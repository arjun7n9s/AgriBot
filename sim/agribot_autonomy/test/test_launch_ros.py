"""Launch files: arguments, bring-up of the autonomy stack, and a bad parameter file.

These start real `ros2 launch` processes (no Gazebo). They need the workspace sourced
and are skipped otherwise.
"""
import os
import pathlib
import signal
import subprocess
import tempfile
import time

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("agribot_interfaces.msg")

# Not imported from conftest: run together with agribot_gazebo's tests, `conftest` names the other package's.
PKG = pathlib.Path(__file__).resolve().parents[1]

ENV = {**os.environ, "ROS_DOMAIN_ID": "88", "ROS_LOCALHOST_ONLY": "1"}


def run(cmd, timeout=60):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=ENV)


def nodes():
    return set(run(["ros2", "node", "list"], timeout=20).stdout.split())


@pytest.fixture()
def launched():
    procs = []

    def start(*args):
        log = tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False)
        p = subprocess.Popen(["ros2", "launch", *args], stdout=log, stderr=subprocess.STDOUT, env=ENV,
                             start_new_session=True)
        procs.append((p, log))
        return p, pathlib.Path(log.name)

    yield start
    for p, log in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        log.close()


def wait_until(condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.5)
    return condition()


def test_autonomy_launch_declares_its_arguments():
    out = run(["ros2", "launch", "agribot_autonomy", "autonomy.launch.py", "--show-args"]).stdout
    for name in ("use_sim_time", "params_file", "harvest", "picks", "max_distance"):
        assert f"'{name}'" in out, name


def test_demo_and_sim_launch_declare_their_arguments():
    demo = run(["ros2", "launch", "agribot_gazebo", "demo.launch.py", "--show-args"]).stdout
    for name in ("gui", "rviz", "harvest", "picks", "mast_pin", "params_file", "capture_dir"):
        assert f"'{name}'" in demo, name
    sim = run(["ros2", "launch", "agribot_gazebo", "sim.launch.py", "--show-args"]).stdout
    for name in ("gui", "rviz", "mast_pin"):
        assert f"'{name}'" in sim, name


def test_autonomy_stack_comes_up_and_goes_down_cleanly(launched):
    proc, log = launched("agribot_autonomy", "autonomy.launch.py")
    want = {"/drive_node", "/row_follower", "/fruit_detector"}
    assert wait_until(lambda: want <= nodes(), 30), f"nodes up: {nodes()}\n{log.read_text()[-2000:]}"
    assert "/harvest_mission" not in nodes(), "the mission starts only with harvest:=true"
    time.sleep(3)
    assert proc.poll() is None, log.read_text()[-2000:]
    os.killpg(proc.pid, signal.SIGINT)
    proc.wait(timeout=20)
    assert wait_until(lambda: not (want & nodes()), 20), "nodes left running after the launch ended"


def test_harvest_argument_starts_the_mission_and_its_client(launched):
    proc, log = launched("agribot_autonomy", "autonomy.launch.py", "harvest:=true", "picks:=1")
    assert wait_until(lambda: "/harvest_mission" in nodes(), 30), log.read_text()[-2000:]
    action = run(["ros2", "action", "list"], timeout=20).stdout
    assert "/harvest" in action
    # With no robot behind it, the client's goal waits for the robot, then fails: the stack must stay up meanwhile.
    time.sleep(3)
    assert proc.poll() is None


def test_a_bad_parameter_file_stops_the_node_with_a_reason(launched, tmp_path):
    bad = yaml.safe_load((PKG / "config/autonomy.yaml").read_text(encoding="utf-8"))
    bad["row_follower"]["ros__parameters"]["speed"] = -1.0
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(bad))
    proc, log = launched("agribot_autonomy", "autonomy.launch.py", f"params_file:={path}")
    assert wait_until(lambda: "process has died" in log.read_text(), 30), log.read_text()[-2000:]
    text = log.read_text()
    assert "row_follower" in text and ("speed" in text or "range" in text.lower())
    assert wait_until(lambda: "/row_follower" not in nodes(), 10)
    assert {"/drive_node", "/fruit_detector"} <= nodes(), "the other nodes should be unaffected"


def test_a_missing_parameter_file_aborts_the_launch_with_a_reason(launched):
    proc, log = launched("agribot_autonomy", "autonomy.launch.py", "params_file:=/nonexistent/params.yaml")
    assert wait_until(lambda: proc.poll() is not None, 30), "the launch carried on with defaults"
    assert proc.returncode != 0
    assert "params_file not found: /nonexistent/params.yaml" in log.read_text()
    assert not ({"/drive_node", "/row_follower", "/fruit_detector"} & nodes())


def test_a_parameter_file_missing_a_node_section_aborts_the_launch(launched, tmp_path):
    partial = yaml.safe_load((PKG / "config/autonomy.yaml").read_text(encoding="utf-8"))
    del partial["fruit_detector"]
    path = tmp_path / "partial.yaml"
    path.write_text(yaml.safe_dump(partial))
    proc, log = launched("agribot_autonomy", "autonomy.launch.py", f"params_file:={path}")
    assert wait_until(lambda: proc.poll() is not None, 30)
    assert "no ros__parameters section for: fruit_detector" in log.read_text()


def test_a_parameter_file_that_is_not_yaml_aborts_the_launch(launched, tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text("drive_node: [unclosed\n  x: : :")
    proc, log = launched("agribot_autonomy", "autonomy.launch.py", f"params_file:={path}")
    assert wait_until(lambda: proc.poll() is not None, 30)
    assert "not valid YAML" in log.read_text()
