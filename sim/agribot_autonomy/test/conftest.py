"""Shared fixtures. Tests that need rclpy are skipped when no ROS 2 environment is sourced,
so the pure-logic tests run anywhere (`python -m pytest sim/agribot_autonomy/test`)."""
import pathlib
import sys

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]
sys.path.insert(0, str(PKG))

try:
    import rclpy  # noqa: F401
    import agribot_interfaces.msg  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

ros = pytest.mark.skipif(not HAVE_ROS, reason="needs a sourced ROS 2 workspace with agribot_interfaces")


@pytest.fixture(scope="session")
def urdf_text():
    """The generated whole-robot URDF with the SCARA mast pin the simulation uses (3).

    The standalone agribot.urdf is written for pin 2 (0.46 m); the simulation's xacro
    default is pin 3 (0.64 m), so that one number is patched here.
    """
    text = (REPO / "sim/agribot_description/urdf/agribot.urdf").read_text(encoding="utf-8")
    pin2 = 'xyz="0 -0.095 0.46"'
    assert text.count(pin2) == 1, "the mast joint moved; update this fixture"
    return text.replace(pin2, 'xyz="0 -0.095 0.64"')


@pytest.fixture(scope="session")
def robot(urdf_text):
    from agribot_autonomy.urdf_fk import Robot
    return Robot(xml=urdf_text)
