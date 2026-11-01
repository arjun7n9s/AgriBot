"""Fixtures for the simulation package's tests (pure Python: no ROS or Gazebo needed)."""
import pathlib
import sys

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
SIM = PKG.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(SIM / "agribot_autonomy"))


@pytest.fixture(scope="session")
def urdf_text():
    """Whole-robot URDF at the simulation's SCARA mast pin (3); see agribot_autonomy/test/conftest.py."""
    text = (SIM / "agribot_description/urdf/agribot.urdf").read_text(encoding="utf-8")
    pin2 = 'xyz="0 -0.095 0.46"'
    assert text.count(pin2) == 1
    return text.replace(pin2, 'xyz="0 -0.095 0.64"')


@pytest.fixture(scope="session")
def robot(urdf_text):
    from agribot_autonomy.urdf_fk import Robot
    return Robot(xml=urdf_text)
