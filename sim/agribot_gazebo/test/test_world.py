"""The generated world is well formed, and the scenery stays out of the way."""
import pathlib
import xml.etree.ElementTree as ET

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
ROOT = ET.parse(PKG / "worlds/demo_row.sdf").getroot()
WORLD = ROOT.find("world")
SCENERY = ("drip_", "crate_stack", "barrel_", "fence_", "tree_", "hill_", "headland")


def pose_of(element):
    values = [float(v) for v in (element.findtext("pose") or "0 0 0 0 0 0").split()]
    return values[:3]


def test_world_has_the_expected_pieces():
    assert WORLD.get("name") == "demo_row"
    names = [m.get("name") for m in WORLD.findall("model")]
    assert len(names) == len(set(names)), "duplicate model names"
    assert "ground" in names
    assert sum(n.startswith("plant_") for n in names) == 24
    for prefix in SCENERY:
        assert any(n.startswith(prefix) for n in names), prefix


def test_scenery_is_visual_only():
    for m in WORLD.findall("model"):
        if m.get("name").startswith(SCENERY):
            assert m.find("static").text == "true"
            assert not list(m.iter("collision")), m.get("name")
            assert list(m.iter("visual")), m.get("name")


def test_the_only_collisions_in_the_world_file_are_the_ground():
    collisions = [(m.get("name"), c.get("name")) for m in WORLD.findall("model") for c in m.iter("collision")]
    assert collisions == [("ground", "plane")]


def test_scenery_keeps_clear_of_the_rovers_corridor():
    """The rover drives the furrow (y = 1) from x = -1.6 toward +x: nothing but beds and plants within 1.2 m of it."""
    for m in WORLD.findall("model"):
        name = m.get("name")
        if not name.startswith(SCENERY) or name.startswith(("hill_", "fence_wire", "drip_")):
            continue
        x, y, _ = pose_of(m)
        assert not (-3.0 < x < 5.2 and abs(y - 1.0) < 1.2), f"{name} at ({x}, {y}) is in the rover's corridor"


def test_drip_lines_lie_on_the_beds():
    for m in WORLD.findall("model"):
        if m.get("name").startswith("drip_"):
            x, y, z = pose_of(m)
            assert x == 0 and z == pytest.approx(0.162)
            bed = 0 if y < 1.0 else 2.0
            assert abs(y - bed) < 0.2, "a drip line must run along a bed's centre"


def test_every_included_model_exists():
    models = PKG.parent / "agribot_field/models"
    for inc in WORLD.iter("include"):
        uri = inc.findtext("uri")
        assert uri.startswith("model://")
        assert (models / uri[len("model://"):] / "model.sdf").exists(), uri
