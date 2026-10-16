"""Pure-Python tests for the map validator and maze layout (no ROS needed)."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
SRC = PKG.parent
sys.path.insert(0, str(PKG / "scripts"))
sys.path.insert(0, str(SRC / "maze_simulation" / "scripts"))

import maze_tools  # noqa: E402
import validate_map  # noqa: E402

validate_map._import_layout_tools(None)


def test_groundtruth_map_matches_layout():
    r = validate_map.validate(PKG / "maps" / "maze_groundtruth.yaml", maze_tools.load())
    assert r["pass"], r


def test_shifted_groundtruth_fails():
    r = validate_map.validate(PKG / "maps" / "maze_groundtruth.yaml", maze_tools.load(), (0.6, 0.6))
    assert not r["pass"]


def test_prototype_test_map_is_not_the_maze():
    p = SRC.parent.parent / "maps" / "test_map.yaml"
    r = validate_map.validate(p, maze_tools.load())
    assert not r["pass"] and r["unknown_fraction"] > 0.5
