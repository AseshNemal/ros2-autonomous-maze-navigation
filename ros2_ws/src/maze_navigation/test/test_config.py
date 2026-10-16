"""Static consistency checks for the Nav2 config against the maze (no ROS needed)."""
import py_compile
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent.parent
SRC = PKG.parent
sys.path.insert(0, str(SRC / "maze_simulation" / "scripts"))
import maze_tools  # noqa: E402

PARAMS = yaml.safe_load((PKG / "config" / "nav2_params.yaml").read_text())
CORRIDOR_FREE = 0.5 - 0.05  # cell size minus wall thickness


def test_robot_fits_corridor_after_inflation():
    for name in ("local_costmap", "global_costmap"):
        p = PARAMS[name][name]["ros__parameters"]
        assert 2 * p["robot_radius"] < CORRIDOR_FREE, name
        infl = p["inflation_layer"]["inflation_radius"]
        # corridor centre is (CORRIDOR_FREE/2) from the wall surface: keep it cheap
        assert infl <= CORRIDOR_FREE / 2 + 0.05, name


def test_planner_is_astar_and_tolerance_small():
    g = PARAMS["planner_server"]["ros__parameters"]["GridBased"]
    assert g["use_astar"] is True and g["tolerance"] <= 0.15


def test_cmd_vel_chain_avoids_stamped_topic_conflict():
    out = PARAMS["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"]
    assert out == "cmd_vel_nav2"  # twist_stamper republishes to /cmd_vel as TwistStamped


def test_goal_tolerance_less_than_half_cell():
    gc = PARAMS["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert gc["xy_goal_tolerance"] < 0.25


def test_goals_reachable_and_distinct():
    layout = maze_tools.load()
    refs = maze_tools.reference_paths(layout)
    assert len(refs) == 3 and len({tuple(r["goal_cell"]) for r in refs}) == 3


def test_python_sources_compile(tmp_path):
    for f in list((PKG / "scripts").glob("*.py")) + list((PKG / "launch").glob("*.py")):
        py_compile.compile(str(f), cfile=str(tmp_path / (f.name + "c")), doraise=True)
