"""Access to the shared maze layout (owned by maze_simulation) for DQN experiments.

Used for sampling curriculum start/goal pairs and optional waypoint guidance.
Pure Python; works in a source checkout or an installed workspace.
"""
import random
import sys
from pathlib import Path


def _tools():
    here = Path(__file__).resolve()
    candidates = [here.parents[2] / "maze_simulation" / "scripts"]  # source tree: ros2_ws/src
    try:
        from ament_index_python.packages import get_package_prefix
        candidates.append(Path(get_package_prefix("maze_simulation")) / "lib" / "maze_simulation")
    except Exception:  # not in a ROS environment
        pass
    for c in candidates:
        if (c / "maze_tools.py").is_file():
            if str(c) not in sys.path:
                sys.path.insert(0, str(c))
            import maze_tools
            return maze_tools
    raise RuntimeError("maze_tools.py not found: build/source the maze_simulation package")


tools = _tools()


def load_layout(path=None):
    return tools.load(path)


def cell_xy(layout, cell):
    return tools.cell_center(layout, *cell)


def spawn_xy(layout):
    return tools.spawn_xy(layout)


def cell_path(layout, a, b):
    return tools.shortest_cells(layout, a, b)


def waypoints(layout, a, b):
    """World (x, y) of each cell centre after `a` up to and including `b`."""
    path = cell_path(layout, a, b)
    if path is None:
        raise ValueError(f"no path {a}->{b}")
    return [cell_xy(layout, c) for c in path[1:]]


def sample_pair(layout, rng, min_cells=1, max_cells=3):
    """Random (start_cell, goal_cell) whose grid distance is in [min_cells, max_cells]."""
    cells = [(r, c) for r in range(layout["rows"]) for c in range(layout["cols"])]
    for _ in range(1000):
        a, b = rng.choice(cells), rng.choice(cells)
        p = cell_path(layout, a, b)
        if p is not None and min_cells <= len(p) - 1 <= max_cells:
            return a, b
    raise ValueError("no start/goal pair in range")


def evaluation_pairs(layout):
    """The fixed start->goal pairs shared with the Nav2 evaluation."""
    start = tuple(layout["start_cell"])
    return [(start, tuple(g)) for g in layout["goal_cells"]]


def rng_from_seed(seed):
    return random.Random(seed)
