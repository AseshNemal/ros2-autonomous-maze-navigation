"""The committed ground-truth map must match the Gazebo world (custom_maze.sdf) wall for wall.

Parses the SDF boxes directly (not via maze_tools.load) so the check is independent of the code
that rasterised the map. Regression test for the missing east outer wall.
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent.parent
SDF = PKG.parent / "maze_simulation" / "worlds" / "custom_maze.sdf"
sys.path.insert(0, str(PKG.parent / "maze_simulation" / "scripts"))

import maze_tools  # noqa: E402


def _map():
    meta = yaml.safe_load((PKG / "maps" / "maze_groundtruth.yaml").read_text())
    raw = (PKG / "maps" / meta["image"]).read_bytes()
    magic, dims, _maxv, body = raw.split(b"\n", 3)
    assert magic == b"P5"
    w, h = map(int, dims.split())
    return meta, w, h, body


def _walls():
    """(x0, y0, x1, y1) world centre lines of every wall box in the SDF."""
    out = []
    for m in ET.parse(SDF).getroot().iter("model"):
        if m.get("name").startswith("ground"):
            continue
        x, y = (float(v) for v in m.findtext("pose").split()[:2])
        sx, sy = (float(v) for v in m.find(".//collision//size").text.split()[:2])
        if sx >= sy:
            out.append((x - sx / 2, y, x + sx / 2, y))
        else:
            out.append((x, y - sy / 2, x, y + sy / 2))
    return out


def _occupied(meta, w, h, body, x, y, reach):
    """True if a wall pixel lies within `reach` metres of world point (x, y)."""
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]
    n = int(round(reach / res))
    ci, cj = int((x - ox) / res), int(h - 1 - (y - oy) / res)
    return any(0 <= i < w and 0 <= j < h and body[j * w + i] == 0
               for i in range(ci - n, ci + n + 1) for j in range(cj - n, cj + n + 1))


def test_every_sdf_wall_is_in_the_map():
    meta, w, h, body = _map()
    res = meta["resolution"]
    for x0, y0, x1, y1 in _walls():
        steps = max(1, int(math.hypot(x1 - x0, y1 - y0) / (res / 2)))
        for k in range(steps + 1):
            x, y = x0 + (x1 - x0) * k / steps, y0 + (y1 - y0) * k / steps
            assert _occupied(meta, w, h, body, x, y, res), ((x0, y0, x1, y1), "missing at", x, y)


def test_no_wall_pixel_away_from_an_sdf_wall():
    meta, w, h, body = _map()
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]
    walls = _walls()
    for j in range(h):
        for i in range(w):
            if body[j * w + i] != 0:
                continue
            x, y = ox + (i + 0.5) * res, oy + (h - j - 0.5) * res
            assert any(min(x0, x1) - 1.5 * res <= x <= max(x0, x1) + 1.5 * res
                       and min(y0, y1) - 1.5 * res <= y <= max(y0, y1) + 1.5 * res
                       for x0, y0, x1, y1 in walls), ("stray wall pixel", i, j, x, y)


def test_outer_boundary_closed_on_all_four_sides():
    meta, w, h, body = _map()
    res = meta["resolution"]
    xs = [v for x0, _y0, x1, _y1 in _walls() for v in (x0, x1)]
    ys = [v for _x0, y0, _x1, y1 in _walls() for v in (y0, y1)]
    x_lo, x_hi, y_lo, y_hi = min(xs), max(xs), min(ys), max(ys)
    ny, nx = int((y_hi - y_lo) / res), int((x_hi - x_lo) / res)
    sides = {"west": [(x_lo, y_lo + k * res) for k in range(ny)],
             "east": [(x_hi, y_lo + k * res) for k in range(ny)],
             "south": [(x_lo + k * res, y_lo) for k in range(nx)],
             "north": [(x_lo + k * res, y_hi) for k in range(nx)]}
    for name, pts in sides.items():
        missing = [p for p in pts if not _occupied(meta, w, h, body, p[0], p[1], res)]
        assert not missing, (name, "outer wall has gaps", missing[:3])


def test_spawn_and_goals_are_free_cells():
    meta, w, h, body = _map()
    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    layout = maze_tools.load()
    pts = [tuple(layout["spawn_xy"])] + [tuple(r["goal_xy"]) for r in maze_tools.reference_paths(layout)]
    assert len(pts) == 4
    for x, y in pts:
        assert not _occupied(meta, w, h, body, x, y, 0.10), ("wall within 0.10 m of", x, y)
