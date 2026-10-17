#!/usr/bin/env python3
"""Maze utilities (pure Python, no ROS needed).

  maze_tools.py map    OUT_PREFIX     ground-truth occupancy map (.pgm/.yaml)
  maze_tools.py paths                 BFS/A* reference path lengths per goal
  maze_tools.py check                 validate layout and print ASCII art

The Gazebo world is main's worlds/custom_maze.sdf (not generated here). The layout is read from that file by load(); the ground-truth map
is rasterised from it; it is NOT a SLAM result.
"""
import argparse
import heapq
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

CELL = 0.5   # Figure 1 cell size in metres (main's generate_maze.py: CELL = 0.5)


def _package_dir():
    """maze_simulation source dir, or its installed share dir."""
    here = Path(__file__).resolve().parent.parent
    if (here / "worlds" / "custom_maze.sdf").is_file():
        return here
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("maze_simulation"))


def load(path=None):
    """Layout read from main's worlds/custom_maze.sdf (the only maze definition), plus the
    start/goal cells from config/mission.yaml.

    The SDF boxes are thin walls on a 0.5 m grid; each is converted to the wall-between-cells
    form used by the planners: vertical_walls [col_line, row], horizontal_walls [row_line, col].
    """
    pkg = _package_dir()
    sdf = Path(path) if path else pkg / "worlds" / "custom_maze.sdf"
    segs, heights = [], []
    for m in ET.parse(sdf).getroot().iter("model"):
        if m.get("name").startswith("ground"):
            continue
        x, y = (float(v) for v in m.findtext("pose").split()[:2])
        sx, sy, sz = (float(v) for v in m.find(".//collision//size").text.split()[:3])
        heights.append(sz)
        if sx >= sy:
            segs.append(("h", y, x - sx / 2, x + sx / 2, sy))
        else:
            segs.append(("v", x, y - sy / 2, y + sy / 2, sx))
    xs = [v for k, c, lo, hi, t in segs for v in ((lo, hi) if k == "h" else (c,))]
    ys = [v for k, c, lo, hi, t in segs for v in ((lo, hi) if k == "v" else (c,))]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    cols, rows = round((x1 - x0) / CELL), round((y1 - y0) / CELL)
    vertical, horizontal = [], []
    for kind, c, lo, hi, t in segs:
        if kind == "v":
            line = round((c - x0) / CELL)
            if 0 < line < cols:
                vertical += [[line, r] for r in range(round((y1 - hi) / CELL), round((y1 - lo) / CELL))]
        else:
            line = round((y1 - c) / CELL)
            if 0 < line < rows:
                horizontal += [[line, col] for col in range(round((lo - x0) / CELL), round((hi - x0) / CELL))]
    layout = {"cell_size_m": CELL, "rows": rows, "cols": cols,
              "center_x_m": (x0 + x1) / 2, "center_y_m": (y0 + y1) / 2,
              "wall_thickness_m": min(t for *_, t in segs), "wall_height_m": max(heights),
              "vertical_walls": sorted(vertical), "horizontal_walls": sorted(horizontal)}
    with open(pkg / "config" / "mission.yaml") as f:
        layout.update(yaml.safe_load(f))
    return layout


def centre(layout):
    """World position of the maze centre (the Figure 1 maze is not centred on the origin)."""
    return layout.get("center_x_m", 0.0), layout.get("center_y_m", 0.0)


def spawn_xy(layout):
    """World (x, y) where the robot is spawned (main's custom_maze.launch.py: 0.25, 0.0)."""
    return tuple(layout["spawn_xy"])


def cell_center(layout, row, col):
    """World (x, y) of a cell centre (maze centre at center_x_m/center_y_m, default origin)."""
    s = layout["cell_size_m"]
    cx, cy = centre(layout)
    return ((col + 0.5) * s - layout["cols"] * s / 2 + cx,
            layout["rows"] * s / 2 - (row + 0.5) * s + cy)


def wall_segments(layout):
    """Return list of (x0, y0, x1, y1) wall centre lines, outer + interior."""
    s, R, C = layout["cell_size_m"], layout["rows"], layout["cols"]
    hx, hy = C * s / 2, R * s / 2
    cx, cy = centre(layout)
    segs = [(-hx, hy, hx, hy), (-hx, -hy, hx, -hy),
            (-hx, -hy, -hx, hy), (hx, -hy, hx, hy)]
    for line, row in layout["vertical_walls"]:
        x = line * s - hx
        segs.append((x, hy - row * s, x, hy - (row + 1) * s))
    for line, col in layout["horizontal_walls"]:
        y = hy - line * s
        segs.append((col * s - hx, y, (col + 1) * s - hx, y))
    return [(x0 + cx, y0 + cy, x1 + cx, y1 + cy) for x0, y0, x1, y1 in segs]


def open_neighbours(layout):
    """Adjacency of cells: neighbours not separated by a wall."""
    R, C = layout["rows"], layout["cols"]
    vw = {tuple(w) for w in layout["vertical_walls"]}
    hw = {tuple(w) for w in layout["horizontal_walls"]}
    adj = {}
    for r in range(R):
        for c in range(C):
            n = []
            if c > 0 and (c, r) not in vw:
                n.append((r, c - 1))
            if c < C - 1 and (c + 1, r) not in vw:
                n.append((r, c + 1))
            if r > 0 and (r, c) not in hw:
                n.append((r - 1, c))
            if r < R - 1 and (r + 1, c) not in hw:
                n.append((r + 1, c))
            adj[(r, c)] = n
    return adj


def shortest_cells(layout, a, b):
    """A* over the cell graph (unit edges = one cell). Returns path or None."""
    adj = open_neighbours(layout)
    a, b = tuple(a), tuple(b)
    h = lambda p: abs(p[0] - b[0]) + abs(p[1] - b[1])
    pq, best, prev = [(h(a), 0, a)], {a: 0}, {}
    while pq:
        _, g, cur = heapq.heappop(pq)
        if cur == b:
            path = [cur]
            while cur in prev:
                cur = prev[cur]
                path.append(cur)
            return path[::-1]
        for n in adj[cur]:
            if g + 1 < best.get(n, 1e9):
                best[n], prev[n] = g + 1, cur
                heapq.heappush(pq, (g + 1 + h(n), g + 1, n))
    return None


def turns(path):
    t = 0
    for i in range(2, len(path)):
        d1 = (path[i-1][0]-path[i-2][0], path[i-1][1]-path[i-2][1])
        d2 = (path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
        t += d1 != d2
    return t


def reference_paths(layout):
    """Grid-optimal lower bounds (centre-line, 0.5 m per cell)."""
    out = []
    for g in layout["goal_cells"]:
        p = shortest_cells(layout, layout["start_cell"], g)
        if p is None:
            raise ValueError(f"goal {g} unreachable from start")
        out.append({"goal_cell": list(g), "goal_xy": cell_center(layout, *g),
                    "cells": len(p) - 1,
                    "length_m": (len(p) - 1) * layout["cell_size_m"],
                    "turns": turns(p)})
    return out


def write_map(layout, prefix, res=0.05, margin=0.25):
    """Rasterise walls into a trinary occupancy grid (free=254, occ=0)."""
    s, R, C = layout["cell_size_m"], layout["rows"], layout["cols"]
    hx, hy = C * s / 2 + margin, R * s / 2 + margin
    cx, cy = centre(layout)
    W, H = round(2 * hx / res), round(2 * hy / res)
    img = [[254] * W for _ in range(H)]
    # a wall thinner than a pixel must still be drawn: raster at least one cell wide
    half = max(layout["wall_thickness_m"], res) / 2
    for x0, y0, x1, y1 in wall_segments(layout):
        xa, xb = min(x0, x1) - half, max(x0, x1) + half
        ya, yb = min(y0, y1) - half, max(y0, y1) + half
        for j in range(H):
            y = cy + hy - (j + 0.5) * res
            if ya <= y <= yb:
                for i in range(W):
                    x = cx + (i + 0.5) * res - hx
                    if xa <= x <= xb:
                        img[j][i] = 0
    pgm = Path(f"{prefix}.pgm")
    pgm.write_bytes(f"P5\n{W} {H}\n255\n".encode() + bytes(v for row in img for v in row))
    Path(f"{prefix}.yaml").write_text(
        f"image: {pgm.name}\nmode: trinary\nresolution: {res}\n"
        f"origin: [{cx - hx:.3f}, {cy - hy:.3f}, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return W, H


def ascii_art(layout):
    R, C = layout["rows"], layout["cols"]
    vw = {tuple(w) for w in layout["vertical_walls"]}
    hw = {tuple(w) for w in layout["horizontal_walls"]}
    lines = ["+" + "--+" * C]
    for r in range(R):
        row = "|"
        for c in range(C):
            row += "  "
            row += "|" if (c == C - 1 or (c + 1, r) in vw) else " "
        lines.append(row)
        lines.append("+" + "".join(("--" if (r == R - 1 or (r + 1, c) in hw) else "  ") + "+" for c in range(C)))
    return "\n".join(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=["map", "paths", "check"])
    p.add_argument("output", nargs="?")
    p.add_argument("--layout", default=None)
    a = p.parse_args(argv)
    layout = load(a.layout)
    if a.command == "map":
        print("wrote %dx%d map" % write_map(layout, a.output))
    elif a.command == "paths":
        for r in reference_paths(layout):
            print(r)
    else:
        print(ascii_art(layout))
        print("spawn", spawn_xy(layout), "start cell", layout["start_cell"],
              cell_center(layout, *layout["start_cell"]))
        for r in reference_paths(layout):
            print("goal", r["goal_cell"], "xy=(%.2f, %.2f)" % tuple(r["goal_xy"]),
                  "cells", r["cells"], "turns", r["turns"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
