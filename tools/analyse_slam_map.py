#!/usr/bin/env python3
"""Offline diagnosis of a saved SLAM map (no ROS): is the failure a global misalignment or a distorted map?

  python3 tools/analyse_slam_map.py [MAP.yaml] [--origin-in-world 0.25 0.0]

Prints (1) validate_map metrics at the assumed alignment, (2) the best rigid alignment (yaw +-12 deg,
translation +-0.2 m) and its precision - if that is still low the map itself is distorted, not just offset -
and (3) the fraction of line-like occupied pixels within 5 deg of a maze axis (the maze has only axis-aligned
walls, so a healthy map is high; the ground-truth map is the reference). Needs numpy + pyyaml.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_mapping/scripts"))
import validate_map as vm  # noqa: E402

DEFAULT = ROOT / "ros2_ws/src/slam_mapping/maps/maze_slam.yaml"
GT = ROOT / "ros2_ws/src/slam_mapping/maps/maze_groundtruth.yaml"


def occupied_points(path):
    cfg, w, h, grid = vm.load_map(path)
    res, (ox, oy) = cfg["resolution"], cfg["origin"][:2]
    return np.array([(ox + (i + .5) * res, oy + (h - 1 - j + .5) * res)
                     for j in range(h) for i in range(w) if grid[j][i] == 1]), grid


def axis_fraction(grid):
    pts = np.argwhere(np.array(grid) == 1)
    angs = []
    for j, i in pts:
        nb = pts[(abs(pts[:, 0] - j) <= 3) & (abs(pts[:, 1] - i) <= 3)]
        if len(nb) < 5:
            continue
        e, v = np.linalg.eigh(np.cov((nb - nb.mean(0)).T))
        if e[1] < 3 * max(e[0], 1e-9):
            continue                                      # not line-like
        a = math.degrees(math.atan2(v[0, 1], v[1, 1])) % 90
        angs.append(min(a, 90 - a))
    angs = np.array(angs)
    return len(angs), float((angs <= 5).mean()) if len(angs) else 0.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml", nargs="?", default=str(DEFAULT))
    ap.add_argument("--origin-in-world", nargs=2, type=float, default=[0.25, 0.0])
    a = ap.parse_args(argv)
    tools = vm._import_layout_tools(None)
    layout = tools.load()
    segs = tools.wall_segments(layout)
    print("validate_map at assumed alignment:", vm.validate(a.map_yaml, layout, tuple(a.origin_in_world)))
    P, grid = occupied_points(a.map_yaml)

    def dist(Q):
        d = np.full(len(Q), 9.0)
        for x0, y0, x1, y1 in segs:
            dx, dy = x1 - x0, y1 - y0
            t = np.clip(((Q[:, 0] - x0) * dx + (Q[:, 1] - y0) * dy) / (dx * dx + dy * dy), 0, 1)
            d = np.minimum(d, np.hypot(Q[:, 0] - (x0 + t * dx), Q[:, 1] - (y0 + t * dy)))
        return d

    ox, oy = a.origin_in_world
    best = (0.0, 0.0, ox, oy)
    for yaw in np.radians(np.arange(-12, 12.1, 1.0)):
        c, s = math.cos(yaw), math.sin(yaw)
        for tx in np.arange(ox - 0.2, ox + 0.2001, 0.025):
            for ty in np.arange(oy - 0.2, oy + 0.2001, 0.025):
                q = np.c_[c * P[:, 0] - s * P[:, 1] + tx, s * P[:, 0] + c * P[:, 1] + ty]
                p = float((dist(q) <= 0.1).mean())
                if p > best[0]:
                    best = (p, math.degrees(yaw), tx, ty)
    print("occupied pixels:", len(P))
    print("best rigid alignment: precision %.3f at yaw %.1f deg, origin (%.3f, %.3f)" % best)
    n, frac = axis_fraction(grid)
    _, gt_grid = occupied_points(GT)
    n0, frac0 = axis_fraction(gt_grid)
    print(f"line-like pixels within 5 deg of an axis: this map {frac:.2f} (n={n}); ground truth {frac0:.2f} (n={n0})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
