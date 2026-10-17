#!/usr/bin/env python3
"""Cloud-side mapping check (no ROS/Gazebo): ray-cast a simulated TurtleBot3 LiDAR along the
coverage tour with PERFECT poses, build an occupancy grid, and run validate_map.py on it.

This is NOT SLAM. It measures the best map the tour + LiDAR geometry + validator can give
if localisation were perfect, so a SLAM failure can be separated from a tour/validator
limit. Optional --yaw-drift (deg per metre driven) injects odometry heading error to show
how much the validator punishes drift. Needs numpy + pyyaml only.

  python3 tools/offline_mapping_check.py [--yaw-drift 0.0] [--noise 0.01] [--hz 5.0]
"""
import argparse
import importlib.util
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "ros2_ws/src/slam_mapping/scripts"
sys.path.insert(0, str(ROOT / "ros2_ws/src/maze_simulation/scripts"))
import maze_tools  # noqa: E402


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def raycast(boxes, px, py, angles, rmin, rmax):
    """Range to the nearest wall box along each ray (inf = none). boxes: (n,4) xmin,ymin,xmax,ymax."""
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        tx0, tx1 = (boxes[:, 0] - px) / dx, (boxes[:, 2] - px) / dx
        ty0, ty1 = (boxes[:, 1] - py) / dy, (boxes[:, 3] - py) / dy
    tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
    tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
    hit = (tmax >= np.maximum(tmin, 0)) & np.isfinite(tmin)
    t = np.where(hit, np.maximum(tmin, 0), np.inf).min(axis=1)
    return np.where((t >= rmin) & (t <= rmax), t, np.inf)


def tour_poses(layout, tools, tour, step):
    """Dense (x, y, yaw) poses along the coverage tour, heading along the motion."""
    wps = [tools.cell_center(layout, r, c) for r, c in tour.coverage_route(layout, tools)]
    poses, yaw, dist = [], 0.0, 0.0
    for (x0, y0), (x1, y1) in zip(wps, wps[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        target = math.atan2(y1 - y0, x1 - x0)
        turn = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        for k in range(int(abs(turn) / 0.1)):          # in-place turn, one scan per ~0.1 rad
            yaw += math.copysign(0.1, turn)
            poses.append((x0, y0, yaw, dist))
        yaw = target
        for k in range(1, int(seg / step) + 1):
            poses.append((x0 + (x1 - x0) * k * step / seg, y0 + (y1 - y0) * k * step / seg, yaw,
                          dist + k * step))
        dist += seg
    return poses


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yaw-drift", type=float, default=0.0, help="deg of heading error per metre driven")
    ap.add_argument("--noise", type=float, default=0.01, help="LiDAR range std-dev (m); TB3 sim uses 0.01")
    ap.add_argument("--hz", type=float, default=5.0, help="LiDAR rate")
    ap.add_argument("--speed", type=float, default=0.12)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    layout = maze_tools.load()
    tour = _load("tour", SCRIPTS / "tour_map_drive.py")
    val = _load("validate_map", SCRIPTS / "validate_map.py")
    val._import_layout_tools(None)

    half = layout["wall_thickness_m"] / 2
    boxes = np.array([(min(x0, x1) - half, min(y0, y1) - half, max(x0, x1) + half, max(y0, y1) + half)
                      for x0, y0, x1, y1 in maze_tools.wall_segments(layout)])
    res, margin = 0.05, 0.25
    hx, hy = layout["cols"] * layout["cell_size_m"] / 2 + margin, layout["rows"] * layout["cell_size_m"] / 2 + margin
    W, H = round(2 * hx / res), round(2 * hy / res)
    cx, cy = maze_tools.centre(layout)          # the maze is not centred on the world origin
    hits, miss = np.zeros((H, W)), np.zeros((H, W))
    angles0 = np.radians(np.arange(360))
    poses = tour_poses(layout, maze_tools, tour, a.speed / a.hz)
    sx, sy = maze_tools.cell_center(layout, *layout["start_cell"])
    for x, y, yaw, dist in poses:
        belief_yaw = yaw + math.radians(a.yaw_drift) * dist          # what the mapper believes
        r = raycast(boxes, x, y, angles0 + yaw, 0.12, 3.5)
        ok = np.isfinite(r)
        r = r + rng.normal(0, a.noise, r.shape)
        ang = angles0 + belief_yaw
        # mapper integrates with believed pose = true pos rotated about the start by the drift
        d = math.radians(a.yaw_drift) * dist
        bx = sx + (x - sx) * math.cos(d) - (y - sy) * math.sin(d)
        by = sy + (x - sx) * math.sin(d) + (y - sy) * math.cos(d)
        for rr, o, an in zip(r, ok, ang):
            if not o:
                continue
            n = int(rr / (res / 2))
            for s in range(n):                                      # free space along the ray
                t = s * res / 2
                i, j = int((bx + t * math.cos(an) - (cx - hx)) / res), int(((cy + hy) - (by + t * math.sin(an))) / res)
                if 0 <= i < W and 0 <= j < H:
                    miss[j, i] += 1
            i, j = int((bx + rr * math.cos(an) - (cx - hx)) / res), int(((cy + hy) - (by + rr * math.sin(an))) / res)
            if 0 <= i < W and 0 <= j < H:
                hits[j, i] += 1
    occ = (hits >= 2) & (hits > 0.3 * miss)
    img = np.full((H, W), 205, np.uint8)
    img[miss > 0] = 254
    img[occ] = 0
    with tempfile.TemporaryDirectory() as d:
        pgm, yml = Path(d) / "m.pgm", Path(d) / "m.yaml"
        pgm.write_bytes(f"P5\n{W} {H}\n255\n".encode() + img.tobytes())
        yml.write_text(f"image: m.pgm\nmode: trinary\nresolution: {res}\norigin: [{cx - hx:.3f}, {cy - hy:.3f}, 0.0]\n"
                       "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        out = val.validate(str(yml), layout, (0.0, 0.0))
    print(f"scans={len(poses)} yaw_drift={a.yaw_drift} deg/m noise={a.noise} m")
    for k, v in out.items():
        print(f"{k}: {v}")
    print("RESULT:", "PASS" if out["pass"] else "FAIL")
    return 0 if out["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
