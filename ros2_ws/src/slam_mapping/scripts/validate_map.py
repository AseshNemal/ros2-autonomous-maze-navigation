#!/usr/bin/env python3
"""Validate an occupancy map against the maze layout (pure Python, no ROS).

  validate_map.py MAP.yaml [--map-origin-in-world X Y] [--layout worlds/custom_maze.sdf]

--map-origin-in-world: world position of the map frame origin. 0 0 (default) for a
map built from the world frame (ground truth); a SLAM map starts where the robot
spawned, e.g. 0.25 0.0. Yaw offset is assumed 0 (spawn yaw 0).

Metrics: wall_coverage = fraction of true wall points with an occupied pixel
within 2 px; precision = fraction of occupied pixels within 0.1 m of a true wall;
free_ok = fraction of cell centres NOT marked occupied. PASS needs coverage>=0.85,
precision>=0.85, free_ok>=0.95. A pass is geometric agreement with the layout, not
proof of navigation performance.
"""
import argparse
import math
import sys
from pathlib import Path

import yaml


def read_pgm(path):
    data = Path(path).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos)
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b"P5":
        raise ValueError("only binary PGM (P5) supported")
    pixels = data[pos + 1:pos + 1 + w * h]
    if len(pixels) != w * h:
        raise ValueError("truncated PGM")
    return w, h, maxval, pixels


def load_map(yaml_path):
    cfg = yaml.safe_load(Path(yaml_path).read_text())
    img = Path(cfg["image"])
    if not img.is_absolute():
        img = Path(yaml_path).parent / img
    w, h, maxval, px = read_pgm(img)
    neg = int(cfg.get("negate", 0))
    occ_t, free_t = cfg.get("occupied_thresh", 0.65), cfg.get("free_thresh", 0.196)
    grid = []  # 1 occupied, 0 free, -1 unknown ; row 0 = top
    for j in range(h):
        row = []
        for i in range(w):
            v = px[j * w + i] / maxval
            p = v if neg else 1.0 - v
            row.append(1 if p > occ_t else (0 if p < free_t else -1))
        grid.append(row)
    return cfg, w, h, grid


def seg_dist(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def validate(map_yaml, layout, offset=(0.0, 0.0)):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    cfg, w, h, grid = load_map(map_yaml)
    res = cfg["resolution"]
    ox, oy = cfg["origin"][0], cfg["origin"][1]

    def to_px(wx, wy):  # world -> (col,row)
        mx, my = wx - offset[0], wy - offset[1]
        return int(math.floor((mx - ox) / res)), h - 1 - int(math.floor((my - oy) / res))

    def occupied_near(wx, wy, r=2):
        c, rr = to_px(wx, wy)
        return any(0 <= rr + dj < h and 0 <= c + di < w and grid[rr + dj][c + di] == 1
                   for dj in range(-r, r + 1) for di in range(-r, r + 1))

    segs = wall_segments(layout)
    hit = tot = 0
    for x0, y0, x1, y1 in segs:
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.025))
        for k in range(n + 1):
            tot += 1
            hit += occupied_near(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
    occ = good = 0
    for j in range(h):
        for i in range(w):
            if grid[j][i] == 1:
                occ += 1
                wx = ox + (i + 0.5) * res + offset[0]
                wy = oy + (h - 1 - j + 0.5) * res + offset[1]
                good += any(seg_dist(wx, wy, *s) <= 0.1 for s in segs)
    free_ok = cnt = 0
    for r in range(layout["rows"]):
        for c in range(layout["cols"]):
            cx, cy = cell_center(layout, r, c)
            col, row = to_px(cx, cy)
            cnt += 1
            inside = 0 <= row < h and 0 <= col < w
            free_ok += inside and grid[row][col] != 1
    unknown = sum(v == -1 for row in grid for v in row) / (w * h)
    res_d = {"size_px": [w, h], "size_m": [round(w * res, 3), round(h * res, 3)],
             "resolution": res, "unknown_fraction": round(unknown, 3),
             "wall_coverage": round(hit / tot, 3), "precision": round(good / occ, 3) if occ else 0.0,
             "free_ok": round(free_ok / cnt, 3)}
    res_d["pass"] = (res_d["wall_coverage"] >= 0.85 and res_d["precision"] >= 0.85
                     and res_d["free_ok"] >= 0.95)
    return res_d


def _import_layout_tools(layout_path):
    global wall_segments, cell_center
    here = Path(__file__).resolve().parent
    candidates = [here.parent.parent / "maze_simulation" / "scripts",
                  Path(layout_path).parent.parent / "scripts" if layout_path else here]
    try:
        from ament_index_python.packages import get_package_prefix
        candidates.append(Path(get_package_prefix("maze_simulation")) / "lib" / "maze_simulation")
    except Exception:
        pass
    for c in candidates:
        if (c / "maze_tools.py").is_file():
            sys.path.insert(0, str(c))
            import maze_tools
            wall_segments, cell_center = maze_tools.wall_segments, maze_tools.cell_center
            return maze_tools
    raise RuntimeError("maze_tools.py (maze_simulation package) not found")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml")
    ap.add_argument("--map-origin-in-world", nargs=2, type=float, default=[0.0, 0.0])
    ap.add_argument("--layout", default=None)
    a = ap.parse_args(argv)
    tools = _import_layout_tools(a.layout)
    result = validate(a.map_yaml, tools.load(a.layout), tuple(a.map_origin_in_world))
    for k, v in result.items():
        print(f"{k}: {v}")
    print("RESULT:", "PASS" if result["pass"] else "FAIL")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
