#!/usr/bin/env python3
"""Static verification that needs no ROS or Gazebo (runs on the Mac).

Checks: Python compiles; every YAML/XML parses; package.xml names match folders;
main's worlds/custom_maze.sdf has the same walls as maze_layout.yaml; the committed
ground-truth map equals a fresh rasterisation; pytest suites for all packages pass.
This proves internal consistency only - NOT that Gazebo/Nav2/SLAM/DQN run.
"""
import compileall
import filecmp
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "ros2_ws" / "src"
sys.path.insert(0, str(SRC / "maze_simulation" / "scripts"))
import maze_tools  # noqa: E402

failures = []


def sdf_matches_layout(tol=0.02):
    """Every layout wall centre line lies inside an SDF wall box, and every SDF wall box
    centre line lies on a layout wall (so the ground-truth map and the world agree)."""
    import math
    boxes = []
    for m in ET.parse(SRC / "maze_simulation/worlds/custom_maze.sdf").getroot().iter("model"):
        if m.get("name").startswith("ground"):
            continue
        x, y = (float(v) for v in m.findtext("pose").split()[:2])
        sx, sy = (float(v) for v in m.find(".//collision//size").text.split()[:2])
        boxes.append((x, y, sx, sy))
    segs = maze_tools.wall_segments(maze_tools.load())

    def pts(x0, y0, x1, y1):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.025))
        return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]

    def in_box(px, py):
        return any(abs(px - x) <= sx / 2 + tol and abs(py - y) <= sy / 2 + tol for x, y, sx, sy in boxes)

    def on_seg(px, py):
        for x0, y0, x1, y1 in segs:
            dx, dy = x1 - x0, y1 - y0
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
            if math.hypot(px - x0 - t * dx, py - y0 - t * dy) <= tol:
                return True
        return False

    if not all(in_box(px, py) for s in segs for px, py in pts(*s)):
        return False
    for x, y, sx, sy in boxes:
        a = (x - sx / 2, y, x + sx / 2, y) if sx >= sy else (x, y - sy / 2, x, y + sy / 2)
        if not all(on_seg(px, py) for px, py in pts(*a)):
            return False
    return True


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f"  {detail}" if detail and not ok else ""))
    if not ok:
        failures.append(name)


check("python compiles", compileall.compile_dir(str(SRC), quiet=2, force=True))
for f in SRC.rglob("*.yaml"):
    try:
        yaml.safe_load(f.read_text())
        ok = True
    except yaml.YAMLError as e:
        ok = False
    check(f"yaml parses: {f.relative_to(ROOT)}", ok)
for f in list(SRC.rglob("package.xml")) + list(SRC.rglob("*.world")) + list(SRC.rglob("*.sdf")):
    try:
        ET.parse(f)
        ok = True
    except ET.ParseError:
        ok = False
    check(f"xml parses: {f.relative_to(ROOT)}", ok)
for pkg in SRC.iterdir():
    px = pkg / "package.xml"
    if px.is_file():
        check(f"package name matches folder: {pkg.name}", ET.parse(px).getroot().findtext("name") == pkg.name)

with tempfile.TemporaryDirectory() as tmp:
    check("custom_maze.sdf walls == maze_layout.yaml walls", sdf_matches_layout())
    maze_tools.main(["map", str(Path(tmp) / "gt")])
    check("committed ground-truth map == regenerated",
          filecmp.cmp(Path(tmp) / "gt.pgm", SRC / "slam_mapping/maps/maze_groundtruth.pgm", shallow=False))

for tests in [SRC / "slam_mapping/test", SRC / "maze_navigation/test", SRC / "dqn_agent/test"]:
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(tests)],
                       cwd=tests.parent, capture_output=True, text=True)
    check(f"pytest {tests.parent.name}", r.returncode == 0, r.stdout[-400:])
    print("      " + (r.stdout.strip().splitlines() or [""])[-1])

print("\nOVERALL:", "FAIL" if failures else "PASS", failures or "")
sys.exit(1 if failures else 0)
