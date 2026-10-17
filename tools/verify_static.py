#!/usr/bin/env python3
"""Static verification that needs no ROS or Gazebo (runs on the Mac).

Checks: Python compiles; every YAML/XML parses; package.xml names match folders;
the maze read from main's worlds/custom_maze.sdf loads and its goals are reachable; the committed
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
    L = maze_tools.load()
    check("layout read from custom_maze.sdf is 6 rows x 7 cols",
          (L["rows"], L["cols"]) == (6, 7), f"{L['rows']}x{L['cols']}")
    check("start and goals reachable",
          all(maze_tools.shortest_cells(L, L["start_cell"], g) for g in L["goal_cells"]))
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
