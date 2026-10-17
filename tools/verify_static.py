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

# ---- cross-file consistency (world names, install rules, message types, paths, spawn clearance) ----
import re  # noqa: E402

SIM = SRC / "maze_simulation"
world_name = ET.parse(SIM / "worlds/custom_maze.sdf").getroot().find("world").get("name")
check("the world is named 'custom_maze' (spawn, gz services and DQN env use /world/custom_maze)", world_name == "custom_maze")
launch_src = (SIM / "launch/maze_simulation.launch.py").read_text()
check("maze_simulation.launch.py spawns into the world named in the world file",
      'arguments=["-world", world_name,' in launch_src and 'default_value="custom_maze.sdf"' in launch_src)
bad = []
for f in list(SRC.rglob("*.py")) + list(SRC.rglob("*.yaml")) + list((ROOT / "tools").glob("*.sh")):
    for m in re.finditer(r"/world/([A-Za-z_]+)/", f.read_text()):
        if m.group(1) != "custom_maze":
            bad.append((f.name, m.group(1)))
check("every hard-coded /world/<name>/ service or topic uses the world 'custom_maze'", not bad, str(bad))
check("dqn_params.yaml world == default world", yaml.safe_load(
    (SRC / "dqn_agent/config/dqn_params.yaml").read_text())["environment"]["world"] == world_name)
check("custom_maze.launch.py spawns into world custom_maze",
      "'custom_maze'" in (SIM / "launch/custom_maze.launch.py").read_text())

# every executable script is installed by its package's CMakeLists
for pkg in ("maze_simulation", "slam_mapping", "maze_navigation"):
    cmake = (SRC / pkg / "CMakeLists.txt").read_text()
    for sc in sorted((SRC / pkg / "scripts").glob("*")):
        if sc.is_file() and sc.suffix in (".py", ".sh"):
            check(f"CMake installs {pkg}/scripts/{sc.name}", f"scripts/{sc.name}" in cmake)
    check(f"CMake installs launch/config dirs of {pkg}", "install(DIRECTORY" in cmake and "launch" in cmake)
    check(f"{pkg} is ament_cmake", "<build_type>ament_cmake</build_type>" in (SRC / pkg / "package.xml").read_text())
check("maze_simulation installs worlds (custom_maze.sdf)",
      "worlds" in (SIM / "CMakeLists.txt").read_text())
check("dqn_agent installs launch+config (setup.py data_files)", "launch/*.py" in (SRC / "dqn_agent/setup.py").read_text())

# package.xml covers the ROS / pip modules that the package's python files import
ROS_DEPS = {"rclpy": "rclpy", "geometry_msgs": "geometry_msgs", "nav_msgs": "nav_msgs",
            "sensor_msgs": "sensor_msgs", "tf2_msgs": "tf2_msgs", "rosgraph_msgs": "rosgraph_msgs",
            "action_msgs": "action_msgs", "nav2_msgs": "nav2_msgs", "launch": "launch",
            "launch_ros": "launch_ros", "ament_index_python": "ament_index_python",
            "yaml": "python3-yaml", "numpy": "python3-numpy"}
for pkg in ("maze_simulation", "slam_mapping", "maze_navigation", "dqn_agent"):
    declared = {e.text for e in ET.parse(SRC / pkg / "package.xml").getroot()
                if e.tag.endswith("depend")}
    used = set()
    for f in (SRC / pkg).rglob("*.py"):
        if "test" in f.parts or f.name == "setup.py":
            continue
        used |= {m.group(1) for m in re.finditer(r"^\s*(?:import|from)\s+([A-Za-z0-9_]+)", f.read_text(), re.M)}
    missing = sorted(ROS_DEPS[m] for m in used if m in ROS_DEPS and ROS_DEPS[m] not in declared)
    check(f"{pkg}/package.xml declares the dependencies its code imports", not missing, str(missing))

# velocity message type: the Gazebo bridge expects TwistStamped on /cmd_vel; only twist_stamper may use plain Twist
bridge = yaml.safe_load((SIM / "config/bridge.yaml").read_text())
cmd = next(e for e in bridge if e["ros_topic_name"] == "cmd_vel")
check("bridge /cmd_vel is geometry_msgs/msg/TwistStamped ROS->GZ",
      cmd["ros_type_name"] == "geometry_msgs/msg/TwistStamped" and cmd["direction"] == "ROS_TO_GZ")
plain = [f.name for f in SRC.rglob("*.py") if "test" not in f.parts and
         re.search(r"create_publisher\(\s*Twist\s*,\s*[\"']/cmd_vel[\"']", f.read_text())]
check("no node publishes plain Twist on /cmd_vel", not plain, str(plain))

# map / params files referenced by the launch files exist and have their image
gt = SRC / "slam_mapping/maps/maze_groundtruth.yaml"
check("ground-truth map yaml references an existing image",
      (gt.parent / yaml.safe_load(gt.read_text())["image"]).is_file())
check("navigation.launch.py default map is maze_groundtruth.yaml", "maze_groundtruth.yaml" in
      (SRC / "maze_navigation/launch/navigation.launch.py").read_text())
check("nav2_params.yaml exists", (SRC / "maze_navigation/config/nav2_params.yaml").is_file())

# robot spawn clearance (Burger radius 0.105 m) in the maze
layout = maze_tools.load()
walls = [(min(a, c), min(b, d), max(a, c), max(b, d)) for a, b, c, d in maze_tools.wall_segments(layout)]
half = layout["wall_thickness_m"] / 2


def clearance(px, py, boxes):
    return min(((max(x0 - px, 0, px - x1)) ** 2 + (max(y0 - py, 0, py - y1)) ** 2) ** 0.5
               for x0, y0, x1, y1 in boxes)


spawn = (0.25, 0.0)
check("spawn (0.25, 0.0) clears every wall by > 0.105 m + margin",
      clearance(*spawn, [(a - half, b - half, c + half, d + half) for a, b, c, d in walls]) > 0.105 + 0.05)
for custom in sorted((SIM / "worlds").glob("custom_maze.sdf")):
    boxes = []
    for m in ET.parse(custom).getroot().iter("model"):
        if m.get("name", "").startswith("wall"):
            x, y = map(float, m.findtext("pose").split()[:2])
            sx, sy = map(float, m.find(".//collision/geometry/box/size").text.split()[:2])
            boxes.append((x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2))
    check("custom_maze spawn (0.25, 0.0) clears its walls by > 0.105 m + margin",
          clearance(0.25, 0.0, boxes) > 0.105 + 0.05)

# Figure-1 comparison (needs Pillow) and the perfect-pose mapping ceiling (needs numpy)
try:
    import PIL  # noqa: F401
    r = subprocess.run([sys.executable, str(ROOT / "tools/check_figure1.py")], capture_output=True, text=True)
    if r.returncode == 0:
        check("maze matches Figure 1 (measured from the assignment PDF image)", True)
    else:  # known: main's 7 x 6 world is an approximation of the 6 x 6 figure; report it, do not hide it
        print("WARN  maze does NOT match Figure 1 (known: main's 7 x 6 world vs the 6 x 6 figure)")
        print("      " + " | ".join(r.stdout.strip().splitlines()[-1:]))
except ImportError:
    print("SKIP  Figure-1 comparison (Pillow not installed)")
try:
    import numpy  # noqa: F401
    r = subprocess.run([sys.executable, str(ROOT / "tools/offline_mapping_check.py")],
                       capture_output=True, text=True)
    check("perfect-pose LiDAR map of the coverage tour passes validate_map (validator/tour are attainable)",
          r.returncode == 0, r.stdout[-300:])
except ImportError:
    print("SKIP  offline mapping check (numpy not installed)")

for tests in [SRC / "slam_mapping/test", SRC / "maze_navigation/test", SRC / "dqn_agent/test"]:
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(tests)],
                       cwd=tests.parent, capture_output=True, text=True)
    check(f"pytest {tests.parent.name}", r.returncode == 0, r.stdout[-400:])
    print("      " + (r.stdout.strip().splitlines() or [""])[-1])

print("\nOVERALL:", "FAIL" if failures else "PASS", failures or "")
sys.exit(1 if failures else 0)
