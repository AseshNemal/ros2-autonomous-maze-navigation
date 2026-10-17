# Gazebo / RViz GUI review (IE4060 Assignment 02)

Goal: the ordinary Gazebo Harmonic and RViz2 interfaces, configured so the maze, robot, goals and path are visible. No custom dashboards, plugins or visual styling.

## What was actually wrong (checked before changing anything)

| # | Where | Finding | Evidence |
|---|---|---|---|
| 1 | `maze_navigation/rviz/navigation.rviz` | Unmodified copy of Nav2's generic `nav2_default_view.rviz` | `cmp` against `/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz` is identical |
| 2 | same | Camera centred at `X = -5.41 m`; the maze is at about (0, 0), so only a corner was on screen | `Views: Current: X: -5.41` |
| 3 | same | RobotModel display disabled, and its QoS (Volatile) cannot receive the latched `/robot_description` | `Enabled: false`, `Durability Policy: Volatile` |
| 4 | same | Displays/panels for hardware we do not have: Bumper Hit (`/mobile_base/sensors/bumper_pointcloud`), Realsense camera panels (`/intel_realsense_r200_*`); also Downsampled Costmap, Voxel Grid, Polygon, Trajectories, Selector, Docking | topics do not exist in the simulation |
| 5 | same | Window `1610x893` at `X=1091` on a `1680x1050` screen: overflows, cannot sit beside Gazebo | `Window Geometry`, `xrandr` |
| 6 | `slam_mapping/rviz/mapping.rviz` | View class `TopDownOrthoView` does not exist (correct: `TopDownOrtho`), so the camera settings were ignored | RViz class list |
| 7 | `maze_simulation/worlds/maze.world` | A custom `<gui>` block replaced Gazebo's default GUI (only 3D view + play button; no entity tree / inspector) | `<gui fullscreen="0">` in the world and in `maze_tools.py` |

Not wrong: maze walls (generated from `maze_layout.yaml`, checked against Figure 1), the TurtleBot3 Burger model and size (stock `turtlebot3_gazebo` model; it only looks small because the maze is 3 m wide).

## What was changed

* **Gazebo:** removed the custom `<gui>` block from the world generator (`maze_tools.py`) and regenerated `maze.world`. Gazebo now opens with its default interface. The maze and robot are unchanged.
* **`navigation.rviz`:** derived from the stock Nav2 file; kept only standard displays: Grid (0.5 m cells = one maze cell), Map, RobotModel (enabled, Transient Local), TF, LaserScan, AMCL particle cloud, Global Costmap, Global Path (`/plan`), Local Costmap, Local Plan. Panels: Displays, Navigation 2, Views. Standard tools incl. **Nav2 Goal** and 2D Pose Estimate. Fixed frame `map`, `TopDownOrtho` view centred on the maze.
* **`mapping.rviz`:** Grid, Map, RobotModel, TF, LaserScan; fixed the view class; same top-down view.
* **Window layout:** both RViz files request the right half of a 1680x1050 screen (`X=840, Width=840`).
* **Scripts:** `demo.sh` / `verify_on_vm.sh` cleanup now also stops `rviz2` and `opennav_docking` (stale copies caused a duplicate RViz and an extra `/cmd_vel` publisher).

## Verified on the VM (2026-10-16)

* `python3 tools/verify_static.py`: PASS (regenerated world equals committed world).
* Headless sim stage: 7/7 PASS with the new world. Default Gazebo GUI + robot spawn: OK.
* Nav2 + RViz with the new config: all lifecycle nodes active; RViz reports no display errors (only a harmless software-GL shader message).
* Goal click: `/goal_pose` (what the RViz Nav2 Goal tool publishes) -> `/plan` in frame `map` -> robot reached the goal.
* Not verified by me: how the windows look on screen (no screenshot tool on the VM) - confirm visually.

## Side-by-side layout for the VIVA

Run `bash tools/demo.sh`. Press **Super+Left** with the Gazebo window focused and **Super+Right** with RViz focused (GNOME window tiling). RViz position hints in the config are ignored by some Wayland sessions, so tiling is the reliable way.
