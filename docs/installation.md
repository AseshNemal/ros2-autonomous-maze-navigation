# Installation (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic)

```bash
git clone https://github.com/AseshNemal/ros2-autonomous-maze-navigation.git && cd ros2-autonomous-maze-navigation
git checkout dev-integration
tools/install_deps_ubuntu.sh        # apt: ros-gz, turtlebot3, nav2, slam-toolbox, rviz2, colcon...
source /opt/ros/jazzy/setup.bash && export TURTLEBOT3_MODEL=burger
cd ros2_ws && colcon build --symlink-install --parallel-workers 1   # 1 worker: small VM
source install/setup.bash
```
The Python-only parts (maze tools, validator, DQN unit tests) run on macOS/Linux without ROS: `python3 tools/verify_static.py` (needs `numpy pyyaml pytest`).
Not verified on a clean machine yet. `requirements.txt` lists the Python packages.
