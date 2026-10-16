**IE4060 Robotics Assignment 02 — Confirmed Software Environment**

I have completed the initial installation and verification of our robotics development environment.

Please use these versions for compatibility across all four members:

**Operating System and Simulation**
- Ubuntu: 24.04.5 LTS (64-bit)
- ROS 2: Jazzy Jalisco
- Gazebo: Harmonic (Gazebo Sim 8.15.0)
- Robot: TurtleBot3 Burger
- ROS–Gazebo integration: ros-gz 1.0.24

**SLAM and Navigation**
- SLAM Toolbox: 2.8.5
- Nav2: 1.3.13

**DQN / Python**
- Python: 3.12.3
- PyTorch: 2.5.1
- NumPy: 1.26.4
- Matplotlib: 3.9.2

My installed PyTorch build is 2.5.1+cu124, but I am running it on the CPU. We should standardize the PyTorch build before integrating DQN.

**Successfully tested:**
- ROS 2 communication
- Gazebo Harmonic simulation
- TurtleBot3 Burger spawning
- LiDAR sensor data
- Odometry and simulation clock
- PyTorch CPU tensor calculations

SLAM Toolbox and Nav2 are installed and recognized by ROS 2. Their full mapping and navigation workflows will be tested during integration.

Please follow this common environment rather than mixing ROS 2 Humble/Jazzy or Gazebo Classic/Harmonic.