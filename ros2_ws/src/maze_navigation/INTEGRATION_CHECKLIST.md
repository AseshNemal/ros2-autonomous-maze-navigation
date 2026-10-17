# Navigation integration checklist

## Environment

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic 8.15.0
- TurtleBot3 Burger
- ros-gz 1.0.24 (team target)
- Nav2 1.3.13

## Preparation completed

- [x] Navigation package builds.
- [x] NavFn A* and initial speed limits configured.
- [x] Initial progress threshold: 0.10 m within 20 seconds.
- [x] MPPI trajectory visualization disabled.
- [x] Nav2 and AMCL launch with optional RViz implemented.
- [x] Launch argument loading checked.
- [x] RViz layout opens.
- [x] Goal evaluation script builds and displays help.

## Offline validation completed

Synthetic inputs were used. These are software checks, not robot
performance results.

Logger checks passed:
- Known odometry distance calculation.
- Elapsed time calculation.
- Missing odometry produces invalid metrics.
- Rejected goal has no elapsed time.
- Backwards odometry timestamps invalidate metrics.
- CSV header and multiple result rows.

Launch preflight checks passed:
- Missing map file rejected.
- Invalid map YAML rejected.
- Missing image entry rejected.
- Missing map image rejected.
- Missing parameters file rejected.
- Relative map path rejected.
- Existing test files accepted by preflight checks.

These checks do not validate the full Nav2 parameter schema,
ROS action communication or physical navigation.

## Required from simulation member

- [ ] Simulation launch command.
- [ ] Robot spawn pose.
- [ ] Actual robot collision geometry and base frame.
- [ ] LiDAR topic, frame, range and publishing rate.
- [ ] Odometry topic and frame IDs.
- [ ] Working simulation clock.
- [ ] Velocity command topic and exact message type.
- [ ] TF from odom to robot base and LiDAR.

Expected topic names are /scan, /odom, /clock and /cmd_vel.
These are proposals until verified with the simulation.

## Required from mapping member

- [ ] Saved map YAML and referenced image.
- [ ] Map path and resolution.
- [ ] Confirmation that the map matches the final maze.
- [ ] Agreement on localization ownership.

The navigation launch starts AMCL and the map server.
Do not run duplicate localization or map-server instances
on the same interfaces.

## Integration and runtime tests

- [ ] Verify TF chain: map -> odom -> robot base -> LiDAR.
- [ ] Verify simulated time on all relevant nodes.
- [ ] Match robot footprint and sensor settings.
- [ ] Verify velocity message compatibility.
- [ ] Confirm required Nav2 lifecycle nodes become active.
- [ ] Set initial pose and verify AMCL localization.
- [ ] Reach three distinct goals clicked in RViz.
- [ ] Verify obstacle avoidance and tune navigation.
- [ ] Test evaluator action communication and CSV measurements.
- [ ] Retest RViz shutdown behaviour.

## Evaluation agreement

- [ ] Choose three reachable goals and record x, y and yaw.
- [ ] Record a common starting pose for each test.
- [ ] Use identical world, map and start/goal pairs for Nav2 and DQN.
- [ ] Agree repetitions, timeout and success criteria.
- [ ] Record outcome, simulation time and odometry distance.
- [ ] Keep failed runs in the results.
- [ ] Record localization/reset procedure between runs.
- [ ] Save actual screenshots and results after runtime tests.

The evaluator sends its own goal; it does not record RViz-clicked goals.
Run only one navigation method at a time.

## Current limits

- End-to-end navigation has not been demonstrated.
- Robot footprint, inflation and goal tolerances need runtime review.
- Logger runtime behaviour and measurement accuracy remain unverified.
- No synthetic data may be presented as real navigation evidence.
