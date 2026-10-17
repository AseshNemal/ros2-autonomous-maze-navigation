# Member 4 - Deep Q-Learning (`dqn_agent`)

Status key: **VERIFIED** (actually run and passed) · **IMPLEMENTED** (code exists, runtime check pending) · **FAILED** · **BLOCKED** · **NOT IMPLEMENTED**.
Runtime results on the current maze (Ubuntu VM): see [testing.md](../testing.md) and `docs/results/vm/`. Summary below under Tests; the post-fix DQN runs are still PENDING.

## Responsibility
DQN navigation: state, actions, reward, Q/target networks, replay, epsilon-greedy, Double DQN, checkpoints, training/evaluation, DQN-vs-Nav2 comparison.

## Provenance
| Item | Origin |
|---|---|
| NumPy Double DQN (`agent.py`, `network.py`, `replay_buffer.py`), `env.py`, `train.py`, `evaluate.py`, `compare.py`, tests, Stage-1 trial CSVs | **Original work on `feature/dqn-agent`** (`Asesh Nemal`, commits `af87978`, `8a76435`), merged via PR #3 (merge commit, intact). |
| Changes below | **Emergency integration code** (commit `ec50373`, `feat(dqn)`), AI-assisted. |
| `feature/dqn-agent-manual` | **Not touched, not read.** |

## Design (unchanged core)
State (15): 12 LiDAR sector minima / 3.5, goal distance/5, sin & cos of bearing error. Actions (5): forward, forward-left, forward-right, spin left, spin right. Reward: +12*progress - 0.05 step - 0.4 only when the nearest obstacle is closer than 0.20 m (it was 0.30 m, which fired on every step in a 0.5 m corridor - see change 7), -40 collision, +80 goal. Q-net 15-64-64-5 (manual NumPy backprop + Adam, no Torch), target net hard-copied every 200 updates, replay 20k, epsilon-greedy 1.0->0.05, **Double DQN** target. Checkpoint `.npz`.

## Changes for the maze
1. **Bug fix - stale odometry after reset.** Gazebo DiffDrive integrates wheel motion, so `/odom` is not reset by the `set_pose` teleport. The old env used odom as the world pose, so from episode 2 onward goal distance/bearing were wrong (this may have contributed to the 0/10 and 0/3 Stage-1 results; not proven). Now `odom_to_world()` calibrates at each reset (unit-tested incl. rotation).
2. Targets the `custom_maze` world; per-episode `reset(start, goal, waypoints)`.
3. `--curriculum-max-cells N`: random start/goal pairs 1..N grid cells apart; `--random-heading`.
4. `--guidance waypoints` (**hybrid, report as such**): the DQN steers toward successive A* cell centres instead of a distant goal behind walls (+5 per waypoint). Default `none` = pure DQN with straight-line goal info, which cannot see around walls and is expected to struggle on long routes.
5. YAML-configurable (`config/dqn_params.yaml`), `--resume`, `dqn_train/dqn_eval.launch.py`.
6. `evaluate_dqn` runs the 3 assignment goals from the fixed start (same pairs as Nav2 `--reset-to-start`), logs grid reference length and efficiency. `compare_navigation` now also reports success-only means.
7. **Final-session fixes (commit `982a11c`, each covered by a unit test):** the near-wall penalty no longer fires at corridor centre (side walls are ~0.245 m away), which had made an early collision almost as good as surviving; a step-limit truncation is no longer stored as a terminal state; a checkpoint keeps its hyperparameters so `--resume` continues epsilon instead of restarting it; after the reset teleport the first state waits for a scan stamped after the teleport; `-inf` LiDAR returns count as close, not free; the default training pair is the shortest goal; the progress line shows loss and successes.

## Topics
Subscribes `/scan`, `/odom`; publishes `/cmd_vel` (**TwistStamped**). Uses `gz service /world/custom_maze/set_pose` for reset. **Never run with Nav2/auto-mapper** (same `/cmd_vel`).

## Run
```bash
ros2 launch dqn_agent dqn_train.launch.py episodes:=300 curriculum_max_cells:=3        # headless sim + training
ros2 launch dqn_agent dqn_eval.launch.py checkpoint:=~/.ros/dqn_agent/checkpoints/dqn_maze.npz
ros2 run dqn_agent compare_navigation --dqn <dqn_eval.csv> --nav2 <nav2_results.csv>
```
Or staged: `tools/verify_on_vm.sh dqn` (env `EPISODES`, `GUIDANCE=waypoints`).

## Tests
- VERIFIED (static, pytest, 18 tests): network/optimizer, target copy, replay, Double-DQN learning + checkpoint round-trip, Bellman target with done mask and target network, observation/reward (incl. corridor-centre reward), `-inf` LiDAR, checkpoint hyperparameters/epsilon, default training pair, truncation, odom->world, curriculum/waypoints, config, comparison.
- Prior VM evidence (before this branch's changes, ROBOTIS Stage 1, **not our maze**): see `docs/results/member4_dqn_trial_notes.md` - loop/reset/checkpoint worked; **0/10 training successes, 0/3 evaluation successes**, 200 actions only. Needs re-verification with the new env.
- Current maze, before the fixes (`docs/results/vm/dqn_train_none_20261017-115922.csv`): 3-episode smoke test - the loop, `burger` reset, CSV and checkpoint work, but all 3 episodes ended in collisions, the checkpoint had zero optimizer updates and `evaluate_dqn` refused it. Earlier 6 x 6 maze (`docs/results/old_6x6_maze/`): 0 of 50 training episodes, 0 of 9 greedy evaluations.
- PENDING (Ubuntu VM): smoke run after the fixes (optimizer updates only start once the replay buffer holds 100 samples, so the transition count must be checked), then one bounded 50-100 episode run and evaluation.

## Honest status
**No evidence that DQN reaches goals** (none on the current maze, none on the earlier one). Do not claim a successful policy until a real `dqn_eval_*.csv` shows it. If pure DQN fails, report that honestly and show the waypoint-guided hybrid separately; the Nav2 comparison is only valid on the same pairs, timeouts and tolerances (`--reset-to-start`).

## VIVA
*Why Double DQN?* Plain DQN's max over the same net overestimates Q; we pick the action with the online net and score it with the target net. *Why a target network / replay?* Stabilise bootstrapped targets / break correlation between consecutive samples. *Why does it struggle in the maze?* Local sensing + straight-line goal gives a non-Markov, deceptive state (dead ends); hence curriculum and waypoint guidance. *Why compare with Nav2?* Nav2 has a map and global planner (privileged information); DQN has only LiDAR + goal.
