# Ownership inventory

Authorship is taken from `git log`; names are Git author names, not claims about who the "Member N" is.

## Original teammate contributions (preserved, merged with merge commits)
| Member | Branch / PR | Commits | Author | Content |
|---|---|---|---|---|
| 1 Simulation | - | **none found on any branch** | - | nothing to preserve |
| 2 SLAM | `feature/slam-mapping`, PR #1 | `7544234` | dhanushka403 | `maps/test_map.pgm/.yaml` (partial prototype map) |
| 3 Navigation | `feature/maze_navigation`, PR #2 | `744dc77` `de04a64` `dd6d60a` `8e8a9b8` `fa84c96` `59bafe8` `e9bcffe` | RavinduYashas | Nav2 params, launch, RViz layout, goal evaluator, docs |
| 4 DQN | `feature/dqn-agent`, PR #3 | `af87978` `8a76435` | Asesh Nemal | NumPy Double DQN, env, train/eval/compare, tests, Stage-1 trial evidence |

## Emergency integration commits (branch `feature/group-integration`, AI-assisted, NOT teammate work)
| Commit | Scope |
|---|---|
| `82efae1` feat(simulation) | `maze_simulation` package, maze world, ground-truth map |
| `3d07b0c` feat(slam) | `slam_mapping` launch/config/tools |
| `4d9c74f` fix(navigation) | Nav2 corridor/cmd_vel fixes, `run_three_goals.py` |
| `ec50373` feat(dqn) | DQN maze adaptation and odom fix |
| docs / tools commits | `tools/`, `docs/` |

## Untouched
`feature/dqn-agent-manual` was neither read nor modified. `main` is unchanged. No teammate branch was modified.

## Merge records
`701b8a5`, `725cc15`, `1c16c1c` are `--no-ff` merge commits of the three source branches into `feature/group-integration`, created locally (the GitHub PR merge button/CLI was not used - see integration.md). Each source branch's commits remain reachable and unmodified.
