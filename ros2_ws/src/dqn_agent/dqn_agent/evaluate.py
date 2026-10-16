"""Deterministic (greedy, epsilon=0) evaluation of a DQN checkpoint in the maze.

Default: the three assignment goals, each from the fixed start cell, N episodes each —
the same start->goal pairs as `run_three_goals.py --reset-to-start` for Nav2, so
`compare_navigation` is like-for-like. Use --guidance waypoints only if the checkpoint
was trained that way (and report it as a hybrid).
"""
import argparse
import csv
import sys
from pathlib import Path

from . import maze
from .agent import DQNAgent
from .config import load_config
from .env import ACTIONS, STATE_DIM, GazeboDQNEnv

FIELDS = ["run_id", "method", "guidance", "start_cell", "goal_cell", "goal_x", "goal_y",
          "success", "collision", "outcome", "elapsed_sim_s", "path_length_m", "steps",
          "distance_to_goal_m", "reference_length_m", "efficiency_ref_over_travelled"]


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--config", default=None)
    p.add_argument("--episodes", type=int, default=3, help="episodes per goal")
    p.add_argument("--max-steps", type=int, default=250)
    p.add_argument("--guidance", choices=["none", "waypoints"], default="none")
    p.add_argument("--world", default=None)
    p.add_argument("--action-duration", type=float, default=None)
    p.add_argument("--csv", default="~/.ros/dqn_agent/results/dqn_evaluation.csv")
    return p.parse_args(argv)


def run(args):
    import rclpy
    if args.episodes < 1 or args.max_steps < 1:
        raise ValueError("episodes and max-steps must be positive")
    cfg = load_config(args.config)
    env_cfg = cfg["environment"]
    agent = DQNAgent.load(args.checkpoint)
    if agent.state_dim != STATE_DIM or agent.action_dim != len(ACTIONS):
        raise ValueError("Checkpoint incompatible with the Gazebo observation/action spaces")
    if agent.train_steps == 0:
        raise ValueError("Checkpoint has zero optimizer updates; not a trained model")
    layout = maze.load_layout()
    path = Path(args.csv).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    env = None
    try:
        env = GazeboDQNEnv(world=args.world or env_cfg["world"],
                           action_duration=args.action_duration or env_cfg["action_duration_s"],
                           goal_tolerance=env_cfg["goal_tolerance_m"],
                           collision_distance=env_cfg["collision_distance_m"],
                           waypoint_tolerance=env_cfg.get("waypoint_tolerance_m", 0.15))
        with path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            writer.writeheader()
            n = 0
            for a, b in maze.evaluation_pairs(layout):
                ref = (len(maze.cell_path(layout, a, b)) - 1) * layout["cell_size_m"]
                sx, sy = maze.cell_xy(layout, a)
                gx, gy = maze.cell_xy(layout, b)
                wps = maze.waypoints(layout, a, b) if args.guidance == "waypoints" else []
                for _ in range(args.episodes):
                    n += 1
                    state = env.reset(start=(sx, sy, 0.0), goal=(gx, gy), waypoints=wps)
                    info = {}
                    for _s in range(args.max_steps):
                        state, _, terminal, info = env.step(agent.act(state, explore=False))
                        if terminal:
                            break
                    outcome = ("success" if info["success"] else
                               "collision" if info["collision"] else "timeout")
                    eff = (round(ref / info["path_length_m"], 3)
                           if info["success"] and info["path_length_m"] > 0 else "")
                    writer.writerow(dict(
                        run_id=f"dqn_{n:03d}", method="dqn" + ("_waypoints" if wps else ""),
                        guidance=args.guidance, start_cell=f"{a[0]},{a[1]}", goal_cell=f"{b[0]},{b[1]}",
                        goal_x=round(gx, 3), goal_y=round(gy, 3), success=info["success"],
                        collision=info["collision"], outcome=outcome,
                        elapsed_sim_s=round(info["elapsed_sim_s"], 3),
                        path_length_m=round(info["path_length_m"], 4), steps=info["steps"],
                        distance_to_goal_m=round(info["distance_to_goal"], 4),
                        reference_length_m=ref, efficiency_ref_over_travelled=eff))
                    file.flush()
                    print(f"run={n} {a}->{b} outcome={outcome} time={info['elapsed_sim_s']:.1f}s "
                          f"path={info['path_length_m']:.2f}m", flush=True)
    finally:
        if env is not None:
            env.close()
        if rclpy.ok():
            rclpy.shutdown()
    print("Evaluation CSV:", path)


def main():
    try:
        run(arguments())
    except (KeyboardInterrupt, RuntimeError, ValueError, OSError) as error:
        print(f"Evaluation failed: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
