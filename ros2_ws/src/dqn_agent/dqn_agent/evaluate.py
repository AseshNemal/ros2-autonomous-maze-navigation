"""Deterministic evaluation of a previously trained DQN checkpoint in Gazebo."""
import argparse
import csv
from pathlib import Path
import sys
from .agent import DQNAgent
from .env import ACTIONS, STATE_DIM, GazeboDQNEnv


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--max-steps", type=int, default=100)
    p.add_argument("--goal-x", type=float, default=1.0)
    p.add_argument("--goal-y", type=float, default=0.0)
    p.add_argument("--world", default="turtlebot3_dqn_stage1")
    p.add_argument("--action-duration", type=float, default=0.30)
    p.add_argument("--csv", default="~/.ros/dqn_agent/results/dqn_evaluation.csv")
    return p.parse_args()


def run(args):
    import rclpy
    if args.episodes < 1 or args.max_steps < 1:
        raise ValueError("episodes and max-steps must be positive")
    agent = DQNAgent.load(args.checkpoint)
    if agent.state_dim != STATE_DIM or agent.action_dim != len(ACTIONS):
        raise ValueError("Checkpoint incompatible with the Gazebo observation/action spaces")
    if agent.train_steps == 0:
        raise ValueError("Checkpoint has zero optimizer updates; not a trained model")
    path = Path(args.csv).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    env = None
    try:
        env = GazeboDQNEnv(goal=(args.goal_x, args.goal_y), world=args.world,
                           action_duration=args.action_duration)
        with path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=[
                "run_id", "method", "world", "goal_x", "goal_y", "success", "collision",
                "outcome", "elapsed_sim_s", "path_length_m", "steps", "distance_to_goal_m"
            ])
            writer.writeheader()
            for i in range(1, args.episodes + 1):
                state = env.reset()
                info = {}
                for _ in range(args.max_steps):
                    state, _, terminal, info = env.step(agent.act(state, explore=False))
                    if terminal:
                        break
                outcome = ("success" if info["success"] else
                           "collision" if info["collision"] else "timeout")
                writer.writerow(dict(run_id=f"dqn_{i:03d}", method="dqn", world=args.world,
                    goal_x=args.goal_x, goal_y=args.goal_y,
                    success=info["success"], collision=info["collision"], outcome=outcome,
                    elapsed_sim_s=round(info["elapsed_sim_s"], 3),
                    path_length_m=round(info["path_length_m"], 4), steps=info["steps"],
                    distance_to_goal_m=round(info["distance_to_goal"], 4)))
                file.flush()
                print(f"run={i} outcome={outcome} time={info['elapsed_sim_s']:.1f}s "
                      f"path={info['path_length_m']:.2f}m", flush=True)
    finally:
        if env is not None:
            env.close()
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
