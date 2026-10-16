"""Train Double DQN against a running TurtleBot3 Gazebo simulation."""
import argparse
import csv
from pathlib import Path
import sys
import numpy as np

from .agent import DQNAgent
from .env import ACTIONS, STATE_DIM, GazeboDQNEnv


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--max-steps", type=int, default=100)
    p.add_argument("--goal-x", type=float, default=1.0)
    p.add_argument("--goal-y", type=float, default=0.0)
    p.add_argument("--world", default="turtlebot3_dqn_stage1")
    p.add_argument("--action-duration", type=float, default=0.30)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--checkpoint", default="~/.ros/dqn_agent/checkpoints/dqn_stage1.npz")
    p.add_argument("--csv", default="~/.ros/dqn_agent/results/training.csv")
    p.add_argument("--checkpoint-every", type=int, default=10)
    return p.parse_args()


def run(args):
    import rclpy
    if args.episodes < 1 or args.max_steps < 1 or args.action_duration <= 0:
        raise ValueError("episodes, max-steps and action-duration must be positive")
    np.random.seed(args.seed)
    agent = DQNAgent(state_dim=STATE_DIM, action_dim=len(ACTIONS), seed=args.seed)
    log_path = Path(args.csv).expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    env = None
    completed = 0
    try:
        env = GazeboDQNEnv(goal=(args.goal_x, args.goal_y),
                           world=args.world, action_duration=args.action_duration)
        with log_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=[
                "episode", "reward", "steps", "success", "collision",
                "elapsed_sim_s", "path_length_m", "epsilon", "loss", "train_steps"
            ])
            writer.writeheader()
            for episode in range(1, args.episodes + 1):
                state = env.reset()
                reward_sum = 0.0
                losses = []
                info = {}
                for _ in range(args.max_steps):
                    action = agent.act(state, explore=True)
                    next_state, reward, terminated, info = env.step(action)
                    # Max steps is a time-limit truncation: do not bootstrap across it.
                    truncated = not terminated and info["steps"] >= args.max_steps
                    agent.observe(state, action, reward, next_state, terminated or truncated)
                    loss = agent.learn()
                    if loss is not None:
                        losses.append(loss)
                    reward_sum += reward
                    state = next_state
                    if terminated or truncated:
                        break
                writer.writerow(dict(episode=episode, reward=round(reward_sum, 4),
                    steps=info["steps"], success=info["success"], collision=info["collision"],
                    elapsed_sim_s=round(info["elapsed_sim_s"], 3),
                    path_length_m=round(info["path_length_m"], 4),
                    epsilon=round(agent.epsilon, 4),
                    loss=round(float(np.mean(losses)), 6) if losses else "",
                    train_steps=agent.train_steps))
                file.flush()
                completed += 1
                print(f"episode={episode} reward={reward_sum:.2f} steps={info['steps']} "
                      f"success={info['success']} collision={info['collision']} "
                      f"epsilon={agent.epsilon:.3f} updates={agent.train_steps}", flush=True)
                if episode % args.checkpoint_every == 0:
                    print("checkpoint:", agent.save(args.checkpoint), flush=True)
    finally:
        if env is not None:
            env.close()
        rclpy.shutdown()
        # Save even after Ctrl+C, but do not claim convergence.
        if completed:
            print(f"completed={completed}; checkpoint={agent.save(args.checkpoint)}", flush=True)
    return completed


def main():
    try:
        run(arguments())
    except (KeyboardInterrupt, RuntimeError, ValueError) as error:
        print(f"Training stopped: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
