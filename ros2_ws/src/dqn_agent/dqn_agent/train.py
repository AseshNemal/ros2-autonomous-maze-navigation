
import argparse
import csv
from datetime import datetime
from pathlib import Path

import rclpy

from .agent import DQNAgent
from .env import GazeboEnvironment


def main():
    parser = argparse.ArgumentParser(
        description="Train Double DQN with TurtleBot3 Gazebo"
    )

    parser.add_argument(
        "--episodes",
        type=int,
        default=2
    )

    parser.add_argument(
        "--max-steps",
        type=int,
        default=10
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.5
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=1.0
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        default=0.0
    )

    parser.add_argument(
        "--checkpoint-every",
        type=int,
        default=5
    )

    args = parser.parse_args()

    if args.episodes <= 0 or args.max_steps <= 0:
        parser.error("Episodes and max-steps must be positive")

    if args.duration <= 0 or args.checkpoint_every <= 0:
        parser.error("Duration and checkpoint-every must be positive")

    # Create a separate directory for every training run
    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S_%f"
    )

    run_dir = (
        Path.home()
        / ".ros"
        / "dqn_agent"
        / "runs"
        / timestamp
    )

    run_dir.mkdir(
        parents=True,
        exist_ok=False
    )

    csv_path = run_dir / "training_results.csv"

    # Create DQN learning agent
    agent = DQNAgent(
        state_size=15,
        action_size=5
    )

    env = None

    columns = [
        "episode",
        "steps",
        "total_reward",
        "status",
        "success",
        "collision",
        "distance_to_goal",
        "path_length",
        "elapsed_sim_s",
        "epsilon",
        "mean_loss",
        "training_updates"
    ]

    print("Starting Double DQN training", flush=True)
    print("Episodes:", args.episodes)
    print("Max steps:", args.max_steps)
    print("Results:", csv_path, flush=True)

    rclpy.init()

    try:
        env = GazeboEnvironment(
            goal=(args.goal_x, args.goal_y),
            world="dqn",
            action_duration=args.duration
        )

        with csv_path.open(
            "w",
            newline="",
            encoding="utf-8"
        ) as csv_file:

            writer = csv.DictWriter(
                csv_file,
                fieldnames=columns
            )

            writer.writeheader()

            # Begin training episodes
            for episode in range(1, args.episodes + 1):

                print(
                    f"\nResetting for episode {episode}...",
                    flush=True
                )

                state = env.reset()

                total_reward = 0.0
                losses = []
                info = None
                steps = 0

                for step in range(1, args.max_steps + 1):

                    # 1. Select action
                    action = agent.choose_action(
                        state,
                        training=True
                    )

                    # 2. Execute in Gazebo
                    next_state, reward, done, info = (
                        env.step(action)
                    )

                    # 3. Store experience
                    agent.remember(
                        state,
                        action,
                        reward,
                        next_state,
                        done
                    )

                    # 4. Train neural network
                    loss = agent.learn()

                    if loss is not None:
                        losses.append(float(loss))

                    # 5. Update statistics
                    total_reward += float(reward)

                    state = next_state
                    steps = step

                    # 6. Stop on goal or collision
                    if done:
                        break

                # Determine episode outcome
                if info["success"]:
                    status = "goal"

                elif info["collision"]:
                    status = "collision"

                else:
                    status = "step_limit"

                mean_loss = (
                    sum(losses) / len(losses)
                    if losses
                    else None
                )

                # Save episode metrics
                writer.writerow({
                    "episode": episode,
                    "steps": steps,
                    "total_reward": round(total_reward, 4),
                    "status": status,
                    "success": int(info["success"]),
                    "collision": int(info["collision"]),
                    "distance_to_goal": round(
                        info["distance"],
                        4
                    ),
                    "path_length": round(
                        info["path_length"],
                        4
                    ),
                    "elapsed_sim_s": round(
                        info["elapsed_sim_s"],
                        3
                    ),
                    "epsilon": round(agent.epsilon, 4),
                    "mean_loss": (
                        round(mean_loss, 6)
                        if mean_loss is not None
                        else ""
                    ),
                    "training_updates": agent.training_steps
                })

                csv_file.flush()

                print(
                    f"Episode {episode}/{args.episodes}"
                    f" | {status}"
                    f" | Steps: {steps}"
                    f" | Reward: {total_reward:.2f}"
                    f" | Epsilon: {agent.epsilon:.3f}"
                    f" | Updates: {agent.training_steps}",
                    flush=True
                )

                # Save an intermediate checkpoint
                if episode % args.checkpoint_every == 0:

                    checkpoint = run_dir / (
                        f"dqn_episode_{episode}.npz"
                    )

                    agent.save(checkpoint)

                    print(
                        "Checkpoint saved:",
                        checkpoint,
                        flush=True
                    )

    except KeyboardInterrupt:
        print(
            "\nTraining interrupted by user.",
            flush=True
        )

    finally:
        if env is not None:
            try:
                env.close()
            finally:
                rclpy.shutdown()
        else:
            rclpy.shutdown()

        # Save the current weights, even if interrupted
        final_model = agent.save(
            run_dir / "dqn_final.npz"
        )

        print("\nTraining session ended.")
        print("CSV:", csv_path)
        print("Model:", final_model)
        print("Total updates:", agent.training_steps)


if __name__ == "__main__":
    main()

