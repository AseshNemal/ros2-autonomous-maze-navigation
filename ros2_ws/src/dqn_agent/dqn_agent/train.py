"""Train Double DQN against a running maze_simulation Gazebo world.

Modes (all use the assignment maze; start the simulation headless first):
  fixed start->goal pair      --pair 5,0:2,0             (cells row,col)
  curriculum                  --curriculum-max-cells 3   random pairs, 1..N cells apart
  waypoint guidance (hybrid)  --guidance waypoints       DQN steers between A* cell
                                                         centres; NOT a pure end-to-end DQN
Do not run Nav2/auto-mapper at the same time: both publish /cmd_vel.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

from . import maze
from .agent import DQNAgent
from .config import agent_kwargs, load_config
from .env import ACTIONS, STATE_DIM, GazeboDQNEnv

FIELDS = ["episode", "start_cell", "goal_cell", "guidance", "reward", "steps", "success",
          "collision", "elapsed_sim_s", "path_length_m", "epsilon", "loss", "train_steps"]


def parse_pair(text):
    a, b = text.split(":")
    return tuple(int(v) for v in a.split(",")), tuple(int(v) for v in b.split(","))


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None, help="YAML experiment file (default: config/dqn_params.yaml)")
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--max-steps", type=int, default=150)
    p.add_argument("--pair", type=parse_pair, default=None, help="fixed 'r,c:r,c' start:goal cells")
    p.add_argument("--curriculum-max-cells", type=int, default=0,
                   help="N>0: sample random pairs 1..N grid cells apart each episode")
    p.add_argument("--guidance", choices=["none", "waypoints"], default="none")
    p.add_argument("--random-heading", action="store_true", help="random start yaw each episode")
    p.add_argument("--world", default=None)
    p.add_argument("--action-duration", type=float, default=None)
    p.add_argument("--epsilon-decay", type=int, default=None,
                   help="env steps over which epsilon decays 1.0 -> 0.05 (default: config, 3000)")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--checkpoint", default="~/.ros/dqn_agent/checkpoints/dqn_maze.npz")
    p.add_argument("--resume", action="store_true", help="continue from --checkpoint if it exists")
    p.add_argument("--csv", default="~/.ros/dqn_agent/results/training.csv")
    p.add_argument("--checkpoint-every", type=int, default=10)
    return p.parse_args(argv)


def pick_pair(layout, args, rng, episode):
    if args.pair:
        return args.pair
    if args.curriculum_max_cells > 0:
        return maze.sample_pair(layout, rng, 1, args.curriculum_max_cells)
    return maze.evaluation_pairs(layout)[1]  # default: the short (5-cell) evaluation goal


def run(args):
    import rclpy
    cfg = load_config(args.config)
    env_cfg = cfg["environment"]
    seed = args.seed if args.seed is not None else cfg["training"]["seed"]
    world = args.world or env_cfg["world"]
    duration = args.action_duration or env_cfg["action_duration_s"]
    if args.episodes < 1 or args.max_steps < 1 or duration <= 0:
        raise ValueError("episodes, max-steps and action-duration must be positive")
    np.random.seed(seed)
    layout = maze.load_layout()
    rng = maze.rng_from_seed(seed)
    ckpt = Path(args.checkpoint).expanduser()
    if args.resume and ckpt.is_file():
        agent = DQNAgent.load(ckpt)
        print(f"resumed from {ckpt}: train_steps={agent.train_steps} env_steps={agent.env_steps}")
    else:
        kwargs = agent_kwargs(cfg, seed)
        if args.epsilon_decay:
            kwargs["epsilon_decay"] = args.epsilon_decay
        agent = DQNAgent(state_dim=STATE_DIM, action_dim=len(ACTIONS), **kwargs)
    log_path = Path(args.csv).expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    env = None
    completed = 0
    try:
        env = GazeboDQNEnv(world=world, action_duration=duration,
                           goal_tolerance=env_cfg["goal_tolerance_m"],
                           collision_distance=env_cfg["collision_distance_m"],
                           waypoint_tolerance=env_cfg.get("waypoint_tolerance_m", 0.15))
        t_start = time.monotonic()
        with log_path.open("a" if args.resume else "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            if file.tell() == 0:
                writer.writeheader()
            for episode in range(1, args.episodes + 1):
                a, b = pick_pair(layout, args, rng, episode)
                yaw = rng.uniform(-3.14159, 3.14159) if args.random_heading else 0.0
                sx, sy = (maze.spawn_xy(layout) if a == tuple(layout["start_cell"])
                          else maze.cell_xy(layout, a))
                wps = maze.waypoints(layout, a, b) if args.guidance == "waypoints" else []
                state = env.reset(start=(sx, sy, yaw), goal=maze.cell_xy(layout, b), waypoints=wps)
                reward_sum, losses, info = 0.0, [], {}
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
                writer.writerow(dict(
                    episode=episode, start_cell=f"{a[0]},{a[1]}", goal_cell=f"{b[0]},{b[1]}",
                    guidance=args.guidance, reward=round(reward_sum, 4), steps=info["steps"],
                    success=info["success"], collision=info["collision"],
                    elapsed_sim_s=round(info["elapsed_sim_s"], 3),
                    path_length_m=round(info["path_length_m"], 4), epsilon=round(agent.epsilon, 4),
                    loss=round(float(np.mean(losses)), 6) if losses else "",
                    train_steps=agent.train_steps))
                file.flush()
                completed += 1
                el = time.monotonic() - t_start
                print(f"[{episode}/{args.episodes}] elapsed={el:.0f}s eta={el / episode * (args.episodes - episode):.0f}s "
                      f"episode={episode} {a}->{b} reward={reward_sum:.2f} steps={info['steps']} "
                      f"success={info['success']} collision={info['collision']} "
                      f"epsilon={agent.epsilon:.3f} updates={agent.train_steps}", flush=True)
                if episode % args.checkpoint_every == 0:
                    print("checkpoint:", agent.save(ckpt), flush=True)
    finally:
        if env is not None:
            env.close()
        if rclpy.ok():
            rclpy.shutdown()
        # Save even after Ctrl+C, but do not claim convergence.
        if completed:
            print(f"completed={completed}; checkpoint={agent.save(ckpt)}", flush=True)
    return completed


def main():
    try:
        run(arguments())
    except (KeyboardInterrupt, RuntimeError, ValueError, OSError) as error:
        print(f"Training stopped: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
