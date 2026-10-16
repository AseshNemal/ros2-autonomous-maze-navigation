#!/usr/bin/env python3
"""Cloud-side DQN check (no ROS/Gazebo): train the real NumPy Double DQN on a KINEMATIC SURROGATE of
the Gazebo environment (unicycle model, ray-cast 360-beam LiDAR on the real maze walls, the same
actions, observation, reward, collision rule and waypoint logic as dqn_agent/env.py).

This is NOT Gazebo evidence and must never be reported as such. It tests whether the learning
code, the reward and the exploration schedule can learn on this maze at all, so that hyper-
parameter changes can be justified before spending VM time. Needs numpy + pyyaml.

  python3 tools/offline_dqn_surrogate.py --episodes 400 --curriculum 3 --guidance none
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/dqn_agent"))
sys.path.insert(0, str(ROOT / "tools"))
from dqn_agent import maze  # noqa: E402
from dqn_agent.agent import DQNAgent  # noqa: E402
from dqn_agent.config import agent_kwargs, load_config  # noqa: E402
from dqn_agent.env import ACTIONS, STATE_DIM, compute_reward, get_observation, wrap_angle  # noqa: E402
from offline_mapping_check import raycast  # noqa: E402


FINE = ((0.14, 0.0), (0.10, 0.4), (0.10, -0.4), (0.0, 0.8), (0.0, -0.8))


class SurrogateEnv:
    """Same step/reset contract and logic as GazeboDQNEnv, on a kinematic model."""

    def __init__(self, layout, goal_tolerance=0.22, collision_distance=0.15, action_duration=0.30,
                 waypoint_tolerance=0.25, waypoint_bonus=5.0, seed=0, noise=0.01, actions=ACTIONS):
        self.actions = actions
        half = layout["wall_thickness_m"] / 2
        self.boxes = np.array([(min(a, c) - half, min(b, d) - half, max(a, c) + half, max(b, d) + half)
                               for a, b, c, d in maze.tools.wall_segments(layout)])
        self.angles = np.radians(np.arange(360))
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.goal_tolerance, self.collision_distance = goal_tolerance, collision_distance
        self.action_duration = action_duration
        self.waypoint_tolerance, self.waypoint_bonus = waypoint_tolerance, waypoint_bonus
        self.waypoints, self.wp_index = [], 0

    def _scan(self):
        r = raycast(self.boxes, self.x, self.y, self.angles + self.yaw, 0.12, 3.5)
        return r + self.rng.normal(0, self.noise, r.shape)

    def target(self):
        return self.waypoints[self.wp_index] if self.wp_index < len(self.waypoints) else self.goal

    def _state(self):
        return get_observation(self._scan(), (self.x, self.y), self.yaw, self.target())

    def reset(self, start, goal, waypoints=()):
        self.x, self.y, self.yaw = start
        self.goal, self.waypoints, self.wp_index = tuple(goal), [tuple(w) for w in waypoints], 0
        state, d, _ = self._state()
        self.previous_distance, self.path, self.steps, self.t = d, 0.0, 0, 0.0
        return state

    def step(self, action):
        v, w = self.actions[int(action)]
        px, py = self.x, self.y
        for _ in range(6):                                  # 6 x 0.05 s = action_duration
            self.yaw = wrap_angle(self.yaw + w * 0.05)
            self.x += v * 0.05 * math.cos(self.yaw)
            self.y += v * 0.05 * math.sin(self.yaw)
        self.t += self.action_duration
        before = self.previous_distance
        hit = False
        while (self.wp_index < len(self.waypoints) and
               math.hypot(self.waypoints[self.wp_index][0] - self.x,
                          self.waypoints[self.wp_index][1] - self.y) <= self.waypoint_tolerance):
            self.wp_index += 1
            hit = True
        state, distance, clearance = self._state()
        self.path += math.hypot(self.x - px, self.y - py)
        self.previous_distance = distance
        self.steps += 1
        collision = clearance < self.collision_distance
        final = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)
        success = final <= self.goal_tolerance
        if hit:
            before = distance
        reward = compute_reward(before, distance, clearance, collision, success)
        if hit and not (collision or success):
            reward += self.waypoint_bonus
        return state, reward, collision or success, {"success": success, "collision": collision,
                                                     "steps": self.steps, "path_length_m": self.path,
                                                     "elapsed_sim_s": self.t, "distance_to_goal": final}


def evaluate(agent, env, layout, guidance, episodes=3, max_steps=250):
    out = []
    for a, b in maze.evaluation_pairs(layout):
        sx, sy = maze.cell_xy(layout, a)
        wps = maze.waypoints(layout, a, b) if guidance == "waypoints" else []
        res = []
        for _ in range(episodes):
            s = env.reset((sx, sy, 0.0), maze.cell_xy(layout, b), wps)
            for _ in range(max_steps):
                s, _, done, info = env.step(agent.act(s, explore=False))
                if done:
                    break
            res.append(info["success"])
        out.append((b, sum(res), len(res)))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--episodes", type=int, default=400)
    p.add_argument("--max-steps", type=int, default=150)
    p.add_argument("--curriculum", type=int, default=3)
    p.add_argument("--guidance", choices=["none", "waypoints"], default="none")
    p.add_argument("--epsilon-decay", type=int, default=None, help="env steps (default: config 20000)")
    p.add_argument("--actions", choices=["default", "fine"], default="default",
                   help="fine = gentler turning while moving (angular 0.4 rad/s); experiment only")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--report-every", type=int, default=100)
    a = p.parse_args(argv)
    cfg = load_config()
    layout = maze.load_layout()
    kw = agent_kwargs(cfg, a.seed)
    if a.epsilon_decay:
        kw["epsilon_decay"] = a.epsilon_decay
    agent = DQNAgent(state_dim=STATE_DIM, action_dim=len(ACTIONS), **kw)
    env = SurrogateEnv(layout, goal_tolerance=cfg["environment"]["goal_tolerance_m"],
                       collision_distance=cfg["environment"]["collision_distance_m"], seed=a.seed,
                       actions=FINE if a.actions == "fine" else ACTIONS)
    rng = maze.rng_from_seed(a.seed)
    wins = coll = 0
    for ep in range(1, a.episodes + 1):
        s0, g = maze.sample_pair(layout, rng, 1, a.curriculum)
        sx, sy = maze.cell_xy(layout, s0)
        wps = maze.waypoints(layout, s0, g) if a.guidance == "waypoints" else []
        s = env.reset((sx, sy, 0.0), maze.cell_xy(layout, g), wps)
        for _ in range(a.max_steps):
            act = agent.act(s)
            ns, r, term, info = env.step(act)
            trunc = not term and info["steps"] >= a.max_steps
            agent.observe(s, act, r, ns, term or trunc)
            agent.learn()
            s = ns
            if term or trunc:
                break
        wins += info["success"]
        coll += info["collision"]
        if ep % a.report_every == 0:
            print(f"ep {ep:4d}: last{a.report_every} success={wins}/{a.report_every} collision={coll} "
                  f"eps={agent.epsilon:.2f} env_steps={agent.env_steps}", flush=True)
            wins = coll = 0
    ev = evaluate(agent, env, layout, a.guidance)
    total = sum(w for _, w, _ in ev)
    print("greedy eval on the 3 assignment goals (surrogate):",
          ", ".join(f"{g}: {w}/{n}" for g, w, n in ev), f"=> {total}/{sum(n for *_, n in ev)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
