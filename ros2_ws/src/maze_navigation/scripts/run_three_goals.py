#!/usr/bin/env python3
"""Navigate to the three evaluation goals with Nav2 and log metrics (CSV).

Goals come from maze_simulation/config/maze_layout.yaml (goal_cells). Metrics per
goal: outcome, simulated seconds, odometry path length, recoveries, grid-optimal
reference length and efficiency (reference / travelled). Reference lengths are
cell-centre A* lower-bound estimates (0.5 m per cell), so efficiency can exceed
1.0 when Nav2 cuts corners; they are a yardstick, not a true optimum.

  ros2 run maze_navigation run_three_goals.py --csv ~/nav2_results.csv \\
      [--map-source world|slam] [--reset-to-start] [--goals 1.25,1.25;-1.25,0.25]

--reset-to-start teleports the robot back to the start cell via Gazebo before each
goal (same start->goal pairs as the DQN evaluation, for a like-for-like comparison).
Without it the goals are visited in sequence (what the VIVA demo does) and the
reference length is measured from the previous goal.
Requires Nav2 + localization active; use_sim_time must be true.
"""
import argparse
import csv
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

ROBOT = "turtlebot3_burger"
WORLD = "maze"


def load_maze_tools():
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, str(Path(get_package_prefix("maze_simulation")) / "lib" / "maze_simulation"))
    import maze_tools
    return maze_tools


class Runner(Node):
    def __init__(self):
        super().__init__("run_three_goals",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.recording = False
        self.prev = None
        self.dist = 0.0
        self.samples = 0
        self.recoveries = 0

    def on_odom(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.recording and self.prev is not None and all(map(math.isfinite, p)):
            self.dist += math.hypot(p[0] - self.prev[0], p[1] - self.prev[1])
            self.samples += 1
        self.prev = p

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_initial_pose(self, x, y, yaw):
        for _ in range(5):
            m = PoseWithCovarianceStamped()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = m.pose.covariance[35] = 0.05
            self.init_pub.publish(m)
            self.spin_for(0.5)

    def teleport(self, world_xy, yaw=0.0):
        req = (f'name: "{ROBOT}" position: {{x: {world_xy[0]}, y: {world_xy[1]}, z: 0.05}} '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        out = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose",
                              "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                              "--timeout", "5000", "--req", req],
                             capture_output=True, text=True, timeout=15)
        if not re.search(r"data:\s*true", out.stdout):
            raise RuntimeError("Gazebo reset failed: " + out.stdout + out.stderr)

    def navigate(self, x, y, timeout_s):
        if not self.client.wait_for_server(timeout_sec=20.0):
            return "SERVER_UNAVAILABLE", 0.0, 0.0, 0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        self.dist, self.samples, self.recoveries, self.recording = 0.0, 0, 0, False

        def feedback(fb):
            self.recoveries = max(self.recoveries, fb.feedback.number_of_recoveries)

        fut = self.client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return "REJECTED", 0.0, 0.0, 0
        t0 = self.now_s()
        self.recording = True
        res = handle.get_result_async()
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.now_s() - t0 > timeout_s:
                handle.cancel_goal_async()
                self.spin_for(2.0)
                self.recording = False
                return "TIMEOUT", self.now_s() - t0, self.dist, self.recoveries
        self.recording = False
        outcome = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
                   GoalStatus.STATUS_CANCELED: "CANCELED"}.get(res.result().status, "UNKNOWN")
        return outcome, self.now_s() - t0, self.dist, self.recoveries


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--run-id", default=time.strftime("%Y%m%d-%H%M%S"))
    ap.add_argument("--map-source", choices=["world", "slam"], default="world",
                    help="world: map frame == world frame (ground-truth map); "
                         "slam: map origin is the spawn pose (SLAM-built map)")
    ap.add_argument("--reset-to-start", action="store_true")
    ap.add_argument("--timeout", type=float, default=120.0, help="sim seconds per goal")
    ap.add_argument("--goals", default=None, help="override: 'x,y;x,y;x,y' in WORLD coords")
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])

    tools = load_maze_tools()
    layout = tools.load()
    start_cell = layout["start_cell"]
    start_w = tools.cell_center(layout, *start_cell)
    off = start_w if args.map_source == "slam" else (0.0, 0.0)
    to_map = lambda w: (w[0] - off[0], w[1] - off[1])

    if args.goals:
        goal_worlds = [tuple(map(float, g.split(","))) for g in args.goals.split(";")]
        goal_cells = [None] * len(goal_worlds)
    else:
        goal_cells = [tuple(c) for c in layout["goal_cells"]]
        goal_worlds = [tools.cell_center(layout, *c) for c in goal_cells]

    rclpy.init()
    node = Runner()
    node.spin_for(1.0)
    rows = []
    prev_cell = tuple(start_cell)
    for i, (gw, gc) in enumerate(zip(goal_worlds, goal_cells), 1):
        if args.reset_to_start:
            node.teleport(start_w)
            node.set_initial_pose(*to_map(start_w), 0.0)
            node.spin_for(3.0)
            ref_from = tuple(start_cell)
        else:
            ref_from = prev_cell
        gx, gy = to_map(gw)
        node.get_logger().info(f"goal {i}: world=({gw[0]:.2f},{gw[1]:.2f}) map=({gx:.2f},{gy:.2f})")
        outcome, elapsed, dist, rec = node.navigate(gx, gy, args.timeout)
        ref = None
        if gc is not None:
            path = tools.shortest_cells(layout, ref_from, gc)
            ref = (len(path) - 1) * layout["cell_size_m"] if path else None
            prev_cell = gc
        eff = round(ref / dist, 3) if (ref and dist > 0 and outcome == "SUCCEEDED") else ""
        rows.append([args.run_id, "nav2_navfn_astar_rpp", i, round(gw[0], 3), round(gw[1], 3),
                     outcome, round(elapsed, 2), round(dist, 3), rec,
                     ref if ref is not None else "", eff, args.reset_to_start, args.map_source])
        node.get_logger().info(f"goal {i}: {outcome} in {elapsed:.1f}s sim, {dist:.2f} m, recoveries {rec}")
    out = Path(args.csv).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    new = not out.exists() or out.stat().st_size == 0
    with out.open("a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["run_id", "method", "goal_index", "goal_world_x", "goal_world_y", "outcome",
                        "elapsed_sim_s", "odom_distance_m", "recoveries", "reference_length_m",
                        "efficiency_ref_over_travelled", "reset_to_start", "map_source"])
        w.writerows(rows)
    ok = sum(r[5] == "SUCCEEDED" for r in rows)
    print(f"{ok}/{len(rows)} goals succeeded; CSV: {out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
