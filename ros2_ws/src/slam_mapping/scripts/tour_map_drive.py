#!/usr/bin/env python3
"""Coverage tour for unattended SLAM mapping (publishes TwistStamped /cmd_vel).

Drives through the centre of EVERY maze cell along open corridors (greedy
nearest-unvisited tour over the cell graph read from custom_maze.sdf), slowly enough for
the ~5 Hz lidar to give overlapping scans. Position comes from /odom only.
SLAM still builds the map purely from the lidar; the layout is used only to choose
where to drive (a wall follower cannot reach interior wall islands).
Mapping phase only: stop it before Nav2 or DQN. Simulation only.
  ros2 run slam_mapping tour_map_drive.py [--speed 0.12]
"""
import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _maze_tools():
    cands = [HERE.parent.parent / "maze_simulation" / "scripts"]
    try:
        from ament_index_python.packages import get_package_prefix
        cands.append(Path(get_package_prefix("maze_simulation")) / "lib" / "maze_simulation")
    except Exception:
        pass
    for c in cands:
        if (c / "maze_tools.py").is_file():
            sys.path.insert(0, str(c))
            import maze_tools
            return maze_tools
    raise RuntimeError("maze_tools.py (maze_simulation package) not found")


def coverage_route(layout, tools):
    """Cell sequence from start_cell that visits every cell via open boundaries."""
    cur = tuple(layout["start_cell"])
    route, seen = [cur], {cur}
    total = layout["rows"] * layout["cols"]
    while len(seen) < total:
        best = None
        for r in range(layout["rows"]):
            for c in range(layout["cols"]):
                if (r, c) in seen:
                    continue
                p = tools.shortest_cells(layout, cur, (r, c))
                if p and (best is None or len(p) < len(best)):
                    best = p
        if best is None:
            break                                   # unreachable cells (none in this maze)
        route.extend(best[1:])
        seen.update(best)
        cur = best[-1]
    return route


def main(argv=None):
    import rclpy
    from rclpy.time import Time
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    p = argparse.ArgumentParser()
    p.add_argument("--speed", type=float, default=0.12)
    p.add_argument("--turn", type=float, default=0.45, help="max in-place turn rate (rad/s)")
    p.add_argument("--max-waypoints", type=int, default=0, help="smoke test: stop after this many waypoints (0 = whole tour)")
    a = p.parse_args(rclpy.utilities.remove_ros_args(argv)[1:] if argv else rclpy.utilities.remove_ros_args()[1:])

    tools = _maze_tools()
    layout = tools.load()
    route = coverage_route(layout, tools)
    wps = [tools.cell_center(layout, r, c) for r, c in route]
    if a.max_waypoints:
        wps = wps[:a.max_waypoints + 1]
    sx, sy = tools.spawn_xy(layout)        # the robot starts at the original spawn pose

    class Tour(Node):
        def __init__(self):
            super().__init__("tour_map_drive")
            self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.pose, self.ref, self.front, self.idx = None, None, float("inf"), 1
            self.left = self.right = float("inf")
            # SLAM's corrected pose (map -> base_footprint) when slam_toolbox runs; plain odometry otherwise.
            # Odometry heading drifts after in-place turns, which in 0.5 m corridors drives the robot into walls.
            from tf2_ros import Buffer, TransformListener
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)
            self.using_slam = False
            self.finished = False
            self.t_log = 0.0
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f"tour: {len(route)} cells, {len(wps)} waypoints")

        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cur = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
            if self.ref is None:
                self.ref = cur                       # robot starts at the spawn pose, facing +x
            ox, oy, oyaw = self.ref
            dx, dy = cur[0] - ox, cur[1] - oy
            c, s = math.cos(-oyaw), math.sin(-oyaw)
            self.pose = (sx + c * dx - s * dy, sy + s * dx + c * dy, cur[2] - oyaw)

        def slam_pose(self):
            """World pose from the SLAM map frame (its origin is the spawn pose, yaw 0), or None."""
            try:
                t = self.tf_buf.lookup_transform("map", "base_footprint", Time())
            except Exception:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (sx + t.transform.translation.x, sy + t.transform.translation.y, yaw)

        def on_scan(self, m):
            best = float("inf")
            n = len(m.ranges)
            for k in range(-15, 16):
                i = int(round((math.radians(k) - m.angle_min) / m.angle_increment)) % n
                r = m.ranges[i]
                # r == range_min is a clipped near return (robot pitching, floor/body hit), not an obstacle
                if math.isfinite(r) and m.range_min + 0.01 < r <= m.range_max:
                    best = min(best, r)
            self.front = best
            # side clearances (+-90 deg): used to stay centred in the 0.5 m corridors
            def side(deg):
                m_ = float("inf")
                for k in range(-8, 9):
                    i = int(round((math.radians(deg + k) - m.angle_min) / m.angle_increment)) % n
                    r = m.ranges[i]
                    if math.isfinite(r) and m.range_min + 0.01 < r <= m.range_max:
                        m_ = min(m_, r)
                return m_
            self.left, self.right = side(90), side(-90)

        def send(self, v, w):
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x, m.twist.angular.z = float(v), float(w)
            self.pub.publish(m)

        def tick(self):
            if self.pose is None or self.finished:
                return
            sp = self.slam_pose()
            if sp is not None:
                if not self.using_slam:
                    self.using_slam = True
                    self.get_logger().info("steering by the SLAM pose (map->base_footprint)")
                self.pose = sp
            if self.idx >= len(wps):
                self.send(0.0, 0.0)
                self.finished = True
                self.get_logger().info("tour complete")
                return
            x, y, yaw = self.pose
            tx, ty = wps[self.idx]
            dist = math.hypot(tx - x, ty - y)
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.t_log > 15.0:
                self.t_log = now
                self.get_logger().info(
                    f"progress wp {self.idx}/{len(wps)-1} pos=({x:.2f},{y:.2f}) yaw={yaw:.2f} "
                    f"target=({tx:.2f},{ty:.2f}) dist={dist:.2f} front={self.front:.2f} sim_t={now:.0f}")
            if dist < 0.07:
                self.idx += 1
                return
            err = math.atan2(ty - y, tx - x) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            if abs(err) > 0.15:
                self.send(0.0, max(-a.turn, min(a.turn, 1.5 * err)))      # turn in place
            else:
                v = min(a.speed, 0.04 + 1.5 * dist)
                if self.front < 0.12:
                    v = 0.0                                               # safety stop
                w = 1.5 * err
                if self.left < 0.45 and self.right < 0.45:           # walls on both sides: stay centred
                    w += 1.5 * (self.left - self.right)
                self.send(v, max(-0.4, min(0.4, w)))

    rclpy.init()
    node = Tour()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
