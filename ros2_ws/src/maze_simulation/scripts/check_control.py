#!/usr/bin/env python3
"""Verify remote control of the TurtleBot3 in the running simulation (PASS/FAIL per check).

Uses only the normal interfaces: the real teleop_twist_keyboard (driven through a
pseudo-terminal), /cmd_vel, /odom, /scan, TF, gz topics, and /goal_pose (what RViz's
"Nav2 Goal" tool publishes).

  check_control.py teleop [--json out.json]   simulation must be running, nothing else driving
  check_control.py nav    [--json out.json]   simulation + Nav2 must be running
"""
import argparse
import json
import math
import os
import pty
import re
import select
import signal
import subprocess
import sys
import time

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append({"check": name, "pass": bool(ok), "detail": detail})
    print(("PASS  " if ok else "FAIL  ") + name + (f"   [{detail}]" if detail else ""), flush=True)
    return ok


def gz_pose():
    """Ground-truth (x, y, yaw) of the robot from Gazebo."""
    out = subprocess.run("timeout 10 gz topic -e -n 1 -t /world/maze/dynamic_pose/info",
                         shell=True, capture_output=True, text=True).stdout
    i = out.find('name: "turtlebot3_burger"')
    if i < 0:
        return None
    blk = out[i:]
    px = float(re.search(r"x: (-?[\d.e-]+)", blk).group(1))
    py = float(re.search(r"y: (-?[\d.e-]+)", blk).group(1))
    o = blk.split("orientation")[1]
    qx, qy, qz, qw = [float(re.search(k + r": (-?[\d.e-]+)", o).group(1)) for k in "xyzw"]
    return px, py, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def cmd_vel_publishers():
    """Node names publishing /cmd_vel right now."""
    out = subprocess.run("ros2 topic info /cmd_vel -v", shell=True, capture_output=True, text=True).stdout
    names, section = set(), None   # the dead-man watchdog is a safety net, not a driver: ignored
    for line in out.splitlines():
        if line.startswith("Publisher count"):
            section = "pub"
        elif line.startswith("Subscription count"):
            section = "sub"
        m = re.match(r"Node name: (\S+)", line)
        if m and section == "pub":
            names.add(m.group(1))
    names.discard("cmd_vel_watchdog")
    return names


def sim_speed(seconds=10):
    """Simulation real-time factor and CPU load: a proxy for how smooth the GUI/remote session is."""
    def stats():
        out = subprocess.run("timeout 10 gz topic -e -n 1 -t /stats", shell=True, capture_output=True, text=True).stdout
        sim = re.search(r"sim_time \{\s*sec: (\d+)\s*nsec: (\d+)", out)
        real = re.search(r"real_time \{\s*sec: (\d+)\s*nsec: (\d+)", out)
        return (int(sim.group(1)) + int(sim.group(2)) * 1e-9, int(real.group(1)) + int(real.group(2)) * 1e-9)
    s0, r0 = stats()
    time.sleep(seconds)
    s1, r1 = stats()
    load = os.getloadavg()[0]
    return (s1 - s0) / max(r1 - r0, 1e-6), load


def angdiff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Probe:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("check_control")
        self.odom, self.scan, self.n_odom, self.n_scan, self.n_tf = None, None, 0, 0, 0
        self.tf_stamp = None
        self.node.create_subscription(Odometry, "/odom", self._o, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, "/scan", self._s, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf", self._t, 100)

    def _o(self, m):
        self.odom, self.n_odom = m, self.n_odom + 1

    def _s(self, m):
        self.scan, self.n_scan = m, self.n_scan + 1

    def _t(self, m):
        for t in m.transforms:
            if t.header.frame_id == "odom" and t.child_frame_id == "base_footprint":
                self.n_tf += 1
                self.tf_stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def odom_pose(self):
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def speed(self):
        return abs(self.odom.twist.twist.linear.x), abs(self.odom.twist.twist.angular.z)


class Teleop:
    """The real teleop_twist_keyboard running in a pseudo-terminal; we type its keys."""

    def __init__(self, speed=0.15, turn=0.5):
        self.master, slave = pty.openpty()
        self.proc = subprocess.Popen(
            ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard", "--ros-args",
             "-p", "stamped:=true", "-p", f"speed:={speed}", "-p", f"turn:={turn}"],
            stdin=slave, stdout=slave, stderr=slave, preexec_fn=os.setsid, close_fds=True)
        os.close(slave)
        time.sleep(4)
        self._drain()

    def _drain(self):
        while select.select([self.master], [], [], 0)[0]:
            try:
                os.read(self.master, 4096)
            except OSError:
                break

    def hold(self, key, seconds, probe):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            os.write(self.master, key.encode())
            probe.spin(0.15)
            self._drain()

    def key(self, key):
        os.write(self.master, key.encode())

    def kill(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)


def run_teleop():
    probe = Probe()
    probe.spin(3)
    gt0 = gz_pose()
    check("robot 'turtlebot3_burger' exists in the Gazebo world", gt0 is not None,
          "pose=(%.2f, %.2f)" % gt0[:2] if gt0 else "not found")
    check("/odom publishing", probe.n_odom > 5, f"{probe.n_odom} msgs in 3 s")
    check("/scan publishing (360 beams)", probe.n_scan > 3 and probe.scan is not None and len(probe.scan.ranges) == 360,
          f"{probe.n_scan} msgs")
    check("TF odom->base_footprint publishing", probe.n_tf > 20, f"{probe.n_tf} msgs")

    info = subprocess.run("ros2 topic info /cmd_vel -v", shell=True, capture_output=True, text=True).stdout
    check("/cmd_vel type is geometry_msgs/msg/TwistStamped", "geometry_msgs/msg/TwistStamped" in info)
    gzinfo = subprocess.run("timeout 10 gz topic -i -t /cmd_vel", shell=True, capture_output=True, text=True).stdout
    check("Gazebo receives /cmd_vel (ros_gz_bridge is its subscriber)",
          "ros_gz_bridge" in info and ("Subscribers" in gzinfo and "gz.msgs.Twist" in gzinfo), "")

    check("nothing publishes /cmd_vel while the robot is idle", not cmd_vel_publishers(),
          "publishers: %s" % sorted(cmd_vel_publishers()))
    tel = Teleop()
    try:
        pubs = cmd_vel_publishers()
        check("only teleop_twist_keyboard publishes /cmd_vel during teleoperation",
              pubs == {"teleop_twist_keyboard"}, "publishers: %s" % sorted(pubs))
        def move(label, key, secs, test):
            probe.spin(0.5)
            o0, g0, scan0, tf0 = probe.odom_pose(), gz_pose(), list(probe.scan.ranges), probe.tf_stamp
            tel.hold(key, secs, probe)
            tel.key("k")
            probe.spin(0.5)
            o1, g1 = probe.odom_pose(), gz_pose()
            ok, detail = test(o0, o1, g0, g1)
            check(label, ok, detail)
            return o0, o1, g0, g1, scan0, tf0

        def fwd(o0, o1, g0, g1):
            d = (o1[0] - o0[0]) * math.cos(o0[2]) + (o1[1] - o0[1]) * math.sin(o0[2])
            gd = (g1[0] - g0[0]) * math.cos(g0[2]) + (g1[1] - g0[1]) * math.sin(g0[2])
            return d > 0.15 and gd > 0.15, f"odom {d:+.2f} m, Gazebo truth {gd:+.2f} m along heading"

        def back(o0, o1, g0, g1):
            d = (o1[0] - o0[0]) * math.cos(o0[2]) + (o1[1] - o0[1]) * math.sin(o0[2])
            gd = (g1[0] - g0[0]) * math.cos(g0[2]) + (g1[1] - g0[1]) * math.sin(g0[2])
            return d < -0.10 and gd < -0.10, f"odom {d:+.2f} m, Gazebo truth {gd:+.2f} m along heading"

        def left(o0, o1, g0, g1):
            d, gd = angdiff(o1[2], o0[2]), angdiff(g1[2], g0[2])
            return d > 0.4 and gd > 0.4, f"odom yaw {d:+.2f} rad, Gazebo truth {gd:+.2f} rad"

        def right(o0, o1, g0, g1):
            d, gd = angdiff(o1[2], o0[2]), angdiff(g1[2], g0[2])
            return d < -0.4 and gd < -0.4, f"odom yaw {d:+.2f} rad, Gazebo truth {gd:+.2f} rad"

        _, _, _, _, scan0, tf0 = move("teleop FORWARD ('i') moves the robot forward", "i", 3.0, fwd)
        check("/scan changes while driving", any(abs(a - b) > 0.02 for a, b in zip(scan0, probe.scan.ranges)
                                                   if math.isfinite(a) and math.isfinite(b)))
        check("TF odom->base_footprint advances while driving", probe.tf_stamp and tf0 and probe.tf_stamp > tf0,
              f"stamp {tf0} -> {probe.tf_stamp}")
        move("teleop BACKWARD (',') moves the robot backward", ",", 2.0, back)
        move("teleop LEFT ('j') turns the robot left", "j", 2.0, left)
        move("teleop RIGHT ('l') turns the robot right", "l", 2.0, right)

        # STOP: drive, press 'k', robot must come to rest
        tel.hold("i", 1.5, probe)
        tel.key("k")
        probe.spin(1.5)
        v, w = probe.speed()
        g_a = gz_pose()
        probe.spin(2.0)
        g_b = gz_pose()
        drift = math.hypot(g_b[0] - g_a[0], g_b[1] - g_a[1])
        check("teleop STOP ('k') brings the robot to rest", v < 0.01 and w < 0.02 and drift < 0.02,
              f"speed {v:.3f} m/s, yaw rate {w:.3f}, drift {drift:.3f} m in 2 s")

        # SAFETY: teleop killed while moving (Ctrl+C / crash / closed terminal)
        tel.hold("i", 1.5, probe)
        g_a = gz_pose()
        tel.kill()
        probe.spin(0.3)
        g_b = gz_pose()
        probe.spin(2.5)
        g_c = gz_pose()
        v, w = probe.speed()
        moved = math.hypot(g_c[0] - g_b[0], g_c[1] - g_b[1])
        check("robot STOPS by itself after teleop is killed mid-motion (no runaway)",
              v < 0.01 and moved < 0.05, f"speed {v:.3f} m/s, moved {moved:.2f} m in the 2.5 s after the kill")
    finally:
        tel.kill()
    return probe


def run_nav():
    from geometry_msgs.msg import PoseStamped
    probe = Probe()
    pub = probe.node.create_publisher(PoseStamped, "/goal_pose", 10)
    probe.spin(2)
    st = subprocess.run("ros2 lifecycle get /bt_navigator", shell=True, capture_output=True, text=True).stdout
    check("Nav2 bt_navigator is active", "active [3]" in st, st.strip())
    g0 = gz_pose()
    goal = (-1.25, 0.25)
    m = PoseStamped()
    m.header.frame_id = "map"
    m.pose.position.x, m.pose.position.y, m.pose.orientation.w = goal[0], goal[1], 1.0
    for _ in range(3):
        m.header.stamp = probe.node.get_clock().now().to_msg()
        pub.publish(m)
        probe.spin(0.3)
    # Nav2 received the goal: the action status topic reports it / the planner publishes /plan
    out = subprocess.run("timeout 20 ros2 topic echo --once /plan --field header.frame_id",
                         shell=True, capture_output=True, text=True).stdout.strip()
    check("Nav2 received the clicked goal and planned a path (/plan, frame map)", out.startswith("map"), out)
    t0 = time.monotonic()
    reached = False
    while time.monotonic() - t0 < 90:
        probe.spin(2)
        g = gz_pose()
        if g and math.hypot(g[0] - goal[0], g[1] - goal[1]) < 0.2:
            reached = True
            break
    g1 = gz_pose()
    check("Nav2 drove the robot to the clicked goal autonomously", reached,
          f"start {g0[:2]} -> end ({g1[0]:.2f}, {g1[1]:.2f}), goal {goal}, {time.monotonic() - t0:.0f} s")
    probe.spin(3)
    v, w = probe.speed()
    check("robot is at rest after reaching the goal", v < 0.02 and w < 0.05, f"{v:.3f} m/s")
    pubs = cmd_vel_publishers()
    check("during Nav2 only the Nav2 chain publishes /cmd_vel (no teleop/SLAM/DQN driver)",
          pubs <= {"twist_stamper", "docking_server"} and "twist_stamper" in pubs, "publishers: %s" % sorted(pubs))
    rtf, load = sim_speed(10)
    check("simulation runs near real time with Nav2 + sim (smooth enough for a remote session)", rtf > 0.5,
          f"real-time factor {rtf:.2f}, load average {load:.1f} on {os.cpu_count()} CPUs (headless: GUI/remote desktop adds load)")
    return probe


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["teleop", "nav"])
    p.add_argument("--json")
    a = p.parse_args()
    probe = run_teleop() if a.mode == "teleop" else run_nav()
    if a.json:
        with open(a.json, "w") as f:
            json.dump(RESULTS, f, indent=2)
    probe.node.destroy_node()
    probe.rclpy.shutdown()
    failed = [r for r in RESULTS if not r["pass"]]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
