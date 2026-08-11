#!/usr/bin/env python3
"""
rail_spray_demo.py
===================
Boustrophedon-sweep demo for the rail-mounted UR5e (ur_spray_rail_demo.launch.py):
3 horizontal lines, one above, one at, and one below the demo_car sweep
height, alternating direction (no wasted return trip) — like an ox plowing
a field. The rail does the horizontal motion each line; the arm only moves
between lines, and holds a single fixed pose during each line itself.

Line order: TOP (above), MIDDLE (the original single-line demo, unchanged
pose), BOTTOM (below) — TOP happens before MIDDLE, BOTTOM after.

  rail_start ──TOP (spray)──> rail_end
                                  |  arm steps down (curved joint move)
  rail_start <──MIDDLE (spray)── rail_end
      |  arm steps down (curved joint move)
  rail_start ──BOTTOM (spray)──> rail_end

Where the TOP/BOTTOM arm poses come from
-------------------------------------------
Joint trajectories only, no Cartesian poses or inverse kinematics: TOP and
BOTTOM are simply the MIDDLE pose (config/cartesian_poses.yaml's centre
waypoint, the same pose the original single-line demo already uses and
which is proven to work) with shoulder_lift_joint offset by a fixed amount
in each direction. No FK/IK/solver involved anywhere in this file.

An earlier version of this script computed every sweep waypoint via
inverse kinematics from scratch and produced visibly wrong arm motion, and
a second version limited IK to a single small step from a good seed - both
were dropped in favour of this: a plain joint-space offset is fully
deterministic, trivial to reason about, and has no solver to misbehave.
The tradeoff is that "up"/"down" here means "rotate shoulder_lift_joint by
this much", not a geometrically exact vertical Cartesian distance - the
row_joint_offset parameter is in radians, not metres, and its sign/scale is
meant to be tuned empirically by watching where each line actually lands,
not computed.

No MoveIt / MoveItPy dependency
--------------------------------
Same reasoning as cartesian_path_executor.py: `moveit_py` isn't installable
via apt for ROS 2 Humble (only Iron/Rolling), so it's absent from this
image. Trajectories are published directly to joint_trajectory_controller /
rail_trajectory_controller, bypassing MoveIt entirely.

Usage
-----
  # In a separate terminal (after ur_spray_rail_demo.launch.py is up and
  # stable — controllers spawn around T+27s):
  ros2 run gz_spray_painting_plugin_demo rail_spray_demo.py
  ros2 run gz_spray_painting_plugin_demo rail_spray_demo.py --ros-args \\
      -p rail_start:=-1.4 -p rail_end:=1.4 -p rail_traverse_duration:=30.0 \\
      -p row_joint_offset:=0.216
"""

import os
import subprocess
import sys
import time
import threading

import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

HOME = {
    "shoulder_pan_joint":  0.0,
    "shoulder_lift_joint": -1.5708,
    "elbow_joint":          1.5708,
    "wrist_1_joint":       -1.5708,
    "wrist_2_joint":       -1.5708,
    "wrist_3_joint":        0.0,
}

# Which joint TOP/BOTTOM offset from MIDDLE, and by how much (radians) by
# default. shoulder_lift_joint is the "shoulder" joint - rotating it swings
# the whole arm up/down for most UR5e configurations, so it's the natural
# first choice for an approximate row-to-row step. Tune ROW_JOINT_OFFSET
# (sign and magnitude) by watching where TOP/BOTTOM actually land in Gazebo.
ROW_JOINT_NAME = "shoulder_lift_joint"

MAX_JOINT_SPEED = 1.0  # rad/s
RAIL_JOINT_NAME = "rail_to_carriage"
RAIL_SETTLE_TOLERANCE_M = 0.02

# The prismatic joint's hard velocity limit is 0.6 m/s (rail_axis.xacro,
# doubled alongside RAIL_MAX_SPEED below - keep the two in sync). RAIL_MAX_SPEED
# stays safely under that so there's margin for accel/decel - a fixed short
# duration is infeasible for anything but small moves: e.g. 5.0s for a 1.4 m
# move demands ~0.28 m/s with zero ramp-up time, which joint_trajectory_controller's
# tolerance check (ur_sim_controllers_rail.yaml's rail_to_carriage trajectory/goal
# tolerances) can reject, truncating the move short of its target. Always
# compute duration from actual distance.
RAIL_MAX_SPEED = 0.4       # m/s
RAIL_MIN_MOVE_DURATION = 3.0   # floor, so tiny moves aren't instantaneous


def _rail_move_duration(y_from: float, y_to: float, max_speed: float = RAIL_MAX_SPEED) -> float:
    return max(RAIL_MIN_MOVE_DURATION, abs(y_to - y_from) / max_speed)


def _move_duration(q_from, q_to, velocity_scaling):
    max_delta = max(abs(a - b) for a, b in zip(q_from, q_to))
    if max_delta < 1e-6:
        return 0.5
    speed = MAX_JOINT_SPEED * max(0.05, min(1.0, velocity_scaling))
    return max(1.5, max_delta / speed)


def _default_poses_file():
    """Resolve config/cartesian_poses.yaml via the installed package share
    directory — the same mechanism cartesian_spray.launch.py uses. (Not a
    path relative to this script's own location: once installed, this
    script lives under lib/gz_spray_painting_plugin_demo/, and config/ is
    under share/gz_spray_painting_plugin_demo/ instead, so a relative
    "../config" guess resolves to a directory that doesn't exist.)"""
    return os.path.join(
        get_package_share_directory("gz_spray_painting_plugin_demo"),
        "config", "cartesian_poses.yaml",
    )


def _offset_config(base_config: dict, joint_name: str, delta: float) -> dict:
    """Copy base_config with joint_name shifted by delta radians. Pure
    joint-space edit - no kinematics involved."""
    cfg = dict(base_config)
    cfg[joint_name] = cfg[joint_name] + delta
    return cfg


# ── Demo node ─────────────────────────────────────────────────────────────────

class RailSprayDemo(Node):

    def __init__(self):
        super().__init__("rail_spray_demo")

        self.declare_parameter("poses_file", _default_poses_file())
        self.declare_parameter("velocity_scaling", 0.35)
        self.declare_parameter("spray_topic", "/spray_paint/trigger")

        # Rail traverse: each line is one continuous pass between these two
        # extremes, alternating direction line to line. Defaults leave a
        # small margin inside the ±1.5 m limit that rail_length:=3.0 (the
        # launch file's default) produces — adjust both if you launch with
        # a different rail_length.
        self.declare_parameter("rail_start", -1.4)
        self.declare_parameter("rail_end", 1.4)
        self.declare_parameter("rail_traverse_duration", 30.0)  # seconds — halved to double speed

        # TOP/BOTTOM = MIDDLE with shoulder_lift_joint offset by +/- this
        # many radians. Not a Cartesian distance - see module docstring.
        # Measured offline with the same DH model used elsewhere in this
        # repo (not used at runtime here - joint trajectories only):
        #   0.15  rad -> ~0.045/-0.047 m  (~0.09 m top-to-bottom span) - overlapped
        #   0.216 rad -> ~0.064/-0.067 m  (~0.13 m top-to-bottom span) - current (0.24 - 10%)
        #   0.24  rad -> ~0.071/-0.075 m  (~0.15 m top-to-bottom span) - previous
        #   0.40  rad -> ~0.114/-0.125 m  (~0.24 m top-to-bottom span) - gapped
        self.declare_parameter("row_joint_offset", 0.216)

        self._arm_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self._rail_pub = self.create_publisher(
            JointTrajectory, "/rail_trajectory_controller/joint_trajectory", 10
        )

        self._current_arm_joints = None
        self._rail_position = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        time.sleep(1.0)
        self.get_logger().info("RailSprayDemo node started")

    # ── State feedback ──────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        try:
            self._current_arm_joints = [msg.position[msg.name.index(j)] for j in JOINT_NAMES]
        except (ValueError, IndexError):
            pass
        if RAIL_JOINT_NAME in msg.name:
            self._rail_position = msg.position[msg.name.index(RAIL_JOINT_NAME)]

    def _wait_for_joints(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._current_arm_joints is not None:
                return list(self._current_arm_joints)
            time.sleep(0.05)
        return None

    def _wait_for_controllers(self, timeout=60.0):
        self.get_logger().info(
            "Waiting for joint_trajectory_controller and rail_trajectory_controller…"
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._arm_pub.get_subscription_count() > 0 and \
                    self._rail_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.5)
        self.get_logger().error("Controllers not ready after timeout")
        return False

    # ── Spray control (direct gz topic, matching cartesian_path_executor.py) ──

    def _spray(self, on: bool):
        spray_topic = self.get_parameter("spray_topic").value
        try:
            subprocess.run(
                ["gz", "topic", "-t", spray_topic, "-m", "gz.msgs.Boolean",
                 "-p", f"data: {'true' if on else 'false'}"],
                check=True, timeout=5.0,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            self.get_logger().warn(f"gz topic spray command failed: {exc}")
        self.get_logger().info(f"Spray {'ON' if on else 'OFF'}")

    # ── Rail control ─────────────────────────────────────────────────────────

    def _move_rail_to(self, y: float, duration_s: float, label: str = "rail"):
        """Send the carriage to world-Y `y` over `duration_s`, and block
        until it settles. A single trajectory point with a long duration is
        exactly what makes this move continuous/slow rather than stepped -
        joint_trajectory_controller interpolates smoothly to it."""
        self.get_logger().info(f"{label}: moving rail to y={y:.3f} m over {duration_s:.1f}s…")

        traj = JointTrajectory()
        traj.joint_names = [RAIL_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [y]
        point.velocities = [0.0]
        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [point]
        self._rail_pub.publish(traj)

        # Generous margin over the commanded duration, not a fixed timeout -
        # a long slow traverse must not be cut off early.
        timeout_s = duration_s + 15.0
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._rail_position is not None and \
                    abs(self._rail_position - y) < RAIL_SETTLE_TOLERANCE_M:
                self.get_logger().info(f"{label}: rail settled at y={self._rail_position:.3f} m")
                return True
            time.sleep(0.2)

        self.get_logger().warn(
            f"{label}: rail did not settle within {timeout_s:.1f}s "
            f"(last known position: {self._rail_position})"
        )
        return False

    # ── Arm trajectory helpers — identical mechanics to cartesian_path_executor.py ──

    def _send_arm_trajectory(self, q_list, vel_scale, label, zero_end_vel=True):
        n = len(q_list)
        if n == 0:
            return True

        times = [0.0]
        for i in range(1, n):
            times.append(times[-1] + _move_duration(q_list[i - 1], q_list[i], vel_scale))

        vels = [[0.0] * 6]
        for i in range(1, n - 1):
            dt = times[i + 1] - times[i - 1]
            v = [(q_list[i + 1][j] - q_list[i - 1][j]) / dt for j in range(6)]
            vels.append(v)
        vels.append([0.0] * 6 if zero_end_vel else vels[-1])

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        for q, v, t in zip(q_list, vels, times):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.velocities = v
            pt.accelerations = [0.0] * 6
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            msg.points.append(pt)

        self._arm_pub.publish(msg)
        total = times[-1]
        self.get_logger().info(f"  {label}: {n} pts, {total:.1f}s — waiting...")
        time.sleep(total + 0.5)
        return True

    def _move_arm_to(self, target_joints: dict, vel_scale: float, label: str = "move"):
        """A 2-point move with zero velocity at both ends. joint_trajectory_controller
        spline-interpolates between them, which is naturally a smooth, eased
        ("curved") joint motion rather than a sharp linear one - used both
        for home/approach moves and for the between-lines steps."""
        current = self._wait_for_joints(timeout=10.0)
        if current is None:
            self.get_logger().warn("No joint state received — sending move anyway")
            current = [target_joints[j] for j in JOINT_NAMES]
        q_to = [target_joints[j] for j in JOINT_NAMES]
        return self._send_arm_trajectory([current, q_to], vel_scale, label)

    def _traverse_line(self, y_from: float, y_to: float, traverse_duration: float, label: str):
        self.get_logger().info(
            f"{label}: rail y={y_from:.2f} -> {y_to:.2f} over {traverse_duration:.1f}s"
        )
        self._spray(True)
        self._move_rail_to(y_to, traverse_duration, label=label)
        self._spray(False)

    # ── Main demo ──────────────────────────────────────────────────────────

    def run(self):
        poses_file = self.get_parameter("poses_file").value
        vel_scale  = self.get_parameter("velocity_scaling").value
        rail_start = self.get_parameter("rail_start").value
        rail_end   = self.get_parameter("rail_end").value
        traverse_s = self.get_parameter("rail_traverse_duration").value
        row_offset = self.get_parameter("row_joint_offset").value

        self.get_logger().info(f"Loading joint configs from {poses_file}")
        try:
            with open(poses_file, "r") as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Poses file not found: {poses_file}")
            return

        joint_configs = [
            {jn: float(entry[jn]) for jn in JOINT_NAMES}
            for entry in data.get("joint_configs", [])
        ]
        if len(joint_configs) == 0:
            self.get_logger().error("No joint configs in the poses file.")
            return

        # MIDDLE = the centre waypoint of the demo_car sweep, unchanged from
        # the single-line version of this script.
        middle_config = joint_configs[len(joint_configs) // 2]
        self.get_logger().info(
            f"Middle pose = joint_configs[{len(joint_configs) // 2}]: {middle_config}"
        )

        # TOP/BOTTOM: plain joint-space offset of middle_config, no
        # kinematics - see module docstring.
        top_config = _offset_config(middle_config, ROW_JOINT_NAME, +row_offset)
        bottom_config = _offset_config(middle_config, ROW_JOINT_NAME, -row_offset)
        self.get_logger().info(
            f"Top pose    = middle with {ROW_JOINT_NAME} {'+' if row_offset >= 0 else ''}"
            f"{row_offset:.3f} rad -> {top_config[ROW_JOINT_NAME]:.3f}"
        )
        self.get_logger().info(
            f"Bottom pose = middle with {ROW_JOINT_NAME} "
            f"{-row_offset:+.3f} rad -> {bottom_config[ROW_JOINT_NAME]:.3f}"
        )

        self.get_logger().info("=== UR5e Rail Boustrophedon Spray Demo (3 lines) ===")

        if not self._wait_for_controllers(timeout=60.0):
            return
        time.sleep(1.0)

        self._spray(False)

        # Rail goes to its starting extreme FIRST, before the arm moves at
        # all - the arm then only ever has to move (home -> top pose) once
        # it's already sitting at that end.
        rail_now = self._rail_position if self._rail_position is not None else 0.0
        self._move_rail_to(rail_start, _rail_move_duration(rail_now, rail_start),
                            label="approach start corner")
        time.sleep(0.5)

        self.get_logger().info("Moving arm to home, then to the TOP line pose...")
        self._move_arm_to(HOME, vel_scale, label="home")
        time.sleep(0.5)
        self._move_arm_to(top_config, vel_scale, label="up to top line")
        time.sleep(0.5)

        traverse_duration = max(traverse_s, _rail_move_duration(rail_start, rail_end))

        # LINE 1 - TOP, rail_start -> rail_end.
        self._traverse_line(rail_start, rail_end, traverse_duration, "LINE 1/3 (top)")

        # Curved step down: top -> middle.
        self._move_arm_to(middle_config, vel_scale, label="step down to middle line")
        time.sleep(0.5)

        # LINE 2 - MIDDLE (the original single-line demo), alternating
        # direction: rail_end -> rail_start, no wasted return trip.
        self._traverse_line(rail_end, rail_start, traverse_duration, "LINE 2/3 (middle)")

        # Curved step down: middle -> bottom.
        self._move_arm_to(bottom_config, vel_scale, label="step down to bottom line")
        time.sleep(0.5)

        # LINE 3 - BOTTOM, alternating back: rail_start -> rail_end.
        self._traverse_line(rail_start, rail_end, traverse_duration, "LINE 3/3 (bottom)")

        self.get_logger().info("All 3 lines complete — returning home")
        self._move_arm_to(HOME, vel_scale, label="home")
        self._move_rail_to(0.0, _rail_move_duration(rail_end, 0.0), label="return to centre")

        self.get_logger().info("=== Demo finished ===")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RailSprayDemo()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node._spray(False)   # safety: always turn spray off on exit
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
