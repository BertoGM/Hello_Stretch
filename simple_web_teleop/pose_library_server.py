#!/usr/bin/env python3
"""World-frame pose library for the simple web teleop.

Uses the two TF functions Lab 13 recommends so we don't reimplement the
math ourselves:

    tf_buffer.lookup_transform(target, source, time)   # capture EE pose
    tf_buffer.transform(pose_stamped, target_frame)    # project for replay

The browser talks to this node over plain string topics through rosbridge:

    /pose_library/save   (sub)  std_msgs/String — name to save under
    /pose_library/play   (sub)  std_msgs/String — name to replay
    /pose_library/delete (sub)  std_msgs/String — name to remove
    /pose_library/names  (pub)  std_msgs/String — JSON array, latched

Run with the rest of the stack already up (stretch_driver + rosbridge):

    python3 pose_library_server.py

Persistence: ~/.stretch_pose_library.json
"""

import json
import math
import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers do_transform_pose for PoseStamped

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg


WORLD_FRAME = "odom"
BASE_FRAME = "base_link"
EE_FRAME = "link_grasp_center"

# Joint clamps — match the browser teleop bounds.
LIFT_MIN, LIFT_MAX = 0.15, 1.09
EXT_MIN, EXT_MAX = 0.0, 0.50

STORAGE = Path.home() / ".stretch_pose_library.json"

WATCHED_JOINTS = (
    "joint_lift",
    "wrist_extension",
    "joint_gripper_finger_left",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
)


def load_db():
    if not STORAGE.exists():
        return {}
    try:
        return json.loads(STORAGE.read_text())
    except json.JSONDecodeError:
        return {}


def save_db(db):
    STORAGE.write_text(json.dumps(db, indent=2))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _yaw_of(q):
    """ZYX yaw from a quaternion (geometry_msgs.Quaternion or dict)."""
    if hasattr(q, "w"):
        x, y, z, w = q.x, q.y, q.z, q.w
    else:
        x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _stamped_to_dict(ts):
    return {
        "translation": {
            "x": ts.transform.translation.x,
            "y": ts.transform.translation.y,
            "z": ts.transform.translation.z,
        },
        "rotation": {
            "x": ts.transform.rotation.x,
            "y": ts.transform.rotation.y,
            "z": ts.transform.rotation.z,
            "w": ts.transform.rotation.w,
        },
    }


class PoseLibraryServer(Node):
    def __init__(self):
        super().__init__("pose_library_server")

        # tf2 buffer + listener — same primitives the lab examples use.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Joint state cache so save() can snapshot current joints.
        self._joints = {}
        self._joints_lock = threading.Lock()
        self.create_subscription(
            JointState, "/stretch/joint_states", self._on_joints, 10
        )

        # Trajectory action — same one the browser uses for direct teleop.
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        # Browser-facing topics.
        self.create_subscription(String, "/pose_library/save", self._on_save, 10)
        self.create_subscription(String, "/pose_library/play", self._on_play, 10)
        self.create_subscription(String, "/pose_library/delete", self._on_delete, 10)

        # Latched (TRANSIENT_LOCAL) so a late-joining browser gets the list.
        names_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.names_pub = self.create_publisher(String, "/pose_library/names", names_qos)

        self.db = load_db()
        self._publish_names()
        self.get_logger().info(
            f"pose library ready ({len(self.db)} saved); storage={STORAGE}"
        )

    # ------------------------------------------------------------------ joints

    def _on_joints(self, msg: JointState):
        with self._joints_lock:
            for n, p in zip(msg.name, msg.position):
                if n in WATCHED_JOINTS:
                    self._joints[n] = float(p)

    def _current_joints(self):
        with self._joints_lock:
            return dict(self._joints)

    # ---------------------------------------------------------------- pub-list

    def _publish_names(self):
        self.names_pub.publish(String(data=json.dumps(sorted(self.db.keys()))))

    # ---------------------------------------------------------------- handlers

    def _on_save(self, msg: String):
        # Accept either "name" or "frame|name". Frame defaults to odom.
        # frame can be any TF frame the buffer can resolve from link_grasp_center,
        # e.g. "odom", "map", or an ArUco frame like "target_frame".
        raw = msg.data.strip()
        if not raw:
            self.get_logger().warn("save called with empty payload")
            return
        if "|" in raw:
            frame, name = (s.strip() for s in raw.split("|", 1))
        else:
            frame, name = WORLD_FRAME, raw
        if not name:
            self.get_logger().warn("save payload missing name")
            return

        # Lab function #1: lookup_transform. We capture the EE pose in two
        # frames so replay can compute deltas without our own forward
        # kinematics for the arm.
        try:
            ee_in_world = self.tf_buffer.lookup_transform(
                frame, EE_FRAME, Time(), Duration(seconds=1.0)
            )
            ee_in_base = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, Time(), Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"TF lookup failed (frame={frame!r}): {e}")
            return

        joints = self._current_joints()
        missing = [j for j in WATCHED_JOINTS if j not in joints]
        if missing:
            self.get_logger().warn(f"missing joints at save time: {missing}")

        entry = {
            "name": name,
            "frame": frame,
            "world_pose": _stamped_to_dict(ee_in_world),
            "base_pose_at_save": _stamped_to_dict(ee_in_base),
            "joints_at_save": joints,
        }
        self.db[name] = entry
        save_db(self.db)
        self._publish_names()

        t = entry["world_pose"]["translation"]
        self.get_logger().info(
            f"saved '{name}' in {frame} at ({t['x']:.3f}, {t['y']:.3f}, {t['z']:.3f})"
        )

    def _on_delete(self, msg: String):
        name = msg.data.strip()
        if name in self.db:
            del self.db[name]
            save_db(self.db)
            self._publish_names()
            self.get_logger().info(f"deleted '{name}'")

    def _on_play(self, msg: String):
        name = msg.data.strip()
        entry = self.db.get(name)
        if not entry:
            self.get_logger().warn(f"no pose named '{name}'")
            return

        # Build PoseStamped in the saved frame (world).
        ps = PoseStamped()
        ps.header.frame_id = entry["frame"]
        # Stamp = 0 ("latest available") avoids "extrapolation into the future"
        # errors that happen when now() is a few ms ahead of the latest TF data.
        ps.header.stamp = Time().to_msg()
        wp = entry["world_pose"]
        ps.pose.position.x = wp["translation"]["x"]
        ps.pose.position.y = wp["translation"]["y"]
        ps.pose.position.z = wp["translation"]["z"]
        ps.pose.orientation.x = wp["rotation"]["x"]
        ps.pose.orientation.y = wp["rotation"]["y"]
        ps.pose.orientation.z = wp["rotation"]["z"]
        ps.pose.orientation.w = wp["rotation"]["w"]

        # Lab function #2: tf_buffer.transform. Reproject the saved world
        # pose into the robot's *current* base_link.
        try:
            ps_in_base = self.tf_buffer.transform(
                ps, BASE_FRAME, timeout=Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return

        # Translation delta in base_link from save time to now. The lab's
        # joint heuristic is x→base, y→arm extension, z→lift.
        save_xyz = entry["base_pose_at_save"]["translation"]
        dx = ps_in_base.pose.position.x - save_xyz["x"]
        dy = ps_in_base.pose.position.y - save_xyz["y"]
        dz = ps_in_base.pose.position.z - save_xyz["z"]

        # Yaw delta — how much the base rotated relative to the saved EE.
        # If the gripper is to keep pointing the same world direction, the
        # wrist yaw must counter-rotate by this amount.
        d_yaw = _yaw_of(ps_in_base.pose.orientation) - _yaw_of(
            entry["base_pose_at_save"]["rotation"]
        )

        joints = entry["joints_at_save"]
        # Stretch's arm extends in the -y direction of base_link, so ee.y in
        # base gets more negative as wrist_extension grows. d(ee.y)/d(ext) = -1,
        # therefore Δext = -Δ(ee.y).
        targets = {
            "joint_lift":                clamp(joints["joint_lift"] + dz, LIFT_MIN, LIFT_MAX),
            "wrist_extension":           clamp(joints["wrist_extension"] - dy, EXT_MIN, EXT_MAX),
            "joint_gripper_finger_left": joints.get("joint_gripper_finger_left", 0.0),
            "joint_wrist_yaw":           joints["joint_wrist_yaw"] + d_yaw,
            "joint_wrist_pitch":         joints["joint_wrist_pitch"],
            "joint_wrist_roll":          joints["joint_wrist_roll"],
        }

        self.get_logger().info(
            f"play '{name}': base-frame target "
            f"({ps_in_base.pose.position.x:+.3f}, {ps_in_base.pose.position.y:+.3f},"
            f" {ps_in_base.pose.position.z:+.3f}); "
            f"deltas dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f} d_yaw={d_yaw:+.3f}"
        )
        self.get_logger().info(
            "  saved joints: " + ", ".join(f"{k}={v:.3f}" for k, v in joints.items())
        )
        self.get_logger().info(
            "  command joints: " + ", ".join(f"{k}={v:.3f}" for k, v in targets.items())
        )
        if abs(dx) > 0.05:
            self.get_logger().warn(
                f"saved EE has |x|={abs(dx):.3f} m forward of base — Stretch's arm "
                "extends sideways only, so this component cannot be reached without "
                "driving the base. Replay will proceed arm-only."
            )

        self._send_trajectory(targets)

    # -------------------------------------------------------------- trajectory

    def _send_trajectory(self, joint_targets, seconds=3):
        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("trajectory action server not available")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_targets.keys())
        pt = JointTrajectoryPoint()
        pt.positions = list(joint_targets.values())
        pt.time_from_start = DurationMsg(sec=seconds, nanosec=0)
        goal.trajectory.points = [pt]
        self.traj_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = PoseLibraryServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
