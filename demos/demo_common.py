#!/usr/bin/env python3
"""
Shared plumbing for the demo scripts.

Not a ROS package -- these are scripts, run with `python3 demos/<name>.py` after
sourcing the workspace. Python puts a script's own directory on sys.path, so
`from demo_common import ...` works from any cwd.

Everything here is deliberately thin. The interesting logic (how to follow a
hand, when to open the gripper) belongs in the demo that implements it, not
hidden in a helper.
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ur5_interfaces.action import Gripper, MoveToNamedPose, MoveToPose
from ur5_interfaces.msg import TrackedTarget

HAND_TARGET_TOPIC = '/perception/hand_target'

GRIPPER_OPEN = 0
GRIPPER_CLOSED = 255


class HandTargetMonitor:
    """Tracks the most recent target and how stale it is.

    Freshness is judged from the message's header stamp, not from arrival time:
    the bridge republishes a smoothed estimate at a fixed rate, and stamps it
    with when the target was actually OBSERVED. A detector that has frozen
    would otherwise look perfectly healthy.

    `label_filter` is an optional predicate on TrackedTarget.label. It exists so
    a demo can say which KIND of target it wants (a hand, a specific tag) while
    the one-topic contract stays intact -- the seam is still
    /perception/hand_target, the consumer just declines to follow a point that
    is not the thing it was asked to follow. Rejected labels are remembered so
    the demo can tell "nothing is publishing" apart from "something is
    publishing, but not what you asked for", which are very different fixes.
    """

    def __init__(self, node: Node, timeout_sec=0.75, topic=HAND_TARGET_TOPIC,
                 label_filter=None):
        self._node = node
        self._timeout = timeout_sec
        self._label_filter = label_filter
        self._position = None
        self._stamp = None
        self._confidence = 0.0
        self._label = ''
        self._ever_seen = False
        self._rejected_labels = set()
        node.create_subscription(TrackedTarget, topic, self._on_target, 10)

    def _on_target(self, msg: TrackedTarget):
        if self._label_filter is not None and not self._label_filter(msg.label):
            self._rejected_labels.add(msg.label)
            return
        self._position = (msg.position.x, msg.position.y, msg.position.z)
        self._stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self._confidence = msg.confidence
        self._label = msg.label
        self._ever_seen = True

    @property
    def ever_seen(self):
        return self._ever_seen

    @property
    def confidence(self):
        return self._confidence

    @property
    def label(self):
        """Label of the most recent ACCEPTED target ('' if there has been none)."""
        return self._label

    @property
    def rejected_labels(self):
        """Labels seen on the topic but refused by label_filter."""
        return frozenset(self._rejected_labels)

    def age(self):
        """Seconds since the target was observed, or None if never seen."""
        if self._stamp is None:
            return None
        return (self._node.get_clock().now() - self._stamp).nanoseconds / 1e9

    def position(self):
        """Latest position if fresh, else None."""
        age = self.age()
        if self._position is None or age is None or age > self._timeout:
            return None
        return self._position


class MotionClients:
    """Action clients for the ur5_control stack."""

    def __init__(self, node: Node):
        self._node = node
        self.move_to_pose = ActionClient(node, MoveToPose, 'move_to_pose')
        self.move_to_named_pose = ActionClient(
            node, MoveToNamedPose, 'move_to_named_pose'
        )
        self.gripper = ActionClient(node, Gripper, 'gripper')

    def wait_for_servers(self, need_gripper=True, timeout_sec=15.0):
        """Block until the required servers are up. Returns True on success."""
        required = [('move_to_pose', self.move_to_pose),
                    ('move_to_named_pose', self.move_to_named_pose)]
        if need_gripper:
            required.append(('gripper', self.gripper))

        for name, client in required:
            self._node.get_logger().info(f"Waiting for '{name}' action server...")
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self._node.get_logger().error(
                    f"'{name}' is not available. Is ur5_bringup control.launch.py running?"
                )
                return False
        return True

    @staticmethod
    def pose_goal(x, y, z, grasp_type='top', grasp_orientation='grasp1',
                  force_linear=False, velocity_scaling=0.0):
        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.grasp_type = grasp_type
        goal.grasp_orientation = grasp_orientation
        goal.force_linear = force_linear
        goal.velocity_scaling = float(velocity_scaling)
        return goal

    @staticmethod
    def gripper_goal(position, force=100, speed=255):
        goal = Gripper.Goal()
        goal.target_position = int(position)
        goal.force = int(force)
        goal.speed = int(speed)
        return goal


def send_and_wait(node: Node, client: ActionClient, goal, timeout_sec=60.0):
    """Send a goal and block until it finishes. Returns (ok, result_or_None).

    For the step-by-step demos. The follow demos must NOT use this -- they
    need to keep processing hand updates while the arm moves.
    """
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout_sec)
    if not send_future.done():
        node.get_logger().error("Timed out waiting for the goal to be accepted.")
        return False, None

    handle = send_future.result()
    if not handle.accepted:
        node.get_logger().error("Goal was REJECTED by the server.")
        return False, None

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)
    if not result_future.done():
        node.get_logger().error("Timed out waiting for the result.")
        return False, None

    result = result_future.result().result
    ok = bool(getattr(result, 'success', False))
    message = getattr(result, 'message', '')
    if ok:
        node.get_logger().info(f"OK: {message}")
    else:
        node.get_logger().error(f"FAILED: {message}")
    return ok, result


def wait_for_target(node: Node, monitor: HandTargetMonitor, timeout_sec=30.0):
    """Spin until a fresh target shows up. Returns its position, or None."""
    node.get_logger().info(
        f"Waiting up to {timeout_sec:.0f}s for a target on {HAND_TARGET_TOPIC}..."
    )
    deadline = node.get_clock().now().nanoseconds / 1e9 + timeout_sec
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        position = monitor.position()
        if position is not None:
            return position
        if node.get_clock().now().nanoseconds / 1e9 > deadline:
            node.get_logger().error(
                f"No target seen. Check that a source is publishing "
                f"{HAND_TARGET_TOPIC} (hand_target_bridge or mock_hand_publisher)."
            )
            return None
    return None


def add_common_args(parser):
    """Arguments every demo understands."""
    parser.add_argument(
        '--dry-run', action='store_true',
        help="Compute and log every command without sending it to the robot. "
             "Run this first when you change anything.",
    )
    parser.add_argument(
        '--velocity-scaling', type=float, default=0.25,
        help="MoveIt velocity scaling, 0-1 (default: 0.25). Demos that run near "
             "a person default low on purpose.",
    )
    return parser


def format_xyz(p):
    return f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"


def die(node, message, code=1):
    node.get_logger().error(message)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)
