#!/usr/bin/env python3
"""
follow_hand: keep the gripper hovering a fixed distance from a tracked hand.

    # safest first run: no robot, no camera, nothing sent anywhere
    ros2 launch ur5_bringup control.launch.py hand_source:=mock gripper_mock:=true
    python3 demos/follow_hand.py --dry-run

    # mock hardware, real motion through MoveIt, synthetic hand
    ros2 launch ur5_bringup robot.launch.py use_mock_hardware:=true
    ros2 launch ur5_bringup moveit.launch.py
    ros2 launch ur5_bringup control.launch.py hand_source:=mock gripper_mock:=true
    python3 demos/follow_hand.py

    # real rig, AprilTag on a wristband
    python3 demos/follow_hand.py --standoff 0.20 --max-step 0.05

Input is /perception/hand_target (ur5_interfaces/TrackedTarget, world frame),
whatever produced it: an AprilTag on a wristband, a hand detector on the Jetson,
or mock_hand_publisher. See docs/CAMERA_STACK.md.

The control chain, in order
---------------------------
  target ->  standoff  ->  safety clamp  ->  step leash  ->  deadband  ->  command

  standoff      Don't drive the gripper AT the hand. Back off along the approach
                axis so a tracking error moves the gripper near the hand, not
                into it.
  safety clamp  Workspace box from ur5_control.geometry, plus --max-y, which
                stops the arm short of where the operator stands. Clamped, not
                rejected: the arm tracks to the edge of its allowed volume and
                waits there, which reads as "it can't reach further" rather
                than "it froze".
  step leash    Each command moves at most --max-step from where the arm
                actually is. A detection that teleports (label swap, reflection,
                someone else's hand) becomes a small move, not a lunge.
  deadband      Ignore target motion under --deadband, so hand tremor and
                detector jitter don't turn into constant replanning.

Modes
-----
plan (default)
    One MoveToPose goal per cycle. Uses the same motion primitive as every
    other demo, respects the MoveIt planning scene, and cannot run away.
    But: a goal in flight is NOT preemptible -- ur5_control/move_to_pose accepts
    a cancel request without forwarding it to move_group -- so the arm always
    finishes the hop it is on before reacting to a new position. Measured
    against mock hardware, that is one hop every ~2.5 s, and the gripper trails
    a target moving at 0.08 m/s by ~0.18 m. Smaller --max-step and higher
    --velocity-scaling shorten each hop; nothing here removes the lag. Fine for
    "hold the tool near my hand", not for "shadow my hand".

servo
    Streams TwistStamped to moveit_servo at --rate Hz: a real closed loop, so
    the tracking is smooth. Costs more setup and drops the collision checking
    that plan mode gets for free (servo does singularity and joint-limit
    handling, not scene collision avoidance). Requires:
        ros2 launch ur5_bringup moveit.launch.py launch_servo:=true
        ros2 control switch_controllers \\
          --deactivate scaled_joint_trajectory_controller \\
          --activate forward_position_controller
    Switch the controllers back before running the plan-mode demos again.

Neither mode controls orientation: the wrist holds whatever --grasp-type asks
for and only the position tracks. Pointing the gripper AT the hand as it moves
is the obvious next step and needs an angular term in servo mode.
"""
import argparse
import math
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import TwistStamped, Vector3Stamped
import tf2_geometry_msgs  # noqa: F401  (registers Vector3Stamped transforms)

from ur5_interfaces.action import MoveToNamedPose

from ur5_control.geometry import (
    TABLE_HEIGHT, clamp_to_workspace, distance, standoff_pose, step_towards,
)

from demo_common import (
    HandTargetMonitor, MotionClients, add_common_args, format_xyz,
)

WORLD_FRAME = 'world'
EEF_LINK = 'tool0'
SERVO_TWIST_TOPIC = '/servo_node/delta_twist_cmds'
SERVO_SWITCH_SRV = '/servo_node/switch_command_type'
SERVO_COMMAND_TYPE_TWIST = 1   # moveit_msgs/srv/ServoCommandType

MODE_PLAN = 'plan'
MODE_SERVO = 'servo'

DEFAULT_RATE = {MODE_PLAN: 2.0, MODE_SERVO: 50.0}


class HandFollower(Node):
    def __init__(self, args):
        super().__init__('follow_hand')
        self.args = args

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._monitor = HandTargetMonitor(self, timeout_sec=args.target_timeout)
        self._motion = MotionClients(self)

        self._goal_handle = None      # in-flight MoveToPose, plan mode
        self._goal_pending = False    # goal sent, not yet accepted/rejected
        self._last_command = None     # last commanded tool0 position
        self._twist_pub = None
        self._lost_since = None
        self._returned_to_watch = False

        if args.mode == MODE_SERVO:
            self._setup_servo()
        elif not args.dry_run:
            if not self._motion.wait_for_servers(need_gripper=False):
                raise RuntimeError("Required action servers are missing.")

        rate = args.rate or DEFAULT_RATE[args.mode]
        self._period = 1.0 / rate
        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"follow_hand: mode={args.mode}, rate={rate:g}Hz, "
            f"standoff={args.standoff:.3f}m, max_step={args.max_step:.3f}m, "
            f"deadband={args.deadband:.3f}m, max_y={args.max_y:.2f}m"
            f"{'  [DRY RUN]' if args.dry_run else ''}"
        )

    # ---- Servo setup ----

    def _setup_servo(self):
        self._twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)

        if self.args.dry_run:
            return

        try:
            from moveit_msgs.srv import ServoCommandType
        except ImportError:
            raise RuntimeError(
                "moveit_msgs/ServoCommandType not found -- moveit_servo is not "
                "installed. Use --mode plan."
            )

        client = self.create_client(ServoCommandType, SERVO_SWITCH_SRV)
        self.get_logger().info(f"Waiting for {SERVO_SWITCH_SRV}...")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f"{SERVO_SWITCH_SRV} not available. Start servo with "
                f"'ros2 launch ur5_bringup moveit.launch.py launch_servo:=true'."
            )

        request = ServoCommandType.Request()
        request.command_type = SERVO_COMMAND_TYPE_TWIST
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result().success:
            raise RuntimeError("servo_node refused to switch to TWIST commands.")
        self.get_logger().info("servo_node is in TWIST mode.")

    # ---- Shared control chain ----

    def _current_tool0(self):
        # Short timeout on purpose: this runs inside the control loop, and the
        # driver publishes tool0 continuously, so a lookup that isn't already
        # satisfiable means something is wrong. Waiting 100 ms for it would
        # throttle a 50 Hz servo loop down to 10 Hz.
        try:
            tf = self._tf_buffer.lookup_transform(
                WORLD_FRAME, EEF_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {WORLD_FRAME}->{EEF_LINK} unavailable: {e}",
                throttle_duration_sec=5.0,
            )
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _resolve_current(self):
        """Where the arm is: TF, else the last command, else (dry run) a guess.

        The guess exists so `--dry-run` is useful with nothing else running at
        all -- no driver means no TF, and refusing to compute would make the
        one command you can safely run print nothing.
        """
        current = self._current_tool0()
        if current is not None:
            return current
        if self._last_command is not None:
            return self._last_command
        if self.args.dry_run:
            self.get_logger().warn(
                f"No TF for {EEF_LINK}; assuming the arm is at "
                f"{format_xyz(self.args.assume_tool0)} (dry run only).",
                throttle_duration_sec=10.0,
            )
            return self.args.assume_tool0
        return None

    def _desired_pose(self, target):
        """target (hand position, world) -> commanded tool0 position, world.

        Returns (position, clamped) or (None, False) if the target is somewhere
        we refuse to follow at all.
        """
        desired = standoff_pose(*target, self.args.grasp_type, self.args.standoff)

        # Stop short of the operator. The workspace box alone allows y up to
        # 1.0, which is past where a person stands at this table.
        y = min(desired[1], self.args.max_y)
        clamped_y = y != desired[1]

        x, y, z, clamped_box = clamp_to_workspace(desired[0], y, desired[2])
        return (x, y, z), (clamped_y or clamped_box)

    def _log_target_state(self, target):
        if target is None:
            age = self._monitor.age()
            if not self._monitor.ever_seen:
                self.get_logger().warn(
                    "No hand target yet on /perception/hand_target.",
                    throttle_duration_sec=5.0,
                )
            else:
                self.get_logger().warn(
                    f"Hand target stale ({age:.1f}s old); holding.",
                    throttle_duration_sec=2.0,
                )

    # ---- Timer ----

    def _tick(self):
        target = self._monitor.position()

        if target is None:
            self._log_target_state(target)
            self._on_target_lost()
            return

        self._lost_since = None
        self._returned_to_watch = False

        if self.args.mode == MODE_SERVO:
            self._tick_servo(target)
        else:
            self._tick_plan(target)

    def _on_target_lost(self):
        if self.args.mode == MODE_SERVO:
            # Servo stops on its own after incoming_command_timeout, but say it
            # explicitly rather than relying on a timeout for a safety stop.
            self._publish_twist((0.0, 0.0, 0.0))
            return

        if self.args.lost_return_sec <= 0.0 or self._returned_to_watch:
            return
        now = self.get_clock().now()
        if self._lost_since is None:
            self._lost_since = now
            return
        lost_for = (now - self._lost_since).nanoseconds / 1e9
        if lost_for < self.args.lost_return_sec or self._goal_pending:
            return

        self.get_logger().info(
            f"Target lost for {lost_for:.0f}s; returning to '{self.args.watch_pose}'."
        )
        self._returned_to_watch = True
        self._last_command = None
        if self.args.dry_run:
            return
        goal = MoveToNamedPose.Goal()
        goal.pose_name = self.args.watch_pose
        self._motion.move_to_named_pose.send_goal_async(goal)

    # ---- Plan mode ----

    def _tick_plan(self, target):
        if self._goal_pending or self._goal_handle is not None:
            # One hop at a time. move_to_pose can't be preempted mid-execution,
            # so queueing a second goal would just make the arm stutter through
            # a stale position after finishing this one.
            return

        desired, clamped = self._desired_pose(target)
        current = self._resolve_current()
        if current is None:
            self.get_logger().warn(
                "Don't know where the arm is yet; waiting for TF.",
                throttle_duration_sec=2.0,
            )
            return

        command = step_towards(current, desired, self.args.max_step)

        if (self._last_command is not None
                and distance(command, self._last_command) < self.args.deadband
                and distance(current, desired) < self.args.deadband):
            return

        self.get_logger().info(
            f"hand={format_xyz(target)} -> command={format_xyz(command)}"
            f"{'  [clamped]' if clamped else ''} "
            f"(error {distance(current, desired):.3f}m)"
        )

        if self.args.dry_run:
            self._last_command = command
            return

        goal = MotionClients.pose_goal(
            *command,
            grasp_type=self.args.grasp_type,
            grasp_orientation=self.args.grasp_orientation,
            force_linear=True,     # short hops: a straight line is predictable
            velocity_scaling=self.args.velocity_scaling,
        )
        self._goal_pending = True
        self._last_command = command
        self._motion.move_to_pose.send_goal_async(goal).add_done_callback(
            self._on_goal_response
        )

    def _on_goal_response(self, future):
        self._goal_pending = False
        handle = future.result()
        if not handle.accepted:
            # Usually the workspace guard in move_to_pose. Keep following:
            # the next target may well be reachable.
            self.get_logger().warn("move_to_pose rejected the goal.")
            self._last_command = None
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_handle = None
        result = future.result().result
        if not result.success:
            self.get_logger().warn(f"Hop failed: {result.message}")
            # Forget the commanded pose so the next cycle re-derives it from
            # where the arm actually ended up.
            self._last_command = None

    # ---- Servo mode ----

    def _tick_servo(self, target):
        desired, clamped = self._desired_pose(target)
        current = self._resolve_current()
        if current is None:
            self._publish_twist((0.0, 0.0, 0.0))
            return

        error = tuple(d - c for d, c in zip(desired, current))
        error_norm = math.sqrt(sum(e * e for e in error))

        if error_norm < self.args.deadband:
            self._publish_twist((0.0, 0.0, 0.0))
            return

        # P control with a hard speed cap. No integral term on purpose: the
        # target moves continuously, so there is no steady-state error worth
        # integrating, and windup near the workspace edge would be dangerous.
        speed = min(self.args.kp * error_norm, self.args.max_speed)
        velocity = tuple(e / error_norm * speed for e in error)

        self.get_logger().debug(
            f"error={error_norm:.3f}m speed={speed:.3f}m/s"
            f"{' [clamped]' if clamped else ''}"
        )
        self._publish_twist(velocity)

    def _publish_twist(self, velocity_world):
        if self._twist_pub is None:
            return

        vx, vy, vz = velocity_world
        if self.args.servo_frame != WORLD_FRAME:
            rotated = self._rotate_into_servo_frame((vx, vy, vz))
            if rotated is None:
                return
            vx, vy, vz = rotated

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.servo_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        # Angular left at zero: position-only tracking, see module docstring.

        if self.args.dry_run:
            self.get_logger().info(
                f"[dry-run] twist {self.args.servo_frame} "
                f"({vx:+.3f}, {vy:+.3f}, {vz:+.3f}) m/s"
            )
            return
        self._twist_pub.publish(msg)

    def _rotate_into_servo_frame(self, v_world):
        """Rotate a world-frame velocity into the servo planning frame.

        On this rig base_link is a pure translation from world, so this is the
        identity -- but the rig is exactly the kind of thing that gets
        re-mounted, and a silently wrong sign here drives the arm the wrong way.
        """
        vec = Vector3Stamped()
        vec.header.frame_id = WORLD_FRAME
        vec.header.stamp = rclpy.time.Time().to_msg()  # latest available
        vec.vector.x, vec.vector.y, vec.vector.z = v_world
        try:
            out = self._tf_buffer.transform(
                vec, self.args.servo_frame, timeout=Duration(seconds=0.05)
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Can't rotate twist into {self.args.servo_frame}: {e}",
                throttle_duration_sec=5.0,
            )
            return None
        return (out.vector.x, out.vector.y, out.vector.z)

    # ---- Shutdown ----

    def stop(self):
        if self.args.mode == MODE_SERVO:
            self._publish_twist((0.0, 0.0, 0.0))
        elif self._goal_handle is not None:
            self.get_logger().info("Cancelling in-flight goal...")
            self._goal_handle.cancel_goal_async()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Follow a tracked hand with the UR5.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    add_common_args(parser)
    parser.add_argument('--mode', choices=[MODE_PLAN, MODE_SERVO], default=MODE_PLAN)
    parser.add_argument(
        '--standoff', type=float, default=0.15,
        help="Metres to hold back from the hand along the approach axis.",
    )
    parser.add_argument(
        '--max-step', type=float, default=0.08,
        help="Max metres the command may move per cycle (plan mode).",
    )
    parser.add_argument(
        '--deadband', type=float, default=0.03,
        help="Ignore target motion smaller than this.",
    )
    parser.add_argument(
        '--max-y', type=float, default=0.65,
        help="Hard limit on +y. The operator stands at y ~0.75; do not raise "
             "this without knowing exactly who is standing there.",
    )
    parser.add_argument('--grasp-type', default='top',
                        choices=['top', 'side', 'diagonal', 'none'])
    parser.add_argument('--grasp-orientation', default='none',
                        choices=['grasp1', 'grasp2', 'none'],
                        help="'none' frees the wrist roll, which gives IK more "
                             "room while following.")
    parser.add_argument('--rate', type=float, default=None,
                        help="Control rate in Hz (default: 2 for plan, 50 for servo).")
    parser.add_argument('--target-timeout', type=float, default=0.75,
                        help="Treat the hand as lost after this many seconds.")
    parser.add_argument('--lost-return-sec', type=float, default=0.0,
                        help="Plan mode: after this long with no target, move to "
                             "--watch-pose. 0 disables.")
    parser.add_argument('--watch-pose', default='watch')
    parser.add_argument('--kp', type=float, default=1.5,
                        help="Servo mode: proportional gain, 1/s.")
    parser.add_argument('--max-speed', type=float, default=0.15,
                        help="Servo mode: end-effector speed cap in m/s.")
    parser.add_argument('--servo-frame', default='base_link',
                        help="Servo mode: must match planning_frame in "
                             "ur_moveit_config/config/ur_servo.yaml.")
    parser.add_argument('--assume-tool0', type=float, nargs=3,
                        metavar=('X', 'Y', 'Z'), default=[0.0, 0.30, 1.15],
                        help="Dry run only: where to pretend the arm is when "
                             "no TF is available.")
    args = parser.parse_args(argv)
    args.assume_tool0 = tuple(args.assume_tool0)
    return args


def main():
    rclpy.init()
    args = parse_args(sys.argv[1:])

    try:
        node = HandFollower(args)
    except RuntimeError as e:
        print(f"follow_hand: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        f"Table surface is at world z={TABLE_HEIGHT}. Ctrl-C to stop."
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
