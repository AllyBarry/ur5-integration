#!/usr/bin/env python3
"""
handover: give a held object to a person's hand.

    python3 demos/handover.py --dry-run
    python3 demos/handover.py --pick-x -0.30 --pick-y 0.25 --pick-z 0.85

Sequence
--------
  1. move to 'ready'
  2. open gripper, descend to the pick pose, close gripper
  3. lift, then wait for a hand target to appear AND hold still
  4. move to a standoff from the hand and open the gripper
  5. retreat to 'watch'

Unlike follow_hand this is a state machine with blocking steps: nothing here
needs to react while the arm is moving, and a handover that runs its stages in
a fixed order is much easier to stop and reason about.

Where this demo is honest about what it can't tell
--------------------------------------------------
Step 2 decides the grasp worked from the gripper's own OBJ status -- it knows
the fingers stopped early on something, not that the something was the object.
Step 4 opens the gripper when the hand has held still for --hold-sec, which is
a proxy for "the person is ready to take it". Neither is perception.
See docs/CAMERA_STACK.md for what would replace both.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from ur5_control.geometry import (
    clamp_to_workspace, distance, standoff_pose, tip_above_table_to_world_z,
)

from demo_common import (
    GRIPPER_CLOSED, GRIPPER_OPEN, HandTargetMonitor, MotionClients,
    add_common_args, format_xyz, send_and_wait,
)

from ur5_interfaces.action import MoveToNamedPose


class Handover(Node):
    def __init__(self, args):
        super().__init__('handover')
        self.args = args
        self._monitor = HandTargetMonitor(self, timeout_sec=args.target_timeout)
        self._motion = MotionClients(self)

    # ---- helpers ----

    def _named_pose(self, name):
        self.get_logger().info(f"--- Moving to '{name}' ---")
        if self.args.dry_run:
            return True
        goal = MoveToNamedPose.Goal()
        goal.pose_name = name
        ok, _ = send_and_wait(self, self._motion.move_to_named_pose, goal)
        return ok

    def _move(self, position, label, grasp_type=None, force_linear=False):
        self.get_logger().info(f"--- {label}: {format_xyz(position)} ---")
        if self.args.dry_run:
            return True
        goal = MotionClients.pose_goal(
            *position,
            grasp_type=grasp_type or self.args.grasp_type,
            grasp_orientation=self.args.grasp_orientation,
            force_linear=force_linear,
            velocity_scaling=self.args.velocity_scaling,
        )
        ok, _ = send_and_wait(self, self._motion.move_to_pose, goal)
        return ok

    def _gripper(self, position, label, force=None):
        self.get_logger().info(f"--- {label} ---")
        if self.args.dry_run:
            return True, None
        goal = MotionClients.gripper_goal(
            position, force=force if force is not None else self.args.grasp_force
        )
        ok, result = send_and_wait(self, self._motion.gripper, goal, timeout_sec=15.0)
        return ok, result

    def _wait_for_steady_hand(self):
        """Return a hand position once it has held still, or None on timeout.

        "Still" means every sample within --steady-tolerance of the first one
        for --hold-sec. A hand that is drifting past on its way somewhere else
        never satisfies this, which is the point: the arm should not offer the
        object to someone walking by.
        """
        self.get_logger().info(
            f"Waiting for a hand to hold still for {self.args.hold_sec:.1f}s "
            f"(within {self.args.steady_tolerance * 1000:.0f}mm)..."
        )
        anchor = None
        steady_since = None
        deadline = time.monotonic() + self.args.handover_timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            position = self._monitor.position()

            if position is None:
                anchor, steady_since = None, None
                continue

            if anchor is None or distance(position, anchor) > self.args.steady_tolerance:
                anchor = position
                steady_since = time.monotonic()
                continue

            if time.monotonic() - steady_since >= self.args.hold_sec:
                self.get_logger().info(f"Hand steady at {format_xyz(position)}.")
                return position

        self.get_logger().error(
            f"No steady hand within {self.args.handover_timeout:.0f}s."
        )
        return None

    # ---- sequence ----

    def run(self):
        if not self.args.dry_run and not self._motion.wait_for_servers():
            return 1

        pick = (self.args.pick_x, self.args.pick_y, self.args.pick_z)
        approach = (pick[0], pick[1], pick[2] + self.args.approach_height)

        if not self._named_pose('ready'):
            return 1

        ok, _ = self._gripper(GRIPPER_OPEN, "Opening gripper")
        if not ok:
            return 1

        if not self._move(approach, "Approach above pick"):
            return 1
        if not self._move(pick, "Descend to pick", force_linear=True):
            return 1

        ok, result = self._gripper(GRIPPER_CLOSED, "Closing on object")
        if not ok:
            return 1
        if result is not None and not result.object_detected:
            # The fingers closed all the way: nothing between them.
            self.get_logger().error(
                "Gripper closed without contact -- nothing was picked up. Stopping."
            )
            self._named_pose('ready')
            return 1

        if not self._move(approach, "Lift", force_linear=True):
            return 1

        hand = self._wait_for_steady_hand()
        if hand is None:
            self.get_logger().warn("Aborting handover; keeping hold of the object.")
            self._named_pose('ready')
            return 1

        offer = standoff_pose(*hand, self.args.grasp_type, self.args.standoff)
        offer_y = min(offer[1], self.args.max_y)
        offer = clamp_to_workspace(offer[0], offer_y, offer[2])[:3]

        if not self._move(offer, "Offering object to hand"):
            self._named_pose('ready')
            return 1

        # A beat before letting go, so the person can close their hand around
        # it rather than catching it.
        time.sleep(self.args.release_delay)

        ok, _ = self._gripper(GRIPPER_OPEN, "Releasing")
        if not ok:
            return 1

        self.get_logger().info("Handover complete. Retreating.")
        self._named_pose(self.args.retreat_pose)
        return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Pick an object and hand it to a tracked hand.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    add_common_args(parser)
    # Defaults put the pick on the left of the table at a height a jar or
    # bottle would sit at. Override them; they are a starting point, not a
    # calibrated object location.
    parser.add_argument('--pick-x', type=float, default=-0.30)
    parser.add_argument('--pick-y', type=float, default=0.25)
    parser.add_argument('--pick-z', type=float,
                        default=round(tip_above_table_to_world_z(0.33), 3),
                        help="World z of the grasp pose (tool0).")
    parser.add_argument('--approach-height', type=float, default=0.10)
    parser.add_argument('--grasp-type', default='top',
                        choices=['top', 'side', 'diagonal', 'none'])
    parser.add_argument('--grasp-orientation', default='grasp1',
                        choices=['grasp1', 'grasp2', 'none'])
    parser.add_argument('--grasp-force', type=int, default=100,
                        help="Gripper force, 0-255.")
    parser.add_argument('--standoff', type=float, default=0.12,
                        help="How far from the hand to offer the object.")
    parser.add_argument('--max-y', type=float, default=0.65,
                        help="Hard limit on +y; see follow_hand.")
    parser.add_argument('--hold-sec', type=float, default=1.5)
    parser.add_argument('--steady-tolerance', type=float, default=0.04)
    parser.add_argument('--handover-timeout', type=float, default=60.0)
    parser.add_argument('--release-delay', type=float, default=1.0)
    parser.add_argument('--target-timeout', type=float, default=0.75)
    parser.add_argument('--retreat-pose', default='watch')
    return parser.parse_args(argv)


def main():
    rclpy.init()
    args = parse_args(sys.argv[1:])
    node = Handover(args)
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
