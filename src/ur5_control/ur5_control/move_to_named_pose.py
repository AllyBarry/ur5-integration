#!/usr/bin/env python3
"""
MoveToNamedPose: collision-aware joint-space move to a configured pose.

Poses come from config/named_poses.yaml, so adding a staging pose is a config
edit rather than a code edit. "ready" is the reset/recovery pose; "watch" is
the one the follow-hand demo parks at while it waits for a target.

Joint-space via OMPL RRTConnect: no Cartesian or orientation constraints, so it
still works when the arm has wound itself somewhere constrained planning can't
solve from. This is the "get me out of trouble" action.
"""
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

from ur5_interfaces.action import MoveToNamedPose

DEFAULT_POSE_NAME = "ready"

# Fallback if named_poses.yaml is missing or unreadable, so the recovery action
# still works on a half-installed workspace.
FALLBACK_POSES = {
    "ready": {
        "shoulder_pan_joint":  0.0,
        "shoulder_lift_joint": -1.57,
        "elbow_joint":         -1.57,
        "wrist_1_joint":       -1.57,
        "wrist_2_joint":        1.57,
        "wrist_3_joint":        0.0,
    },
}

JOINT_TOLERANCE = 0.01     # radians, ~0.6 deg
PLANNING_GROUP = "ur_manipulator"
PIPELINE_ID = "ompl"
PLANNER_ID = "RRTConnectkConfigDefault"   # much faster than RRT* for joint goals
PLANNING_TIME = 10.0
PLANNING_ATTEMPTS = 5
VELOCITY_SCALING = 0.4
ACCEL_SCALING = 0.1


def load_named_poses(logger=None):
    """Load config/named_poses.yaml from the installed share directory."""
    try:
        path = os.path.join(
            get_package_share_directory('ur5_control'), 'config', 'named_poses.yaml'
        )
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        poses = data.get('named_poses', {})
        if poses:
            return poses
        if logger:
            logger.warn(f"{path} has no 'named_poses' key; using fallback.")
    except Exception as e:  # noqa: BLE001 - config problems must not be fatal
        if logger:
            logger.warn(f"Could not load named_poses.yaml ({e}); using fallback.")
    return dict(FALLBACK_POSES)


class MoveToNamedPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_named_pose_server')

        self._poses = load_named_poses(self.get_logger())
        self.get_logger().info(f"Named poses: {sorted(self._poses)}")

        self._cb_group = ReentrantCallbackGroup()

        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self._cb_group,
        )
        self.get_logger().info("Waiting for move_group action server...")
        self._move_group_client.wait_for_server()
        self.get_logger().info("move_group is up.")

        self._action_server = ActionServer(
            self, MoveToNamedPose, 'move_to_named_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info("move_to_named_pose action server ready.")

    def goal_callback(self, goal_request):
        name = goal_request.pose_name or DEFAULT_POSE_NAME
        if name not in self._poses:
            self.get_logger().warn(
                f"Rejecting: unknown pose '{name}'. Known: {sorted(self._poses)}"
            )
            return GoalResponse.REJECT
        self.get_logger().info(f"MoveToNamedPose goal: '{name}'")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _build_move_group_goal(self, joint_positions):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = PLANNING_GROUP
        goal_msg.request.pipeline_id = PIPELINE_ID
        goal_msg.request.planner_id = PLANNER_ID
        goal_msg.request.allowed_planning_time = PLANNING_TIME
        goal_msg.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal_msg.request.max_velocity_scaling_factor = VELOCITY_SCALING
        goal_msg.request.max_acceleration_scaling_factor = ACCEL_SCALING

        constraints = Constraints()
        for joint_name, position in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = JOINT_TOLERANCE
            jc.tolerance_below = JOINT_TOLERANCE
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        name = goal_handle.request.pose_name or DEFAULT_POSE_NAME

        fb = MoveToNamedPose.Feedback()
        fb.status = f"Planning joint-space move to '{name}'..."
        goal_handle.publish_feedback(fb)

        mg_goal = self._build_move_group_goal(self._poses[name])
        mg_handle = await self._move_group_client.send_goal_async(mg_goal)

        result = MoveToNamedPose.Result()
        if not mg_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = f"move_group rejected the '{name}' goal."
            self.get_logger().error(result.message)
            return result

        fb.status = "move_group accepted. Executing trajectory..."
        goal_handle.publish_feedback(fb)

        mg_result = await mg_handle.get_result_async()
        error_code = mg_result.result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            goal_handle.succeed()
            result.success = True
            result.message = f"Reached '{name}'."
            self.get_logger().info(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"MoveToNamedPose('{name}') failed with error code {error_code}."
            )
            self.get_logger().error(result.message)
        return result


def main():
    rclpy.init()
    node = MoveToNamedPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
