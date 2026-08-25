#!/usr/bin/env python3
"""
MoveToPose: the one Cartesian motion primitive in this stack.

Takes a world-frame tool0 target plus a grasp type/orientation and drives it
through MoveIt's move_group action. Everything else in the stack (demos,
sequencers) is built out of calls to this.

Planner selection
-----------------
  constrained,   near : Pilz LIN   -- straight Cartesian line, predictable
  constrained,   far  : OMPL RRT*  -- collision-aware, wanders a bit
  unconstrained, near : Pilz PTP   -- joint-space hop
  unconstrained, far  : OMPL RRT*
"near" means the Cartesian distance from the current tool0 pose is under
DIRECT_DISTANCE_THRESHOLD. force_linear=True on the goal overrides the distance
test and demands LIN (falling back to PTP if there is no orientation to hold).

Follow-style demos want force_linear=True on every goal: the hops are short,
and a LIN plan between two nearby poses is the one thing you can predict the
shape of while a person is standing next to the arm.

Conventions
-----------
  - goal z is world z, targeting tool0 (see ur5_interfaces/MoveToPose)
  - workspace bounds and grasp quaternions come from ur5_control.geometry
  - velocity_scaling is per-goal; 0 means "use the server default"
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, MoveItErrorCodes,
    PositionConstraint, BoundingVolume, OrientationConstraint,
    WorkspaceParameters,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from ur5_interfaces.action import MoveToPose

from ur5_control.geometry import (
    GRASP_NONE, GRASP_QUATERNIONS, ORIENT_1, ORIENT_NONE,
    VALID_GRASP_TYPES, VALID_ORIENTATIONS,
    X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX,
    in_workspace,
)

# ---- Planning ----
PLANNING_TIME = 30.0
PLANNING_ATTEMPTS = 25
DEFAULT_VELOCITY_SCALING = 0.6
DEFAULT_ACCEL_SCALING = 0.1

# Below this Cartesian distance a motion counts as "direct".
DIRECT_DISTANCE_THRESHOLD = 0.20  # metres

OMPL_PIPELINE = "ompl"
OMPL_PLANNER = "RRTstarkConfigDefault"
PILZ_PIPELINE = "pilz_industrial_motion_planner"
PILZ_LIN_PLANNER = "LIN"
PILZ_PTP_PLANNER = "PTP"

PLANNING_GROUP = "ur_manipulator"
WORLD_FRAME = "world"
EEF_LINK = "tool0"

# Goal position tolerance (radius of the constraint sphere).
TOL_DIRECT = 0.001
TOL_PLANNED = 0.02


class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')

        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self._cb_group,
        )
        self.get_logger().info("Waiting for move_group action server...")
        self._move_group_client.wait_for_server()
        self.get_logger().info("move_group is up.")

        self._action_server = ActionServer(
            self, MoveToPose, 'move_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info("move_to_pose action server ready.")

    # ---- Goal validation ----

    def goal_callback(self, goal_request):
        x, y, z = goal_request.x, goal_request.y, goal_request.z

        if not in_workspace(x, y, z):
            self.get_logger().warn(
                f"Rejecting goal ({x:.3f}, {y:.3f}, {z:.3f}): outside workspace "
                f"x[{X_MIN}, {X_MAX}] y[{Y_MIN}, {Y_MAX}] z[{Z_MIN:.2f}, {Z_MAX:.2f}]"
            )
            return GoalResponse.REJECT
        if goal_request.grasp_type not in VALID_GRASP_TYPES:
            self.get_logger().warn(
                f"Rejecting goal: grasp_type='{goal_request.grasp_type}' "
                f"not in {sorted(VALID_GRASP_TYPES)}"
            )
            return GoalResponse.REJECT
        if goal_request.grasp_orientation not in VALID_ORIENTATIONS:
            self.get_logger().warn(
                f"Rejecting goal: grasp_orientation='{goal_request.grasp_orientation}' "
                f"not in {sorted(VALID_ORIENTATIONS)}"
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepting goal: ({x:.3f}, {y:.3f}, {z:.3f}) "
            f"{goal_request.grasp_type}/{goal_request.grasp_orientation}"
            f"{' force_linear' if goal_request.force_linear else ''}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    # ---- Helpers ----

    def _current_tool0_xyz(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                WORLD_FRAME, EEF_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup {WORLD_FRAME}->{EEF_LINK} failed: {e}")
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _is_direct_move(self, tx, ty, tz):
        """True if the target is close enough to plan as a direct move.

        Unknown current pose -> False, i.e. fall back to the collision-aware
        planner. Guessing "it's probably close" here would be a guess about
        where a robot arm is.
        """
        current = self._current_tool0_xyz()
        if current is None:
            self.get_logger().info(
                "No current tool0 pose; falling back to constrained planning."
            )
            return False

        cx, cy, cz = current
        dist = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2 + (tz - cz) ** 2)
        direct = dist < DIRECT_DISTANCE_THRESHOLD
        self.get_logger().info(
            f"current=({cx:.3f},{cy:.3f},{cz:.3f}) target=({tx:.3f},{ty:.3f},{tz:.3f}) "
            f"dist={dist:.3f} -> {'DIRECT' if direct else 'PLANNED (OMPL)'}"
        )
        return direct

    def _select_planner(self, req, unconstrained):
        """Return (pipeline_id, planner_id, is_direct, description)."""
        if req.force_linear:
            if unconstrained:
                # Pilz LIN needs an orientation to hold the line for. With
                # grasp_type='none' there isn't one, so PTP is the honest
                # interpretation of "just get there directly".
                self.get_logger().warn(
                    "force_linear with grasp_type='none' is ambiguous "
                    "(LIN needs an orientation). Using PTP."
                )
                return PILZ_PIPELINE, PILZ_PTP_PLANNER, True, "force_linear PTP"
            return (PILZ_PIPELINE, PILZ_LIN_PLANNER, True,
                    f"force_linear LIN ({req.grasp_orientation})")

        direct = self._is_direct_move(req.x, req.y, req.z)
        if direct and not unconstrained:
            return (PILZ_PIPELINE, PILZ_LIN_PLANNER, True,
                    f"direct LIN ({req.grasp_orientation})")
        if direct and unconstrained:
            return PILZ_PIPELINE, PILZ_PTP_PLANNER, True, "direct PTP"
        if unconstrained:
            return OMPL_PIPELINE, OMPL_PLANNER, False, "unconstrained (OMPL)"
        return (OMPL_PIPELINE, OMPL_PLANNER, False,
                f"{req.grasp_type}/{req.grasp_orientation} (OMPL)")

    # ---- Execution ----

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        req = goal_handle.request

        unconstrained = (req.grasp_type == GRASP_NONE)
        free_roll = (not unconstrained and req.grasp_orientation == ORIENT_NONE)

        if unconstrained:
            target_quat = None
        else:
            key = ORIENT_1 if free_roll else req.grasp_orientation
            target_quat = GRASP_QUATERNIONS[(req.grasp_type, key)]

        pipeline_id, planner_id, is_direct, mode = self._select_planner(
            req, unconstrained
        )

        fb = MoveToPose.Feedback()
        fb.status = f"Planning {mode} to ({req.x:.3f}, {req.y:.3f}, {req.z:.3f})"
        goal_handle.publish_feedback(fb)

        mg_goal = self._build_move_group_goal(
            req.x, req.y, req.z,
            target_quat=target_quat,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            is_direct=is_direct,
            unconstrained=unconstrained,
            free_roll=free_roll,
            velocity_scaling=(
                req.velocity_scaling if req.velocity_scaling > 0.0
                else DEFAULT_VELOCITY_SCALING
            ),
        )

        mg_handle = await self._move_group_client.send_goal_async(mg_goal)

        result = MoveToPose.Result()
        if not mg_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = "move_group rejected the goal."
            self.get_logger().error(result.message)
            return result

        fb.status = "move_group accepted. Executing trajectory..."
        goal_handle.publish_feedback(fb)

        mg_result = await mg_handle.get_result_async()
        error_code = mg_result.result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            goal_handle.succeed()
            result.success = True
            result.message = f"Reached pose ({mode})."
            self.get_logger().info(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"MoveIt failed with error code {error_code} "
                f"(planner: {pipeline_id}/{planner_id}, mode: {mode})."
            )
            self.get_logger().error(result.message)
        return result

    # ---- Goal construction ----

    def _build_move_group_goal(
        self, tx, ty, tz, target_quat, pipeline_id, planner_id,
        is_direct, unconstrained, free_roll, velocity_scaling,
    ):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = PLANNING_GROUP
        goal_msg.request.pipeline_id = pipeline_id
        goal_msg.request.planner_id = planner_id
        goal_msg.request.allowed_planning_time = PLANNING_TIME
        goal_msg.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal_msg.request.max_velocity_scaling_factor = float(velocity_scaling)
        goal_msg.request.max_acceleration_scaling_factor = DEFAULT_ACCEL_SCALING

        # Sampling hint for OMPL, not a collision boundary.
        ws = WorkspaceParameters()
        ws.header.frame_id = WORLD_FRAME
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -0.8, -0.3, 0.7
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 0.8, 1.0, 1.8
        goal_msg.request.workspace_parameters = ws

        # --- Position constraint on tool0 (always) ---
        pos_con = PositionConstraint()
        pos_con.header.frame_id = WORLD_FRAME
        pos_con.link_name = EEF_LINK

        target_p = Pose()
        target_p.position.x = tx
        target_p.position.y = ty
        target_p.position.z = tz
        if target_quat is not None:
            target_p.orientation = target_quat
        else:
            target_p.orientation.w = 1.0  # identity; ignored on this path

        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [TOL_DIRECT if is_direct else TOL_PLANNED]

        bv = BoundingVolume()
        bv.primitives.append(s)
        bv.primitive_poses.append(target_p)
        pos_con.constraint_region = bv
        pos_con.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)

        # --- Orientation constraint on tool0 (unless unconstrained) ---
        # absolute_z_axis_tolerance is about the link's local Z, which is the
        # approach axis by construction -- so opening it to PI is exactly
        # "let the wrist roll, keep pointing the same way".
        if not unconstrained:
            oc = OrientationConstraint()
            oc.header.frame_id = WORLD_FRAME
            oc.link_name = EEF_LINK
            oc.orientation = target_quat
            oc.absolute_x_axis_tolerance = 0.05
            oc.absolute_y_axis_tolerance = 0.05
            oc.absolute_z_axis_tolerance = math.pi if free_roll else 0.05
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg


def main():
    rclpy.init()
    node = MoveToPoseServer()
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
