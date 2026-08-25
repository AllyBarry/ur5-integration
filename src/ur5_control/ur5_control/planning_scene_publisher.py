#!/usr/bin/env python3
"""
planning_scene_publisher: push the room in as CollisionObjects.

OFF BY DEFAULT, and you probably don't want it. Read this before enabling it.

On this machine's ROS Jazzy, `ur_moveit_config/launch/ur_moveit.launch.py` does
NOT build its own URDF -- it starts `wait_for_robot_description` and move_group
picks the model up from the /robot_description topic, i.e. from whatever
robot_state_publisher published. Launched through ur5_bringup, that is
ur5_hand_e.urdf.xacro, table and keep-out slabs included. MoveIt already knows
about them, and adding them again here would leave the robot permanently in
collision with a duplicate of its own table.

Enable it (publish_scene:=true) only when move_group is running against a
description that does NOT contain the table -- an older ur_moveit_config that
loads its own URDF, a hand-written moveit config, or move_group started before
robot_state_publisher. Check which you have:

    ros2 param get /robot_state_publisher robot_description | grep -c workspace_obstacle
    # 0 -> the boxes are missing, this node is useful
    # >0 -> they're already there, leave this node off

It is also a reasonable starting point for pushing in *dynamic* obstacles
(a detected person, a tray that moved), which never belong in the URDF.

Parameters
----------
world_frame           (str,   default "world")
republish_period_sec  (float, default 0.0)
    0 -> apply once. Set >0 if something clears the scene (RViz's scene reset
    does) and you want it restored.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from ur5_control.geometry import SCENE_BOXES

SERVICE_WAIT_SEC = 5.0


class PlanningScenePublisher(Node):
    def __init__(self):
        super().__init__('planning_scene_publisher')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('republish_period_sec', 0.0)

        self._world_frame = self.get_parameter('world_frame').value
        period = float(self.get_parameter('republish_period_sec').value)

        self.get_logger().warn(
            "Adding table/ceiling/keep-out boxes to the planning scene. If "
            "move_group already loaded ur5_bringup's URDF (the normal case), "
            "this DUPLICATES geometry the robot is attached to and every plan "
            "will fail. See this node's docstring."
        )

        self._client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info("Waiting for /apply_planning_scene...")
        while not self._client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            self.get_logger().warn(
                "/apply_planning_scene not available yet -- is move_group running?"
            )

        self._apply()
        if period > 0.0:
            self.create_timer(period, self._apply)

    def _build_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        for name, size, centre in SCENE_BOXES:
            obj = CollisionObject()
            obj.header.frame_id = self._world_frame
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.id = name
            obj.operation = CollisionObject.ADD

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [float(s) for s in size]

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = [
                float(c) for c in centre
            ]
            pose.orientation.w = 1.0

            obj.primitives.append(box)
            obj.primitive_poses.append(pose)
            scene.world.collision_objects.append(obj)

        return scene

    def _apply(self):
        request = ApplyPlanningScene.Request()
        request.scene = self._build_scene()
        future = self._client.call_async(request)
        future.add_done_callback(self._on_applied)

    def _on_applied(self, future):
        try:
            response = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"apply_planning_scene call failed: {e}")
            return
        names = ", ".join(name for name, _, _ in SCENE_BOXES)
        if response.success:
            self.get_logger().info(f"Planning scene updated: {names}")
        else:
            self.get_logger().error(
                f"move_group rejected the planning scene diff ({names})."
            )


def main(args=None):
    rclpy.init(args=args)
    node = PlanningScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
