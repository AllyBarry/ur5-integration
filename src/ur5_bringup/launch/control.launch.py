"""Start the ur5_control stack: motion primitives, gripper, perception bridge.

    # real rig, hand tracked by an AprilTag on a wristband
    ros2 launch ur5_bringup control.launch.py

    # no camera, no gripper -- just enough to run the demos
    ros2 launch ur5_bringup control.launch.py hand_source:=mock gripper_mock:=true

Assumes robot.launch.py and moveit.launch.py are already up (move_to_pose and
move_to_named_pose block on move_group at startup).

hand_source
  apriltag    TF world -> <tag_frame>. Works with the Jetson stack as it is
              today: apriltag_ros + ur_camera/extrinsics_publisher.
  detections  vision_msgs/Detection3DArray filtered by class label.
              CAMERA-STACK: needs a hand detector that does not exist yet --
              see docs/CAMERA_STACK.md.
  mock        SIM-HOOK. Synthetic hand, no camera at all.
  none        No perception; run the bridge yourself, or publish
              /perception/hand_target from your own node.

The bridge labels what it publishes after the source -- the tag frame for
apriltag ('tag_1'), the detection class for detections ('hand') -- so a consumer
can insist on one kind of target (demos/follow_hand.py --target tag).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hand_source = LaunchConfiguration('hand_source')
    tag_frame = LaunchConfiguration('tag_frame')
    gripper_mock = LaunchConfiguration('gripper_mock')
    gripper_port = LaunchConfiguration('gripper_port')
    publish_scene = LaunchConfiguration('publish_scene')

    declared_args = [
        DeclareLaunchArgument('hand_source', default_value='apriltag'),
        DeclareLaunchArgument(
            'tag_frame', default_value='tag_1',
            description="AprilTag TF frame to follow. tag_1 is the 33 mm tag, "
                        "the one small enough to wear.",
        ),
        DeclareLaunchArgument(
            'gripper_mock', default_value='false',
            description="SIM-HOOK. Answer gripper goals without a serial port.",
        ),
        DeclareLaunchArgument('gripper_port', default_value='/tmp/ttyUR'),
        DeclareLaunchArgument(
            'publish_scene', default_value='false',
            description="Push the table/ceiling/keep-out boxes into MoveIt as "
                        "CollisionObjects. Off because move_group normally "
                        "loads them from /robot_description already; enabling "
                        "it then duplicates geometry and every plan fails. See "
                        "planning_scene_publisher's docstring.",
        ),
    ]

    nodes = [
        Node(package='ur5_control', executable='move_to_pose',
             name='move_to_pose_server', output='screen', emulate_tty=True),
        Node(package='ur5_control', executable='move_to_named_pose',
             name='move_to_named_pose_server', output='screen', emulate_tty=True),
        Node(package='ur5_control', executable='gripper_server',
             name='gripper_server', output='screen', emulate_tty=True,
             parameters=[{
                 'use_mock': gripper_mock,
                 'serial_port': gripper_port,
             }]),
        Node(package='ur5_control', executable='planning_scene_publisher',
             name='planning_scene_publisher', output='screen', emulate_tty=True,
             condition=IfCondition(publish_scene)),
    ]

    # Perception: exactly one of these, or neither.
    perception = [
        Node(package='ur5_control', executable='hand_target_bridge',
             name='hand_target_bridge', output='screen', emulate_tty=True,
             parameters=[{'source': 'apriltag', 'tag_frame': tag_frame}],
             condition=IfCondition(EqualsSubstitution(hand_source, 'apriltag'))),
        Node(package='ur5_control', executable='hand_target_bridge',
             name='hand_target_bridge', output='screen', emulate_tty=True,
             parameters=[{'source': 'detections'}],
             condition=IfCondition(EqualsSubstitution(hand_source, 'detections'))),
        Node(package='ur5_control', executable='mock_hand_publisher',
             name='mock_hand_publisher', output='screen', emulate_tty=True,
             condition=IfCondition(EqualsSubstitution(hand_source, 'mock'))),
    ]

    return LaunchDescription(declared_args + nodes + perception)
