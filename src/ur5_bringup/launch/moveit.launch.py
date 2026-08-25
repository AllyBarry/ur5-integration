"""MoveIt for the UR5.

    ros2 launch ur5_bringup moveit.launch.py
    ros2 launch ur5_bringup moveit.launch.py launch_servo:=true   # for follow_hand --mode servo

A thin wrapper over ur_moveit_config so the whole stack has one launch style,
plus the two facts that bite everyone on this rig:

1. ROBOT MODEL. ur_moveit_config builds no URDF of its own: it runs
   `wait_for_robot_description` and move_group takes the model off the
   /robot_description topic. So MoveIt plans against whatever
   robot_state_publisher published -- launched through ur5_bringup that is
   ur5_hand_e.urdf.xacro, table and keep-out slabs included. Start
   robot.launch.py FIRST, or move_group sits waiting for a description.

   The SRDF is still stock `ur_moveit_config/srdf/ur.srdf.xacro`, which names
   only the six arm links. Nothing disables collisions for the Hand-E links,
   the table or `grasped_object`, so any pair of those that overlaps without
   being directly jointed reads as a permanent self-collision -- which is why
   `enable_grasped_object` defaults to false. Generating a MoveIt config from
   this workspace's URDF is the real fix; see docs/SIMULATION.md.

2. SERVO. launch_servo:=true starts moveit_servo, which follow_hand's servo
   mode streams twists to. Servo publishes to /forward_position_controller,
   which ur_control leaves INACTIVE, so you must also switch controllers:

     ros2 control switch_controllers \\
       --deactivate scaled_joint_trajectory_controller \\
       --activate forward_position_controller

   and switch back before using the planning-based demos again. Servo config
   (velocity limits, singularity thresholds) is ur_moveit_config's
   config/ur_servo.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "launch_servo", default_value="false",
            description="Start moveit_servo for velocity-streaming control. "
                        "Requires a controller switch -- see this file's docstring.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="SIM-HOOK. Set true when driving the stack from a "
                        "simulator that publishes /clock.",
        ),
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": LaunchConfiguration("launch_servo"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(declared_args + [moveit])
