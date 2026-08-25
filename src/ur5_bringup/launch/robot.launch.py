"""Bring up the UR5 driver with THIS workspace's description.

    ros2 launch ur5_bringup robot.launch.py                      # real robot
    ros2 launch ur5_bringup robot.launch.py use_mock_hardware:=true

Wraps ur_robot_driver/ur_control.launch.py, redirecting its
`description_launchfile` to ur5_bringup/launch/rsp.launch.py so the arm comes up
with the table, the ceiling, the workspace obstacles and the Hand-E attached --
not the bare UR5 that ur_robot_driver would otherwise publish.

Deliberately NOT passed: use_tool_communication. On this rig the gripper is
reached through gripper_bridge.py on the controller plus socat, and the driver's
tool-communication path fights it. See docs/BRINGUP.md.

Next steps after this launch is up:
  1. Press play on external.urp on the pendant (skip if use_mock_hardware:=true).
     Look for "Robot connected to reverse interface".
  2. ros2 launch ur5_bringup moveit.launch.py
  3. ros2 launch ur5_bringup control.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Default rig IP; override with robot_ip:=...
DEFAULT_ROBOT_IP = "10.10.187.168"

# Arm calibration produced by ur_calibration. Without it the kinematic model is
# the nominal UR5 rather than this specific arm. Defaulted only if the file is
# actually present, so a fresh machine still launches.
CALIBRATION_PATH = os.path.expanduser("~/my_robot_calibration.yaml")


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    default_kinematics = (
        CALIBRATION_PATH if os.path.isfile(CALIBRATION_PATH)
        else PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type,
            "default_kinematics.yaml",
        ])
    )

    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument(
            "robot_ip", default_value=DEFAULT_ROBOT_IP,
            description="Ignored when use_mock_hardware:=true.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="SIM-HOOK. Run the whole stack against mocked joints: "
                        "no robot, no pendant, no network. Motion is instant "
                        "and perfect, so it validates plumbing and geometry, "
                        "not dynamics or reachability at speed.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "kinematics_params_file", default_value=default_kinematics,
        ),
        DeclareLaunchArgument("enable_grasped_object", default_value="false"),
        DeclareLaunchArgument("gripper_use_fake_hardware", default_value="true"),
    ]

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "description_launchfile": PathJoinSubstitution([
                FindPackageShare("ur5_bringup"), "launch", "rsp.launch.py",
            ]),
        }.items(),
    )

    return LaunchDescription(declared_args + [ur_control])
