"""robot_state_publisher for the UR5 + Hand-E + table description.

Why this exists instead of using ur_robot_driver's ur_rsp.launch.py directly:
that launch file builds its xacro command from a FIXED list of arguments, so
any argument our description adds (enable_grasped_object,
gripper_use_fake_hardware) is unreachable through it. ur_control.launch.py
exposes a `description_launchfile` argument for exactly this case, and
robot.launch.py points it here.

Contract with ur_control.launch.py: it includes this file passing only
`robot_ip` and `ur_type`. Everything else arrives by launch-configuration
inheritance from the parent scope, which is why each argument below is
declared with a default rather than assumed.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    enable_grasped_object = LaunchConfiguration("enable_grasped_object")
    gripper_use_fake_hardware = LaunchConfiguration("gripper_use_fake_hardware")

    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur5_bringup"), "urdf", "ur5_hand_e.urdf.xacro",
            ]),
            description="Absolute path is used here deliberately: xacro resolves "
                        "a relative description_file against the CWD, which is "
                        "the classic 'works in my terminal' failure on this rig.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_description"), "config", ur_type,
                "default_kinematics.yaml",
            ]),
            description="Robot-specific calibration. On this rig, pass "
                        "$HOME/my_robot_calibration.yaml -- without it the arm "
                        "is accurate to a few mm at best.",
        ),
        DeclareLaunchArgument(
            "enable_grasped_object",
            default_value="false",
            description="Attach a collision cube to the gripper, for planning "
                        "while carrying something. Off by default: it makes "
                        "close-to-person poses fail to plan when empty-handed.",
        ),
        DeclareLaunchArgument(
            "gripper_use_fake_hardware",
            default_value="true",
            description="Keep true while ur5_control/gripper_server owns "
                        "/tmp/ttyUR. See the note in the xacro.",
        ),
    ]

    joint_limit_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml",
    ])
    physical_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type,
        "physical_parameters.yaml",
    ])
    visual_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type,
        "visual_parameters.yaml",
    ])
    script_filename = PathJoinSubstitution([
        FindPackageShare("ur_client_library"), "resources", "external_control.urscript",
    ])
    input_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt",
    ])
    output_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt",
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        description_file, " ",
        "name:=", ur_type, " ",
        "tf_prefix:=", tf_prefix, " ",
        "robot_ip:=", robot_ip, " ",
        "joint_limit_params:=", joint_limit_params, " ",
        "kinematics_params:=", kinematics_params_file, " ",
        "physical_params:=", physical_params, " ",
        "visual_params:=", visual_params, " ",
        "script_filename:=", script_filename, " ",
        "input_recipe_filename:=", input_recipe_filename, " ",
        "output_recipe_filename:=", output_recipe_filename, " ",
        "use_mock_hardware:=", use_mock_hardware, " ",
        "mock_sensor_commands:=", mock_sensor_commands, " ",
        "enable_grasped_object:=", enable_grasped_object, " ",
        "gripper_use_fake_hardware:=", gripper_use_fake_hardware,
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                robot_description_content, value_type=str
            ),
        }],
    )

    return LaunchDescription(declared_args + [robot_state_publisher])
