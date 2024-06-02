from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the 'sim' argument
    declare_sim_arg = DeclareLaunchArgument(
        "sim", default_value="False", description="Simulation mode flag"
    )
    sim = LaunchConfiguration("sim")

    # Get real robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pupper_v3_description"),
                    "description",
                    "pupper_v3.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get simulation robot description
    sim_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pupper_v3_description"),
                    "description",
                    "pupper_v3_mujoco.urdf.xacro",
                ]
            ),
        ]
    )
    sim_robot_description = {"robot_description": sim_robot_description_content}

    # Declare and get config file argument
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("neural_controller"), "launch", "config.yaml"]
        ),
        description="Path to the config file for the controller",
    )
    config_file = LaunchConfiguration("config_file")
    robot_controller_parameters = ParameterFile(config_file, allow_substs=True)

    # Launch nodes for joystick teleop
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[robot_controller_parameters],
        output="both",
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[robot_controller_parameters],
        output="both",
    )

    # Launch different managers depending on sim argument
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controller_parameters],
        output="both",
        condition=UnlessCondition(sim),
    )

    control_node_sim = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[sim_robot_description, robot_controller_parameters],
        output="both",
        condition=IfCondition(sim),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Launch a joint state broadcaster so we can view and log joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    nodes = [
        config_file_arg,
        declare_sim_arg,
        control_node,
        control_node_sim,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        joy_node,
        teleop_twist_joy_node,
    ]

    return LaunchDescription(nodes)
