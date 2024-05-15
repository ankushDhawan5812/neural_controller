
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
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
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("neural_controller"),
            "launch",
            "config.yaml",
        ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[robot_controllers],
        output="both",
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[robot_controllers],
        output="both",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["neural_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )

    nodes = [
        control_node,
        robot_controller_spawner,
        joy_node,
        teleop_twist_joy_node
    ]

    return LaunchDescription(nodes)
