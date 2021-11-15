from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
        "config",
        "control.yaml"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_husky_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["husky_velocity_controller"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_husky_velocity_controller)

    return ld