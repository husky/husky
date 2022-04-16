from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_husky_ekf = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
        "config",
        "localization.yaml"],
    )

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[config_husky_ekf],
        )
    ld = LaunchDescription()
    ld.add_action(node_ekf)

    return ld