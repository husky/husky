from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_husky_ekf = PathJoinSubstitution(
        [FindPackageShare('husky_control'),
        'config',
        'localization.yaml'],
    )

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[config_husky_ekf],
        )

    config_imu_filter = PathJoinSubstitution(
        [FindPackageShare('husky_control'),
        'config',
        'imu_filter.yaml'],
    )
    node_imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[config_imu_filter]
    )
    
    ld = LaunchDescription()
    ld.add_action(node_ekf)
    ld.add_action(node_imu_filter)

    return ld