import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_config', default_value='teleop_logitech'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('husky_control'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.yaml')]),

        launch_ros.actions.Node(
            namespace='joy_teleop',
            package='joy', executable='joy_node', name='joy_node',
            parameters=[config_filepath]),
        launch_ros.actions.Node(
            namespace='joy_teleop',
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath]),
        launch_ros.actions.Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', '/husky_velocity_controller/cmd_vel_unstamped')},
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare("husky_control"), "config", "twist_mux.yaml"]
                )]
        ),
            
    ])

