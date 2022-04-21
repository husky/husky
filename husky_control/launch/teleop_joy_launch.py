from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='logitech')


    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare('husky_control'), 'config', ('teleop_' + joy_type.perform(lc) + '.yaml')]
    )

    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy]
    )

    node_teleop_twist_joy = Node(
        namespace='joy_teleop',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[filepath_config_joy]
    )


    ld = LaunchDescription()
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    return ld

