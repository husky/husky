from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    world_file = PathJoinSubstitution(
        [FindPackageShare("husky_gazebo"),
        "worlds",
        "clearpath_playpen.world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("husky_gazebo"),
        "launch",
        "gazebo.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim)

    return ld
