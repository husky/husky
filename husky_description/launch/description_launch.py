#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def get_parsed_urdf_string():
    shared_dir = get_package_share_directory('husky_description')
    path = os.path.join(shared_dir, 'urdf', 'husky.urdf.xacro')
    return xacro.process(path)

def generate_launch_description():
    urdf_description_string = get_parsed_urdf_string()
    robot_description = { 'robot_description': urdf_description_string }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
    ])



def generate_launch_description():

    robot_description_path = os.path.join(
         get_package_share_directory('husky_description'),
        'urdf',
        'husky.urdf.xacro',
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    return LaunchDescription(
        [
        ]
    )
