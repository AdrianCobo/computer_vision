# Copyright (c) 2023 José Miguel Guerrero Hernández
#
# This file is licensed under the terms of the MIT license.
# See the LICENSE file in the root of this repository.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='computer_vision',
            namespace='computer_vision',
            executable='depth2pcl',
            output='both',
            emulate_tty=True,
            # Use topics from robot
            remappings=[
                ('/depth_input', '/computer_vision/image_depth1'),
                ('/depth_camera_info', '/c1/stereo/camera_info'),
                ('/pcl_output', '/pcl_from_depth1'),
            ],
        ),
        Node(
            package='computer_vision',
            namespace='computer_vision',
            executable='depth2pcl',
            output='both',
            emulate_tty=True,
            # Use topics from robot
            remappings=[
                ('/depth_input', '/computer_vision/image_depth2'),
                ('/depth_camera_info', '/c2/stereo/camera_info'),
                ('/pcl_output', '/pcl_from_depth2'),
            ],
        ),
        Node(
            package='computer_vision',
            namespace='computer_vision',
            executable='depth2pcl',
            output='both',
            emulate_tty=True,
            # Use topics from robot
            remappings=[
                ('/depth_input', '/computer_vision/image_depth3'),
                ('/depth_camera_info', '/c3/stereo/camera_info'),
                ('/pcl_output', '/pcl_from_depth3'),
            ],
        )
    ])
