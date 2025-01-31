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
                ('/depth_input', '/c1/stereo/depth'),
                ('/depth_camera_info', '/c1/right_rect/camera_info'),
                #('/pcl_from_depth', '/pcl_from_depth1'),
            ],
        )
    ])
