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
            executable='pclsyncIcppcl',
            output='both',
            emulate_tty=True,
            # Set to True to process just if there is a subscription,
            # False to process always
            parameters=[
                {"check_subscription_count": False}
            ],
            # Use topics from robot
            remappings=[
                ('/pointcloud_in1', '/computer_vision/pcl_sync'),
                ('/pointcloud_in2', '/rslidar_points')
            ],
        )
    ])
