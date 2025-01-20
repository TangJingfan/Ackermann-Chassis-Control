#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_control',  # package name
            executable='chassis_control',   # name of executable
            name='chassis_control',    # name of node
            output='screen',                # send log to screen
            parameters=[                    # parameters
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200}
            ],
            remappings=[                    # remapping of topic
                # ('/topic_in', '/topic_out')
            ]
        ),
    ])
