from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_manual_bridge',
            executable='joy_to_manual_bridge',
            name='joy_to_manual_bridge',
            parameters=[{
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ])
