"""Xbox 360 모드 패드(GameSir 등)용 조이스틱 쪽 한 방 런치.

joy_node + joy_remap(홈<->X 스왑) + joy_controller(ds4 프로파일, /joy_remapped 입력)
+ joy_manual_bridge 를 같이 띄운다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jc_share = get_package_share_directory('autoware_joy_controller')
    bridge_share = get_package_share_directory('joy_manual_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('throttle_scale', default_value='1.0'),
        DeclareLaunchArgument('joy_type', default_value='ds4'),
        # output.buttons[i] = input.buttons[map[i]]  (기본: 2<->8 스왑)
        DeclareLaunchArgument('button_map', default_value='[0, 1, 8, 3, 4, 5, 6, 7, 2, 9, 10]'),

        Node(
            package='joy_manual_bridge',
            executable='joy_remap',
            name='joy_remap',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/joy',
                'output_topic': '/joy_remapped',
                'button_map': LaunchConfiguration('button_map'),
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(jc_share, 'launch', 'joy_controller.launch.xml')),
            launch_arguments={
                'input_joy': '/joy_remapped',
                'config_file': [jc_share, '/config/joy_controller_',
                                LaunchConfiguration('joy_type'), '.param.yaml'],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(bridge_share, 'launch', 'joy_manual_bridge.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'throttle_scale': LaunchConfiguration('throttle_scale'),
            }.items(),
        ),
    ])
