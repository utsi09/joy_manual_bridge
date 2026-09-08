from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 시뮬레이터(CARLA)에서는 true, 실차에서는 false
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 시작 기어 (2=DRIVE, 20=REVERSE, 1=NEUTRAL, 22=PARK)
        DeclareLaunchArgument('gear_command', default_value='2'),
        # 시작할 때 selector REMOTE / gate EXTERNAL을 알아서 잡을지.
        # 실차에서 운전자가 Options 버튼으로 직접 넘기고 싶으면 false.
        DeclareLaunchArgument('select_remote_on_start', default_value='true'),
        DeclareLaunchArgument('gate_external_on_start', default_value='true'),
        # 주행 중 D<->R 전환 거부 속도 [m/s], 0 이하면 보호 끔
        DeclareLaunchArgument('gear_change_max_speed', default_value='0.5'),
        # 페달 안 밟았을 때 넣어줄 브레이크 (크리프 방지). 0이면 끔
        DeclareLaunchArgument('idle_brake', default_value='0.2'),
        # PARK에서 잡아둘 브레이크, 0이면 끔
        DeclareLaunchArgument('park_brake', default_value='0.5'),
        # 기어별 최대 속도 [m/s], 0이면 제한 없음
        DeclareLaunchArgument('low_gear_max_speed', default_value='3.0'),
        DeclareLaunchArgument('drive_gear_max_speed', default_value='6.0'),
        DeclareLaunchArgument('reverse_gear_max_speed', default_value='3.0'),
        # gate가 AUTO로 돌아가면 다시 EXTERNAL로 되돌릴지
        DeclareLaunchArgument('hold_gate_external', default_value='false'),
        # 스로틀 배율. 기본 accel map(0.5까지)을 쓰면 0.5 권장
        DeclareLaunchArgument('throttle_scale', default_value='1.0'),
        # joy_controller 하트비트 없이도 계속 ready를 쏠지 (테스트용)
        DeclareLaunchArgument('heartbeat_always', default_value='false'),

        Node(
            package='joy_manual_bridge',
            executable='joy_to_manual_bridge',
            name='joy_to_manual_bridge',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gear_command': LaunchConfiguration('gear_command'),
                'select_remote_on_start':
                    LaunchConfiguration('select_remote_on_start'),
                'gate_external_on_start':
                    LaunchConfiguration('gate_external_on_start'),
                'gear_change_max_speed':
                    LaunchConfiguration('gear_change_max_speed'),
                'idle_brake': LaunchConfiguration('idle_brake'),
                'throttle_scale': LaunchConfiguration('throttle_scale'),
                'park_brake': LaunchConfiguration('park_brake'),
                'low_gear_max_speed':
                    LaunchConfiguration('low_gear_max_speed'),
                'drive_gear_max_speed':
                    LaunchConfiguration('drive_gear_max_speed'),
                'reverse_gear_max_speed':
                    LaunchConfiguration('reverse_gear_max_speed'),
                'hold_gate_external':
                    LaunchConfiguration('hold_gate_external'),
                'heartbeat_always': LaunchConfiguration('heartbeat_always'),
            }],
            output='screen',
        ),
    ])
