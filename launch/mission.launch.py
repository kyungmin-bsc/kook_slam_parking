"""
미션 주행 시작. bringup.launch.py가 먼저 떠 있어야 한다.

bringup으로 지도/AMCL/Nav2를 올리고 RViz에서 위치추정이 수렴한 걸 눈으로 확인한
뒤에 이걸 실행한다.

실행 예:
  ros2 launch parking_mission mission.launch.py
  ros2 launch parking_mission mission.launch.py autostart:=true

autostart:=false(기본)면 노드가 뜨기만 하고 대기한다. 시작 신호는 따로 준다:
  ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"

이렇게 해두면 노드를 미리 띄워놓고 심판 신호에 맞춰 정확히 출발시킬 수 있다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    park_speed = LaunchConfiguration('park_speed')
    radius_scale = LaunchConfiguration('radius_scale')

    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='true면 노드 기동 즉시 출발. false면 /mission/start 대기'),
        DeclareLaunchArgument(
            'park_speed', default_value='0.30',
            description='주차 기동 속도(m/s). 느릴수록 정확하다'),
        DeclareLaunchArgument(
            'radius_scale', default_value='1.0',
            description='실제 회전반경이 설계보다 크게 나오면(덜 꺾이면) 1보다 크게'),
        Node(
            package='parking_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'park_speed': park_speed,
                'radius_scale': radius_scale,
            }],
        ),
    ])
