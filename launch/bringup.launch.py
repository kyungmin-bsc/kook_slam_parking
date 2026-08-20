"""
지도 / 위치추정 / 경로계획 / 경로추종 - 주행 준비 전체.

이 launch는 "차를 움직일 준비"까지만 한다. 실제 미션 주행은
mission.launch.py를 따로 띄워야 시작된다.

왜 나눠놨나: 대회 당일 전원 켜자마자 차가 튀어나가면 안 된다. 이걸 먼저 띄워서
RViz로 지도가 뜨는지, AMCL 파티클이 출발 위치에 수렴했는지, /scan이 벽과
겹치는지 눈으로 확인한 뒤에 미션을 시작하는 2단계 스타트가 안전하다.

이 launch가 다루지 않는 것 (별도로 이미 떠 있어야 함)
  - 라이다/카메라 등 센서 브링업
  - ROS1 쪽: 모터 드라이버, VESC 드라이버, vesc_odom_publisher.py
  - ros1_bridge (dynamic_bridge --bridge-all-topics)
    -> ROS1의 /vesc_odom(Odometry)과 tf(odom->base_link)를 ROS2로,
       ROS2의 /xycar_motor(Float32MultiArray)를 ROS1으로 넘겨준다.
       셋 다 표준 타입이라 별도 매핑 설정 없이 통과한다.

실행 예:
  ros2 launch parking_mission bringup.launch.py
  ros2 launch parking_mission bringup.launch.py map:=/path/to/parking_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('parking_mission')
    default_params = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'parking_map.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'parking_view.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf')
    use_obstacle_monitor = LaunchConfiguration('use_obstacle_monitor')

    declares = [
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='주최측 배포 지도 yaml. 당일 갱신되면 이 값만 바꿔 실행'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='lifecycle_manager가 configure/activate 자동 수행'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('use_obstacle_monitor', default_value='true'),
        DeclareLaunchArgument(
            'publish_lidar_tf', default_value='true',
            description='base_link->laser_frame 정적 TF를 여기서 발행. '
                        '센서 브링업이 이미 발행한다면 false로 꺼서 충돌 방지'),
    ]

    lifecycle_nodes = [
        'map_server', 'amcl', 'planner_server',
        'controller_server', 'behavior_server', 'bt_navigator',
    ]

    nav2_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params_file, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params_file]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_parking', output='screen',
             parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}]),
    ]

    # 실차 측정 완료: base_link(뒷차축 중심) 기준 라이다가 x=+0.41m 앞, 정면 장착.
    # /scan의 frame_id는 실측으로 'laser_frame' 확인됨.
    # /tf_static에 이 변환이 없는 것으로 확인되어 여기서 직접 발행한다.
    lidar_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser_tf', output='screen',
        arguments=[
            '--x', '0.41', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
        ],
        condition=IfCondition(publish_lidar_tf),
    )

    cmd_vel_bridge = Node(
        package='parking_mission', executable='cmd_vel_bridge',
        name='cmd_vel_bridge', output='screen',
    )

    # 지도에 없는 장애물만 골라 빨간 마커로 띄운다. costmap은 벽과 새 장애물을
    # 똑같이 '비용 높은 셀'로 보여줘서 눈으로 구분이 안 되기 때문에 따로 둔다.
    obstacle_monitor = Node(
        package='parking_mission', executable='obstacle_monitor',
        name='obstacle_monitor', output='screen',
        condition=IfCondition(use_obstacle_monitor),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declares + nav2_nodes + [lidar_tf, cmd_vel_bridge, obstacle_monitor, rviz])
