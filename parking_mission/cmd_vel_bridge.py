#!/usr/bin/env python3
"""
Nav2 controller_server의 /cmd_vel(Twist)을 자이카 모터 토픽
/xycar_motor(std_msgs/Float32MultiArray [angle, speed])으로 변환하는 브릿지.

이 노드는 ROS2에서 /xycar_motor를 발행하고, ros1_bridge가 그걸 ROS1 쪽
모터 드라이버와 vesc_odom_publisher에게 전달한다. Float32MultiArray는
표준 타입이라 ros1_bridge가 별도 설정 없이 넘겨준다.

게이팅
-----
/nav_drive_enable(Bool)이 false면 아무것도 발행하지 않는다.

이게 필요한 이유: 슬롯 안 정밀 주차 기동은 mission_manager가 /xycar_motor를
**직접** 잡고 돌린다. 그 순간 이 브릿지도 같은 토픽에 쓰면 두 노드가 모터를
두고 싸운다. Nav2가 목표 도달 후 알아서 조용해질 거라는 가정에만 기대지 않고,
mission_manager가 기동 직전에 명시적으로 이 브릿지를 꺼버린다.

실측 완료 값 (track_parking/MEASUREMENTS.md, 실차 측정)
  wheelbase = 0.33 m
  angle 단위 = degree 그대로 (라디안*120 스케일이 아님이 실측으로 확인됨)
  angle 한계 = ±35도 (명령 40 이상 줘도 기계적으로 더 안 꺾임)
  speed_gain = 9.86 (speed 단위 per m/s)
  min_forward_speed = 3.0 (speed 2는 정지, 3부터 실제로 굴러감)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray


def latched_qos() -> QoSProfile:
    """늦게 뜬 구독자도 마지막 값을 받도록 하는 QoS (ROS1 latched 상당)."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # 실차 측정 완료 값
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('angle_limit', 35.0)
        self.declare_parameter('speed_gain', 9.86)
        self.declare_parameter('min_forward_speed', 3.0)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('enable_topic', '/nav_drive_enable')
        # bringup만 띄우고 RViz로 수동 테스트할 때를 위해 기본은 활성.
        # 미션 중에는 mission_manager가 구간마다 명시적으로 켜고 끈다.
        self.declare_parameter('default_enabled', True)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.angle_limit = float(self.get_parameter('angle_limit').value)
        self.speed_gain = float(self.get_parameter('speed_gain').value)
        self.min_forward_speed = float(self.get_parameter('min_forward_speed').value)
        self.enabled = bool(self.get_parameter('default_enabled').value)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        motor_topic = self.get_parameter('motor_topic').value
        enable_topic = self.get_parameter('enable_topic').value

        self._pub = self.create_publisher(Float32MultiArray, motor_topic, 10)
        self._sub = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self._enable_sub = self.create_subscription(
            Bool, enable_topic, self._on_enable, latched_qos())

        self.get_logger().info(
            '%s(Twist) -> %s([angle,speed]) 브릿지 시작 | '
            'wheelbase=%.2f angle_limit=%.1f speed_gain=%.2f min_speed=%.1f | 초기 %s'
            % (cmd_vel_topic, motor_topic, self.wheelbase, self.angle_limit,
               self.speed_gain, self.min_forward_speed,
               '활성' if self.enabled else '비활성'))

    def _on_enable(self, msg: Bool) -> None:
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        self.get_logger().info('Nav2 구동 게이트 -> %s' % ('활성' if self.enabled else '비활성'))
        if not self.enabled:
            # 끄는 순간 정지 명령을 한 번 확실히 박아둔다
            self._publish(0.0, 0.0)

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not self.enabled:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 속도가 0에 가까우면 곡률 계산이 0으로 나누기가 되므로 조향을 0으로 고정.
        if abs(linear_x) < 1e-3:
            steer_rad = 0.0
        else:
            # 자전거 모델: tan(delta) = wheelbase * (angular.z / linear.x)
            # 후진(linear.x < 0) 시 atan2가 각도를 뒤집어버리므로 크기로 나눈다.
            steer_rad = math.atan2(self.wheelbase * angular_z, abs(linear_x))

        angle = math.degrees(steer_rad)
        angle = max(-self.angle_limit, min(self.angle_limit, angle))

        speed = linear_x * self.speed_gain
        # 정지마찰 구간 건너뛰기 (speed 2는 안 움직임). 단, 진짜 0은 0으로 둔다.
        if 0.0 < speed < self.min_forward_speed:
            speed = self.min_forward_speed
        elif -self.min_forward_speed < speed < 0.0:
            speed = -self.min_forward_speed

        self._publish(angle, speed)

    def _publish(self, angle: float, speed: float) -> None:
        cmd = Float32MultiArray()
        cmd.data = [float(angle), float(speed)]
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
