#!/usr/bin/env python3
"""その場で指定角度だけ旋回するサンプル。

使い方:
    ros2 run xgo_lite2_driver spin_in_place    # 既定: 左 360deg, 1.0 rad/s
    python3 spin_in_place.py --angle 180 --speed 0.8 --direction right
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SpinInPlace(Node):
    def __init__(self, angle_deg: float, speed_rad_s: float,
                 direction: str, publish_hz: float = 20.0) -> None:
        super().__init__('spin_in_place')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angle_rad = math.radians(abs(angle_deg))
        self.omega = abs(speed_rad_s) * (1.0 if direction == 'left' else -1.0)
        self.publish_hz = publish_hz

    def run(self) -> None:
        # cmd_vel_timeout(0.5s) より短い周期で連続 publish する
        period = 1.0 / self.publish_hz
        duration = self.angle_rad / abs(self.omega)
        self.get_logger().info(
            f'Spinning: omega={self.omega:.3f} rad/s, '
            f'duration={duration:.2f} s')

        twist = Twist()
        twist.angular.z = float(self.omega)

        # 少し待って publisher の接続確立を確認
        time.sleep(0.3)

        start = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start) < duration:
            self.pub.publish(twist)
            time.sleep(period)

        # 停止指令を数回送って確実に止める
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(period)
        self.get_logger().info('done')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--angle', type=float, default=360.0,
                        help='旋回角度 [deg] (既定: 360)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='角速度 [rad/s] (既定: 1.0, 上限 ~1.7)')
    parser.add_argument('--direction', choices=['left', 'right'],
                        default='left', help='旋回方向 (既定: left)')
    args = parser.parse_args()

    rclpy.init()
    node = SpinInPlace(args.angle, args.speed, args.direction)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
