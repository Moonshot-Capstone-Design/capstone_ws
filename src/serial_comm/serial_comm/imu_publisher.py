#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

NUM_RE = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')

class EbimuPublisher(Node):
    def __init__(self):
        super().__init__('ebimu_publisher')

        # 파라미터 (한 줄 실행 가능)
        self.declare_parameter('port', '/dev/ttyUSB_IMU')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'ebimu_link')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(
            self.get_parameter('baud').get_parameter_value().integer_value
            or self.get_parameter('baud').get_parameter_value().string_value
            or 115200
        )
        self.get_logger().info(f'EBIMU Port: {port}')
        self.get_logger().info(f'Baudrate: {baud}')

        # 시리얼
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')
            raise

        # 센서 데이터 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_imu = self.create_publisher(Imu, '/ebimu/imu', qos)
        self.pub_raw = self.create_publisher(String, '/ebimu/raw', 10)

        # 200 Hz 시도(실제는 timeout/라인 길이에 따름)
        self.timer = self.create_timer(0.005, self.loop)

        self.get_logger().info('Starting ebimu_publisher..')

    def loop(self):
        try:
            line = self.ser.readline()
            if not line:
                return
            s = line.decode('utf-8', errors='ignore').strip()
            if not s:
                return

            # 원문도 같이 퍼블리시 (디버깅용)
            self.pub_raw.publish(String(data=s))

            # 예상 포맷: [qw, qx, qy, qz, roll, pitch, yaw, ax, ay, az, (gx, gy, gz)? ...]
            vals = [float(x) for x in NUM_RE.findall(s)]
            if len(vals) < 10:
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

            # 쿼터니언
            qw, qx, qy, qz = vals[0], vals[1], vals[2], vals[3]
            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz

            # 가속도 (EBIMU가 이미 중력 제거를 출력한다고 가정)
            ax, ay, az = vals[7], vals[8], vals[9]
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            # 자이로가 뒤에 있으면 채움
            if len(vals) >= 13:
                gx, gy, gz = vals[10], vals[11], vals[12]
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz

            self.pub_imu.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'read/parse error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = EbimuPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
