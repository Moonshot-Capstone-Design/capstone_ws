#!/usr/bin/env python3
"""
Elevator door open/close detector using LiDAR ROI on LaserScan.

- 전방 방향(기본 0deg)에 ROI를 잡고,
- ROI 안의 거리 분포를 보고 "문이 열렸는지/닫혔는지"만 Bool 플래그로 퍼블리시한다.
"""

import math
import signal
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ElevatorDoorROIDetector(Node):
    def __init__(self):
        super().__init__('elevator_door_roi_detector')

        # -----------------------------
        # Parameters
        # -----------------------------
        # Scan input
        self.declare_parameter('scan_topic', '/scan')

        # ROI definition (degrees, base_link frame)
        # roi_center_deg: direction to door (0 = front, +CCW)
        # roi_width_deg : total width of ROI
        # 👉 기본값을 전방(0deg)으로 세팅
        self.declare_parameter('roi_center_deg', 140.0)
        self.declare_parameter('roi_width_deg', 30.0)

        # Door distance model (meters)
        # closed_ref_dist: typical distance to door when closed
        # open_delta      : how much farther than closed_ref to consider open
        self.declare_parameter('closed_ref_dist', 1.2)
        self.declare_parameter('open_delta', 0.7)

        # Validity / smoothing
        self.declare_parameter('min_valid_ratio', 0.25)
        self.declare_parameter('median_window', 3)

        # Hysteresis to reduce toggling (meters)
        self.declare_parameter('hysteresis', 0.1)

        # 결과 플래그 토픽
        self.declare_parameter('door_flag_topic', '/elevator/door_open_flag')

        # -----------------------------
        # Load parameters
        # -----------------------------
        self.scan_topic = self.get_parameter('scan_topic').value

        self.roi_center = math.radians(
            float(self.get_parameter('roi_center_deg').value)
        )
        self.roi_width = math.radians(
            float(self.get_parameter('roi_width_deg').value)
        )

        self.closed_ref = float(self.get_parameter('closed_ref_dist').value)
        self.open_delta = float(self.get_parameter('open_delta').value)

        self.min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self.median_window = int(self.get_parameter('median_window').value)
        self.hysteresis = float(self.get_parameter('hysteresis').value)

        self.door_flag_topic = self.get_parameter('door_flag_topic').value

        # Internal state (for hysteresis)
        self.door_open = False

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos
        )

        # 문 열림 여부만 Bool로 퍼블리시
        self.pub_open = self.create_publisher(Bool, self.door_flag_topic, 10)

        self.get_logger().info(
            f'[ElevatorDoorROIDetector] scan_topic={self.scan_topic}, '
            f'ROI center={math.degrees(self.roi_center):.1f} deg, '
            f'width={math.degrees(self.roi_width):.1f} deg, '
            f'closed_ref={self.closed_ref:.2f} m, open_delta={self.open_delta:.2f} m, '
            f'flag_topic={self.door_flag_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        try:
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            n = ranges.size
            if n == 0:
                return

            # 각 인덱스별 각도 배열
            angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

            # ROI 마스크
            half = 0.5 * self.roi_width
            roi_mask = (angles >= self.roi_center - half) & (angles <= self.roi_center + half)
            roi_ranges = ranges[roi_mask]

            if roi_ranges.size == 0:
                return

            # 유효 거리 필터: finite & 센서 min/max 범위 안
            valid_mask = np.isfinite(roi_ranges) \
                         & (roi_ranges > msg.range_min) \
                         & (roi_ranges < msg.range_max)

            valid_ranges = roi_ranges[valid_mask]
            valid_ratio = float(valid_ranges.size) / float(roi_ranges.size)

            # Optional median filter
            if valid_ranges.size >= self.median_window and self.median_window > 1:
                valid_ranges = self.median_filter_1d(valid_ranges, self.median_window)

            # 문 열림/닫힘 판정
            if valid_ranges.size == 0 or valid_ratio < self.min_valid_ratio:
                # 거의 리턴이 없으면 앞이 뻥 뚫린 상태 → "열림"으로 간주
                roi_median = float('inf')
                open_now = True
            else:
                roi_median = float(np.median(valid_ranges))

                # 히스테리시스 적용
                open_th = self.closed_ref + self.open_delta
                close_th = open_th - self.hysteresis

                if self.door_open:
                    # 이미 open 상태라면 조금 줄어들어도 close_th 이상이면 계속 open
                    open_now = (roi_median > close_th)
                else:
                    # 닫힌 상태에서 open으로 넘어갈 땐 open_th 넘겨야 함
                    open_now = (roi_median > open_th)

            self.door_open = open_now

            # Bool 플래그만 퍼블리시
            out = Bool()
            out.data = open_now
            self.pub_open.publish(out)

        except Exception as e:
            self.get_logger().warn(f'scan_callback exception: {e}')

    @staticmethod
    def median_filter_1d(x: np.ndarray, k: int) -> np.ndarray:
        """Simple 1D median filter."""
        if k <= 1:
            return x
        pad = k // 2
        xp = np.pad(x, (pad, pad), mode='edge')
        out = np.empty_like(x)
        for i in range(len(x)):
            out[i] = np.median(xp[i:i + k])
        return out


def main():
    rclpy.init()
    node = ElevatorDoorROIDetector()

    def _shutdown(*_):
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
