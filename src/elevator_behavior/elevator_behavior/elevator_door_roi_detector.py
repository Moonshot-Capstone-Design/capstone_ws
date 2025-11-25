#!/usr/bin/env python3
"""
Elevator door open/close detector using LiDAR ROI on LaserScan.

Core idea:
- Pick an angular ROI where elevator door exists.
- Compare median distance (or valid ratio) in ROI to a reference for "closed".
- If distance becomes much larger or returns become sparse -> "open".
"""

import math
import signal
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


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
        self.declare_parameter('roi_center_deg', 0.0)
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

        # Debug
        self.declare_parameter('publish_debug', True)

        self.scan_topic = self.get_parameter('scan_topic').value

        self.roi_center = math.radians(float(self.get_parameter('roi_center_deg').value))
        self.roi_width = math.radians(float(self.get_parameter('roi_width_deg').value))

        self.closed_ref = float(self.get_parameter('closed_ref_dist').value)
        self.open_delta = float(self.get_parameter('open_delta').value)

        self.min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self.median_window = int(self.get_parameter('median_window').value)
        self.hysteresis = float(self.get_parameter('hysteresis').value)

        self.publish_debug = bool(self.get_parameter('publish_debug').value)

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

        self.pub_open = self.create_publisher(Bool, '/elevator_door_open', 10)

        if self.publish_debug:
            self.pub_median = self.create_publisher(Float32, '/elevator_door_roi_median', 10)
            self.pub_valid_ratio = self.create_publisher(Float32, '/elevator_door_roi_valid_ratio', 10)

        self.get_logger().info(
            f'ROI center={math.degrees(self.roi_center):.1f} deg, '
            f'width={math.degrees(self.roi_width):.1f} deg, '
            f'closed_ref={self.closed_ref:.2f} m, open_delta={self.open_delta:.2f} m'
        )

    def scan_callback(self, msg: LaserScan):
        try:
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            n = ranges.size
            if n == 0:
                return

            # Build angle array for each index
            angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

            # ROI mask
            half = 0.5 * self.roi_width
            roi_mask = (angles >= self.roi_center - half) & (angles <= self.roi_center + half)
            roi_ranges = ranges[roi_mask]

            if roi_ranges.size == 0:
                return

            # Valid filter: finite and within min/max range
            valid_mask = np.isfinite(roi_ranges) \
                         & (roi_ranges > msg.range_min) \
                         & (roi_ranges < msg.range_max)

            valid_ranges = roi_ranges[valid_mask]
            valid_ratio = float(valid_ranges.size) / float(roi_ranges.size)

            # Optional median filter to reduce spikes
            if valid_ranges.size >= self.median_window and self.median_window > 1:
                valid_ranges = self.median_filter_1d(valid_ranges, self.median_window)

            # Decide open / closed
            # Case 1) too few valid returns -> likely open (empty space)
            if valid_ranges.size == 0 or valid_ratio < self.min_valid_ratio:
                roi_median = float('inf')
                open_now = True
            else:
                roi_median = float(np.median(valid_ranges))

                open_th = self.closed_ref + self.open_delta
                close_th = open_th - self.hysteresis

                # Hysteresis based state update
                if self.door_open:
                    open_now = (roi_median > close_th)
                else:
                    open_now = (roi_median > open_th)

            self.door_open = open_now

            # Publish result
            out = Bool()
            out.data = open_now
            self.pub_open.publish(out)

            # Debug topics
            if self.publish_debug:
                m_msg = Float32()
                m_msg.data = roi_median if math.isfinite(roi_median) else float(msg.range_max)
                self.pub_median.publish(m_msg)

                r_msg = Float32()
                r_msg.data = valid_ratio
                self.pub_valid_ratio.publish(r_msg)

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
