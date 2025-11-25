#!/usr/bin/env python3
"""
Elevator Spot Detector (ROS2 rclpy)

- Subscribes:
    1) floor/elevator move flag (std_msgs/Bool)
    2) odometry (nav_msgs/Odometry)
- Publishes:
    1) elevator spot arrived (std_msgs/Bool)

Logic:
- When move flag is False:
    -> reset state, publish "not arrived"
- When move flag is True:
    -> compute distance from current pose to elevator spot
    -> if within tolerance for N consecutive cycles => arrived True
    -> else arrived False
"""

import math
import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class ElevatorSpotDetector(Node):
    def __init__(self):
        super().__init__('elevator_spot_detector')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('flag_topic', '/elevator_move_flag')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('arrived_topic', '/elevator_spot_arrived')

        # Elevator spot pose in map/odom frame (choose consistent frame with odom)
        self.declare_parameter('spot_x', 0.0)
        self.declare_parameter('spot_y', 0.0)
        self.declare_parameter('spot_yaw', 0.0)   # rad, optional use

        # Tolerances
        self.declare_parameter('pos_tolerance', 0.25)   # meters
        self.declare_parameter('yaw_tolerance', 0.35)   # rad (~20 deg), set large if not used

        # Stability (debounce)
        self.declare_parameter('stable_count_required', 5)

        # Publish rate
        self.declare_parameter('check_rate_hz', 10.0)

        # Use yaw check or not
        self.declare_parameter('use_yaw_check', False)

        self.flag_topic = self.get_parameter('flag_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.arrived_topic = self.get_parameter('arrived_topic').value

        self.spot_x = float(self.get_parameter('spot_x').value)
        self.spot_y = float(self.get_parameter('spot_y').value)
        self.spot_yaw = float(self.get_parameter('spot_yaw').value)

        self.pos_tol = float(self.get_parameter('pos_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)

        self.stable_required = int(self.get_parameter('stable_count_required').value)
        self.check_rate_hz = float(self.get_parameter('check_rate_hz').value)

        self.use_yaw_check = bool(self.get_parameter('use_yaw_check').value)

        # -------------------------
        # Internal state
        # -------------------------
        self.move_flag = False
        self.last_odom: Optional[Odometry] = None
        self.stable_counter = 0
        self.arrived = False

        # -------------------------
        # ROS interfaces
        # -------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub_flag = self.create_subscription(
            Bool, self.flag_topic, self.flag_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos
        )

        self.pub_arrived = self.create_publisher(Bool, self.arrived_topic, 10)

        period = 1.0 / max(1e-3, self.check_rate_hz)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'ElevatorSpotDetector started. spot=({self.spot_x:.2f}, {self.spot_y:.2f}, yaw={self.spot_yaw:.2f}) '
            f'pos_tol={self.pos_tol:.2f}, stable_required={self.stable_required}, use_yaw_check={self.use_yaw_check}'
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def flag_callback(self, msg: Bool):
        self.move_flag = bool(msg.data)

        if not self.move_flag:
            # Reset when elevator not needed
            self.stable_counter = 0
            self.arrived = False
            self.publish_arrived(False)

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def timer_callback(self):
        if not self.move_flag:
            # Do nothing when elevator sequence not active
            return

        if self.last_odom is None:
            return

        x, y, yaw = self.extract_pose(self.last_odom)

        dist = math.hypot(x - self.spot_x, y - self.spot_y)

        if self.use_yaw_check:
            yaw_err = self.normalize_angle(yaw - self.spot_yaw)
            yaw_ok = abs(yaw_err) < self.yaw_tol
        else:
            yaw_ok = True

        pos_ok = dist < self.pos_tol

        if pos_ok and yaw_ok:
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        arrived_now = (self.stable_counter >= self.stable_required)

        if arrived_now != self.arrived:
            self.arrived = arrived_now
            self.publish_arrived(self.arrived)

    # -------------------------
    # Helpers
    # -------------------------
    @staticmethod
    def extract_pose(odom: Odometry):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return float(p.x), float(p.y), float(yaw)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def publish_arrived(self, value: bool):
        msg = Bool()
        msg.data = value
        self.pub_arrived.publish(msg)


def main():
    rclpy.init()
    node = ElevatorSpotDetector()

    def _shutdown(*_):
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
