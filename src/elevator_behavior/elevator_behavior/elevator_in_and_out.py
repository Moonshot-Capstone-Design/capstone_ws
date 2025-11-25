#!/usr/bin/env python3
"""
elevator_in_and_out.py

Inputs (topics):
- /current_floor (std_msgs/Int32) : robot's current floor (before boarding)
- /target_floor  (std_msgs/Int32) : destination floor (after boarding)
- /elevator_spot_arrived (std_msgs/Bool) : True when robot is at elevator waiting spot
- /elevator_door_open   (std_msgs/Bool) : True when elevator door is open

Outputs (topics):
- /elevator_state (std_msgs/String) : debug state
- /in_elevator (std_msgs/Bool) : internal state broadcast (optional for other modules)

Actions:
- /navigate_to_pose (nav2_msgs/action/NavigateToPose)

Behavior:
1) If at spot and door open -> enter elevator (NavigateToPose to enter_pose).
2) After enter success -> inside state.
3) If inside and door open AND target_floor received -> exit elevator (NavigateToPose to exit_pose).
4) After exit success -> reset to idle.
"""

import math
import signal
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float):
    """yaw(rad) -> quaternion (z,w only, planar)"""
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class State(Enum):
    IDLE = 0
    WAIT_DOOR_TO_ENTER = 1
    ENTERING = 2
    INSIDE_WAIT_TARGET = 3
    WAIT_DOOR_TO_EXIT = 4
    EXITING = 5


class ElevatorInOut(Node):
    def __init__(self):
        super().__init__('elevator_in_and_out')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('frame_id', 'map')

        # Enter pose (inside elevator)
        self.declare_parameter('enter_x', 0.0)
        self.declare_parameter('enter_y', 0.0)
        self.declare_parameter('enter_yaw', 0.0)

        # Exit pose (outside at destination floor)
        self.declare_parameter('exit_x', 0.0)
        self.declare_parameter('exit_y', 0.0)
        self.declare_parameter('exit_yaw', 0.0)

        # Topics
        self.declare_parameter('current_floor_topic', '/current_floor')
        self.declare_parameter('target_floor_topic', '/target_floor')
        self.declare_parameter('spot_arrived_topic', '/elevator_spot_arrived')
        self.declare_parameter('door_open_topic', '/elevator_door_open')

        # Simple debounce
        self.declare_parameter('door_open_hold_count', 3)
        self.declare_parameter('timer_hz', 10.0)

        self.nav_action_name = self.get_parameter('navigate_action').value
        self.frame_id = self.get_parameter('frame_id').value

        self.enter_x = float(self.get_parameter('enter_x').value)
        self.enter_y = float(self.get_parameter('enter_y').value)
        self.enter_yaw = float(self.get_parameter('enter_yaw').value)

        self.exit_x = float(self.get_parameter('exit_x').value)
        self.exit_y = float(self.get_parameter('exit_y').value)
        self.exit_yaw = float(self.get_parameter('exit_yaw').value)

        self.current_floor_topic = self.get_parameter('current_floor_topic').value
        self.target_floor_topic = self.get_parameter('target_floor_topic').value
        self.spot_arrived_topic = self.get_parameter('spot_arrived_topic').value
        self.door_open_topic = self.get_parameter('door_open_topic').value

        self.door_open_hold_count = int(self.get_parameter('door_open_hold_count').value)
        self.timer_hz = float(self.get_parameter('timer_hz').value)

        # -------------------------
        # Internal state
        # -------------------------
        self.state = State.IDLE

        self.current_floor: Optional[int] = None  # saved before boarding
        self.target_floor: Optional[int] = None   # received after boarding

        self.spot_arrived = False
        self.door_open = False

        self._door_open_counter = 0

        self._active_nav_goal = False
        self._nav_goal_handle = None

        # -------------------------
        # ROS interfaces
        # -------------------------
        qos_scanlike = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.sub_current_floor = self.create_subscription(
            Int32, self.current_floor_topic, self.cb_current_floor, 10
        )
        self.sub_target_floor = self.create_subscription(
            Int32, self.target_floor_topic, self.cb_target_floor, 10
        )
        self.sub_spot = self.create_subscription(
            Bool, self.spot_arrived_topic, self.cb_spot_arrived, 10
        )
        self.sub_door = self.create_subscription(
            Bool, self.door_open_topic, self.cb_door_open, qos_scanlike
        )

        self.pub_state = self.create_publisher(String, '/elevator_state', 10)
        self.pub_in_elevator = self.create_publisher(Bool, '/in_elevator', 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        period = 1.0 / max(1e-3, self.timer_hz)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info('ElevatorInOut node started.')

    # -------------------------
    # Topic callbacks
    # -------------------------
    def cb_current_floor(self, msg: Int32):
        # Always store latest current floor (used at boarding)
        self.current_floor = int(msg.data)

    def cb_target_floor(self, msg: Int32):
        self.target_floor = int(msg.data)

    def cb_spot_arrived(self, msg: Bool):
        self.spot_arrived = bool(msg.data)

    def cb_door_open(self, msg: Bool):
        self.door_open = bool(msg.data)
        if self.door_open:
            self._door_open_counter += 1
        else:
            self._door_open_counter = 0

    # -------------------------
    # Main state machine tick
    # -------------------------
    def tick(self):
        # publish debug
        self.publish_state()

        # If navigating, do nothing here (wait for result callback)
        if self._active_nav_goal:
            return

        # Stable door open?
        door_open_stable = (self._door_open_counter >= self.door_open_hold_count)

        if self.state == State.IDLE:
            # Wait in front of elevator spot
            if self.spot_arrived:
                self.state = State.WAIT_DOOR_TO_ENTER

        elif self.state == State.WAIT_DOOR_TO_ENTER:
            # Need current floor to be known before boarding
            if self.current_floor is None:
                return

            # If door open at spot -> enter
            if self.spot_arrived and door_open_stable:
                self.send_nav_goal(self.enter_x, self.enter_y, self.enter_yaw)
                self.state = State.ENTERING

        elif self.state == State.INSIDE_WAIT_TARGET:
            # Wait for target floor after boarding
            if self.target_floor is not None:
                self.state = State.WAIT_DOOR_TO_EXIT

        elif self.state == State.WAIT_DOOR_TO_EXIT:
            # If inside and door open (at destination) -> exit
            if door_open_stable:
                self.send_nav_goal(self.exit_x, self.exit_y, self.exit_yaw)
                self.state = State.EXITING

        # ENTERING / EXITING states handled by nav result callbacks

    # -------------------------
    # Nav2 action client
    # -------------------------
    def send_nav_goal(self, x: float, y: float, yaw: float):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose_stamped(x, y, yaw)

        self._active_nav_goal = True

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav goal rejected.')
            self._active_nav_goal = False
            # rollback state
            if self.state == State.ENTERING:
                self.state = State.WAIT_DOOR_TO_ENTER
            elif self.state == State.EXITING:
                self.state = State.WAIT_DOOR_TO_EXIT
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        self._active_nav_goal = False
        result = future.result().result
        # result is nav2_msgs/action/NavigateToPose.Result
        success = bool(result)

        if self.state == State.ENTERING:
            if success:
                # Now inside
                self.state = State.INSIDE_WAIT_TARGET
                self.publish_in_elevator(True)
            else:
                # retry by waiting door again
                self.state = State.WAIT_DOOR_TO_ENTER

        elif self.state == State.EXITING:
            if success:
                # Exit complete -> reset
                self.state = State.IDLE
                self.publish_in_elevator(False)
                # clear target floor after exit
                self.target_floor = None
            else:
                self.state = State.WAIT_DOOR_TO_EXIT

    # -------------------------
    # Helpers
    # -------------------------
    def make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()

        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0

        qz, qw = yaw_to_quat(yaw)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def publish_state(self):
        s = String()
        s.data = self.state.name
        self.pub_state.publish(s)

    def publish_in_elevator(self, inside: bool):
        m = Bool()
        m.data = inside
        self.pub_in_elevator.publish(m)

    # -------------------------
    # Shutdown
    # -------------------------
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ElevatorInOut()

    def _sig(*_):
        node.shutdown()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
