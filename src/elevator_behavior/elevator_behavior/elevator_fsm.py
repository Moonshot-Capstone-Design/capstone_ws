#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class ElevatorState(enum.Enum):
    IDLE = 0           # 대기
    WAIT_DOOR = 1      # align 후 문 열리기 대기
    WAIT_FLOOR = 2     # in 이후 목적층 도착 대기
    DONE = 3           # 시퀀스 완료


class ElevatorFSM(Node):
    def __init__(self):
        super().__init__('elevator_fsm')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('destination_topic', '/destination')
        self.declare_parameter('door_flag_topic', '/elevator/door_open_flag')
        self.declare_parameter('out_flag_topic', '/elevator/out_flag')
        self.declare_parameter('cmd_topic', '/elevator_fsm_cmd')
        self.declare_parameter('state_topic', '/elevator_fsm_state')

        # 문 열림 플래그가 얼마나 연속으로 들어와야 인정할지
        self.declare_parameter('door_open_min_count', 3)

        self.destination_topic = self.get_parameter('destination_topic').value
        self.door_flag_topic = self.get_parameter('door_flag_topic').value
        self.out_flag_topic = self.get_parameter('out_flag_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.door_open_min_count = int(self.get_parameter('door_open_min_count').value)

        # -----------------------------
        # Publishers / Subscribers
        # -----------------------------
        # Navigator로 보낼 목적지 명령
        self.dest_pub = self.create_publisher(String, self.destination_topic, 10)

        # FSM 상태(문자열로 publish)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        # 시작/리셋 명령
        self.cmd_sub = self.create_subscription(
            String,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        # 문 열림 플래그 (LiDAR ROI detector)
        self.door_sub = self.create_subscription(
            Bool,
            self.door_flag_topic,
            self.door_flag_callback,
            10
        )

        # 목적층 도착 플래그 (floor detector)
        self.out_flag_sub = self.create_subscription(
            Bool,
            self.out_flag_topic,
            self.out_flag_callback,
            10
        )

        # -----------------------------
        # 내부 상태
        # -----------------------------
        self.state = ElevatorState.IDLE

        # 문 열림 안정성 카운터
        self.door_open_counter = 0

        # 목적층 도착 플래그의 rising edge 체크용
        self.last_out_flag = False

        self.get_logger().info(
            f'[ElevatorFSM] dest_topic={self.destination_topic}, '
            f'door_flag_topic={self.door_flag_topic}, '
            f'out_flag_topic={self.out_flag_topic}, '
            f'cmd_topic={self.cmd_topic}'
        )
        self.publish_state()

    # ------------------------------------------------------------------
    # 상태 유틸
    # ------------------------------------------------------------------
    def set_state(self, new_state: ElevatorState):
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(f'[FSM] state -> {self.state.name}')
            self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def send_destination(self, label: str):
        """Navigator에게 /destination으로 문자열 label 전송."""
        m = String()
        m.data = label
        self.dest_pub.publish(m)
        self.get_logger().info(f'[FSM] /destination <- {label}')

    # ------------------------------------------------------------------
    # CMD 콜백
    # ------------------------------------------------------------------
    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'[FSM] cmd: {cmd}')

        if cmd == 'start':
            if self.state not in (ElevatorState.IDLE, ElevatorState.DONE):
                self.get_logger().warn('[FSM] Already running. Ignore start.')
                return
            self.start_sequence()

        elif cmd == 'reset':
            self.reset_fsm()

    def start_sequence(self):
        """
        전체 시퀀스:
        1) elevator_align 으로 엘리베이터 앞까지 이동
        2) 문 열림 감지 → elevator_in
        3) 목적층 도착(out_flag=True) → elevator_out
        """
        self.get_logger().info('[FSM] Sequence start.')
        self.door_open_counter = 0
        self.last_out_flag = False

        # 1단계: 엘리베이터 앞 정렬
        self.send_destination('elevator_align')
        self.set_state(ElevatorState.WAIT_DOOR)

    def reset_fsm(self):
        self.get_logger().info('[FSM] Reset requested.')
        self.door_open_counter = 0
        self.last_out_flag = False
        self.set_state(ElevatorState.IDLE)

    # ------------------------------------------------------------------
    # 문 열림 플래그 콜백
    # ------------------------------------------------------------------
    def door_flag_callback(self, msg: Bool):
        if self.state != ElevatorState.WAIT_DOOR:
            return

        flag = bool(msg.data)

        # 간단한 디바운스: True면 카운트++, False면 살짝 감소
        if flag:
            self.door_open_counter += 1
        else:
            if self.door_open_counter > 0:
                self.door_open_counter -= 1

        self.get_logger().debug(
            f'[FSM] door_flag={flag}, counter={self.door_open_counter}'
        )

        if self.door_open_counter >= self.door_open_min_count:
            self.handle_door_open()

    def handle_door_open(self):
        self.get_logger().info('[FSM] Door open detected (from ROI detector).')
        # 2단계: 엘리베이터 탑승
        self.send_destination('elevator_in')
        # 이제 층 도착(out_flag)만 기다린다.
        self.set_state(ElevatorState.WAIT_FLOOR)

    # ------------------------------------------------------------------
    # 목적층 도착 플래그 콜백
    # ------------------------------------------------------------------
    def out_flag_callback(self, msg: Bool):
        flag = bool(msg.data)

        # rising edge만 사용
        if self.state == ElevatorState.WAIT_FLOOR and flag and not self.last_out_flag:
            self.handle_target_floor_reached()

        self.last_out_flag = flag

    def handle_target_floor_reached(self):
        self.get_logger().info('[FSM] Target floor reached (out_flag=True). Exiting elevator.')

        # 3단계: 엘리베이터 하차
        self.send_destination('elevator_out')

        # 필요하면 여기서 "EXITING" 같은 중간 state 를 두고,
        # 나중에 Navigator가 out 지점 도달했다고 알려줄 때 DONE으로 넘겨도 된다.
        self.set_state(ElevatorState.DONE)


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
