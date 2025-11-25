#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


# -------------------------------
# 층 라벨 ↔ 인덱스 변환
# 내부 인덱스: 0=B1, 1=1F, 2=2F, 3=3F, 4=4F, 5=5F
# -------------------------------
VALID_FLOOR_INDEX = set(range(0, 6))


def label_to_index(s: str) -> int:
    s = s.strip().upper()
    if s == 'B1':
        return 0
    if s.isdigit():
        n = int(s)
        if 1 <= n <= 5:
            return n
    raise ValueError("유효하지 않은 층입니다. (허용: B1, 1, 2, 3, 4, 5)")


def index_to_label(i: int) -> str:
    if i == 0:
        return 'B1'
    return f'{i}'


class ElevatorFloorNode(Node):
    """
    기존에 시리얼(COM7)로 EBIMU 데이터를 읽어서
    - 0.5초 윈도우마다 z_acc 통계를 내고
    - "올라갑니다 / 내려갑니다 / 올라가지도 내려가지도 않습니다" 상태를 판단한 뒤
    - 연속 상태와 타이머(상승/하강 시간)로 층수를 추정하던 로직을

    ROS 2 IMU 토픽(/ebimu/imu) 구독 기반으로 그대로 옮긴 노드입니다.

    시리얼 부분만 ROS 2 구독으로 바꾸고,
    ACC_Z_THRESHOLD, THRESH_COUNT, WINDOW, 타이머/층수 로직,
    로그 문구(올라갑니다/내려갑니다/타이머 시작/종료/경과 시간 등)는 유지합니다.
    """

    def __init__(self):
        super().__init__('elevator_floor_node')

        # -------------------------------
        # 파라미터 / 설정
        # -------------------------------
        self.declare_parameter('imu_topic', '/ebimu/imu')
        imu_topic = self.get_parameter(
            'imu_topic'
        ).get_parameter_value().string_value

        # 원본 코드에서 사용하던 상수들 (건들지 않음)
        self.ACC_Z_THRESHOLD = 0.02   # z_acc 임계값
        self.THRESH_COUNT = 30        # 0.5초 동안 임계값 이상 카운트 개수
        self.WINDOW = 0.5             # 0.5초마다 판정

        # 층 정보 (내부 인덱스: 0=B1, 1=1F, ... 5=5F)
        self.current_floor_idx = 1    # 기본 1F (start_session에서 덮어씀)
        self.dest_floor_idx = None    # 목적 층 인덱스

        # 상태 기록
        self.consecutive_up = 0
        self.consecutive_down = 0

        # 타이머 관련
        self.timer_running = False
        self.timer_start_time = 0.0
        self.elevator_mode = None   # "UP" 또는 "DOWN"

        # 윈도우/카운터
        self.window_start = None
        self.count_up = 0
        self.count_down = 0
        self.count_still = 0

        # 동작 플래그 (s 눌러서 시작)
        self.active = False

        # -------------------------------
        # QoS 설정 (IMU 퍼블리셔와 맞추기: BEST_EFFORT)
        # -------------------------------
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # ROS 입출력
        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.on_imu,
            imu_qos
        )
        # 퍼블리시는 기본 QoS로 충분
        # Int32 값은 "내부 인덱스" (0=B1, 1=1F, ..., 5=5F)
        self.pub_floor = self.create_publisher(Int32, 'current_floor', 10)

        self.get_logger().info(
            f"[ElevatorFloorNode] IMU topic='{imu_topic}', "
            f"ACC_Z_THRESHOLD={self.ACC_Z_THRESHOLD}, "
            f"THRESH_COUNT={self.THRESH_COUNT}, WINDOW={self.WINDOW}"
        )

    # -------------------------------
    # 세션 시작 (시작/목적 층 설정 후 호출)
    # -------------------------------
    def start_session(self, start_floor_idx: int, dest_floor_idx: int):
        self.current_floor_idx = start_floor_idx
        self.dest_floor_idx = dest_floor_idx

        # 상태/타이머 초기화
        self.consecutive_up = 0
        self.consecutive_down = 0
        self.timer_running = False
        self.timer_start_time = 0.0
        self.elevator_mode = None

        self.window_start = None
        self.count_up = 0
        self.count_down = 0
        self.count_still = 0

        self.active = True

        start_label = index_to_label(self.current_floor_idx)
        dest_label = index_to_label(self.dest_floor_idx)

        self.get_logger().info(
            f"[START] 엘리베이터 층수 추정을 시작합니다. 시작 층 = {start_label}, 목적 층 = {dest_label}"
        )
        print(f"[START] 시작 층: {start_label}, 목적 층: {dest_label}")

    # -------------------------------
    # IMU 콜백
    # -------------------------------
    def on_imu(self, msg: Imu):
        # s를 눌러 세션 시작 전에는 아무 것도 하지 않음
        if not self.active:
            return

        # z축 가속도 (원본 코드의 z_acc에 해당)
        # 원본 로직을 그대로 가져오기 위해 추가 처리(필터, 단위 변환 등)는 하지 않음
        z_acc = float(msg.linear_acceleration.z)

        now = self.get_clock().now().nanoseconds * 1e-9

        # 윈도우 시작 초기화
        if self.window_start is None:
            self.window_start = now

        # ------------------------------------
        # 0.5초 동안 카운팅 (원본 코드 그대로)
        # ------------------------------------
        if z_acc >= self.ACC_Z_THRESHOLD:
            self.count_up += 1
        elif z_acc <= -self.ACC_Z_THRESHOLD:
            self.count_down += 1
        else:
            self.count_still += 1

        # 0.5초 경과 시 판정/타이머/층수 업데이트
        if (now - self.window_start) >= self.WINDOW:
            # ------------------------------
            # 0.5초 상태 판단 (원본 로직)
            # ------------------------------
            if self.count_down >= self.THRESH_COUNT:
                state = "가속도 방향 ↓"
            elif self.count_up >= self.THRESH_COUNT:
                state = "가속도 방향 ↑"
            elif self.count_still >= self.THRESH_COUNT:
                state = "z축 등속운동 진행"
            else:
                state = "z축 등속운동 진행"

            # ------------------------------
            # 연속 상태 세기 (원본 로직)
            # ------------------------------
            if state == "올라갑니다":
                self.consecutive_up += 1
                self.consecutive_down = 0
            elif state == "내려갑니다":
                self.consecutive_down += 1
                self.consecutive_up = 0
            else:
                self.consecutive_up = 0
                self.consecutive_down = 0

            # ------------------------------
            # 1) 타이머 종료 먼저 처리
            # ------------------------------
            if self.timer_running:
                # 올라가던 중인데 내려감이 3번 연속 → 상승 타이머 종료
                if self.elevator_mode == "UP" and self.consecutive_down >= 3:
                    self.timer_running = False
                    elapsed = now - self.timer_start_time
                    diff = 0
                    if elapsed >= 15.3:
                        diff = 5
                    elif elapsed >= 12.5:
                        diff = 4
                    elif elapsed >= 8.5:
                        diff = 3
                    elif elapsed >= 5.8:
                        diff = 2
                    elif elapsed >= 2.6:
                        diff = 1

                    self.current_floor_idx += diff
                    # 0~5 범위로 클램프 (B1~5F)
                    if self.current_floor_idx < 0:
                        self.current_floor_idx = 0
                    if self.current_floor_idx > 5:
                        self.current_floor_idx = 5

                    # 같은 구간에서 다시 시작 못 하게 초기화
                    self.consecutive_up = 0
                    self.consecutive_down = 0

                    # 타이머 종료 출력 (원본 print 유지 + ROS 로그)
                    label = index_to_label(self.current_floor_idx)
                    msg_txt = (
                        f"타이머 종료: 모드=UP, 경과={elapsed:.2f}s, "
                        f"층변경=+{diff}, 현재층={label}"
                    )
                    print(msg_txt)
                    self.get_logger().info(msg_txt)

                    # 목적층 도달 여부 체크
                    self._check_arrival()

                # 내려가던 중인데 올라감이 3번 연속 → 하강 타이머 종료
                elif self.elevator_mode == "DOWN" and self.consecutive_up >= 3:
                    self.timer_running = False
                    elapsed = now - self.timer_start_time
                    diff = 0
                    if elapsed >= 15.3:
                        diff = 5
                    elif elapsed >= 12.5:
                        diff = 4
                    elif elapsed >= 8.5:
                        diff = 3
                    elif elapsed >= 5.8:
                        diff = 2
                    elif elapsed >= 2.6:
                        diff = 1

                    self.current_floor_idx -= diff
                    # 0~5 범위로 클램프
                    if self.current_floor_idx < 0:
                        self.current_floor_idx = 0
                    if self.current_floor_idx > 5:
                        self.current_floor_idx = 5

                    # 같은 구간에서 다시 시작 못 하게 초기화
                    self.consecutive_up = 0
                    self.consecutive_down = 0

                    # 타이머 종료 출력
                    label = index_to_label(self.current_floor_idx)
                    msg_txt = (
                        f"타이머 종료: 모드=DOWN, 경과={elapsed:.2f}s, "
                        f"층변경=-{diff}, 현재층={label}"
                    )
                    print(msg_txt)
                    self.get_logger().info(msg_txt)

                    # 목적층 도달 여부 체크
                    self._check_arrival()

            # ------------------------------
            # 2) 타이머가 꺼져 있는 경우에만 시작
            #    (종료 처리 후에 체크)  ─ 원본 로직
            # ------------------------------
            if not self.timer_running:
                if state == "올라갑니다" and self.consecutive_up >= 3:
                    self.elevator_mode = "UP"
                    self.timer_running = True
                    self.timer_start_time = now
                    msg_txt = (
                        f"타이머 시작: 모드=UP, "
                        f"시작시간={time.strftime('%H:%M:%S', time.localtime(self.timer_start_time))}"
                    )
                    print(msg_txt)
                    self.get_logger().info(msg_txt)

                elif state == "내려갑니다" and self.consecutive_down >= 3:
                    self.elevator_mode = "DOWN"
                    self.timer_running = True
                    self.timer_start_time = now
                    msg_txt = (
                        f"타이머 시작: 모드=DOWN, "
                        f"시작시간={time.strftime('%H:%M:%S', time.localtime(self.timer_start_time))}"
                    )
                    print(msg_txt)
                    self.get_logger().info(msg_txt)

            # ------------------------------
            # 상태 출력 (원본 로직 유지 + 층 라벨 반영)
            # ------------------------------
            label = index_to_label(self.current_floor_idx)
            msg_txt = f"[현재 층 : {label}], [{state}]"
            print(msg_txt)
            self.get_logger().info(msg_txt)

            # ------------------------------
            # 층 정보 퍼블리시 (내부 인덱스 0~5)
            # ------------------------------
            floor_msg = Int32()
            floor_msg.data = int(self.current_floor_idx)
            self.pub_floor.publish(floor_msg)

            # ------------------------------
            # 윈도우/카운터 초기화
            # ------------------------------
            self.window_start = now
            self.count_down = 0
            self.count_up = 0
            self.count_still = 0

    # -------------------------------
    # 목적층 도달 체크
    # -------------------------------
    def _check_arrival(self):
        if self.dest_floor_idx is None:
            return
        if self.current_floor_idx == self.dest_floor_idx:
            label = index_to_label(self.dest_floor_idx)
            msg_txt = f"목적층인 {label}층에 도달했습니다."
            print(msg_txt)
            self.get_logger().info(msg_txt)
            # 필요하면 여기서 self.active = False 로 추정 멈출 수 있음
            # self.active = False


# -------------------------------
# 층 입력 유틸 (B1, 1~5)
# -------------------------------
def prompt_floor_label(prompt_text: str) -> int:
    while True:
        raw = input(prompt_text).strip()
        try:
            idx = label_to_index(raw)
            return idx
        except Exception as e:
            print("입력 오류:", e)


# -------------------------------
# 메인
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ElevatorFloorNode()

    try:
        # 시작/목적 층수 입력 (B1, 1, 2, 3, 4, 5)
        start_idx = prompt_floor_label("시작 층을 입력하세요 [B1, 1~5]: ")
        dest_idx = prompt_floor_label("목적 층을 입력하세요 [B1, 1~5]: ")

        # s 입력 시 시작
        go = input("start now? (s 입력 시 시작): ").strip().lower()
        if go != 's':
            print("시작이 취소되었습니다. 노드를 종료합니다.")
            rclpy.shutdown()
            return

        node.start_session(start_idx, dest_idx)

        # ROS 스핀 (Ctrl+C로 종료)
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("KeyboardInterrupt: 종료합니다.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
