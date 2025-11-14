#!/usr/bin/env python3

#=====================================================#
# 기능: MDrobot MDH100, MD200T Motor Driver
# - 모터굴리기..!
# - 일단 귀찮으니 대충 적어놓고,,
#
# TODO : 할거 많음..
# 최종 수정일: 2025.11.09
# 편집자: 김형진
#=====================================================#
import time
import numpy as np
import serial
from serial_comm.helper import Helper

'''
< 송신 데이터 포맷 (PC → MotorDriver) >
┌──────┬──────┬────────────┬────────┬──────┬──────────┬──────────────┬──────────────┬──────┐
│ RMID │ TMID │ Driver ID  │  PID   │ LEN  │   DATA   │     ...      │     ...      │ CHK  │
├──────┼──────┼────────────┼────────┼──────┼──────────┼──────────────┼──────────────┼──────┤
│ 183  │ 184  │     1      │ 0~255  │  N   │  값(0~255)│  ...payload │  ...payload │ 계산값 │
└──────┴──────┴────────────┴────────┴──────┴──────────┴──────────────┴──────────────┴──────┘

설명:
 - RMID : 송신자 ID (Master, PC)
 - TMID : 수신자 ID (Slave, Motor Driver)
 - Driver ID : 제어 대상 드라이버 번호
 - PID : 명령 코드 (예: 속도 제어, 상태 요청 등)
 - LEN : DATA 구간의 바이트 길이
 - DATA : 명령 데이터 (예: 목표 속도, 방향 등)
 - CHK : 체크섬 (모든 바이트의 합을 1바이트로 더한 뒤 보수 + 1)

예시:
 [183, 184, 1, 1, 4, 0, 100, 0, 200, 58]
 RMID | TMID | ID | PID | LEN | DATA0 | DATA1 | DATA2 | DATA3 | CHK


< 수신 데이터 포맷 (MotorDriver → PC) >
┌──────┬──────┬────────────┬────────┬──────┬──────────┬──────────────┬──────────────┬──────┐
│ RMID │ TMID │ Driver ID  │  PID   │ LEN  │   DATA   │     ...      │     ...      │ CHK  │
├──────┼──────┼────────────┼────────┼──────┼──────────┼──────────────┼──────────────┼──────┤
│ 184  │ 183  │     1      │ 0~255  │  N   │  값(0~255)│  ...payload │  ...payload │ 계산값 │
└──────┴──────┴────────────┴────────┴──────┴──────────┴──────────────┴──────────────┴──────┘

설명:
 - RMID : 송신자 ID (Slave, Motor Driver)
 - TMID : 수신자 ID (Master, PC)
 - Driver ID : 응답한 드라이버 번호
 - PID : 응답 코드 (예: 상태 값, 에러 등)
 - LEN : DATA 구간의 바이트 길이
 - DATA : 상태 데이터 (예: 속도, 전류, 엔코더 등)
 - CHK : 체크섬 (모든 바이트의 합을 1바이트로 더한 뒤 보수 + 1)

예시:
 [184, 183, 1, 129, 6, 0, 30, 0, 2, 0, 60, 177]
 RMID | TMID | ID | PID | LEN | DATA0 | DATA1 | DATA2 | DATA3 | DATA4 | DATA5 | CHK
'''


class MotorDriver:
    def __init__(self, port="/dev/ttyUSB_RS485", baudrate=115200, timeout=0.1):
        # RS485 직렬 포트 오픈
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

        # Master / Slave ID
        self.RMID = 183   # PC(Master)
        self.TMID = 184   # 모터드라이버(Slave)
        self.driverID = 1

        # 엔코더 해상도 (원래 코드 그대로 유지)
        self.encoder_gain = 250

    # ----------------- 내부 공통 유틸 ----------------- #
    def _build_frame(self, pid: int, payload: np.ndarray) -> np.ndarray:
        """
        프로토콜:
        [ RMID, TMID, driverID, PID, LEN, DATA..., CHK ]
        CHK = ~sum(all bytes except CHK) + 1  (uint8)
        """
        if payload is None:
            payload = np.array([], dtype=np.uint8)
        payload = payload.astype(np.uint8)

        header = np.array(
            [self.RMID, self.TMID, self.driverID, pid, len(payload)],
            dtype=np.uint8
        )
        body = np.concatenate((header, payload))

        byChkSend = np.array(np.sum(body, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)

        frame = np.concatenate((body, [chk]))
        return frame

    def _send_frame(self, frame: np.ndarray):
        """프레임 전송 + 디버그 출력"""
        frame = frame.astype(np.uint8)
        print(f"TX ({len(frame)} bytes):", frame.tolist())
        self.ser.write(frame.tobytes())

    # ----------------- Recv Method ----------------- #
    def recv_motor_state(self):
        """
        모터드라이버가 주기적으로 쏴주는 상태 프레임을 그냥 읽어서 찍어본다.
        정확한 길이를 아직 확정하지 않았으니, 일단 최대 32바이트 받아서 확인.
        """
        readbytes = self.ser.read(32)

        if not readbytes:
            print("[recv_motor_state] no data")
            return

        data = np.frombuffer(readbytes, dtype=np.uint8)
        print(f"RX ({len(data)} bytes):", data.tolist())

        # 아주 간단한 헤더 체크만 해본다
        if len(data) >= 5:
            if data[0] == self.TMID and data[1] == self.RMID:
                print("  -> header OK (TMID/RMID)")
            else:
                print("  -> header mismatch (expected TMID/RMID =",
                      self.TMID, self.RMID, ")")


    # ----------------- Send Methods ----------------- #
    def send_torque_cmd(self, t1: int, t2: int):
        """
        PID 209 : 토크 명령
        형식: [1, t1(2B), 1, t2(2B), 2]
        방향 보정은 필요에 따라 조정 가능 (여기선 원래 코드와 동일하게 사용 X)
        """
        pid = 209
        datanum = 7

        t1_ = np.array(
            Helper.int16_to_uint8arr(np.array(int(t1), dtype=np.int16)),
            dtype=np.uint8
        )
        t2_ = np.array(
            Helper.int16_to_uint8arr(np.array(int(t2), dtype=np.int16)),
            dtype=np.uint8
        )

        payload = np.array([1], dtype=np.uint8)
        payload = np.append(payload, t1_)
        payload = np.append(payload, np.array([1], dtype=np.uint8))
        payload = np.append(payload, t2_)
        payload = np.append(payload, np.array([2], dtype=np.uint8))

        frame = self._build_frame(pid, payload)
        self._send_frame(frame)

    def send_position_cmd(self, p1: int, p2: int, mv1: int, mv2: int):
        """
        PID 206 : 위치 제어 명령
        형식: [1, p1(4B), mv1(2B), 1, p2(4B), mv2(2B), 2]
        여기서는 원래 사용하던 방향 보정을 적용:
          - 왼쪽(p1)은 부호 반전해서 보냄
          - 오른쪽(p2)은 그대로
        """
        pid = 206
        datanum = 15

        p1_ = np.array(
            Helper.int32_to_uint8arr(np.array(-int(p1), dtype=np.int32)),
            dtype=np.uint8
        )
        mv1_ = np.array(
            Helper.int16_to_uint8arr(np.array(int(mv1), dtype=np.int16)),
            dtype=np.uint8
        )
        p2_ = np.array(
            Helper.int32_to_uint8arr(np.array(int(p2), dtype=np.int32)),
            dtype=np.uint8
        )
        mv2_ = np.array(
            Helper.int16_to_uint8arr(np.array(int(mv2), dtype=np.int16)),
            dtype=np.uint8
        )

        payload = np.array([1], dtype=np.uint8)
        payload = np.append(payload, p1_)
        payload = np.append(payload, mv1_)
        payload = np.append(payload, np.array([1], dtype=np.uint8))
        payload = np.append(payload, p2_)
        payload = np.append(payload, mv2_)
        payload = np.append(payload, np.array([2], dtype=np.uint8))

        frame = self._build_frame(pid, payload)
        self._send_frame(frame)

    def send_vel_cmd(self, v1: int, v2: int):
        """
        PID 207 : 속도 제어 명령
        형식: [1, v1(2B), 1, v2(2B), 2]
        방향 보정:
          - 왼쪽(v1)은 부호 반전해서 보냄
          - 오른쪽(v2)은 그대로
        """
        pid = 207
        datanum = 7

        v1_ = np.array(
            Helper.int16_to_uint8arr(np.array(-int(v1), dtype=np.int16)),
            dtype=np.uint8
        )
        v2_ = np.array(
            Helper.int16_to_uint8arr(np.array(-int(v2), dtype=np.int16)),
            dtype=np.uint8
        )

        payload = np.array([1], dtype=np.uint8)
        payload = np.append(payload, v1_)
        payload = np.append(payload, np.array([1], dtype=np.uint8))
        payload = np.append(payload, v2_)
        payload = np.append(payload, np.array([2], dtype=np.uint8))

        frame = self._build_frame(pid, payload)
        self._send_frame(frame)


# ----------------- 간단 테스트용 main ----------------- #
if __name__ == "__main__":
    md = MotorDriver()

    # 1) 정지 명령 한 번 쏘고
    md.send_vel_cmd(0, 0)

    # 2) 잠깐 대기 후 들어오는 상태 패킷을 한 번 읽어본다
    time.sleep(0.1)
    md.recv_motor_state()