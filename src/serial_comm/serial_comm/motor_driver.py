import time
import numpy as np
import serial
from serial_comm.helper import Helper


'''
test

이 부분에 송신 수신 데이터 프레임 상세히 명시


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

    # while True:
    #     md.send_vel_cmd(0, 0)
    #     time.sleep(0.1)
    #     md.recv_motor_state()
    #     time.sleep(0.1)
