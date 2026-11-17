#!/usr/bin/env python3
"""
gripper.py — RG2 그리퍼 (Modbus TCP) 공용 모듈
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 필요한 기능은 관제에게 요청.

11/17 세 파트(탐지·판별·적재)의 그리퍼를 디지털 출력(DO) → Modbus로 통일.
  - DO: 미리 정한 잡기/놓기만 가능, 폭을 못 읽음
  - Modbus: 힘·목표 폭을 명령하고, 폭과 잡힘 여부를 읽음 → 집기 실패 감지, 판별 폭 측정
RG2 클래스는 판별 파트(rg2_sync_grip.py, 11/14~17 실측)의 코드를 옮긴 것이다. 보정값도 그때 실측값.

파트 알고리즘은 이 모듈을 직접 만들지 않는다. robot(DsrIO)이 Goal 동안만 접속을 열어 준다.
  robot.grip_open(width_mm, force_n)   열기
  robot.grip_close(force_n) -> GripResult(grabbed, width_mm, raw_mm)   닫기 + 잡힘 여부·폭
  robot.gripper_width() -> mm          지금 폭 (패드 사이, 보정 반영)
  robot.gripper                        RG2 객체 그대로 (판별 SyncGripper처럼 세밀한 제어가 필요할 때)

주의
  - Compute Box(192.168.1.1:502)는 Modbus 동시 접속이 1개뿐이다.
    OnRobot 드라이버(m0609_rg2_bringup mode:=real의 OnRobotRGControllerServer)가 떠 있으면 접속 실패 → 끄고 쓸 것
      pkill -f OnRobotRGControllerServer
  - 환경변수 BOX_GRIPPER=fake 이면 FakeRG2 (에뮬레이터·fake_run SIM=1). 기본은 modbus.
    BOX_FAKE_GRIP_FAILS=N 이면 FakeRG2가 처음 N번 닫기를 빈손으로 돌려줌 (집기 실패 시험용)
  - 폭 기준: 원시 폭(레지스터 267) = 알루미늄 핑거 사이, 반영 폭(275) = 패드 사이.
    보정 폭 = (원시 − cal_offset) / cal_gain − 패드 두께 합 (패드 장착 실측: 실제 = 원시 − 10.30)
================================================================
"""

import os
import statistics
import struct
import time
from typing import NamedTuple

from box_comm.common import GRIPPER

RAW_MAX_MM = 110.0
CTRL_GRIP_W_OFFSET = 16
SETTLE_TIMEOUT = 1.5
SETTLE_TOL_MM = 0.1


class GripResult(NamedTuple):
    grabbed: bool        # 물체를 잡았는지 (잡힘 비트 + 패드 사이 간격이 empty_width_mm보다 큼)
    width_mm: float      # 보정 폭 (패드 사이 실제 간격) [mm]
    raw_mm: float        # 원시 폭 (알루미늄 핑거 사이) [mm]


def _call(fn, *args, **kw):
    """pymodbus 버전에 따라 slave= / device_id= 이름이 달라서 둘 다 시도."""
    try:
        return fn(*args, slave=GRIPPER['slave'], **kw)
    except TypeError:
        return fn(*args, device_id=GRIPPER['slave'], **kw)


class RG2:
    """Compute Box에 Modbus TCP로 직접 붙는 클래스. 접속은 1개뿐이니 쓰고 나면 close()."""

    def __init__(self, ip=None, port=None):
        from pymodbus.client import ModbusTcpClient          # apt: python3-pymodbus
        ip = ip or GRIPPER['ip']
        port = port or GRIPPER['port']
        self.c = ModbusTcpClient(host=ip, port=port, timeout=1)
        if not self.c.connect():
            raise ConnectionError(f'RG2 Modbus 연결 실패 {ip}:{port} '
                                  f'(OnRobot 드라이버가 떠 있으면 끌 것: pkill -f OnRobotRGControllerServer)')
        st = self.status()
        self.pad_total = st['width_raw_mm'] - st['width_mm']     # 양쪽 패드 두께 합

    def close(self):
        self.c.close()

    # ---- 판별 원본 그대로 (rg2_sync_grip.py) ----
    def status(self):
        r = _call(self.c.read_holding_registers, 258, count=18)
        if r.isError():
            raise IOError(f'그리퍼 상태 읽기 실패: {r}')
        v = r.registers
        word = v[10]
        return {
            'offset_mm': struct.unpack('<h', struct.pack('<H', v[0]))[0] / 10,
            'width_raw_mm': v[9] / 10,    # 267: 오프셋 미반영 (알루미늄 핑거 사이)
            'width_mm': v[17] / 10,       # 275: 오프셋 반영 (패드 사이)
            'busy': word & 1,
            'grip': (word >> 1) & 1,
            'safety': [(word >> b) & 1 for b in (2, 3, 4, 5, 6)],
        }

    def command(self, force_n, pad_width_mm):
        """목표 힘 [N], 목표 '패드 간격' [mm]으로 움직인다."""
        f = max(0, min(400, int(round(force_n * 10))))
        w = max(0, min(int(RAW_MAX_MM * 10), int(round(pad_width_mm * 10))))
        r = _call(self.c.write_registers, 0, [f, w, CTRL_GRIP_W_OFFSET])
        if r.isError():
            raise IOError(f'그리퍼 명령 쓰기 실패: {r}')

    def wait_idle(self, timeout=5.0):
        time.sleep(0.15)  # 명령 직후 busy가 올라올 시간
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not self.status()['busy']:
                return True
            time.sleep(0.05)
        return False

    def open(self, force_n, pad_width_mm, tol=3.0, timeout=3.0):
        """패드 간격 pad_width_mm로 열고, 실제로 그 폭이 될 때까지 기다린다. 원시 폭을 돌려준다.
        최대 폭과 똑같이 명령하면 무시되는 일이 있어 최대보다 0.5mm 작게 보낸다."""
        target = min(pad_width_mm, RAW_MAX_MM - self.pad_total - 0.5)
        self.command(force_n, target)
        self.wait_idle()
        t0, last = time.time(), None
        while time.time() - t0 < timeout:
            st = self.status()
            w = st['width_mm']
            if abs(w - target) <= tol and last is not None and abs(w - last) < 0.2 and not st['busy']:
                return st['width_raw_mm']
            last = w
            time.sleep(0.1)
        st = self.status()
        raise RuntimeError(f'그리퍼가 패드 간격 {target:.1f} mm로 열리지 않음 (현재 {st["width_mm"]:.1f}, '
                           f'잡힘 {st["grip"]}, 안전 {st["safety"]})')

    def grip_until_done(self, force_n, timeout=5.0):
        """닫기(목표 폭 0) 명령을 보내고 동작이 끝날 때까지 기다린다. 끝났으면 True."""
        self.command(force_n, 0)
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.status()
            if not st['busy'] and time.time() - t0 > 0.15:
                return True
        return False

    def settled_width(self, n_samples=10, interval=0.05):
        """폭이 안정될 때까지 기다린 뒤 n회 읽어 (원시 폭 중앙값, 편차) 반환."""
        t0, recent = time.time(), []
        while time.time() - t0 < SETTLE_TIMEOUT:
            recent = (recent + [self.status()['width_raw_mm']])[-3:]
            if len(recent) == 3 and max(recent) - min(recent) <= SETTLE_TOL_MM:
                break
            time.sleep(interval)
        raw = []
        for _ in range(n_samples):
            raw.append(self.status()['width_raw_mm'])
            time.sleep(interval)
        return statistics.median(raw), max(raw) - min(raw)

    # ---- 공용 (관제 추가) ----
    def calibrated(self, raw_mm):
        """원시 폭 → 패드 사이 실제 간격 [mm]"""
        return max(0.0, (raw_mm - GRIPPER['cal_offset_mm']) / GRIPPER['cal_gain'] - self.pad_total)

    def grip(self, force_n, timeout=None) -> GripResult:
        """닫고(목표 폭 0) 다 닫힐 때까지 기다린 뒤 잡힘 여부·폭을 돌려준다. 예외는 통신 오류일 때만."""
        if not self.grip_until_done(force_n, timeout or GRIPPER['timeout_s']):
            raise RuntimeError('그리퍼 닫기가 시간 안에 끝나지 않음')
        raw, _ = self.settled_width(n_samples=3)
        st = self.status()
        width = self.calibrated(raw)
        grabbed = bool(st['grip']) and width > GRIPPER['empty_width_mm']
        return GripResult(grabbed, width, raw)

    def width(self) -> float:
        return self.calibrated(self.status()['width_raw_mm'])


class FakeRG2:
    """에뮬레이터용 가짜 그리퍼 (RG2와 같은 함수). 닫으면 fake_raw_mm 폭을 잡았다고 돌려준다.
    BOX_FAKE_GRIP_FAILS=N → 처음 N번 닫기는 빈손 (집기 실패 시험)."""

    _fails_left = None                  # 프로세스 전체에서 한 번만 환경변수를 읽음

    def __init__(self, fake_raw_mm=50.0):
        self.pad_total = 10.0
        self.fake_raw = fake_raw_mm
        self._raw = RAW_MAX_MM - 0.5
        self._grip = 0
        if FakeRG2._fails_left is None:
            FakeRG2._fails_left = int(os.environ.get('BOX_FAKE_GRIP_FAILS', '0') or 0)

    def close(self):
        pass

    def status(self):
        return {'offset_mm': self.pad_total / 2, 'width_raw_mm': self._raw,
                'width_mm': self._raw - self.pad_total, 'busy': 0, 'grip': self._grip,
                'safety': [0, 0, 0, 0, 0]}

    def command(self, force_n, pad_width_mm):
        if pad_width_mm <= 0:
            self._close()
        else:
            self._raw, self._grip = min(pad_width_mm + self.pad_total, RAW_MAX_MM - 0.5), 0

    def _close(self):
        if FakeRG2._fails_left > 0:
            FakeRG2._fails_left -= 1
            self._raw, self._grip = self.pad_total + 0.3, 1   # 빈손: 패드끼리 닿음 (실물처럼 원시 폭≈패드 두께, 잡힘 비트도 켜짐)
        else:
            self._raw, self._grip = self.fake_raw, 1
        time.sleep(0.3)

    def wait_idle(self, timeout=5.0):
        return True

    def open(self, force_n, pad_width_mm, tol=3.0, timeout=3.0):
        self.command(force_n, min(pad_width_mm, RAW_MAX_MM - self.pad_total - 0.5))
        time.sleep(0.3)
        return self._raw

    def grip_until_done(self, force_n, timeout=5.0):
        self._close()
        return True

    def settled_width(self, n_samples=10, interval=0.05):
        return self._raw, 0.0

    def calibrated(self, raw_mm):
        return max(0.0, (raw_mm - GRIPPER['cal_offset_mm']) / GRIPPER['cal_gain'] - self.pad_total)

    def grip(self, force_n, timeout=None) -> GripResult:
        self._close()
        width = self.calibrated(self._raw)
        grabbed = bool(self._grip) and width > GRIPPER['empty_width_mm']
        return GripResult(grabbed, width, self._raw)

    def width(self) -> float:
        return self.calibrated(self._raw)


def gripper_mode() -> str:
    """'modbus'(기본) 또는 'fake' — 환경변수 BOX_GRIPPER"""
    mode = os.environ.get('BOX_GRIPPER', 'modbus').strip().lower()
    return 'fake' if mode == 'fake' else 'modbus'


def make_gripper():
    """설정에 맞는 그리퍼 객체 (접속까지). 실패하면 ConnectionError"""
    return FakeRG2() if gripper_mode() == 'fake' else RG2()
