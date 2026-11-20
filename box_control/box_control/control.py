#!/usr/bin/env python3
"""
control.py — 관제 메인 (작업 시퀀스 · 재고 반영 · 중단 처리)
================================================================
구성
  control.py            : 이 파일. 상태 흐름과 판단만 한다 (ROS 호출은 self.io에 맡김)
  control_interface.py  : 통신 계층 (Action·Service·이벤트 로그·정지 감시). start(Control)로 실행
  preflight.py          : 시작 전 점검 (PREFLIGHT 단계에서 run_all 호출)
  recovery.py           : 관제 복구 동작 (관제가 직접 로봇을 움직임 — 지금은 측정존 비우기)
  common.py             : 공통 정의 (에러 코드, 상태, 좌표 기본값, 적재 규칙 ...)

상주 모드 (11/18, 기본) — 관제를 켜 두면 READY에서 시작 명령을 기다린다
  READY ──(UI 시작: DB 노드 → /job_command start, 또는 터미널 s)──▶ 작업 한 번 ──▶ READY
  - 결과(DONE/ERROR/STOPPED)는 DB status·reason에 남고, 단계(stage)만 READY로 돌아온다.
  - READY에서도 로봇 상태는 계속 조회·발행한다 (이상이 있어도 정지할 작업이 없으니 표시만).
  - 카운트다운 중 취소: UI 취소(/job_command cancel) 또는 터미널 q → 작업 안 하고 READY.
  - 카운트다운 생략(바로 시작): 현장 관리자만 — 관리자 GUI(/job_command start_now) 또는 터미널 s.
    웹 UI(source='ui')에서 온 start_now는 거절한다. 누가 했는지 이벤트 로그(JOB_COMMAND)에 남는다.
  - READY에서 터미널 q → 관제 종료.
  - -p auto_start:=true → 예전처럼 바로 작업 한 번 하고 종료 (fake_run.sh·시험용).

작업 한 번의 흐름 (인터페이스 문서 2.5 관제 상태)
  BEFORE → GetJobData → PREFLIGHT(움직임 없는 사전 점검) → STANDBY(시작 대기)
  → RUNNING → START_SETTING(그리퍼 열기·HOME 이동·닫기)
  → INVENTORY_CHECK(A/B/C/D 시작 수량 측정)
  → [SCANNING → SCAN_DONE → JUDGING → JUDGE_DONE → PLACING → PLACE_DONE
     → UpdateInventory] 반복
  → /scan found=False → DONE
  중단: RUNNING 전 실패 → cancel_start (DB status BEFORE + 취소 이유)
        RUNNING 후 실패 → abort_job   (DB에 ERROR 또는 STOPPED + 이유)

규칙
  - "-ING" 상태는 Goal 보내기 직전, "_DONE" 상태는 Result 받은 직후에 바꾼다.
  - 단계 상태는 관제 내부에만 둔다 (DB에는 BEFORE/RUNNING/DONE만).
  - 상태 값은 영어(State), 로그에는 한글 표시를 함께 붙인다.
  - 고정 좌표(HOME, 측정존)는 DB가 기준이다. GetJobData로 받은 값을
    모든 Action Goal(/inventory_check, /scan, /judge, /place)에 실어 보내고, 파트는 그 값으로 움직인다.
  - box_id는 '<작업 시작 시각>_<순번>' (예: 20261114_130512_001).
    관제를 다시 실행해도 이전 작업의 box_id와 겹치지 않는다.
  - 재고 검증 (v0.4): 적재는 실제로 놓은 위치로 잰 result_class 수량(count)만 돌려준다.
    (단·층·z는 적재 내부 정보라 관제는 모른다)
    관제는 기대값(적재 전 재고의 result_class 수량 + 1)과 비교해서 기록하고,
    DB에는 항상 '측정값'을 반영한다 (실제 적재 상태가 기준). 다르면 경고 로그를 남기고
    작업을 계속한다. 다른 클래스 수량은 그대로 둔다. (불일치 기록을 DB에 남길지는 DB/UI와 협의 중)
    단, 수량이 1보다 작게 오면(적재가 수량을 안 채움) 반영하지 않는다 → 다음 같은 클래스 적재 때 바로잡힘.
  - 시작 재고: DB 응답의 재고값은 사용하지 않는다. 시작 대기 후 A~D 구역을 측정하고,
    U는 0으로 시작한다. 적재 결과에서 실제 수량을 계속 반영하며, 구역 6개가 찬 뒤의 물품은 U로 보낸다.
  - 미분류 U (v0.5): 판별이 'U'를 내면 적재에 'U'로 보낸다. 적재는 미분류 위치에 놓기만 하고
    수량을 재지 않는다(count 0). 관제가 적재 성공 후 재고 U = 이전 + 1로 집계한다 (검증 없음).
  - 집기 실패 (11/17): 파트가 GRIP_RETRY번 다시 집어 보고 포기하면 GRASP_FAILED로 Goal을 끝낸다.
    관제는 작업을 끝내지 않는다. 판별·적재였으면 CLEARING(측정존 비우기, recovery.py — 관제가 로봇을 움직임),
    탐지였으면 바로, BOX_RETRY 경고를 남긴 뒤 공간 스캔부터 다시 한다 (재고·박스 번호 그대로).
    연속 RECOVER_LIMIT번을 넘으면 RETRY_LIMIT로 종료. 박스 하나를 적재까지 끝내면 연속 횟수는 0으로.
    측정존을 비우지 못하면(RECOVERY_FAILED) 측정존 사용 불가 → 바로 작업 종료.
    11/18 임시: 밀어내기 경로(common.RECOVERY push_path)가 없으면 비우기를 fake로 넘기고(WARN 로그) 바로 재스캔
    → 스캔하는 동안 사람이 측정존을 치운다.
  - 진행 추적: 중단 시 해당 파트의 마지막 Feedback stage와 경과 시간을 함께 기록한다.
  - 작업 중 중단 (abort_job): DB status를 ERROR(공정 오류) 또는 STOPPED(로봇 정지)로 바꾸고
    이유를 함께 기록한다 (예: 'ERROR : Fail to pick').
    abort_job 자체는 로봇에 정지 명령을 보내지 않는다. 로봇 정지(move_stop)와 파트 Goal cancel은
    control_interface의 request_stop()이 맡는다 (비상정지 요청, 로봇 상태 이상, 진행 없음).
    정지 요청이 있었으면 파트가 돌려준 CANCELED 대신 정지 원인 코드를 DB에 남긴다.
  - 이벤트 로그: /event_log 토픽으로 보내기만 한다 (DB가 저장·UI 표시).
    관제 이벤트(JOB_START / INVENTORY_CHECK_DONE / BOX_DONE / COUNT_MISMATCH /
    JOB_ERROR / JOB_STOPPED / JOB_CANCELED / JOB_DONE)와 파트가 report.log()로 보낸 것
    (PART_LOG, control_interface가 자동으로 넘김).

사전 점검 (PREFLIGHT) — box_control/preflight.py
  분류 기준, 고정 좌표, 적재 구역, 로봇 상태를 점검한다.
  하나라도 FAIL이면 작업을 시작하지 않는다. WARN은 기록만 하고 진행한다.

정지 감시 (control_interface, 파라미터)
  robot_monitor        : dsr(기본, 실제·에뮬레이터) / fake(테스트) / off(bringup 없이 테스트할 때만)
  progress_timeout_sec : 파트 Feedback이 이 시간 동안 없으면 정지 (0 = 끔, 기본)
  감시는 RUNNING 동안만 켠다 (arm_monitor). 비상정지 요청(/emergency_stop)은 언제든 동작.

시작 대기 (STANDBY)
  GetJobData 후 start_delay_sec 동안 카운트다운한다 (기본: 상주 모드 600초 = 10분, auto_start 10초).
  작업공간에 사람·파손 위험 물품이 없는지 확인하는 시간이다.
    q / UI 취소 : 작업 시작 취소 (상주 모드면 READY로, auto_start면 관제 종료)
    s : 바로 시작
    입력 없음 : 시간이 지나면 자동 시작
  시험할 때 짧게: --ros-args -p start_delay_sec:=10.0
  터미널 입력을 받을 수 없는 환경(launch 등)에서는 중단 기능 없이 대기 후 시작한다.

관제 코드 오류 (11/18): run_job에서 예상 못 한 예외 → 로봇 정지 + 파트 cancel → ERROR : Control error → READY
  (관제 프로세스는 죽지 않음. 관제가 정말 죽었다가 다시 켜지면 DB 노드가 RUNNING으로 남은 작업을 ERROR로 정리)
파트 노드 죽음 (11/18): 작업 중 파트 Action 서버가 사라지면 control_interface가 PART_DIED로 정지 → ERROR

TODO(나중에): error_code별 정책 (재시도 / 이 박스 건너뛰기)

실행: ros2 run box_control control                                     (상주: READY에서 시작 대기)
      ros2 run box_control control --ros-args -p auto_start:=true      (바로 한 번 실행하고 종료)
      ros2 run box_control control --ros-args -p start_delay_sec:=3.0
      ros2 run box_control control --ros-args -p progress_timeout_sec:=60.0
      ros2 run box_control control --ros-args -p robot_monitor:=off   (bringup 없이 fake 파트로 테스트)
      ros2 run box_control control --ros-args -p gui:=false           (모니터 GUI 창 없이)
모니터 GUI (11/18): 관제를 실행하면 monitor_gui 창이 같이 뜬다 (별도 프로세스, box_control/monitor_gui.py).
  관제가 끝나도 창은 남는다 (결과 확인용). 관제 상태는 /control_status로 보낸다.
================================================================
"""

import math
import os
import select
import shutil
import subprocess
import sys
import threading
import time
import traceback
from enum import Enum

try:                       # 한 글자 입력(q/s)을 Enter 없이 받기 위함 (Linux)
    import termios
    import tty
except ImportError:        # pragma: no cover
    termios = tty = None

from box_comm.common import (INVENTORY_KEYS, RECOVER_LIMIT, UNCLASSIFIED, default_poses,
                             error_text, status_for)
from box_comm.control_interface import start
from box_control import preflight
from box_control.recovery import Recovery


class State(str, Enum):
    """관제 상태. 값은 영어, label은 로그용 한글."""

    READY = 'READY'             # 대기     (상주 모드: 시작 명령 기다림, 11/18)
    BEFORE = 'BEFORE'           # 작업전   (DB 기록)
    PREFLIGHT = 'PREFLIGHT'     # 사전점검 (관제 내부)
    STANDBY = 'STANDBY'         # 시작대기 (카운트다운, 관제 내부)
    START_SETTING = 'START_SETTING'  # 대기 종료 후 시작 그리퍼 설정
    RUNNING = 'RUNNING'         # 작업중   (DB 기록)
    INVENTORY_CHECK = 'INVENTORY_CHECK'  # 시작 재고 점검
    SCANNING = 'SCANNING'       # 스캔중
    SCAN_DONE = 'SCAN_DONE'     # 스캔완료
    JUDGING = 'JUDGING'         # 판별중
    JUDGE_DONE = 'JUDGE_DONE'   # 판별완료
    PLACING = 'PLACING'         # 적재중
    PLACE_DONE = 'PLACE_DONE'   # 적재완료
    CLEARING = 'CLEARING'       # 측정존비우기 (집기 실패 뒤 관제가 직접 로봇을 움직임, 11/17)
    DONE = 'DONE'               # 작업완료 (DB 기록)
    ERROR = 'ERROR'             # 오류중단 (DB 기록, 공정 오류 — 관제가 판단해서 멈춤)
    STOPPED = 'STOPPED'         # 정지     (DB 기록, 로봇이 물리적으로 멈춤 — 정지 기능 단계에서 연결)

    @property
    def label(self) -> str:
        return _STATE_LABEL[self]


_STATE_LABEL = {
    State.READY: '대기',
    State.BEFORE: '작업전',
    State.PREFLIGHT: '사전점검',
    State.STANDBY: '시작대기',
    State.START_SETTING: '시작 설정',
    State.RUNNING: '작업중',
    State.INVENTORY_CHECK: '시작 재고 점검',
    State.SCANNING: '스캔중',
    State.SCAN_DONE: '스캔완료',
    State.JUDGING: '판별중',
    State.JUDGE_DONE: '판별완료',
    State.PLACING: '적재중',
    State.PLACE_DONE: '적재완료',
    State.CLEARING: '측정존비우기',
    State.DONE: '작업완료',
    State.ERROR: '오류중단',
    State.STOPPED: '정지',
}


# 단계 상태 → 해당 Action 이름 (진행 추적 조회용)
_CANCELABLE = (State.BEFORE, State.PREFLIGHT, State.STANDBY)   # UI 취소를 받는 단계 (작업 시작 전)


class _Keys:
    """터미널 한 글자 입력 (Enter 없이). 터미널이 아니면 tty=False, read()는 기다리기만 한다.
    11/18: 터미널 입력에서 예상 못 한 오류가 나도 관제를 멈추지 않는다 → 키 입력만 끄고 계속
    (error에 이유를 남김, 호출한 쪽이 '주의' 로그를 한 번 남김). 비상정지는 UI·관리자 GUI·펜던트로."""

    def __enter__(self):
        self.fd = None
        self.old = None
        self.error = ''
        try:
            if termios is not None and sys.stdin.isatty():
                fd = sys.stdin.fileno()
                self.old = termios.tcgetattr(fd)
                tty.setcbreak(fd)               # Ctrl+C는 그대로 동작
                self.fd = fd
        except Exception as e:                  # 터미널 설정 실패 → 키 입력 없이
            self.error = f'{type(e).__name__}: {e}'
        return self

    def __exit__(self, *_):
        if self.old is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old)   # 터미널 설정 원래대로
            except Exception:
                pass

    @property
    def tty(self):
        return self.fd is not None

    def read(self, timeout) -> str:
        if self.fd is None:
            time.sleep(timeout)
            return ''
        try:
            ready, _, _ = select.select([self.fd], [], [], timeout)
            return os.read(self.fd, 1).decode(errors='ignore').lower() if ready else ''
        except Exception as e:                  # 키 입력 오류 → 이후 키 입력 끔 (관제는 계속)
            self.error = f'{type(e).__name__}: {e}'
            self.fd = None
            time.sleep(timeout)
            return ''


_STATE_ACTION = {
    State.INVENTORY_CHECK: 'inventory_check',
    State.SCANNING: 'scan', State.SCAN_DONE: 'scan',
    State.JUDGING: 'judge', State.JUDGE_DONE: 'judge',
    State.PLACING: 'place', State.PLACE_DONE: 'place',
}


class Control:
    def __init__(self, io):
        self.io = io                       # ControlInterface (통신 담당)
        self.log = io.get_logger()
        self.state = None
        self.specs = []
        self.poses = default_poses()       # 고정 좌표. GetJobData 후 DB 값으로 무조건 덮어씀 → 모든 Goal에 실어 보냄
        self.inventory = {c: 0 for c in INVENTORY_KEYS}     # A~D + U(미분류)
        self._inventory_available = False
        self._inventory_needs_recheck = False
        self.box_seq = 0
        self.job_id = ''                   # 작업마다 새로 (_new_job). READY에서는 마지막 작업 ID 유지
        self._current_class = ''          # 지금 박스의 판별 결과 (중단 이유 표시용)
        self._current_box = ''            # 지금 박스 box_id (모니터 GUI 표시용)
        self.count_mismatches = []         # 재고 불일치 기록 (box_id, class, 기대 수량, 측정 수량)
        self.grasp_retries = 0             # 집기 실패 → 재스캔 연속 횟수 (박스 하나 적재 완료 시 0)
        self.grasp_retry_total = 0         # 이번 작업 전체 재스캔 횟수 (요약용)
        self.recovery = Recovery(io, self.log)                     # 관제 복구 동작 (측정존 비우기 등)
        self.auto_start = bool(io.declare_parameter('auto_start', False).value)   # True: 바로 한 번 실행 후 종료
        self.inventory_check_enabled = bool(io.declare_parameter('inventory_check_enabled', True).value)
        self.robot_hardware_mode = str(io.declare_parameter('robot_hardware_mode', 'unknown').value)
        delay = float(io.declare_parameter('start_delay_sec', -1.0).value)     # 시작 대기 시간(초)
        self.start_delay = delay if delay >= 0 else (10.0 if self.auto_start else 600.0)  # 기본: 상주 10분, 시험 10초
        self._boot_id = time.strftime('%Y%m%d_%H%M%S')
        # 원격 명령 (/job_command, executor 스레드에서 들어옴 → 신호만 켜고 바로 응답)
        self._cmd_lock = threading.Lock()
        self._start_req = threading.Event()
        self._inventory_changed_req = threading.Event()
        self._cancel_req = threading.Event()
        self._now_req = threading.Event()          # 카운트다운 생략 (현장 관리자)
        self._now_source = ''
        self._start_source = ''
        self._start_requested_at = None
        self._start_requested_at_wall = None
        self._cancel_source = ''
        io.set_job_command_handler(self.on_job_command)

    def apply_db_poses(self, db_poses):
        """관제가 가진 고정 좌표를 DB 값으로 무조건 덮어쓴다 (비교해서 멈추지 않음).
        DB에 좌표가 없으면 None → 사전 점검에서 FAIL (파트에 보낼 좌표가 없으므로)."""
        old = self.poses
        self.poses = db_poses
        if db_poses is None:
            self.log.error('[control] DB에 고정 좌표 없음')
            return
        for key in ('home_j', 'measure_zone'):
            before = old.get(key) if old else None
            after = db_poses.get(key)
            mark = '' if before == after else f'  (덮어씀, 이전 {before})'
            self.log.info(f'[control] 고정 좌표 {key} = {after} ← DB{mark}')

    def set_state(self, state: State):
        self.state = state
        self.log.info(f'[control] ===== {state.value} ({state.label}) =====')
        self.publish_status()

    def publish_status(self, reason=None):
        """모니터 GUI용 상태 발행 (/control_status). 단계 바뀔 때·재고 반영 후·중단 시 호출."""
        self.io.publish_status(
            state=self.state.value if self.state else '', job_id=self.job_id,
            box_seq=self.box_seq, box_id=self._current_box, class_id=self._current_class,
            inventory=self.inventory, mismatches=len(self.count_mismatches),
            retries=self.grasp_retry_total,
            standby_sec=self._standby_remaining() if self.state == State.STANDBY else 0,
            standby_until=(self._start_requested_at_wall + self.start_delay
                           if self.state == State.STANDBY and self._start_requested_at_wall is not None else None),
            inventory_needs_recheck=self._inventory_needs_recheck,
            reason=reason)

    def _standby_remaining(self):
        if self._start_requested_at is None:
            return self.start_delay
        return max(0.0, self.start_delay - (time.monotonic() - self._start_requested_at))

    # ------------------------------------------------------------------
    # 메인 시퀀스
    # ------------------------------------------------------------------
    def run(self):
        if self.auto_start:
            self._start_source = 'terminal'
            self._start_requested_at = time.monotonic()
            self._start_requested_at_wall = time.time()
            self.run_job_safe()
            return
        self.io.set_log_context(job_id=f'idle_{self._boot_id}')    # 작업 밖 로그(알람 등)용 파일
        self.log.info('[control] 상주 모드: READY에서 시작 명령을 기다립니다 '
                      '(UI 시작 버튼 / 터미널 s). 관제 종료는 READY에서 q 또는 Ctrl+C')
        while True:
            # 지난 시작 신호는 READY를 알리기 '전에' 지운다. (알린 뒤에 지우면, READY를 보자마자
            #   UI가 보낸 시작이 '수락' 응답만 받고 지워짐 — 11/18 수정)
            self._start_req.clear()
            self.set_state(State.READY)
            if not self.wait_for_command():
                self.log.info('[control] 터미널 q → 관제 종료')
                return
            self.run_job_safe()

    def run_job_safe(self):
        """작업 한 번. 관제 코드 자체에서 예상 못 한 예외가 나도 관제는 죽지 않는다 (11/18).
        → 로봇 정지(move_stop) + 진행 중인 파트 Goal cancel → DB에 ERROR : Control error — 이유 → READY로."""
        try:
            if self._start_requested_at is None:
                self._start_requested_at = time.monotonic()
                self._start_requested_at_wall = time.time()
            self.run_job()
        except Exception as e:                           # KeyboardInterrupt(Ctrl+C)는 그대로 통과
            self.on_control_error(e)

    def warn_and_continue(self, where, detail):
        """작업을 멈출 일이 아닌 관제 오류 (대기·카운트다운 중 터미널 입력 등) → '주의' 로그만 남기고 계속 (11/18).
        멈춰야 하면 UI·관리자 GUI 비상정지 또는 펜던트."""
        msg = f'주의! {where} 오류 — {detail} → 키 입력 없이 계속 (비상정지는 UI·관리자 GUI·펜던트)'
        self.log.warn(f'[control] {msg}')
        self.io.log_event('WARN', 'CONTROL_WARN', msg)

    def on_control_error(self, exc):
        detail = f'{type(exc).__name__}: {exc}'
        where = f'{self.state.value} ({self.state.label})' if self.state else '-'
        self.log.error(f'[control] !!! 관제 코드 오류 [{where}]: {detail}\n{traceback.format_exc()}')
        try:
            self.io.request_stop('CONTROL_ERROR', detail)   # 로봇 정지 + 파트 cancel (이미 정지 중이면 그 원인 유지)
            self.io.log_event('ERROR', 'CONTROL_ERROR', f'관제 코드 오류 [{where}]: {detail} → 작업 종료')
            if self.state in (None, State.READY, State.BEFORE, State.PREFLIGHT, State.STANDBY):
                self.cancel_start(f'관제 오류 ({detail})')
            else:
                self.abort_job('CONTROL_ERROR')
        except Exception as e2:                          # 마무리도 실패하면 기록만 하고 READY로
            self.log.error(f'[control] 오류 마무리 중 또 오류: {type(e2).__name__}: {e2}')

    def _new_job(self):
        """작업 한 번마다 초기화 (상주 모드에서 작업을 여러 번 하므로)"""
        job_id = time.strftime('%Y%m%d_%H%M%S')
        if job_id == self.job_id:                       # 같은 초에 다시 시작한 경우
            job_id += '_2'
        self.job_id = job_id
        self.box_seq = 0
        self._current_class = ''
        self._current_box = ''
        self.count_mismatches = []
        self.grasp_retries = 0
        self.grasp_retry_total = 0
        self._cancel_req.clear()
        self._now_req.clear()
        self.io.reset_stop()                            # 지난 작업의 정지 요청은 지움 (로봇 상태는 사전 점검이 다시 확인)

    def run_job(self):
        self._new_job()
        # BEFORE: 분류 기준·좌표를 받고, 시작 재고는 뒤에서 실물 점검
        self.set_state(State.BEFORE)
        self.log.info(f'[control] job_id={self.job_id} (시작: {self._start_source or "-"})')
        self.io.set_log_context(job_id=self.job_id)
        self._inventory_available = False
        self._inventory_needs_recheck = False
        self.specs, db_inventory, db_poses, err = self.io.get_job_data()
        if err:
            return self.cancel_start(f'작업 데이터를 받지 못함 ({err})')
        self.inventory = {c: int(db_inventory.get(c, 0)) for c in INVENTORY_KEYS}
        self._inventory_available = True       # 직전 DB 재고를 기준값으로 보존, 실측 후 A~D 교체
        if self.inventory_check_enabled:
            self.log.info(f'[control] specs {len(self.specs)}개; 시작 재고는 로봇 점검으로 확인합니다')
        else:
            self.log.info(f'[control] specs {len(self.specs)}개; 시작 재고 점검 생략, DB 재고 유지: {self.inventory}')
        self.apply_db_poses(db_poses)

        # PREFLIGHT: 시작 전 점검 (box_control/preflight.py)
        self.set_state(State.PREFLIGHT)
        if not preflight.run_all(self):
            return self.cancel_start('사전 점검 실패 (로그의 FAIL 항목 확인)')

        # STANDBY: 작업공간 안전 확인 시간 (q로 취소)
        self.set_state(State.STANDBY)
        started, why = self.wait_for_start()
        if (stop := self.io.stop_reason()):               # 시작 전 비상정지 → 시작하지 않음
            return self.cancel_start(f'정지 요청 ({stop[0]}{" — " + stop[1] if stop[1] else ""})')
        if not started:
            return self.cancel_start(why)

        self.set_state(State.RUNNING)
        self.io.arm_monitor(True)                        # 작업 중에만 로봇 이상·진행 없음을 정지로 처리
        if err := self.io.set_job_status(State.RUNNING.value):
            return self.abort_job(err)
        self.set_state(State.START_SETTING)
        if err := self.recovery.prepare_start(self.poses):
            return self.abort_job(err)
        start_msg = ('그리퍼 시작 설정 완료, 초기 재고 점검 시작' if self.inventory_check_enabled
                     else '그리퍼 시작 설정 완료, 초기 재고 점검 생략')
        self.io.log_event('INFO', 'JOB_START',
                          f'작업 시작 ({self._start_source or "-"}) — {start_msg}', box_id='')

        if self.inventory_check_enabled:
            # 시작 재고는 DB 값이 아니라 실물 점검 결과를 기준으로 한다.
            self.set_state(State.INVENTORY_CHECK)
            measured, err = self.io.inventory_check(self.poses, self.inventory)
            if err:
                self.inventory.update(measured)     # 완료된 분류는 새 실측값, 나머지는 기존 DB 기준값
                self._inventory_needs_recheck = True
                self.publish_status()
                return self.abort_job(err)
            inventory = {c: 0 for c in INVENTORY_KEYS}
            inventory.update(measured)
            self.inventory = inventory
            self._inventory_needs_recheck = False
            self.publish_status()
            self.log.info(f'[control] 시작 재고 점검 완료: {self.inventory}')
            if err := self.io.update_inventory(f'{self.job_id}_initial', self.inventory):
                return self.abort_job(err)
            self.io.log_event('INFO', 'INVENTORY_CHECK_DONE',
                              f'시작 재고 점검 완료, DB 반영: {self.inventory}')
        else:
            self.log.warning(f'[control] 시작 재고 점검 생략 — DB 재고를 기준으로 스캔부터 시작: {self.inventory}')

        while True:
            box_id = f'{self.job_id}_{self.box_seq + 1:03d}'   # 이번 박스 (적재까지 끝나야 확정)
            self._current_box = box_id
            self._current_class = ''
            self.io.set_log_context(box_id=box_id, class_id='')

            # ---- 탐지 ----
            self.set_state(State.SCANNING)
            found, err = self.io.scan(self.poses)
            if err:
                if (err := self.on_part_error(err, 'scan')) is None:
                    continue                   # 집기 실패 → 공간 스캔부터 다시
                return self.abort_job(err)
            if not found:
                self._current_box = ''     # 만들어 둔 다음 번호는 안 씀 → UI에 남지 않게
                break                      # 물품 없음 → 관제가 작업완료로 판단
            self.set_state(State.SCAN_DONE)

            # ---- 판별 ----
            self.set_state(State.JUDGING)
            result_class, err = self.io.judge(self.specs, self.poses)
            if err:
                if (err := self.on_part_error(err, 'judge')) is None:
                    continue
                return self.abort_job(err)
            self.set_state(State.JUDGE_DONE)
            self.log.info(f'[control] 판별 결과: {result_class}')
            self._current_class = result_class
            self.io.set_log_context(class_id=result_class)

            # ---- 적재 ----
            self.set_state(State.PLACING)
            measured, placed_class, err = self.io.place(result_class, self.poses, self.inventory)
            if err:
                if (err := self.on_part_error(err, 'place')) is None:
                    continue
                return self.abort_job(err)
            self.set_state(State.PLACE_DONE)

            # ---- 재고 검증 → 반영 (Place Result를 받은 뒤에만) ----
            self.box_seq += 1
            if result_class == UNCLASSIFIED:
                # 미분류: 적재는 놓기만 함 → 관제가 +1 (측정·비교 없음)
                inventory_new = dict(self.inventory)
                inventory_new[UNCLASSIFIED] = self.inventory.get(UNCLASSIFIED, 0) + 1
                done_msg = f'미분류 투하 완료 → 미분류 {inventory_new[UNCLASSIFIED]}개 (관제 집계)'
            elif placed_class == UNCLASSIFIED:
                inventory_new, done_msg = self.zone_full(box_id, result_class, measured)
            else:
                self.log.info(f'[control] 적재 측정 수량: {result_class} {measured}개')
                inventory_new = self.verify_inventory(box_id, result_class, measured)
                done_msg = f'{result_class} 적재 완료 → {result_class} {inventory_new[result_class]}개'
            self.inventory = inventory_new
            self.publish_status()                  # 재고 바뀜 → GUI 갱신
            self.grasp_retries = 0                 # 박스 하나를 끝까지 처리 → 연속 집기 실패 횟수 초기화
            if err := self.io.update_inventory(box_id, self.inventory):
                self.log.error(f'[control] 박스별 재고 DB 기록 실패 ({box_id}): {err}')
            self.log.info(f'[control] {box_id} {done_msg} → inventory={self.inventory}')
            self.io.log_event('INFO', 'BOX_DONE', done_msg)

        # DONE
        self.io.arm_monitor(False)
        self.io.set_log_context(box_id='', class_id='')
        if err := self.io.update_inventory(f'{self.job_id}_final', self.inventory):
            self.log.error(f'[control] 최종 재고 DB 기록 실패: {err}')
        self.set_state(State.DONE)
        if err := self.io.set_job_status(State.DONE.value):
            self.log.error(f'[control] DONE 상태 DB 기록 실패: {err} '
                           f'— UI에는 RUNNING으로 남아 있을 수 있습니다. DB 상태를 확인하세요.')
        self.io.log_event('INFO', 'JOB_DONE',
                          f'작업 완료: {self.box_seq}개 처리, 재고 {self.inventory}, '
                          f'불일치 {len(self.count_mismatches)}건, 집기 실패 재스캔 {self.grasp_retry_total}회')
        self.log.info(f'[control] 총 {self.box_seq}개 처리, 최종 inventory={self.inventory}')
        if self.count_mismatches:
            self.log.warn(f'[control] 재고 불일치 {len(self.count_mismatches)}건 — 확인 필요: '
                          + ', '.join(f'{b}({c} 기대 {e}/측정 {m})'
                                      for b, c, e, m in self.count_mismatches))

    # ------------------------------------------------------------------
    # 파트 실패 처리 — 집기 실패면 공간 스캔부터 다시 (11/17)
    # ------------------------------------------------------------------
    def on_part_error(self, err, part):
        """파트 Result 실패 코드 → None이면 공간 스캔부터 다시, 아니면 abort_job에 넘길 코드.
        GRASP_FAILED만 되돌아간다. 판별·적재였으면 측정존부터 비운다(관제가 로봇을 움직임).
        정지 요청이 있었거나, 연속 한도를 넘었거나, 측정존을 못 비우면 작업 종료."""
        if err != 'GRASP_FAILED' or self.io.stop_reason():
            return err
        self.grasp_retries += 1
        if self.grasp_retries > RECOVER_LIMIT:
            self.log.error(f'[control] 집기 실패 재스캔 연속 {RECOVER_LIMIT}회 초과 ({part}) → 작업 종료')
            return 'RETRY_LIMIT'
        where = ''
        if part in ('judge', 'place'):                     # 박스가 측정존에 있음 → 비우고 다시
            if (clear_err := self.clear_measure_zone(part)):
                return clear_err
            where = '측정존 비움, '
        self.grasp_retry_total += 1
        msg = (f'{part} 집기 실패 → {where}공간 스캔부터 다시 '
               f'({self.grasp_retries}/{RECOVER_LIMIT})')
        stage, _ = self.io.progress(part)
        self.log.warn(f'[control] {msg} (마지막 stage="{stage}")')
        self.io.log_event('WARN', 'BOX_RETRY', msg, stage=stage)
        self._current_class = ''
        return None

    def clear_measure_zone(self, part):
        """CLEARING: 측정존 박스를 스캔 공간으로 밀어내고 비었는지 확인 (box_control/recovery.py).
        성공 None, 실패하면 abort_job에 넘길 코드 (RECOVERY_FAILED 또는 정지 코드)."""
        self.set_state(State.CLEARING)
        self.io.log_event('WARN', 'ZONE_CLEAR', f'{part} 집기 실패 → 측정존 비우기 시작')
        err = self.recovery.clear_measure_zone(self.poses)
        if err:
            why = self.recovery.last_error or err
            self.log.error(f'[control] 측정존 비우기 실패: {why}')
            self.io.log_event('ERROR', 'ZONE_CLEAR', f'측정존 비우기 실패: {why}')
            return err
        if getattr(self.recovery, 'skipped', False):
            self.io.log_event('WARN', 'ZONE_CLEAR', '측정존 비우기 (fake): 밀어내기 경로 미정 → 사람이 치움, 바로 재스캔')
        else:
            self.io.log_event('INFO', 'ZONE_CLEAR', '측정존 비움')
        return None

    # ------------------------------------------------------------------
    # 시작 대기 (카운트다운)
    # ------------------------------------------------------------------
    def wait_for_start(self):
        """start_delay 동안 카운트다운. (True, '') = 시작, (False, 이유) = 취소 (터미널 q / UI 취소 / 정지)."""
        delay = self._standby_remaining()
        how = ('중단하려면 q(또는 UI 취소), 바로 시작하려면 s 를 누르세요.' if sys.stdin.isatty()
               else '중단은 UI·관리자 GUI의 시작 취소, 바로 시작은 관리자 GUI 버튼으로 (launch라 터미널 키 없음).')
        print(
            f'\n  ⚠ {delay:.0f}초 후에 작업을 시작합니다.\n'
            f'  ⚠ 작업공간 안에 사람이나 파손 위험이 있는 물품이 없는지 확인하세요.\n'
            f'  {how} {delay:.0f}초 동안 입력이 없으면 자동으로 시작합니다.\n',
            flush=True)
        if delay <= 0:
            return True, ''

        with _Keys() as keys:
            if not keys.tty:
                self.log.warn('[control] 터미널 입력을 받을 수 없어 q 입력 없이 대기합니다. (UI 취소·비상정지는 동작)')
            end = time.monotonic() + delay
            shown = None
            warned = keys.error
            while True:
                if keys.error and keys.error != warned:
                    warned = keys.error
                    self.warn_and_continue('카운트다운 중 터미널 입력', keys.error)
                if self.io.stop_reason():
                    print('\r  비상정지 요청 — 작업을 시작하지 않습니다.' + ' ' * 20, flush=True)
                    return False, '정지 요청'
                if self._cancel_req.is_set():
                    print('\r  원격 취소 — 작업을 시작하지 않습니다.' + ' ' * 20, flush=True)
                    return False, f'{self._cancel_source}에서 취소'
                if self._now_req.is_set():
                    print(f'\r  바로 시작합니다 ({self._now_source}).' + ' ' * 30, flush=True)
                    self.io.log_event('WARN', 'START_NOW', f'카운트다운 생략 — 현장 즉시 시작 ({self._now_source})')
                    return True, ''
                remain = end - time.monotonic()
                if remain <= 0:
                    print('\r  시작합니다.' + ' ' * 40, flush=True)
                    return True, ''
                sec = math.ceil(remain)
                if keys.tty and sec != shown:
                    print(f'\r  시작까지 {sec}초 ...  (q: 중단 / s: 바로 시작) ', end='', flush=True)
                    shown = sec
                key = keys.read(min(0.1, remain))
                if key == 'q':
                    print('\r  작업 시작을 취소했습니다.' + ' ' * 30, flush=True)
                    return False, '터미널 q로 취소'
                if key == 's':
                    print('\r  바로 시작합니다.' + ' ' * 40, flush=True)
                    self.io.log_event('WARN', 'START_NOW', '카운트다운 생략 — 현장 즉시 시작 (terminal)')
                    return True, ''

    # ------------------------------------------------------------------
    # 상주 모드: 시작 명령 대기 + 원격 명령 (/job_command)
    # ------------------------------------------------------------------
    def wait_for_command(self) -> bool:
        """READY에서 시작 명령을 기다린다. True = 시작, False = 관제 종료 (터미널 q)."""
        print('\n  [READY] 시작 명령을 기다립니다 — UI 시작 버튼 / 터미널 s: 시작, q: 관제 종료\n', flush=True)
        stop_seen = self.io.stop_reason()
        warned = ''
        with _Keys() as keys:
            while True:
                try:
                    if (stop := self.io.stop_reason()) and stop != stop_seen:
                        stop_seen = stop
                        self._record_idle_stop(stop)
                    if self._start_req.is_set():
                        print(f'  시작 명령 받음 ({self._start_source})', flush=True)
                        return True
                    with self._cmd_lock:
                        inventory_changed = self._inventory_changed_req.is_set()
                        self._inventory_changed_req.clear()
                    if inventory_changed:
                        _specs, inventory, _poses, err = self.io.get_job_data()
                        if err:
                            self.log.error(f'[control] UI 재고 변경 후 DB 재조회 실패: {err}')
                        else:
                            self.inventory = {c: int(inventory.get(c, 0)) for c in INVENTORY_KEYS}
                            self._inventory_available = True
                            self._inventory_needs_recheck = False
                            self.publish_status()
                            self.log.info(f'[control] UI 재고 변경 반영: {self.inventory}')
                    key = keys.read(0.2)
                    if keys.error and keys.error != warned:
                        warned = keys.error
                        self.warn_and_continue('대기 중 터미널 입력', keys.error)
                    if key == 's':
                        with self._cmd_lock:
                            self._start_source = 'terminal'
                            self._start_requested_at = time.monotonic()
                            self._start_requested_at_wall = time.time()
                            self._start_req.set()
                    elif key == 'q':
                        return False
                except Exception as e:          # 대기 중 예상 못 한 오류 → 주의 로그만, 계속 대기 (Ctrl+C는 통과)
                    self.warn_and_continue('대기', f'{type(e).__name__}: {e}')
                    time.sleep(1.0)

    def _record_idle_stop(self, stop):
        """READY에서 비상정지 → 정지할 작업은 없지만 DB status를 STOPPED로 (UI 작업 상태 빨간불 '비상정지').
        다음 시작 때 지워지고(reset_stop), 사전 점검이 로봇 상태를 다시 확인한다."""
        code, detail = stop
        reason = error_text(code) + (f' — {detail}' if detail else '') + ' (대기 중)'
        self.log.warn(f'[control] 대기 중 정지 요청: {status_for(code)} : {reason}')
        self.publish_status(reason=f'{status_for(code)} : {reason}')
        if (db_err := self.io.set_job_status(status_for(code), reason)):
            self.log.warn(f'[control] 정지 DB 기록 실패: {db_err}')

    def on_job_command(self, command, source):
        """/job_command 처리 (executor 스레드). 작업을 여기서 돌리지 않고 신호만 켠다 → 바로 응답."""
        with self._cmd_lock:
            state = self.state
            now = f'{state.value} ({state.label})' if state else '-'
            if command == 'start':
                if state != State.READY:
                    return False, f'지금은 시작할 수 없음 (관제 단계: {now})'
                if self._start_req.is_set():
                    return False, '이미 시작 명령을 받음'
                self._start_source = source
                self._start_requested_at = time.monotonic()
                self._start_requested_at_wall = time.time()
                self._start_req.set()
                return True, '시작 명령 수락 — 사전 점검 후 카운트다운'
            if command == 'start_now':
                if source == 'ui':
                    return False, '바로 시작은 현장(관리자 GUI·관제 터미널)에서만 가능'
                if state != State.STANDBY:
                    return False, f'카운트다운 중이 아님 (관제 단계: {now})'
                self._now_source = source
                self._now_req.set()
                return True, '카운트다운 생략 — 바로 시작'
            if command == 'cancel':
                if state not in _CANCELABLE:
                    return False, f'취소할 카운트다운 없음 (관제 단계: {now})'
                self._cancel_source = source
                self._cancel_req.set()
                return True, '카운트다운 취소 수락 — 작업을 시작하지 않고 대기로'
            if command == 'inventory_changed':
                if source != 'db':
                    return False, '재고 변경 알림은 DB 노드에서만 받을 수 있음'
                if state != State.READY:
                    return False, f'대기(READY)일 때만 재고 변경을 반영할 수 있음 (관제 단계: {now})'
                self._inventory_changed_req.set()
                return True, '재고 변경 알림 수락'
            return False, f'알 수 없는 명령: {command}'

    # ------------------------------------------------------------------
    # 재고 검증
    # ------------------------------------------------------------------
    def zone_full(self, box_id, result_class, measured):
        """구역이 꽉 차서 적재가 미분류로 보낸 경우 (11/18). 작업은 계속.
        재고: result_class = 적재가 확인한 정원(measured, 0이면 그대로), U +1. 로그 ZONE_FULL (WARN)."""
        inventory_new = dict(self.inventory)
        if measured and measured >= 1:
            inventory_new[result_class] = int(measured)
        inventory_new[UNCLASSIFIED] = self.inventory.get(UNCLASSIFIED, 0) + 1
        msg = (f'{result_class} 구역 꽉 참({inventory_new[result_class]}개) → 미분류로 보냄, '
               f'미분류 {inventory_new[UNCLASSIFIED]}개 (구역을 비우세요)')
        self.log.warn(f'[control] {box_id} {msg}')
        self.io.log_event('WARN', 'ZONE_FULL', msg, box_id=box_id, class_id=result_class,
                          expected=self.expected_count(result_class), measured=int(measured or 0))
        return inventory_new, msg

    def expected_count(self, result_class) -> int:
        """기대 수량 = 적재 전 재고의 result_class 수량 + 1"""
        return self.inventory[result_class] + 1

    def verify_inventory(self, box_id, result_class, measured) -> dict:
        """적재가 잰 수량(measured)과 기대값 비교 → DB에 반영할 재고를 돌려준다.
        정책 (a): 항상 측정값 반영 (실제 선반 상태가 기준), 불일치는 기록 + 경고 후 계속.
        예외: 수량이 1보다 작으면 = '적재 현황 확인 불가' (11/18: place_interface가 범위 밖 수량을 0으로 바꿔 보냄,
              박스는 놓였으므로 작업은 계속) → 반영하지 않는다.
              재고가 0으로 덮어써지는 것을 막고, 같은 클래스를 다음에 적재할 때 측정값으로 바로잡힌다
              (그때 기대값과 한 번 어긋나므로 COUNT_MISMATCH가 한 번 더 남는다)."""
        expected = self.expected_count(result_class)
        inventory_new = dict(self.inventory)
        measured = int(measured)
        if measured < 1:
            self.count_mismatches.append((box_id, result_class, expected, measured))
            self.log.error(
                f'[control] {box_id}: {result_class} 적재는 성공했는데 적재 현황 확인 불가(수량 {measured}) '
                f'→ 반영하지 않고 이전 {self.inventory[result_class]}개 유지 (다음 {result_class} 적재 때 바로잡힘)')
            self.io.log_event('WARN', 'COUNT_MISMATCH',
                              f'{result_class} 적재 현황 확인 불가 → 재고 반영 안 함, '
                              f'이전 {self.inventory[result_class]}개 유지',
                              box_id=box_id, class_id=result_class,
                              expected=expected, measured=measured)
            return inventory_new

        inventory_new[result_class] = measured
        if measured == expected:
            return inventory_new

        self.count_mismatches.append((box_id, result_class, expected, measured))
        self.log.warn(
            f'[control] 재고 불일치 ({time.strftime("%H:%M:%S")}) {box_id}: '
            f'class={result_class}, 적재 전 {self.inventory[result_class]}개, '
            f'기대 {expected}개, 적재 측정 {measured}개 (차이 {measured - expected:+d}) '
            f'→ 측정값으로 반영, 확인 필요')
        self.io.log_event('WARN', 'COUNT_MISMATCH',
                          f'{result_class} 재고 불일치: 기대 {expected}, 측정 {measured} '
                          f'({measured - expected:+d}) → 측정값 반영',
                          box_id=box_id, class_id=result_class,
                          expected=expected, measured=measured)
        return inventory_new

    # ------------------------------------------------------------------
    # 중단 — 두 가지로 통일
    #   cancel_start(): RUNNING 전 (데이터 실패 / 사전 점검 실패 / q·UI 취소)
    #                   로봇이 움직이기 전 → DB status BEFORE + 취소 이유 (UI에 왜 안 시작했는지 보이게)
    #   abort_job()   : RUNNING 이후 (작업 중 오류) → DB에 ERROR 또는 STOPPED + 이유 기록
    # ------------------------------------------------------------------
    def cancel_start(self, reason):
        where = f'{self.state.value} ({self.state.label})' if self.state else '-'
        self.log.warn(f'[control] ===== 작업을 시작하지 않습니다 [{where}]: {reason} =====')
        self.io.log_event('WARN', 'JOB_CANCELED', f'작업을 시작하지 않음 [{where}]: {reason}')
        if (stop := self.io.stop_reason()):            # 시작 전 비상정지 → UI에 '비상정지'(빨강)로 보이게
            code, detail = stop
            db_status = status_for(code)
            db_reason = error_text(code) + (f' — {detail}' if detail else '') + ' (시작 전)'
        else:
            db_status, db_reason = State.BEFORE.value, f'시작 안 함: {reason}'
        if (db_err := self.io.set_job_status(db_status, db_reason)):
            self.log.warn(f'[control] 시작 취소 DB 기록 실패: {db_err}')
        self.log.warn('[control] 로봇은 움직이지 않았습니다. 원인을 해결한 뒤 '
                      + ('관제를 다시 실행하세요.' if self.auto_start else '다시 시작하세요 (READY로 돌아감).'))

    def abort_job(self, err):
        """작업 중 오류로 작업을 끝낸다 (로봇을 멈추는 함수가 아님 — 두산 stop()과 다름).
        err 코드로 ERROR / STOPPED를 정하고, DB에 status·reason을 기록한다.
          ERROR   : 공정 오류 (GRASP_FAILED, SERVER_DOWN ...)       → 'ERROR : Fail to pick'
          STOPPED : 로봇 정지 (EMERGENCY_STOP, ROBOT_SAFETY_STOP ...) → 'STOPPED : EMERGENCY STOP'
        """
        # TODO: error_code별 정책 (재시도 / 이 박스 건너뛰기) — 지금은 모든 오류에서 작업 종료
        self.io.arm_monitor(False)
        detail = ''
        if (stop := self.io.stop_reason()):              # 정지 요청이 있었으면 그 원인이 우선 (파트의 CANCELED 등보다)
            err, detail = stop
        where = f'{self.state.value} ({self.state.label})' if self.state else '-'
        action = _STATE_ACTION.get(self.state)
        status = status_for(err)
        reason = error_text(err, part=action or '', class_id=self._current_class)
        if detail:
            reason += f' — {detail}'

        # Any mid-job interruption can leave an item in the gripper, on the
        # measuring zone, or between a physical placement and its result.
        # Keep the last confirmed controller snapshot, but flag it for recount.
        self._inventory_needs_recheck = True
        if self._inventory_available:
            if inv_err := self.io.update_inventory(f'{self.job_id}_final', self.inventory):
                self.log.error(f'[control] 중단 시 최종 재고 DB 기록 실패: {inv_err}')

        msg = f'[control] {where} 단계에서 중단: {err}'
        stage = ''
        if action:
            stage, sec = self.io.progress(action)
            msg += f' | /{action} 마지막 stage="{stage}" ({sec:.1f}초 전)'
        if self._inventory_needs_recheck:
            reason += ' — 재고 재측정 필요 (작업 중단으로 현재 수량이 확정되지 않음)'
        self.log.error(msg)

        self.set_state(State.STOPPED if status == 'STOPPED' else State.ERROR)
        self.publish_status(reason=f'{status} : {reason}')
        db_err = self.io.set_job_status(status, reason)      # DB가 원인이어도 한 번은 시도
        self.log.error(f'[control] ===== {status} : {reason} =====' +
                       (f' (DB 기록 실패: {db_err})' if db_err else ''))
        self.io.log_event('ERROR', 'JOB_STOPPED' if status == 'STOPPED' else 'JOB_ERROR',
                          f'{status} : {reason} — {msg.replace("[control] ", "")}', stage=stage)

def _launch_monitor_gui():
    """관제와 같이 모니터 GUI 창을 띄운다 (별도 프로세스 → GUI가 죽어도 관제는 계속).
    끄기: --ros-args -p gui:=false  /  화면(DISPLAY)이 없으면 자동으로 안 띄움.
    robot_monitor:=off/fake로 실행하면 GUI도 로봇 상태 조회를 끈다."""
    argv = ' '.join(sys.argv)
    if 'gui:=false' in argv.replace(' ', '').lower():
        return None
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('[control] 화면(DISPLAY)이 없어 모니터 GUI를 띄우지 않습니다.', flush=True)
        return None
    poll = 'false' if ('robot_monitor:=off' in argv or 'robot_monitor:=fake' in argv) else 'true'
    exe = shutil.which('ros2')
    cmd = ([exe, 'run', 'box_control', 'monitor_gui'] if exe
           else [sys.executable, '-m', 'box_control.monitor_gui'])
    cmd += ['--ros-args', '-p', f'robot_poll:={poll}']
    try:
        return subprocess.Popen(cmd, stdin=subprocess.DEVNULL, start_new_session=True)
    except OSError as e:
        print(f'[control] 모니터 GUI 실행 실패: {e}', flush=True)
        return None


def main():
    gui = _launch_monitor_gui()
    try:
        start(Control)
    finally:
        # 관제가 끝나도 GUI는 남겨 둔다 (마지막 결과·로그 확인용). 창은 직접 닫는다.
        if gui is not None and gui.poll() is None:
            print('[control] 관제 종료 — 모니터 GUI 창은 결과 확인용으로 남겨 둡니다 (창을 닫으면 종료).',
                  flush=True)


if __name__ == '__main__':
    main()
