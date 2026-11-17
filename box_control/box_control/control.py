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

흐름 (인터페이스 문서 2.5 관제 상태)
  BEFORE → GetJobData → PREFLIGHT(사전 점검) → STANDBY(시작 대기) → RUNNING
  → [SCANNING → SCAN_DONE → JUDGING → JUDGE_DONE → PLACING → PLACE_DONE
     → UpdateInventory] 반복
  → /scan found=False → DONE
  중단: RUNNING 전 실패 → cancel_start (DB는 BEFORE 그대로)
        RUNNING 후 실패 → abort_job   (DB에 ERROR 또는 STOPPED + 이유)

규칙
  - "-ING" 상태는 Goal 보내기 직전, "_DONE" 상태는 Result 받은 직후에 바꾼다.
  - 단계 상태는 관제 내부에만 둔다 (DB에는 BEFORE/RUNNING/DONE만).
  - 상태 값은 영어(State), 로그에는 한글 표시를 함께 붙인다.
  - 고정 좌표(HOME, 측정존)는 DB가 기준이다. GetJobData로 받은 값을
    모든 Action Goal(/scan, /judge, /place)에 실어 보내고, 파트는 그 값으로 움직인다.
  - box_id는 '<작업 시작 시각>_<순번>' (예: 20261114_130512_001).
    관제를 다시 실행해도 이전 작업의 box_id와 겹치지 않는다.
  - 재고 검증 (v0.4): 적재는 실제로 놓은 위치로 잰 result_class 수량(count)만 돌려준다.
    (단·층·z는 적재 내부 정보라 관제는 모른다)
    관제는 기대값(적재 전 재고의 result_class 수량 + 1)과 비교해서 기록하고,
    DB에는 항상 '측정값'을 반영한다 (실제 적재 상태가 기준). 다르면 경고 로그를 남기고
    작업을 계속한다. 다른 클래스 수량은 그대로 둔다. (불일치 기록을 DB에 남길지는 DB/UI와 협의 중)
    단, 수량이 1보다 작게 오면(적재가 수량을 안 채움) 반영하지 않는다 → 다음 같은 클래스 적재 때 바로잡힘.
  - 미분류 U (v0.5): 재고는 DB에서 A, B, C, D, U 다섯 개를 받는다 (U는 DB 초기값 0).
    판별이 'U'를 내면 적재에 'U'로 보낸다. 적재는 미분류 위치에 놓기만 하고
    수량을 재지 않는다(count 0). 관제가 적재 성공 후 재고 U = 이전 + 1로 DB에 반영한다 (검증 없음).
  - 집기 실패 (11/17): 파트가 GRIP_RETRY번 다시 집어 보고 포기하면 GRASP_FAILED로 Goal을 끝낸다.
    관제는 작업을 끝내지 않는다. 판별·적재였으면 CLEARING(측정존 비우기, recovery.py — 관제가 로봇을 움직임),
    탐지였으면 바로, BOX_RETRY 경고를 남긴 뒤 공간 스캔부터 다시 한다 (재고·박스 번호 그대로).
    연속 RECOVER_LIMIT번을 넘으면 RETRY_LIMIT로 종료. 박스 하나를 적재까지 끝내면 연속 횟수는 0으로.
    측정존을 비우지 못하면(RECOVERY_FAILED) 측정존 사용 불가 → 바로 작업 종료.
  - 진행 추적: 중단 시 해당 파트의 마지막 Feedback stage와 경과 시간을 함께 기록한다.
  - 작업 중 중단 (abort_job): DB status를 ERROR(공정 오류) 또는 STOPPED(로봇 정지)로 바꾸고
    이유를 함께 기록한다 (예: 'ERROR : Fail to pick').
    abort_job 자체는 로봇에 정지 명령을 보내지 않는다. 로봇 정지(move_stop)와 파트 Goal cancel은
    control_interface의 request_stop()이 맡는다 (비상정지 요청, 로봇 상태 이상, 진행 없음).
    정지 요청이 있었으면 파트가 돌려준 CANCELED 대신 정지 원인 코드를 DB에 남긴다.
  - 이벤트 로그: /event_log 토픽으로 보내기만 한다 (DB가 저장·UI 표시).
    관제 이벤트(JOB_START / BOX_DONE / COUNT_MISMATCH / JOB_ERROR / JOB_STOPPED / JOB_CANCELED /
    JOB_DONE)와 파트가 report.log()로 보낸 것(PART_LOG, control_interface가 자동으로 넘김).

사전 점검 (PREFLIGHT) — box_control/preflight.py
  분류 기준, 고정 좌표, 적재 구역, 로봇 상태를 점검한다.
  하나라도 FAIL이면 작업을 시작하지 않는다. WARN은 기록만 하고 진행한다.

정지 감시 (control_interface, 파라미터)
  robot_monitor        : dsr(기본, 실제·에뮬레이터) / fake(테스트) / off(bringup 없이 테스트할 때만)
  progress_timeout_sec : 파트 Feedback이 이 시간 동안 없으면 정지 (0 = 끔, 기본)
  감시는 RUNNING 동안만 켠다 (arm_monitor). 비상정지 요청(/emergency_stop)은 언제든 동작.

시작 대기 (STANDBY)
  GetJobData 후 start_delay_sec(기본 10초) 동안 카운트다운한다.
  작업공간에 사람·파손 위험 물품이 없는지 확인하는 시간이다.
    q : 작업 시작 취소 (DB는 BEFORE 그대로, 관제 종료)
    s : 바로 시작
    입력 없음 : 시간이 지나면 자동 시작
  실제 운용: --ros-args -p start_delay_sec:=600.0 (10분)
  터미널 입력을 받을 수 없는 환경(launch 등)에서는 중단 기능 없이 대기 후 시작한다.

TODO(나중에): UI 시작 버튼 연동, error_code별 정책 (재시도 / 이 박스 건너뛰기), 집기 실패 처리

실행: ros2 run box_control control
      ros2 run box_control control --ros-args -p start_delay_sec:=3.0
      ros2 run box_control control --ros-args -p progress_timeout_sec:=60.0
      ros2 run box_control control --ros-args -p robot_monitor:=off   (bringup 없이 fake 파트로 테스트)
================================================================
"""

import math
import os
import select
import sys
import time
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

    BEFORE = 'BEFORE'           # 작업전   (DB 기록)
    PREFLIGHT = 'PREFLIGHT'     # 사전점검 (관제 내부)
    STANDBY = 'STANDBY'         # 시작대기 (카운트다운, 관제 내부)
    RUNNING = 'RUNNING'         # 작업중   (DB 기록)
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
    State.BEFORE: '작업전',
    State.PREFLIGHT: '사전점검',
    State.STANDBY: '시작대기',
    State.RUNNING: '작업중',
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
_STATE_ACTION = {
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
        self.box_seq = 0
        self.job_id = time.strftime('%Y%m%d_%H%M%S')   # 실행마다 고유한 작업 ID
        self._current_class = ''          # 지금 박스의 판별 결과 (중단 이유 표시용)
        self.count_mismatches = []         # 재고 불일치 기록 (box_id, class, 기대 수량, 측정 수량)
        self.grasp_retries = 0             # 집기 실패 → 재스캔 연속 횟수 (박스 하나 적재 완료 시 0)
        self.grasp_retry_total = 0         # 이번 작업 전체 재스캔 횟수 (요약용)
        self.start_delay = float(
            io.declare_parameter('start_delay_sec', 10.0).value)   # 시작 대기 시간(초)
        self.recovery = Recovery(io, self.log)                     # 관제 복구 동작 (측정존 비우기 등)

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

    # ------------------------------------------------------------------
    # 메인 시퀀스
    # ------------------------------------------------------------------
    def run(self):
        # BEFORE: 물품값과 재고 받아오기
        self.set_state(State.BEFORE)
        self.log.info(f'[control] job_id={self.job_id}')
        self.io.set_log_context(job_id=self.job_id)
        self.specs, self.inventory, db_poses, err = self.io.get_job_data()
        if err:
            return self.cancel_start(f'작업 데이터를 받지 못함 ({err})')
        self.log.info(f'[control] specs {len(self.specs)}개, inventory={self.inventory}')
        self.apply_db_poses(db_poses)

        # PREFLIGHT: 시작 전 점검 (box_control/preflight.py)
        self.set_state(State.PREFLIGHT)
        if not preflight.run_all(self):
            return self.cancel_start('사전 점검 실패 (위 FAIL 항목 확인 후 재설정)')

        # STANDBY: 작업공간 안전 확인 시간 (q로 취소)
        self.set_state(State.STANDBY)
        started = self.wait_for_start()
        if (stop := self.io.stop_reason()):               # 시작 전 비상정지 → 시작하지 않음
            return self.cancel_start(f'정지 요청 ({stop[0]}{" — " + stop[1] if stop[1] else ""})')
        if not started:
            return self.cancel_start('사용자가 q로 취소')

        self.set_state(State.RUNNING)
        self.io.arm_monitor(True)                        # 작업 중에만 로봇 이상·진행 없음을 정지로 처리
        if err := self.io.set_job_status(State.RUNNING.value):
            return self.abort_job(err)
        self.io.log_event('INFO', 'JOB_START', f'작업 시작 (재고 {self.inventory})', box_id='')

        while True:
            box_id = f'{self.job_id}_{self.box_seq + 1:03d}'   # 이번 박스 (적재까지 끝나야 확정)
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
            measured, err = self.io.place(result_class, self.poses)
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
            else:
                self.log.info(f'[control] 적재 측정 수량: {result_class} {measured}개')
                inventory_new = self.verify_inventory(box_id, result_class, measured)
                done_msg = f'{result_class} 적재 완료 → {result_class} {inventory_new[result_class]}개'
            if err := self.io.update_inventory(box_id, inventory_new):
                return self.abort_job(err)
            self.inventory = inventory_new
            self.grasp_retries = 0                 # 박스 하나를 끝까지 처리 → 연속 집기 실패 횟수 초기화
            self.log.info(f'[control] {box_id} {done_msg} → inventory={self.inventory}')
            self.io.log_event('INFO', 'BOX_DONE', done_msg)

        # DONE
        self.io.arm_monitor(False)
        self.io.set_log_context(box_id='', class_id='')
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
        self.io.log_event('INFO', 'ZONE_CLEAR', '측정존 비움')
        return None

    # ------------------------------------------------------------------
    # 시작 대기 (카운트다운)
    # ------------------------------------------------------------------
    def wait_for_start(self) -> bool:
        """start_delay 동안 카운트다운. True=시작, False=취소(q)."""
        delay = self.start_delay
        print(
            f'\n  ⚠ {delay:.0f}초 후에 작업을 시작합니다.\n'
            f'  ⚠ 작업공간 안에 사람이나 파손 위험이 있는 물품이 없는지 확인하세요.\n'
            f'  중단하려면 q, 바로 시작하려면 s 를 누르세요. '
            f'{delay:.0f}초 동안 입력이 없으면 자동으로 시작합니다.\n',
            flush=True)

        if delay <= 0:
            return True

        if termios is None or not sys.stdin.isatty():
            self.log.warn('[control] 터미널 입력을 받을 수 없어 q 입력 없이 대기합니다. (비상정지는 동작)')
            end = time.monotonic() + delay
            while time.monotonic() < end:
                if self.io.stop_reason():
                    return False
                time.sleep(0.1)
            return True

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)                       # Enter 없이 한 글자씩 읽기 (Ctrl+C는 그대로 동작)
            end = time.monotonic() + delay
            shown = None
            while True:
                if self.io.stop_reason():
                    print('\r  비상정지 요청 — 작업을 시작하지 않습니다.' + ' ' * 20, flush=True)
                    return False
                remain = end - time.monotonic()
                if remain <= 0:
                    print('\r  시작합니다.' + ' ' * 40, flush=True)
                    return True
                sec = math.ceil(remain)
                if sec != shown:
                    print(f'\r  시작까지 {sec}초 ...  (q: 중단 / s: 바로 시작) ',
                          end='', flush=True)
                    shown = sec
                ready, _, _ = select.select([fd], [], [], min(0.1, remain))
                if ready:
                    key = os.read(fd, 1).decode(errors='ignore').lower()
                    if key == 'q':
                        print('\r  작업 시작을 취소했습니다.' + ' ' * 30, flush=True)
                        return False
                    if key == 's':
                        print('\r  바로 시작합니다.' + ' ' * 40, flush=True)
                        return True
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)   # 터미널 설정 원래대로

    # ------------------------------------------------------------------
    # 재고 검증
    # ------------------------------------------------------------------
    def expected_count(self, result_class) -> int:
        """기대 수량 = 적재 전 재고의 result_class 수량 + 1"""
        return self.inventory[result_class] + 1

    def verify_inventory(self, box_id, result_class, measured) -> dict:
        """적재가 잰 수량(measured)과 기대값 비교 → DB에 반영할 재고를 돌려준다.
        정책 (a): 항상 측정값 반영 (실제 선반 상태가 기준), 불일치는 기록 + 경고 후 계속.
        예외: 수량이 1보다 작으면(적재 성공인데 수량을 안 채움 = 적재 쪽 계약 위반) 반영하지 않는다.
              재고가 0으로 덮어써지는 것을 막고, 같은 클래스를 다음에 적재할 때 측정값으로 바로잡힌다
              (그때 기대값과 한 번 어긋나므로 COUNT_MISMATCH가 한 번 더 남는다)."""
        expected = self.expected_count(result_class)
        inventory_new = dict(self.inventory)
        measured = int(measured)
        if measured < 1:
            self.count_mismatches.append((box_id, result_class, expected, measured))
            self.log.error(
                f'[control] {box_id}: {result_class} 적재는 성공했는데 수량이 {measured}개 '
                f'(적재 쪽 수량 측정 확인 필요) → 반영하지 않고 이전 {self.inventory[result_class]}개 유지')
            self.io.log_event('WARN', 'COUNT_MISMATCH',
                              f'{result_class} 적재 수량 없음({measured}) → 반영 안 함, '
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
    #   cancel_start(): RUNNING 전 (데이터 실패 / 사전 점검 실패 / q 취소)
    #                   로봇이 움직이기 전이므로 DB는 BEFORE 그대로 두고 종료
    #   abort_job()   : RUNNING 이후 (작업 중 오류) → DB에 ERROR 또는 STOPPED + 이유 기록
    # ------------------------------------------------------------------
    def cancel_start(self, reason):
        where = f'{self.state.value} ({self.state.label})' if self.state else '-'
        self.log.warn(f'[control] ===== 작업을 시작하지 않습니다 [{where}]: {reason} =====')
        self.log.warn('[control] 로봇은 움직이지 않았고, DB 상태는 BEFORE 그대로입니다. '
                      '원인을 해결한 뒤 관제를 다시 실행하세요.')
        self.io.log_event('WARN', 'JOB_CANCELED', f'작업을 시작하지 않음 [{where}]: {reason}')

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

        msg = f'[control] {where} 단계에서 중단: {err}'
        stage = ''
        if action:
            stage, sec = self.io.progress(action)
            msg += f' | /{action} 마지막 stage="{stage}" ({sec:.1f}초 전)'
        self.log.error(msg)

        self.set_state(State.STOPPED if status == 'STOPPED' else State.ERROR)
        db_err = self.io.set_job_status(status, reason)      # DB가 원인이어도 한 번은 시도
        self.log.error(f'[control] ===== {status} : {reason} =====' +
                       (f' (DB 기록 실패: {db_err})' if db_err else ''))
        self.io.log_event('ERROR', 'JOB_STOPPED' if status == 'STOPPED' else 'JOB_ERROR',
                          f'{status} : {reason} — {msg.replace("[control] ", "")}', stage=stage)

def main():
    start(Control)


if __name__ == '__main__':
    main()
