#!/usr/bin/env python3
"""
control.py — 관제 메인 (예시: 정상 경로만)
================================================================
흐름 (인터페이스 문서 2.5 관제 상태)
  BEFORE → GetJobData → RUNNING
  → [SCANNING → SCAN_DONE → JUDGING → JUDGE_DONE → PLACING → PLACE_DONE
     → UpdateInventory] 반복
  → /scan found=False → DONE

규칙
  - "-ING" 상태는 Goal 보내기 직전, "_DONE" 상태는 Result 받은 직후에 바꾼다.
  - 단계 상태는 관제 내부에만 둔다 (DB에는 BEFORE/RUNNING/DONE만).
  - 상태 값은 영어(State), 로그에는 한글 표시를 함께 붙인다.
  - box_id는 '<작업 시작 시각>_<순번>' (예: 20261114_130512_001).
    관제를 다시 실행해도 이전 작업의 box_id와 겹치지 않는다.
  - 재고 검증: 적재 Result의 inventory를 관제 기대값(적재 전 재고에서
    result_class 칸만 +1)과 비교한다. 다르면 관제 기대값으로 DB에 반영하고
    경고 로그를 남긴 뒤 작업을 계속한다. (작업 로그를 DB에 남길지는 DB/UI와 협의 중)
  - 진행 추적: 중단 시 해당 파트의 마지막 Feedback stage와 경과 시간을 함께 기록한다.

TODO(나중에): 시작 버튼 연동, error_code별 정책, ERROR 상태, 정지

실행: ros2 run box_control control
================================================================
"""

import time
from enum import Enum

from box_comm.control_interface import start


class State(str, Enum):
    """관제 상태. 값은 영어, label은 로그용 한글."""

    BEFORE = 'BEFORE'           # 작업전   (DB 기록)
    RUNNING = 'RUNNING'         # 작업중   (DB 기록)
    SCANNING = 'SCANNING'       # 스캔중
    SCAN_DONE = 'SCAN_DONE'     # 스캔완료
    JUDGING = 'JUDGING'         # 판별중
    JUDGE_DONE = 'JUDGE_DONE'   # 판별완료
    PLACING = 'PLACING'         # 적재중
    PLACE_DONE = 'PLACE_DONE'   # 적재완료
    DONE = 'DONE'               # 작업완료 (DB 기록)

    @property
    def label(self) -> str:
        return _STATE_LABEL[self]


_STATE_LABEL = {
    State.BEFORE: '작업전',
    State.RUNNING: '작업중',
    State.SCANNING: '스캔중',
    State.SCAN_DONE: '스캔완료',
    State.JUDGING: '판별중',
    State.JUDGE_DONE: '판별완료',
    State.PLACING: '적재중',
    State.PLACE_DONE: '적재완료',
    State.DONE: '작업완료',
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
        self.inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.box_seq = 0
        self.job_id = time.strftime('%Y%m%d_%H%M%S')   # 실행마다 고유한 작업 ID
        self.count_mismatches = []         # 재고 불일치 기록 (box_id, 기대값, 받은 값)

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
        self.specs, self.inventory, err = self.io.get_job_data()
        if err:
            return self.stop(err)
        self.log.info(f'[control] specs {len(self.specs)}개, inventory={self.inventory}')

        # TODO: 시작 버튼 연동. 지금은 바로 시작
        self.set_state(State.RUNNING)
        if err := self.io.set_job_status(State.RUNNING.value):
            return self.stop(err)

        while True:
            # ---- 탐지 ----
            self.set_state(State.SCANNING)
            found, err = self.io.scan()
            if err:
                return self.stop(err)
            if not found:
                break                      # 물품 없음 → 관제가 작업완료로 판단
            self.set_state(State.SCAN_DONE)

            # ---- 판별 ----
            self.set_state(State.JUDGING)
            result_class, err = self.io.judge(self.specs)
            if err:
                return self.stop(err)
            self.set_state(State.JUDGE_DONE)
            self.log.info(f'[control] 판별 결과: {result_class}')

            # ---- 적재 ----
            self.set_state(State.PLACING)
            inventory_after, err = self.io.place(result_class, self.inventory)
            if err:
                return self.stop(err)
            self.set_state(State.PLACE_DONE)

            # ---- 재고 검증 → 반영 (Place Result를 받은 뒤에만) ----
            self.box_seq += 1
            box_id = f'{self.job_id}_{self.box_seq:03d}'
            inventory_new = self.verify_inventory(box_id, result_class, inventory_after)
            if err := self.io.update_inventory(box_id, inventory_new):
                return self.stop(err)
            self.inventory = inventory_new
            self.log.info(f'[control] {box_id} 적재 완료 → inventory={self.inventory}')

        # DONE
        self.set_state(State.DONE)
        self.io.set_job_status(State.DONE.value)
        self.log.info(f'[control] 총 {self.box_seq}개 처리, 최종 inventory={self.inventory}')
        if self.count_mismatches:
            self.log.warn(f'[control] 재고 불일치 {len(self.count_mismatches)}건 — 확인 필요: '
                          + ', '.join(b for b, _, _ in self.count_mismatches))

    # ------------------------------------------------------------------
    # 재고 검증
    # ------------------------------------------------------------------
    def expected_inventory(self, result_class) -> dict:
        """적재 전 재고에서 result_class 칸만 +1 한 값"""
        expected = dict(self.inventory)
        expected[result_class] += 1
        # TODO: 미분류('X')가 생기면 그때는 변화 0을 기대값으로
        return expected

    def verify_inventory(self, box_id, result_class, received) -> dict:
        """적재 Result와 관제 기대값 비교. DB에 반영할 값을 돌려준다 (관제 기대값 우선)."""
        expected = self.expected_inventory(result_class)
        if received == expected:
            return received

        self.count_mismatches.append((box_id, expected, received))
        self.log.warn(
            f'[control] 재고 카운팅 오류 ({time.strftime("%H:%M:%S")}) {box_id}: '
            f'class={result_class}, 적재 전={self.inventory}, '
            f'기대={expected}, 적재 Result={received} → 기대값으로 반영, 확인 필요')
        return expected

    # ------------------------------------------------------------------
    # 중단
    # ------------------------------------------------------------------
    def stop(self, err):
        # TODO: error_code별 정책 (재시도 / 건너뛰기 / ERROR 상태 기록)
        where = f'{self.state.value} ({self.state.label})' if self.state else '-'
        msg = f'[control] {where} 단계에서 중단: {err}'
        action = _STATE_ACTION.get(self.state)
        if action:
            stage, sec = self.io.progress(action)
            msg += f' | /{action} 마지막 stage="{stage}" ({sec:.1f}초 전)'
        self.log.error(msg)


def main():
    start(Control)


if __name__ == '__main__':
    main()
