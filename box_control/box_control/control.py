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


class Control:
    def __init__(self, io):
        self.io = io                       # ControlInterface (통신 담당)
        self.log = io.get_logger()
        self.state = None
        self.specs = []
        self.inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.box_seq = 0
        self.job_id = time.strftime('%Y%m%d_%H%M%S')   # 실행마다 고유한 작업 ID

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

            # ---- 재고 반영 (Place Result를 받은 뒤에만) ----
            self.box_seq += 1
            box_id = f'{self.job_id}_{self.box_seq:03d}'
            if err := self.io.update_inventory(box_id, inventory_after):
                return self.stop(err)
            self.inventory = inventory_after
            self.log.info(f'[control] {box_id} 적재 완료 → inventory={self.inventory}')

        # DONE
        self.set_state(State.DONE)
        self.io.set_job_status(State.DONE.value)
        self.log.info(f'[control] 총 {self.box_seq}개 처리, 최종 inventory={self.inventory}')

    def stop(self, err):
        # TODO: error_code별 정책 (재시도 / 건너뛰기 / ERROR 상태 기록)
        where = f'{self.state.value} ({self.state.label})' if self.state else '-'
        self.log.error(f'[control] {where} 단계에서 중단: {err}')


def main():
    start(Control)


if __name__ == '__main__':
    main()
