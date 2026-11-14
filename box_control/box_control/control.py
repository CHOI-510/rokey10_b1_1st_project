#!/usr/bin/env python3
"""
control.py — 관제 메인 (예시: 정상 경로만)
================================================================
흐름 (인터페이스 문서 2.5 관제 상태)
  작업전 → GetJobData → 작업중(RUNNING)
  → [스캔중 → 스캔완료 → 판별중 → 판별완료 → 적재중 → 적재완료 → UpdateInventory] 반복
  → /scan found=False → 작업완료(DONE)

규칙
  - "-중" 상태는 Goal 보내기 직전, "-완료" 상태는 Result 받은 직후에 바꾼다.
  - 단계 상태는 관제 내부에만 둔다 (DB에는 BEFORE/RUNNING/DONE만).
  - box_id는 '<작업 시작 시각>_<순번>' (예: 20261114_130512_001).
    관제를 다시 실행해도 이전 작업의 box_id와 겹치지 않는다.

TODO(나중에): 시작 버튼 연동, error_code별 정책, ERROR 상태, 정지

실행: ros2 run box_control control
================================================================
"""

import time

from box_comm.control_interface import start


class Control:
    def __init__(self, io):
        self.io = io                       # ControlInterface (통신 담당)
        self.log = io.get_logger()
        self.state = None
        self.specs = []
        self.inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.box_seq = 0
        self.job_id = time.strftime('%Y%m%d_%H%M%S')   # 실행마다 고유한 작업 ID

    def set_state(self, state: str):
        self.state = state
        self.log.info(f'[control] ===== 상태: {state} =====')

    # ------------------------------------------------------------------
    # 메인 시퀀스
    # ------------------------------------------------------------------
    def run(self):
        # 작업전: 물품값과 재고 받아오기
        self.set_state('작업전')
        self.log.info(f'[control] job_id={self.job_id}')
        self.specs, self.inventory, err = self.io.get_job_data()
        if err:
            return self.stop(err)
        self.log.info(f'[control] specs {len(self.specs)}개, inventory={self.inventory}')

        # TODO: 시작 버튼 연동. 지금은 바로 시작
        self.set_state('작업중')
        if err := self.io.set_job_status('RUNNING'):
            return self.stop(err)

        while True:
            # ---- 탐지 ----
            self.set_state('스캔중')
            found, err = self.io.scan()
            if err:
                return self.stop(err)
            if not found:
                break                      # 물품 없음 → 작업완료
            self.set_state('스캔완료')

            # ---- 판별 ----
            self.set_state('판별중')
            result_class, err = self.io.judge(self.specs)
            if err:
                return self.stop(err)
            self.set_state('판별완료')
            self.log.info(f'[control] 판별 결과: {result_class}')

            # ---- 적재 ----
            self.set_state('적재중')
            inventory_after, err = self.io.place(result_class, self.inventory)
            if err:
                return self.stop(err)
            self.set_state('적재완료')

            # ---- 재고 반영 (Place Result를 받은 뒤에만) ----
            self.box_seq += 1
            box_id = f'{self.job_id}_{self.box_seq:03d}'
            if err := self.io.update_inventory(box_id, inventory_after):
                return self.stop(err)
            self.inventory = inventory_after
            self.log.info(f'[control] {box_id} 적재 완료 → inventory={self.inventory}')

        # 작업완료
        self.set_state('작업완료')
        self.io.set_job_status('DONE')
        self.log.info(f'[control] 총 {self.box_seq}개 처리, 최종 inventory={self.inventory}')

    def stop(self, err):
        # TODO: error_code별 정책 (재시도 / 건너뛰기 / ERROR 상태 기록)
        self.log.error(f'[control] "{self.state}" 단계에서 중단: {err}')


def main():
    start(Control)


if __name__ == '__main__':
    main()
