#!/usr/bin/env python3
"""
scan_main.py — 탐지 노드 실행 진입점 (ros2 run box_scan scan_main)
================================================================
탐지 알고리즘: box_scan/scan_algorithm.py, 설정: box_scan/scan_config.py (담당자)
로봇 명령·정지 확인: box_comm/dsr_io.py, Action·TCP·실패 처리: box_comm/scan_interface.py (관제)

실행
  ros2 run box_scan scan_main
  ros2 run box_scan scan_main --ros-args -p csv_dir:=~/ROkEY_협동1/04_실험데이터   # 탐색 CSV 기록
단독 테스트 (Goal 한 번 = 물품 1개)
  ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback
================================================================
"""
from box_comm.scan_interface import start_algorithm
from box_scan.scan_algorithm import ScanAlgorithm


def main():
    start_algorithm(ScanAlgorithm)


if __name__ == '__main__':
    main()
