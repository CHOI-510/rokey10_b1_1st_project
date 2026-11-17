#!/usr/bin/env python3
"""
place_main.py — 적재 노드 실행 진입점 (ros2 run box_place place_main)
================================================================
적재 알고리즘: box_place/place_algorithm.py (담당자)
로봇 명령·정지 확인: box_comm/dsr_io.py, Action·TCP·실패 처리: box_comm/place_interface.py (관제)

실행
  ros2 run box_place place_main
  ros2 run box_place place_main --ros-args -p clear_pending:=true      # 멈춘 뒤 확인 완료
  ros2 run box_place place_main --ros-args -p show_force:=true         # 하강 중 힘 출력
  ros2 run box_place place_main --ros-args -p sim_contact:=true        # 에뮬레이터 전용
  파라미터 설명은 place_algorithm.py 머리말 참고
================================================================
"""
from box_comm.place_interface import start_algorithm
from box_place.place_algorithm import PlaceAlgorithm


def main():
    start_algorithm(PlaceAlgorithm)


if __name__ == '__main__':
    main()
