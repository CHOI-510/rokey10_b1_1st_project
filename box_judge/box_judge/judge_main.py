#!/usr/bin/env python3
"""judge_main.py — 판별 파트 실행 진입점 (judge_template.py 권장 구성)
setup.py entry_points 에 'judge_main = box_judge.judge_main:main' 추가."""
from box_comm.judge_interface import start_algorithm

from box_judge.judge_algorithm import JudgeAlgorithm


def main():
    start_algorithm(JudgeAlgorithm)


if __name__ == '__main__':
    main()
