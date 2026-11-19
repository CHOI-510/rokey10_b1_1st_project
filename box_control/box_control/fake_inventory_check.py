#!/usr/bin/env python3
"""가짜 시작 재고 점검 Action 서버 (virtual/fake launch 전용)."""

import time

from box_comm import part_server
from box_comm.common import GRIPPER, STACK, VALID_CLASSES
from box_comm.inventory_check_interface import SPEC

# inventory_check_algorithm.py와 동일한 구역별 자세 및 2번 자리 오프셋
SLOT_LAYOUT = {
    'A': {'offset': [0.0, -45.0], 'orient': [120.0, 180.0, 120.0]},
    'B': {'offset': [0.0, -45.0], 'orient': [120.0, -180.0, 120.0]},
    'C': {'offset': [0.0, -30.0], 'orient': [175.0, -180.0, -175.0]},
    'D': {'offset': [0.0, -30.0], 'orient': [3.5, 180.0, 4.0]},
}


class FakeInventoryCheck:
    def __init__(self, node, robot=None):
        self.robot = robot
        node.declare_parameter('fake_inventory_delay_sec', 0.1)
        node.declare_parameter('fake_inventory_counts', [0, 0, 0, 0])
        # Physical/simulator motion is opt-in; pure fake runs must never move a robot.
        node.declare_parameter('fake_move_robot', False)
        self.move_robot = bool(node.get_parameter('fake_move_robot').value)
        counts = node.get_parameter('fake_inventory_counts').value
        if len(counts) != len(VALID_CLASSES):
            raise ValueError(f'fake_inventory_counts는 {len(VALID_CLASSES)}개 필요')
        self.counts = dict(zip(VALID_CLASSES, map(int, counts)))
        invalid = {
            category: count for category, count in self.counts.items()
            if not 0 <= count <= STACK['levels'] * STACK['max_tiers']
        }
        if invalid:
            raise ValueError(f'가짜 재고 점검 수량 범위 오류: {invalid}')
        self.delay = float(node.get_parameter('fake_inventory_delay_sec').value)

    def _move_slot(self, result_class, first_xy, home, tier, report):
        layout = SLOT_LAYOUT[result_class]
        dx, dy = layout['offset']
        xy = [first_xy[0] + (tier - 1) * dx, first_xy[1] + (tier - 1) * dy]

        report('moving')
        self.robot.movej(home, vel=30.0, acc=30.0)
        self.robot.grip_close(GRIPPER['grip_force_n'])
        # 상부(travel_z=160) 이동 후 가상 접촉 높이(85mm)까지만 왕복
        self.robot.movel([*xy, 160.0, *layout['orient']], vel=60.0, acc=100.0)
        report('placing')
        self.robot.movel([*xy, 85.0, *layout['orient']], vel=40.0, acc=80.0)
        if self.delay > 0:
            time.sleep(self.delay)
        self.robot.movel([*xy, 160.0, *layout['orient']], vel=60.0, acc=100.0)
        self.robot.movej(home, vel=30.0, acc=30.0)

    def measure(self, result_class, poses, report):
        count = self.counts[result_class]
        # 에뮬레이터(robot 연결됨)일 때만 실제 경로 이동
        if self.robot is not None and self.move_robot:
            xy = (poses.get('place_zones') or {}).get(result_class)
            home = poses.get('home_j')
            self._move_slot(result_class, xy, home, 1, report)
            # 첫 자리 3층(만재)이면 2번 자리도 추가 점검하는 실제 알고리즘 흐름 그대로 재현
            if count >= STACK['levels']:
                self._move_slot(result_class, xy, home, 2, report)
        else:
            report('moving')
            if self.delay > 0:
                time.sleep(self.delay)
            report('placing')
            if self.delay > 0:
                time.sleep(self.delay)

        report.log(f'{result_class} 시작 재고 점검 (fake) = {count}')
        return count


def main():
    # use_robot=True로 켜두면 virtual에서는 로봇을 움직이고,
    # robot_monitor:=fake (robot_setup:=off)인 완전 fake 모드에서도 안전하게 동작합니다.
    part_server.start_algorithm(
        SPEC, FakeInventoryCheck, 'measure', 'inventory_check_dsr', use_robot=True)


if __name__ == '__main__':
    main()
