#!/usr/bin/env python3
"""
robot_test.py — TCP·툴 자동 확인 점검 도구 (관제 테스트 전용)
================================================================
/scan Goal을 여러 번 보내면서 box_comm/robot.py가 의도대로 동작하는지 확인하고
마지막에 PASS / FAIL 표를 출력한다. 시뮬레이션(가상 bringup)과 실제 로봇 모두에서 쓸 수 있다.

점검 항목
  1. 첫 Goal          : 성공 + 로봇에 기본 TCP·툴이 걸려 있는지
  2. 같은 Goal 한 번 더 : 성공 + 기본값 유지 (scan 로그에 'TCP OK, TOOL OK'가 나와야 함)
  3. 다른 TCP·툴로 바꿔 놓고 Goal (reapply_test:=true일 때)
       테스트용 TCP·툴(box_test_tcp / box_test_tool)을 등록해 현재값으로 걸어 둔 뒤 Goal
       → 성공 + 기본값으로 되돌아왔는지
       (scan 로그에 '재적용'이 나오면 자동 모드에서 바로 적용 가능, '등록·적용'이면 수동 모드 경로)
       끝나면 테스트용 TCP·툴을 삭제한다.

준비 (터미널 2개)
  1) 두산 bringup (가상 또는 실제)
  2) /scan 서버 — 둘 중 하나
       ros2 run box_control test_scan_motion --ros-args -p robot_setup:=on   ← 로봇이 실제로 움직임
       ros2 run box_scan scan --ros-args -p robot_setup:=on -p fake_delay_sec:=0.3   ← 움직임 없음

실행
  ros2 run box_control robot_test
  ros2 run box_control robot_test --ros-args -p reapply_test:=false   (컨트롤러 설정을 건드리기 싫을 때)
================================================================
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from box_interfaces.action import Scan

from box_comm.common import ROBOT, TCP, TOOL

DUMMY_TCP = {'name': 'box_test_tcp', 'pos': [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]}
DUMMY_TOOL = {'name': 'box_test_tool', 'weight': 0.5, 'cog': [0.0, 0.0, 10.0],
              'inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

SERVICES = {
    'set_mode':    ('system/set_robot_mode', 'SetRobotMode'),
    'tcp_create':  ('tcp/config_create_tcp', 'ConfigCreateTcp'),
    'tcp_delete':  ('tcp/config_delete_tcp', 'ConfigDeleteTcp'),
    'tcp_set':     ('tcp/set_current_tcp', 'SetCurrentTcp'),
    'tcp_get':     ('tcp/get_current_tcp', 'GetCurrentTcp'),
    'tool_create': ('tool/config_create_tool', 'ConfigCreateTool'),
    'tool_delete': ('tool/config_delete_tool', 'ConfigDeleteTool'),
    'tool_set':    ('tool/set_current_tool', 'SetCurrentTool'),
    'tool_get':    ('tool/get_current_tool', 'GetCurrentTool'),
}
MANUAL, AUTONOMOUS = 0, 1
GOAL_TIMEOUT = 180.0


class RobotTest:
    def __init__(self):
        import dsr_msgs2.srv as dsr
        self.dsr = dsr
        self.node = rclpy.create_node('robot_setup_test')
        self.node.declare_parameter('reapply_test', True)
        self.reapply = bool(self.node.get_parameter('reapply_test').value)
        self.log = self.node.get_logger()
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        ns = ROBOT['service_ns'].rstrip('/')
        self.cli = {k: self.node.create_client(getattr(dsr, t), f'{ns}/{n}')
                    for k, (n, t) in SERVICES.items()}
        self.scan = ActionClient(self.node, Scan, '/scan')
        self.rows = []

    # ---------------- 도우미 ----------------
    def _wait(self, fut, timeout):
        self.ex.spin_until_future_complete(fut, timeout_sec=timeout)
        return fut.done() and fut.result() is not None

    def call(self, key, **fields):
        cli = self.cli[key]
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{SERVICES[key][0]} 서비스 없음')
        req = getattr(self.dsr, SERVICES[key][1]).Request()
        for k, v in fields.items():
            if hasattr(req, k):
                setattr(req, k, v)
        fut = cli.call_async(req)
        if not self._wait(fut, 3.0):
            raise RuntimeError(f'{SERVICES[key][0]} 응답 없음')
        return fut.result()

    def current(self):
        return (str(self.call('tcp_get').info).strip(), str(self.call('tool_get').info).strip())

    def send_scan(self):
        fut = self.scan.send_goal_async(Scan.Goal())
        if not self._wait(fut, 5.0):
            return None, 'Goal 전송 실패'
        handle = fut.result()
        if not handle.accepted:
            return None, 'Goal 거절'
        res_fut = handle.get_result_async()
        if not self._wait(res_fut, GOAL_TIMEOUT):
            return None, f'{GOAL_TIMEOUT:.0f}초 안에 Result 없음'
        r = res_fut.result().result
        return r, None

    def record(self, name, ok, detail):
        self.rows.append((name, ok, detail))
        (self.log.info if ok else self.log.error)(f'[robot_test] {"PASS" if ok else "FAIL"} {name}: {detail}')

    def goal_step(self, name):
        self.log.info(f'[robot_test] ---- {name}: /scan Goal 전송 ----')
        r, err = self.send_scan()
        if err:
            self.record(name, False, err)
            return False
        tcp, tool = self.current()
        ok = r.success and tcp == TCP['name'] and tool == TOOL['name']
        self.record(name, ok, f'success={r.success} error_code="{r.error_code}" '
                              f'| 로봇 TCP="{tcp}" TOOL="{tool}"')
        return ok

    def set_dummy(self):
        """테스트용 TCP·툴을 등록해서 현재값으로 걸어 둔다."""
        self.call('set_mode', robot_mode=MANUAL)
        try:
            self.call('tcp_create', name=DUMMY_TCP['name'], pos=DUMMY_TCP['pos'])
            self.call('tool_create', name=DUMMY_TOOL['name'], weight=DUMMY_TOOL['weight'],
                      cog=DUMMY_TOOL['cog'], inertia=DUMMY_TOOL['inertia'])
            self.call('tcp_set', name=DUMMY_TCP['name'])
            self.call('tool_set', name=DUMMY_TOOL['name'])
        finally:
            self.call('set_mode', robot_mode=AUTONOMOUS)
        return self.current()

    def cleanup_dummy(self):
        try:
            self.call('set_mode', robot_mode=MANUAL)
            self.call('tcp_delete', name=DUMMY_TCP['name'])
            self.call('tool_delete', name=DUMMY_TOOL['name'])
        finally:
            self.call('set_mode', robot_mode=AUTONOMOUS)

    # ---------------- 점검 순서 ----------------
    def run(self):
        self.log.info('[robot_test] /scan 서버와 두산 서비스 기다리는 중...')
        if not self.scan.wait_for_server(timeout_sec=10.0):
            self.record('준비', False, '/scan 서버 없음 (test_scan_motion 또는 box_scan scan 실행 확인)')
            return
        try:
            before = self.current()
        except RuntimeError as e:
            self.record('준비', False, f'{e} (두산 bringup 확인)')
            return
        self.record('준비', True, f'시작 전 로봇 TCP="{before[0]}" TOOL="{before[1]}"')

        self.goal_step('1. 첫 Goal')
        self.goal_step('2. 같은 Goal 한 번 더 (scan 로그에 TCP OK, TOOL OK)')

        if self.reapply:
            try:
                tcp, tool = self.set_dummy()
                self.record('3-준비. 테스트용 TCP·툴 걸기',
                            tcp == DUMMY_TCP['name'] and tool == DUMMY_TOOL['name'],
                            f'로봇 TCP="{tcp}" TOOL="{tool}"')
                self.goal_step('3. 다른 TCP·툴 상태에서 Goal → 기본값 복귀 (scan 로그에 재적용/등록·적용)')
            finally:
                try:
                    self.cleanup_dummy()
                    self.record('정리. 테스트용 TCP·툴 삭제', True, f'{DUMMY_TCP["name"]}, {DUMMY_TOOL["name"]}')
                except RuntimeError as e:
                    self.record('정리. 테스트용 TCP·툴 삭제', False, str(e))
        else:
            self.log.info('[robot_test] reapply_test=false → 3번 건너뜀')

    def summary(self):
        print('\n================ robot_test 결과 ================')
        for name, ok, detail in self.rows:
            print(f' {"PASS" if ok else "FAIL"}  {name}\n        {detail}')
        n_fail = sum(1 for _, ok, _ in self.rows if not ok)
        print('=================================================')
        print(f' {"전체 통과" if n_fail == 0 else f"실패 {n_fail}건"}')
        print(' scan 노드 로그에서 각 Goal의 robot_setup 줄(OK / 재적용 / 등록·적용)도 확인하세요.\n')


def main(args=None):
    rclpy.init(args=args)
    t = RobotTest()
    try:
        t.run()
    except KeyboardInterrupt:
        pass
    finally:
        t.summary()
        t.ex.shutdown()
        t.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
