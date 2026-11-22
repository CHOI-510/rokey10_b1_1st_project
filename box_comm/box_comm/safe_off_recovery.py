"""Explicit operator-requested SAFE_OFF recovery. Never starts a job or movement."""
import threading
import time


def _rpc(client, request, timeout):
    if not client.service_is_ready():
        raise RuntimeError('로봇 상태/복구 서비스가 준비되지 않았습니다.')
    future = client.call_async(request)
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout):
        raise RuntimeError('로봇 복구 응답 시간 초과. 실제 상태를 다시 확인하세요.')
    response = future.result()
    if response is None or response.success is not True:
        raise RuntimeError('로봇이 상태 조회/SAFE_OFF 해제 요청을 거절했습니다.')
    return response


def recover_safe_off(io):
    with io._stop_lock:
        if io._recovering:
            return False, '이미 복구 요청을 처리 중입니다.'
        if io._armed or io._active is not None:
            return False, '진행 중인 작업이 종료된 뒤 복구할 수 있습니다.'
        if time.monotonic() < io._stop_hold_until or getattr(io, '_stop_hold_count', 0) > 0:
            return False, '로봇 정지 명령 처리 중입니다. 잠시 후 복구를 눌러주세요.'
        revision = io._stop_revision
        io._recovering = True
    reset_sent = False
    completed = False
    try:
        def interrupted():
            with io._stop_lock:
                return io._stop_revision != revision

        def robot_state():
            return _rpc(io._cli_robot_state, io._dsr.GetRobotState.Request(), 1.0).robot_state

        if io._monitor == 'fake':
            state = int(io.get_parameter('fake_robot_state').value)
            if state != 1:
                return False, '가짜 로봇도 STANDBY 상태에서 복구할 수 있습니다.'
        elif io._monitor == 'dsr' and io._dsr is not None:
            state = robot_state()
            if interrupted():
                return False, '새 정지 요청으로 복구를 취소했습니다.'
            if state == 3:  # STATE_SAFE_OFF only. Do not reset other safety states.
                if interrupted():
                    return False, '새 정지 요청으로 복구를 취소했습니다.'
                reset_sent = True
                _rpc(io._cli_robot_control, io._dsr.SetRobotControl.Request(robot_control=3), 2.0)
                deadline = time.monotonic() + 3.0
                while not interrupted():
                    state = robot_state()
                    if state == 1:
                        break
                    if state != 3 or time.monotonic() >= deadline:
                        return False, f'SAFE_OFF 해제 후 STANDBY 복귀를 확인하지 못했습니다 (상태 {state}).'
                    time.sleep(0.1)
            elif state != 1:
                return False, f'현재 로봇 상태({state})는 자동 복구 대상이 아닙니다. 펜던트에서 원인을 확인하세요.'
        else:
            return False, '실제 로봇 상태를 확인할 수 없어 복구하지 않았습니다.'
        with io._stop_lock:
            if io._stop_revision != revision or io._armed or io._active is not None:
                return False, '새 정지 요청 또는 작업 상태 변경으로 복구를 취소했습니다.'
            io._stop = None
            io._last_alarm = ''
            io._stop_event.clear()
            completed = True
        return True, '복구 완료 — 로봇 STANDBY 확인. 새 작업을 시작할 수 있습니다.'
    except Exception as error:
        return False, f'복구 실패: {error}'
    finally:
        # A timeout or new emergency must not leave an unconfirmed servo reset running.
        if reset_sent and not completed:
            io._robot_move_stop_hold()
        with io._stop_lock:
            io._recovering = False
