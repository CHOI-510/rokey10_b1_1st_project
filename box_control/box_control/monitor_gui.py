#!/usr/bin/env python3
"""
monitor_gui.py — 관제 모니터 GUI (관리자·개발 확인용)
================================================================
관리: 관제(정쓰). DB/UI 연동 전까지 관제 PC에서 상황을 한눈에 보려고 만든 화면.
사용자 UI(웹 대시보드)와는 별개이며, 작업 흐름에는 끼어들지 않는다 (보기만 함 + 비상정지 버튼).

보는 것 (관제 코드를 고치지 않아도 보이는 것)
  - /event_log                       : 이벤트 로그 (관제 이벤트 + 파트 report.log)
  - /scan|judge|place/_action/status   : 파트 Goal 상태 (실행중/성공/실패/취소)
  - /scan|judge|place/_action/feedback : 파트 진행 stage, 마지막 소식 후 경과 시간
  - 로봇 상태: 관제가 살아 있으면 /control_status의 robot_state를 그대로 씀,
    관제가 없거나 robot_monitor≠dsr일 때만 직접 get_robot_state (1초 주기) + /dsr01/error 알람 (dsr_msgs2가 있을 때만)
  - 연결 상태: /emergency_stop(관제), /get_job_data(DB), 파트 Action 서버, 로봇 서비스

관제가 /control_status(std_msgs/String, JSON)를 발행하면 추가로 보는 것
  - 관제 단계(BEFORE~STOPPED), job_id, 처리 박스 수, 재고 A~D·U, 불일치·재스캔 횟수, 중단 이유
  - 없으면 이벤트 로그와 파트 Goal 상태로 단계를 '추정'해서 보여 준다.

버튼
  - 비상정지: 확인창('비상 정지를 하시겠습니까?') [정지]/[취소] → /emergency_stop (source='monitor_gui')
      → 작업 상태가 빨간 '비상정지'로 바뀜 (웹 UI와 같은 동작, 11/18)
  - 시작 / 시작 취소 (11/18, 관제 상주 모드): /job_command 호출 (source='monitor_gui')
      관제 READY → [▶ 시작] (확인창 [시작]/[취소]) → 사전 점검 → 10분 카운트다운 → 작업
      관제 BEFORE·PREFLIGHT·STANDBY → [✕ 시작 취소] → 작업 안 하고 READY
      관제 STANDBY → [⏩ 바로 시작] (확인창 [바로 시작]/[대기 후 시작]) → 카운트다운 생략
        현장 관리자 전용: 관리자 GUI는 관제 PC(로봇 옆)에서만 뜨고 작업공간을 직접 볼 수 있으므로 허용.
        웹 UI에는 없음 (DB 노드 HTTP 창구가 start_now를 받지 않음). 누가 했는지 이벤트 로그에 남음
      그 밖(작업 중·관제 없음) → 버튼 비활성. 작업 중에 멈추려면 비상정지
화면 톤은 사용자 웹 대시보드와 맞춤 (밝은 배경·흰 카드·청록, 규격 A 청록 · B 파랑 · C 주황 · D 보라).

실행
  ros2 run box_control monitor_gui
  ros2 run box_control monitor_gui --ros-args -p robot_poll:=false   (로봇 상태 조회 끔)
  (setup.py 등록 전) python3 monitor_gui.py
필요: sudo apt install python3-tk
================================================================
"""

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from box_interfaces.action import Judge, Place, Scan
from box_interfaces.msg import LogEvent
from box_interfaces.srv import EmergencyStop, GetJobData, JobCommand

from box_comm.common import (INVENTORY_KEYS, LOG_TOPIC, ROBOT, ROBOT_STATE, SAFETY, STAGES,
                             log_qos)

try:
    from box_comm.common import ROBOT_OK_STATES
except ImportError:
    ROBOT_OK_STATES = (1, 2)
try:
    from box_comm.common import stack_capacity
    CAPACITY = int(stack_capacity())
except Exception:
    CAPACITY = 6

STATUS_TOPIC = '/control_status'
PARTS = (('inventory_check', '초기 재고', Place), ('scan', '탐지', Scan),
         ('judge', '판별', Judge), ('place', '적재', Place))
MAX_LOG_ROWS = 3000
STALE_WARN_SEC = 30.0          # 파트 Feedback이 이 시간 넘게 없으면 주황색
CTRL_FRESH_SEC = 5.0           # 관제 /control_status가 이 안에 왔으면 로봇 상태는 관제 값을 씀 (직접 조회 안 함)

# ---- 색 (웹 대시보드와 같은 톤) ----
C = {
    'bg': '#eef2f6', 'card': '#ffffff', 'border': '#e3e8ee', 'tile': '#f3f5f8', 'line': '#e3e8ee',
    'text': '#1f2933', 'dim': '#8a94a3',
    'teal': '#14806f', 'teal_l': '#dff3ee',
    'green': '#16a34a', 'orange': '#d97706', 'red': '#dc2626', 'purple': '#7c3aed', 'blue': '#2f6fe0',
}
LIGHT = {'teal': '#dff3ee', 'green': '#dcfce7', 'orange': '#fdf1d8', 'red': '#fde2e2',
         'purple': '#ede4fe', 'blue': '#e3edff', 'dim': '#eceff3'}
CLASS_COLOR = {'A': '#1fb39e', 'B': '#3b82f6', 'C': '#f59e0b', 'D': '#8b5cf6', 'U': '#9aa3ae'}
# 로그 출처 배지 (배경, 글자)
SRC_BADGE = {
    'inventory_check': ('#dff3ee', '#14806f'),
    'scan': ('#fdf1d3', '#b7791f'), 'judge': ('#e3edff', '#2f6fe0'), 'place': ('#dff3ee', '#14806f'),
    'control': ('#eceff3', '#4b5563'), 'robot': ('#fde2e2', '#c42b2b'), 'monitor': ('#ede4fe', '#6d3fd6'),
}
SRC_LABEL = {'inventory_check': '초기 재고', 'scan': '탐지', 'judge': '판별', 'place': '적재',
             'control': '관제', 'robot': '로봇',
             'monitor': 'GUI'}

RUN_GOALS = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
GOAL_TEXT = {
    GoalStatus.STATUS_UNKNOWN: ('대기', 'dim'),
    GoalStatus.STATUS_ACCEPTED: ('수락됨', 'teal'),
    GoalStatus.STATUS_EXECUTING: ('실행중', 'teal'),
    GoalStatus.STATUS_CANCELING: ('취소중', 'orange'),
    GoalStatus.STATUS_SUCCEEDED: ('성공', 'green'),
    GoalStatus.STATUS_CANCELED: ('취소됨', 'orange'),
    GoalStatus.STATUS_ABORTED: ('실패', 'red'),
}

STATE_LABEL = {
    'READY': '대기 (시작 가능)', 'BEFORE': '작업전', 'PREFLIGHT': '사전점검', 'STANDBY': '시작대기', 'RUNNING': '작업중',
    'INVENTORY_CHECK': '시작 재고 점검',
    'SCANNING': '스캔중', 'SCAN_DONE': '스캔완료', 'JUDGING': '판별중', 'JUDGE_DONE': '판별완료',
    'PLACING': '적재중', 'PLACE_DONE': '적재완료', 'CLEARING': '측정존 비우기',
    'DONE': '작업완료', 'ERROR': '오류중단', 'STOPPED': '정지', 'CANCELED': '시작취소',
}
STATE_COLOR = {
    'READY': 'blue', 'BEFORE': 'dim', 'PREFLIGHT': 'orange', 'STANDBY': 'orange', 'RUNNING': 'teal',
    'INVENTORY_CHECK': 'teal',
    'SCANNING': 'teal', 'SCAN_DONE': 'teal', 'JUDGING': 'teal', 'JUDGE_DONE': 'teal',
    'PLACING': 'teal', 'PLACE_DONE': 'teal', 'CLEARING': 'orange',
    'DONE': 'green', 'ERROR': 'red', 'STOPPED': 'red', 'CANCELED': 'orange',
}
STATE_PART = {'SCANNING': 'scan', 'SCAN_DONE': 'scan', 'JUDGING': 'judge', 'JUDGE_DONE': 'judge',
              'PLACING': 'place', 'PLACE_DONE': 'place', 'INVENTORY_CHECK': 'inventory_check'}
END_STATES = ('DONE', 'ERROR', 'STOPPED', 'CANCELED', 'READY')   # READY = 상주 모드 대기 (지난 결과 유지)

# 이벤트 로그 code → 추정 단계 (/control_status가 없을 때만 사용)
CODE_STATE = {'JOB_START': 'RUNNING', 'JOB_DONE': 'DONE', 'JOB_ERROR': 'ERROR',
              'JOB_STOPPED': 'STOPPED', 'JOB_CANCELED': 'CANCELED'}
PART_STATE = {'inventory_check': 'INVENTORY_CHECK', 'scan': 'SCANNING',
              'judge': 'JUDGING', 'place': 'PLACING'}


# ======================================================================
# ROS 쪽: 구독 결과를 큐에 넣기만 한다 (화면은 Tk 스레드에서만 건드림)
# ======================================================================
class MonitorNode(Node):
    def __init__(self, q: queue.Queue):
        super().__init__('control_monitor')
        self.q = q
        self.robot_poll = bool(self.declare_parameter('robot_poll', True).value)

        self.create_subscription(LogEvent, LOG_TOPIC, lambda m: self.q.put(('log', m)), log_qos())
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, latched)

        self._action_clients = {}
        for name, _, action in PARTS:
            self.create_subscription(action.Impl.FeedbackMessage, f'/{name}/_action/feedback',
                                     lambda m, n=name: self._on_feedback(n, m), 10)
            self.create_subscription(GoalStatusArray, f'/{name}/_action/status',
                                     lambda m, n=name: self._on_goal_status(n, m), latched)
            self._action_clients[name] = ActionClient(self, action, f'/{name}')   # 연결 확인용

        self._cli_estop = self.create_client(EmergencyStop, '/emergency_stop')
        self._cli_db = self.create_client(GetJobData, '/get_job_data')            # 연결 확인용 (호출 안 함)
        self._cli_job = self.create_client(JobCommand, '/job_command')            # 시작·취소 (관제 상주 모드)

        self._dsr = None
        self._state_pending = False
        self._ctrl_robot = (0.0, None, None)    # (받은 시각, robot_monitor, robot_state) — 관제 /control_status
        if self.robot_poll:
            try:
                import dsr_msgs2.msg as dsr_msg
                import dsr_msgs2.srv as dsr_srv
                self._dsr = dsr_srv
                ns = ROBOT['service_ns'].rstrip('/')
                self._cli_robot = self.create_client(dsr_srv.GetRobotState,
                                                     f'{ns}/{SAFETY["srv_robot_state"]}')
                self.create_subscription(dsr_msg.RobotError, SAFETY['topic_error'], self._on_alarm, 10)
            except ImportError:
                self.get_logger().warn('[monitor] dsr_msgs2 없음 → 로봇 상태 표시 안 함')
        self.create_timer(1.0, self._tick)
        self.get_logger().info('[monitor] 관제 모니터 시작')

    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
            self._ctrl_robot = (time.monotonic(), st.get('robot_monitor'), st.get('robot_state'))
            self.q.put(('status', st))
        except ValueError:
            self.get_logger().warn(f'[monitor] {STATUS_TOPIC} JSON 아님: {msg.data[:80]}')

    def _on_feedback(self, name, msg):
        fb = msg.feedback
        self.q.put(('fb', name, fb.stage, fb.log_level, fb.log_msg))

    def _on_goal_status(self, name, msg):
        if not msg.status_list:
            return
        latest = max(msg.status_list,
                     key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        self.q.put(('goal', name, latest.status))

    def _on_alarm(self, msg):
        text = ' / '.join(t for t in (msg.msg1, msg.msg2, msg.msg3) if t)
        self.q.put(('alarm', int(msg.level), f'alarm {msg.group}-{msg.code}' + (f': {text}' if text else '')))

    def _tick(self):
        conn = {
            '관제': self._cli_estop.service_is_ready(),
            'DB': self._cli_db.service_is_ready(),
        }
        for name, label, _ in PARTS:
            conn[label] = self._action_clients[name].server_is_ready()
        if self._dsr is not None:
            conn['로봇'] = self._cli_robot.service_is_ready()
            seen, monitor, ctrl_state = self._ctrl_robot
            if conn['로봇'] and monitor == 'dsr' and time.monotonic() - seen < CTRL_FRESH_SEC:
                # 관제가 살아서 로봇 상태를 0.5초마다 읽고 있음 → 그 값을 그대로 씀 (컨트롤러에 같은 질의를 두 번 안 함, 11/18)
                self.q.put(('robot', ctrl_state))
            elif conn['로봇'] and not self._state_pending:
                self._state_pending = True
                fut = self._cli_robot.call_async(self._dsr.GetRobotState.Request())
                fut.add_done_callback(self._on_robot_state)
            elif not conn['로봇']:
                self.q.put(('robot', None))
        self.q.put(('conn', conn))

    def _on_robot_state(self, fut):
        self._state_pending = False
        res = fut.result()
        self.q.put(('robot', int(res.robot_state) if res is not None and res.success else None))

    def emergency_stop(self):
        if not self._cli_estop.service_is_ready():
            self.q.put(('estop', False, '/emergency_stop 서비스 없음 (관제가 안 떠 있음)'))
            return
        fut = self._cli_estop.call_async(EmergencyStop.Request(source='monitor_gui'))
        fut.add_done_callback(lambda f: self.q.put(
            ('estop', bool(f.result() and f.result().success),
             f.result().message if f.result() else '응답 없음')))

    def job_command(self, command):
        if not self._cli_job.service_is_ready():
            self.q.put(('job', command, False, '/job_command 없음 (관제가 안 떠 있거나 auto_start 모드)'))
            return
        fut = self._cli_job.call_async(JobCommand.Request(command=command, source='monitor_gui'))
        fut.add_done_callback(lambda f: self.q.put(
            ('job', command, bool(f.result() and f.result().success),
             f.result().message if f.result() else '응답 없음')))


# ======================================================================
# 화면 — 웹 대시보드(사용자 UI)와 같은 톤: 밝은 배경, 흰 카드, 청록 포인트, 규격별 색
# ======================================================================
class MonitorGui:
    def __init__(self, root: tk.Tk, node, q: queue.Queue):
        self.root, self.node, self.q = root, node, q
        self.status = None              # 마지막 /control_status
        self.status_at = 0.0
        self.standby_deadline = None
        self.inferred = 'BEFORE'        # /control_status 없을 때 추정 단계
        self.job_id = ''
        self.job_started = None         # monotonic
        self.job_ended = None           # 끝난 시각 (경과 시간 고정)
        self.reason = ''
        self.part = {n: {'goal': None, 'stage': '', 'at': None, 'msg': ''} for n, _, _ in PARTS}
        self.robot = None
        self.conn = {}
        self.log_count = {'INFO': 0, 'WARN': 0, 'ERROR': 0}
        self.rows = []                  # (level, record) — 필터 다시 그릴 때 사용

        root.title('박스 분류 관제 — 관리자 모니터')
        root.geometry('1360x860')
        root.minsize(1100, 700)
        root.configure(bg=C['bg'])
        self._fonts()
        self._build()
        root.after(100, self._drain)
        root.after(250, self._tick)

    # ---------------- 모양 ----------------
    def _fonts(self):
        fams = set(tkfont.families())
        fam = next((f for f in ('Pretendard', 'Noto Sans CJK KR', 'NanumGothic', 'Noto Sans KR', 'UnDotum')
                    if f in fams), 'TkDefaultFont')
        mono = next((f for f in ('D2Coding', 'Noto Sans Mono CJK KR', 'NanumGothicCoding', 'DejaVu Sans Mono')
                     if f in fams), 'TkFixedFont')
        self.f = {
            'title': (fam, 16, 'bold'), 'sub': (fam, 10), 'h': (fam, 13, 'bold'),
            'big': (fam, 30, 'bold'), 'n': (fam, 11), 'nb': (fam, 11, 'bold'), 's': (fam, 10),
            'sb': (fam, 10, 'bold'), 'tile': (fam, 18, 'bold'), 'pill': (fam, 12, 'bold'),
            'clock': (fam, 20, 'bold'), 'mono': (mono, 10), 'btn': (fam, 12, 'bold'),
        }
        st = ttk.Style(self.root)
        st.theme_use('clam')
        st.configure('Vertical.TScrollbar', background=C['line'], troughcolor=C['card'],
                     bordercolor=C['card'], arrowcolor=C['dim'], relief='flat')

    def card(self, parent, title=None, right=None):
        """흰 카드 (얇은 테두리). title/right가 있으면 머리줄을 만든다. 반환: (카드, 본문 프레임, 오른쪽 라벨)"""
        fr = tk.Frame(parent, bg=C['card'], highlightbackground=C['border'], highlightthickness=1)
        body = tk.Frame(fr, bg=C['card'])
        rlbl = None
        if title:
            head = tk.Frame(fr, bg=C['card'])
            head.pack(fill='x', padx=16, pady=(12, 6))
            tk.Label(head, text=title, font=self.f['h'], fg=C['text'], bg=C['card']).pack(side='left')
            rlbl = tk.Label(head, text=right or '', font=self.f['s'], fg=C['dim'], bg=C['card'])
            rlbl.pack(side='right')
        body.pack(fill='both', expand=True, padx=16, pady=(0, 12))
        return fr, body, rlbl

    def tile(self, parent, label, value='—', fg='text'):
        """회색 둥근 타일 (라벨 + 큰 값)"""
        fr = tk.Frame(parent, bg=C['tile'])
        heading = tk.Label(fr, text=label, font=self.f['s'], fg=C['dim'], bg=C['tile'], anchor='w')
        heading.pack(fill='x', padx=12, pady=(8, 0))
        if label == '남은 시간':
            self.t_time_heading = heading
        val = tk.Label(fr, text=value, font=self.f['tile'], fg=C[fg], bg=C['tile'], anchor='w')
        val.pack(fill='x', padx=12, pady=(0, 8))
        return fr, val

    # ---------------- 배치 ----------------
    def _build(self):
        r = self.root
        wrap = tk.Frame(r, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=12, pady=10)

        # ---- 머리 카드: 제목 · 시계 · 연결 칩 · 비상정지 ----
        hdr, hb, _ = self.card(wrap)
        hdr.pack(fill='x')
        hb.pack_configure(pady=10)
        icon = tk.Label(hb, text='⬡', font=('TkDefaultFont', 22, 'bold'), fg='white', bg=C['teal'],
                        width=2)
        icon.pack(side='left', ipady=4)
        tt = tk.Frame(hb, bg=C['card'])
        tt.pack(side='left', padx=12)
        tk.Label(tt, text='박스 분류 관제 · 관리자 모니터', font=self.f['title'], fg=C['text'],
                 bg=C['card'], anchor='w').pack(fill='x')
        tk.Label(tt, text='M0609 · RG2 자동 분류 · 재고 관리 — 관제 PC 전용',
                 font=self.f['sub'], fg=C['dim'], bg=C['card'], anchor='w').pack(fill='x')

        self.estop_btn = tk.Button(hb, text='■  비상정지', font=self.f['btn'], fg='white', bg=C['red'],
                                   activebackground='#ef4444', activeforeground='white', relief='flat',
                                   bd=0, padx=18, pady=10, cursor='hand2', command=self._estop)
        self.estop_btn.pack(side='right', padx=(8, 0))
        self.job_btn = tk.Button(hb, text='▶  시작', font=self.f['btn'], fg='white', bg=C['dim'],
                                 activeforeground='white', relief='flat', bd=0, padx=18, pady=10,
                                 cursor='hand2', state='disabled', command=self._job_btn)
        self.job_btn.pack(side='right', padx=(8, 0))
        self._job_mode = None             # 'start' / 'cancel' / None (비활성)
        self.now_btn = tk.Button(hb, text='⏩  바로 시작', font=self.f['btn'], fg='white', bg=C['teal'],
                                 activebackground=C['teal'], activeforeground='white', relief='flat', bd=0,
                                 padx=18, pady=10, cursor='hand2', command=self._start_now)
        self._now_shown = False           # STANDBY에서만 보임
        chips = tk.Frame(hb, bg=C['card'])
        chips.pack(side='right')
        self.conn_lamps = {}
        for name in ('관제', 'DB', '탐지', '판별', '적재', '로봇'):
            chip = tk.Label(chips, text=f'● {name}', font=self.f['s'], fg=C['dim'], bg=C['tile'],
                            padx=7, pady=5)
            chip.pack(side='left', padx=3)
            self.conn_lamps[name] = chip
        self.robot_lbl = tk.Label(chips, text='로봇 —', font=self.f['s'], fg=C['dim'], bg=C['tile'],
                                  padx=7, pady=5)
        self.robot_lbl.pack(side='left', padx=3)
        clock = tk.Frame(hb, bg=C['card'])
        clock.pack(side='right', padx=16)
        self.date_lbl = tk.Label(clock, text='', font=self.f['s'], fg=C['dim'], bg=C['card'])
        self.date_lbl.pack()
        self.clock_lbl = tk.Label(clock, text='', font=self.f['clock'], fg=C['text'], bg=C['card'])
        self.clock_lbl.pack()

        # ---- 가운데 줄: 작업 상태 | 파트 진행 ----
        mid = tk.Frame(wrap, bg=C['bg'])
        mid.pack(fill='x', pady=(10, 0))
        mid.columnconfigure(0, weight=5, uniform='mid')
        mid.columnconfigure(1, weight=7, uniform='mid')

        sc, sb, self.job_lbl = self.card(mid, '작업 상태', 'job —')
        sc.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        row = tk.Frame(sb, bg=C['card'])
        row.pack(fill='x', pady=(0, 8))
        self.dot = tk.Canvas(row, width=26, height=26, bg=C['card'], highlightthickness=0)
        self.dot_halo = self.dot.create_oval(1, 1, 25, 25, fill=C['teal_l'], outline='')
        self.dot_core = self.dot.create_oval(6, 6, 20, 20, fill=C['teal'], outline='')
        self.dot.pack(side='left', padx=(0, 10))
        self.state_lbl = tk.Label(row, text='연결 대기', font=self.f['big'], fg=C['text'], bg=C['card'])
        self.state_lbl.pack(side='left')
        self.state_sub = tk.Label(row, text='', font=self.f['s'], fg=C['dim'], bg=C['card'])
        self.state_sub.pack(side='left', padx=12, pady=(14, 0))

        pills = tk.Frame(sb, bg=C['card'])
        pills.pack(fill='x', pady=(0, 8))
        self.pills = {}
        for i, (name, label, _) in enumerate(PARTS):
            p = tk.Label(pills, text=label, font=self.f['pill'], fg=C['dim'], bg=C['tile'], pady=8)
            p.grid(row=0, column=i, sticky='we', padx=(0 if i == 0 else 6, 0))
            pills.columnconfigure(i, weight=1, uniform='p')
            self.pills[name] = p

        tiles = tk.Frame(sb, bg=C['card'])
        tiles.pack(fill='x')
        t1, self.t_done = self.tile(tiles, '처리 완료', '—')
        t2, self.t_time = self.tile(tiles, '남은 시간', '—')
        t3, self.t_cond = self.tile(tiles, '대기 상태', '—')
        t4, self.t_box = self.tile(tiles, '현재 박스', '—')
        t5, self.t_retry = self.tile(tiles, '집기 실패 재스캔', '—', 'orange')
        t6, self.t_mis = self.tile(tiles, '재고 불일치', '—', 'green')
        for i, t in enumerate((t1, t2, t3, t4, t5, t6)):
            t.grid(row=i // 3, column=i % 3, sticky='nsew', padx=(0 if i % 3 == 0 else 6, 0),
                   pady=(0 if i < 3 else 6, 0))
        for i in range(3):
            tiles.columnconfigure(i, weight=1, uniform='t')
        self.reason_lbl = tk.Label(sb, text='', font=self.f['s'], fg=C['red'], bg=C['card'], anchor='w',
                                   justify='left', wraplength=480)
        self.reason_lbl.pack(fill='x', pady=(6, 0))

        pc, pb, _ = self.card(mid, '파트 진행', 'Action Goal · Feedback stage')
        pc.grid(row=0, column=1, sticky='nsew')
        self.part_w = {}
        for i, (name, label, _) in enumerate(PARTS):
            col = tk.Frame(pb, bg=C['tile'])
            col.grid(row=0, column=i, sticky='nsew', padx=(0 if i == 0 else 8, 0))
            pb.columnconfigure(i, weight=1, uniform='pc')
            pb.rowconfigure(0, weight=1)
            head = tk.Frame(col, bg=C['tile'])
            head.pack(fill='x', padx=10, pady=(8, 4))
            bg_, fg_ = SRC_BADGE[name]
            tk.Label(head, text=f' {label} ', font=self.f['sb'], fg=fg_, bg=bg_).pack(side='left')
            tk.Label(head, text=f'/{name}', font=self.f['s'], fg=C['dim'], bg=C['tile']).pack(side='left', padx=4)
            goal = tk.Label(head, text=' 대기 ', font=self.f['sb'], fg=C['dim'], bg=C['card'])
            goal.pack(side='right')
            stages = {}
            for st_name in STAGES.get(name, ()):
                lb = tk.Label(col, text=f'   {st_name}', font=self.f['s'], fg=C['dim'], bg=C['tile'],
                              anchor='w')
                lb.pack(fill='x', padx=6)
                stages[st_name] = lb
            ago = tk.Label(col, text='소식 —', font=self.f['s'], fg=C['dim'], bg=C['tile'], anchor='w')
            ago.pack(fill='x', padx=10, pady=(6, 0))
            msg = tk.Label(col, text='', font=self.f['s'], fg=C['text'], bg=C['tile'], anchor='w',
                           justify='left', wraplength=230)
            msg.pack(fill='x', padx=10, pady=(2, 8))
            self.part_w[name] = {'goal': goal, 'stages': stages, 'ago': ago, 'msg': msg}

        # ---- 아래 줄: 적재 현황 | 작업 로그 ----
        low = tk.Frame(wrap, bg=C['bg'])
        low.pack(fill='both', expand=True, pady=(10, 0))
        low.columnconfigure(0, weight=5, uniform='low')
        low.columnconfigure(1, weight=7, uniform='low')
        low.rowconfigure(0, weight=1)

        ic, ib, _ = self.card(low, '적재 현황', f'규격별 수량 · 정원 {CAPACITY}')
        ic.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        self.inv_w = {}
        for k in INVENTORY_KEYS:
            line = tk.Frame(ib, bg=C['card'])
            line.pack(fill='x', pady=4)
            tk.Frame(ib, bg=C['border'], height=1).pack(fill='x')
            sw = tk.Canvas(line, width=14, height=14, bg=C['card'], highlightthickness=0)
            sw.create_rectangle(1, 1, 13, 13, fill=CLASS_COLOR[k], outline='')
            sw.pack(side='left', padx=(0, 8))
            tk.Label(line, text='미분류' if k == 'U' else f'규격 {k}', font=self.f['n'], fg=C['text'],
                     bg=C['card'], width=7, anchor='w').pack(side='left')
            num = tk.Label(line, text='—', font=self.f['h'], fg=C['text'], bg=C['card'], width=3, anchor='e')
            num.pack(side='left')
            cap = tk.Label(line, text='' if k == 'U' else f'/ {CAPACITY}', font=self.f['s'], fg=C['dim'],
                           bg=C['card'], width=5, anchor='w')
            cap.pack(side='left', padx=(2, 8))
            bar = None
            if k != 'U':
                bar = tk.Canvas(line, height=10, bg=C['card'], highlightthickness=0)
                bar.pack(side='left', fill='x', expand=True, padx=(0, 4))
                bar.bind('<Configure>', lambda e, kk=k: self._draw_bar(kk))
            self.inv_w[k] = {'num': num, 'bar': bar, 'n': None}
        foot = tk.Frame(ib, bg=C['card'])
        foot.pack(fill='x', pady=(10, 0))
        tk.Label(foot, text='전체 적재', font=self.f['n'], fg=C['dim'], bg=C['card']).pack(side='left')
        self.total_lbl = tk.Label(foot, text='—', font=self.f['tile'], fg=C['text'], bg=C['card'])
        self.total_lbl.pack(side='right')

        lc, lb, self.count_lbl = self.card(low, '작업 로그', '/event_log')
        lc.grid(row=0, column=1, sticky='nsew')
        bar = tk.Frame(lb, bg=C['card'])
        bar.pack(fill='x', pady=(0, 6))
        self.show = {}
        for lv in ('INFO', 'WARN', 'ERROR'):
            v = tk.BooleanVar(value=True)
            tk.Checkbutton(bar, text=lv, variable=v, command=self._refilter, font=self.f['s'],
                           fg=C['text'], bg=C['card'], selectcolor=C['card'], activebackground=C['card'],
                           highlightthickness=0, bd=0).pack(side='left', padx=(0, 8))
            self.show[lv] = v
        self.follow = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text='자동 스크롤', variable=self.follow, font=self.f['s'], fg=C['text'],
                       bg=C['card'], selectcolor=C['card'], activebackground=C['card'],
                       highlightthickness=0, bd=0).pack(side='left', padx=(8, 0))
        tk.Button(bar, text='지우기', font=self.f['s'], command=self._clear_log, relief='flat', bd=0,
                  bg=C['tile'], fg=C['text'], padx=10, pady=2, cursor='hand2').pack(side='right')

        tf = tk.Frame(lb, bg=C['card'])
        tf.pack(fill='both', expand=True)
        self.text = tk.Text(tf, font=self.f['n'], bg=C['card'], fg=C['text'], relief='flat', bd=0,
                            wrap='none', cursor='arrow', spacing1=3, spacing3=3, highlightthickness=0,
                            tabs=(self._px(76), self._px(140)))
        sbar = ttk.Scrollbar(tf, orient='vertical', command=self.text.yview)
        self.text.configure(yscrollcommand=sbar.set)
        self.text.pack(side='left', fill='both', expand=True)
        sbar.pack(side='right', fill='y')
        self.text.tag_configure('time', foreground=C['dim'], font=self.f['mono'])
        for name, (bg_, fg_) in SRC_BADGE.items():
            self.text.tag_configure(f'src_{name}', background=bg_, foreground=fg_, font=self.f['sb'])
        self.text.tag_configure('msg_WARN', foreground=C['orange'])
        self.text.tag_configure('msg_ERROR', foreground=C['red'], font=self.f['nb'])
        self.text.tag_configure('meta', foreground=C['dim'], font=self.f['s'])
        self.text.tag_configure('sep', foreground=C['dim'], justify='center', font=self.f['s'])
        self.text.configure(state='disabled')

    def _px(self, n):
        return f'{n}p'

    def _draw_bar(self, k):
        w = self.inv_w[k]
        cv, n = w['bar'], w['n'] or 0
        cv.delete('all')
        width, h = cv.winfo_width(), 10
        if width < 10:
            return
        cv.create_rectangle(0, 0, width, h, fill=C['tile'], outline='')
        fill = int(width * min(n, CAPACITY) / max(CAPACITY, 1))
        if fill > 0:
            cv.create_rectangle(0, 0, fill, h, fill=C['red'] if n > CAPACITY else CLASS_COLOR[k], outline='')

    # ---------------- 큐 처리 ----------------
    def _drain(self):
        if not rclpy.ok():                 # 터미널 Ctrl+C → rclpy 종료 → 창도 닫음
            self.root.destroy()
            return
        try:
            for _ in range(500):
                item = self.q.get_nowait()
                getattr(self, f'_on_{item[0]}')(*item[1:])
        except queue.Empty:
            pass
        self.root.after(100, self._drain)

    def _new_job(self, job_id):
        if self.job_id and job_id != self.job_id:
            self._add_row('INFO', {'sep': f'──────  새 작업 {job_id}  ──────'})
        self.job_id = job_id
        self.reason, self.job_started, self.job_ended = '', None, None

    def _on_status(self, st):
        self.status, self.status_at = st, time.monotonic()
        if st.get('state') == 'STANDBY':
            self.standby_deadline = float(st.get('standby_until') or
                                          (time.time() + max(0.0, float(st.get('standby_sec') or 0))))
        elif st.get('state') != 'STANDBY':
            self.standby_deadline = None
        if st.get('job_id') and st['job_id'] != self.job_id:
            self._new_job(st['job_id'])
        state = st.get('state')
        if state == 'RUNNING' and self.job_started is None:
            self.job_started = time.monotonic()
        if state in END_STATES and self.job_started and self.job_ended is None:
            self.job_ended = time.monotonic()
        if st.get('reason'):
            self.reason = st['reason']
        inv = st.get('inventory') or {}
        total = 0
        for k, w in self.inv_w.items():
            if k in inv:
                n = int(inv[k])
                w['n'] = n
                w['num'].config(text=str(n), fg=C['red'] if k != 'U' and n > CAPACITY else C['text'])
                if w['bar'] is not None:
                    total += n
                    self._draw_bar(k)
        if inv:
            self.total_lbl.config(text=f'{total} / {CAPACITY * (len(INVENTORY_KEYS) - 1)}')
        if 'retries' in st:
            self.t_retry.config(text=str(st['retries']))
        if 'mismatches' in st:
            self.t_mis.config(text=str(st['mismatches']), fg=C['orange'] if st['mismatches'] else C['green'])

    def _on_log(self, m):
        if m.job_id and m.job_id != self.job_id:
            self._new_job(m.job_id)
        if m.code in CODE_STATE:
            self.inferred = CODE_STATE[m.code]
            if m.code == 'JOB_START':
                self.job_started, self.job_ended = time.monotonic(), None
            elif self.job_started and self.job_ended is None:
                self.job_ended = time.monotonic()
            if m.code in ('JOB_ERROR', 'JOB_STOPPED', 'JOB_CANCELED'):
                self.reason = m.message
        lv = m.level if m.level in self.log_count else 'INFO'
        t = m.stamp.split(' ')[-1] if m.stamp else time.strftime('%H:%M:%S')
        box = m.box_id.split('_')[-1] if m.box_id else ''
        meta = ' · '.join(x for x in (m.code, f'#{box}' if box else '', m.class_id, m.stage) if x)
        self._add_row(lv, {'time': t, 'src': m.source, 'msg': m.message, 'meta': meta})
        if m.source in self.part and m.message:
            self.part[m.source]['msg'] = m.message

    def _on_fb(self, name, stage, level, msg):
        p = self.part[name]
        p['stage'], p['at'] = stage, time.monotonic()

    def _on_goal(self, name, status):
        p = self.part[name]
        if status != p['goal'] and status in RUN_GOALS and p['goal'] not in RUN_GOALS:
            p['stage'], p['at'], p['msg'] = 'goal_sent', time.monotonic(), ''   # 새 Goal 시작
        p['goal'] = status

    def _on_alarm(self, level, text):
        lv = 'ERROR' if level >= 3 else 'WARN' if level == 2 else 'INFO'
        self._add_row(lv, {'time': time.strftime('%H:%M:%S'), 'src': 'robot', 'msg': text,
                           'meta': 'DSR_ALARM'})

    def _on_robot(self, state):
        self.robot = state

    def _on_conn(self, conn):
        self.conn = conn

    def _on_job(self, command, ok, message):
        what = {'start': '시작', 'cancel': '시작 취소', 'start_now': '바로 시작'}.get(command, command)
        self._add_row('INFO' if ok else 'WARN', {'time': time.strftime('%H:%M:%S'), 'src': 'monitor',
                                                'msg': f'{what} 요청 → {"수락" if ok else "거절"}: {message}',
                                                'meta': 'JOB_COMMAND'})

    def _on_estop(self, ok, message):
        self._add_row('WARN' if ok else 'ERROR', {'time': time.strftime('%H:%M:%S'), 'src': 'monitor',
                                                 'msg': f'비상정지 요청 → {message}', 'meta': 'ESTOP_SENT'})

    # ---------------- 주기 갱신 ----------------
    def _tick(self):
        now = time.monotonic()
        now_wall = time.time()
        self.date_lbl.config(text=time.strftime('%m월 %d일 ').lstrip('0') + '월화수목금토일'[time.localtime().tm_wday] + '요일')
        self.clock_lbl.config(text=time.strftime('%H:%M:%S'))

        # 관제 단계
        live = self.status is not None
        st = self.status or {}
        state = st.get('state') or self.inferred
        if not live:
            for name, _, _ in PARTS:               # 실행 중인 파트로 단계 추정
                if self.part[name]['goal'] in RUN_GOALS:
                    state = PART_STATE[name]
        if not self.conn.get('관제') and state not in END_STATES:
            text, color, sub = '관제 없음', 'dim', '/emergency_stop 서비스가 안 보임'
        else:
            text = STATE_LABEL.get(state, state)
            color = STATE_COLOR.get(state, 'dim')
            sub = state + ('' if live else ' · 로그로 추정')
            if live and state == 'STANDBY' and self.standby_deadline is not None:
                remain = self.standby_deadline - now_wall
                if remain > 0:
                    sub = f'시작까지 {remain:.0f}초 · 관제 터미널 q/s · UI 취소'
        self.state_lbl.config(text=text)
        stop = st.get('stop')
        if state == 'STOPPED' or (state == 'READY' and stop):    # 비상정지 → 빨간불 '비상정지'
            emergency = 'EMERGENCY' in (self.reason or '') or (stop and stop[0] == 'EMERGENCY_STOP')
            text, color = ('비상정지' if emergency else '정지'), 'red'
            if state == 'READY':
                text += ' · 대기'
            self.state_lbl.config(text=text)
        self.state_sub.config(text=sub)
        self._update_job_btn(state, live)
        self.dot.itemconfig(self.dot_core, fill=C[color])
        self.dot.itemconfig(self.dot_halo, fill=LIGHT.get(color, C['tile']))

        # 스캔 · 판별 · 적재 알약 (지난 단계 연한 청록, 지금 진한 청록, 남은 단계 회색)
        cur = STATE_PART.get(state)
        order = [n for n, _, _ in PARTS]
        for i, name in enumerate(order):
            if cur is None:
                bg_, fg_ = C['tile'], C['dim']
            elif name == cur:
                bg_, fg_ = C['teal'], 'white'
            elif i < order.index(cur):
                bg_, fg_ = C['teal_l'], C['teal']
            else:
                bg_, fg_ = C['tile'], C['dim']
            self.pills[name].config(bg=bg_, fg=fg_)

        # 타일
        self.job_lbl.config(text=f'job {self.job_id}' if self.job_id else 'job —')
        self.t_done.config(text=f'{st["box_seq"]}개' if 'box_seq' in st else '—')
        if st.get('box_id') and state not in END_STATES:
            self.t_box.config(text=f'#{str(st["box_id"]).split("_")[-1]}'
                              + (f' [{st["class_id"]}]' if st.get('class_id') else ''))
        else:
            self.t_box.config(text='—')
        if state == 'STANDBY' and self.standby_deadline is not None:
            remain = max(0, int(self.standby_deadline - now_wall + 0.999))
            self.t_time_heading.config(text='남은 시간')
            self.t_time.config(text=f'{remain // 60:02d}:{remain % 60:02d}')
        elif self.job_started is not None:
            self.t_time_heading.config(text='경과 시간')
            sec = int((self.job_ended or now) - self.job_started)
            self.t_time.config(text=f'{sec // 3600:d}:{sec // 60 % 60:02d}:{sec % 60:02d}' if sec >= 3600
                               else f'{sec // 60:02d}:{sec % 60:02d}')
        else:
            self.t_time_heading.config(text='남은 시간')
            self.t_time.config(text='—')
        stop = st.get('stop')
        if st.get('inventory_needs_recheck'):
            self.t_cond.config(text='재고 재측정 필요', fg=C['red'])
        elif state in ('ERROR', 'STOPPED') or stop:
            self.t_cond.config(text='정지' if (state == 'STOPPED' or stop) else '오류', fg=C['red'])
        elif state == 'CANCELED':
            self.t_cond.config(text='시작 취소', fg=C['orange'])
        elif any(self.part[n]['goal'] in RUN_GOALS and self.part[n]['at']
                 and now - self.part[n]['at'] > STALE_WARN_SEC for n in self.part):
            self.t_cond.config(text='지연', fg=C['orange'])
        else:
            self.t_cond.config(text='정상', fg=C['text'])
        reason = self.reason or (f'정지 요청: {stop[0]} {stop[1] if len(stop) > 1 else ""}' if stop else '')
        self.reason_lbl.config(text=reason)

        # 연결 칩
        for name, chip in self.conn_lamps.items():
            if name not in self.conn:
                chip.config(fg=C['dim'], text=f'○ {name}')
            else:
                chip.config(fg=C['green'] if self.conn[name] else C['red'], text=f'● {name}')
        if '로봇' in self.conn:
            if self.robot is None:
                self.robot_lbl.config(text='로봇 응답 없음', fg=C['red'] if self.conn['로봇'] else C['dim'])
            else:
                name = ROBOT_STATE.get(self.robot, str(self.robot))
                self.robot_lbl.config(text=f'{name} ({self.robot})',
                                      fg=C['green'] if self.robot in ROBOT_OK_STATES else C['red'])
        else:
            self.robot_lbl.config(text='로봇 조회 꺼짐', fg=C['dim'])

        # 파트 진행
        for name, _, _ in PARTS:
            p, w = self.part[name], self.part_w[name]
            gtext, gcolor = GOAL_TEXT.get(p['goal'], ('대기', 'dim'))
            w['goal'].config(text=f' {gtext} ', fg=C[gcolor], bg=LIGHT.get(gcolor, C['card']))
            running = p['goal'] in RUN_GOALS
            for st_name, lb in w['stages'].items():
                on = st_name == p['stage']
                lb.config(text=(' ▶ ' if on else '   ') + st_name,
                          fg='white' if on and running else (C['text'] if on else C['dim']),
                          bg=C['teal'] if on and running else C['tile'])
            if p['at'] is None:
                w['ago'].config(text='소식 —', fg=C['dim'])
            else:
                sec = now - p['at']
                stale = running and sec > STALE_WARN_SEC
                w['ago'].config(text=f'{p["stage"]} · {sec:.0f}초 전', fg=C['orange'] if stale else C['dim'])
            w['msg'].config(text=p['msg'])

        self.count_lbl.config(text=f'INFO {self.log_count["INFO"]} · WARN {self.log_count["WARN"]} · '
                                   f'ERROR {self.log_count["ERROR"]}')
        self.root.after(250, self._tick)

    # ---------------- 로그 ----------------
    def _add_row(self, level, rec):
        if 'sep' not in rec:
            self.log_count[level] += 1
        self.rows.append((level, rec))
        if len(self.rows) > MAX_LOG_ROWS:
            self.rows = self.rows[-MAX_LOG_ROWS:]
            self._refilter()
            return
        if 'sep' in rec or self.show[level].get():
            self._write(level, rec)

    def _write(self, level, rec):
        t = self.text
        t.configure(state='normal')
        if 'sep' in rec:
            t.insert('end', rec['sep'] + '\n', 'sep')
        else:
            src = rec['src'] if rec['src'] in SRC_BADGE else 'control'
            label = SRC_LABEL.get(rec['src'], rec['src'])
            t.insert('end', rec['time'], 'time')
            t.insert('end', '\t')
            t.insert('end', f'  {label}  ', f'src_{src}')
            t.insert('end', '\t')
            t.insert('end', rec['msg'], f'msg_{level}' if level != 'INFO' else ())
            if rec.get('meta'):
                t.insert('end', f'   {rec["meta"]}', 'meta')
            t.insert('end', '\n')
        t.configure(state='disabled')
        if self.follow.get():
            t.see('end')

    def _refilter(self):
        self.text.configure(state='normal')
        self.text.delete('1.0', 'end')
        self.text.configure(state='disabled')
        for level, rec in self.rows:
            if 'sep' in rec or self.show[level].get():
                self._write(level, rec)

    def _clear_log(self):
        self.rows.clear()
        self._refilter()
        self.log_count = {k: 0 for k in self.log_count}

    def _ask(self, title, message, ok_text, cancel_text, ok_color):
        """확인창: [ok_text] / [cancel_text]. ok면 True. 창을 닫거나 Esc도 취소."""
        win = tk.Toplevel(self.root)
        win.title(title)
        win.configure(bg=C['card'])
        win.resizable(False, False)
        win.transient(self.root)
        result = {'ok': False}
        tk.Label(win, text=title, font=self.f['h'], fg=C[ok_color], bg=C['card']).pack(
            anchor='w', padx=24, pady=(20, 6))
        tk.Label(win, text=message, font=self.f['n'], fg=C['text'], bg=C['card'], justify='left').pack(
            anchor='w', padx=24, pady=(0, 18))
        bar = tk.Frame(win, bg=C['card'])
        bar.pack(fill='x', padx=24, pady=(0, 20))

        def close(ok):
            result['ok'] = ok
            win.destroy()
        tk.Button(bar, text=cancel_text, font=self.f['btn'], fg=C['text'], bg=C['tile'], relief='flat', bd=0,
                  padx=16, pady=8, cursor='hand2', command=lambda: close(False)).pack(side='right')
        tk.Button(bar, text=ok_text, font=self.f['btn'], fg='white', bg=C[ok_color],
                  activebackground=C[ok_color], activeforeground='white', relief='flat', bd=0,
                  padx=16, pady=8, cursor='hand2', command=lambda: close(True)).pack(side='right', padx=(0, 8))
        win.bind('<Escape>', lambda _e: close(False))
        win.protocol('WM_DELETE_WINDOW', lambda: close(False))
        win.update_idletasks()
        x = self.root.winfo_rootx() + (self.root.winfo_width() - win.winfo_width()) // 2
        y = self.root.winfo_rooty() + (self.root.winfo_height() - win.winfo_height()) // 3
        win.geometry(f'+{max(x, 0)}+{max(y, 0)}')
        win.grab_set()
        win.focus_set()
        self.root.wait_window(win)
        return result['ok']

    def _estop(self):
        if self._ask('비상 정지', '비상 정지를 하시겠습니까?\n로봇이 현재 위치에서 정지합니다.',
                     '정지', '취소', 'red'):
            self.node.emergency_stop()

    def _start_now(self):
        if self._ask('바로 시작', '카운트다운을 기다리지 않고 바로 시작하시겠습니까?\n'
                     '작업공간에 사람이나 파손 위험 물품이 없는지 직접 확인하세요.\n'
                     '(현장 관리자 전용 · 이벤트 로그에 기록됩니다)',
                     '바로 시작', '대기 후 시작', 'teal'):
            self.node.job_command('start_now')

    def _job_btn(self):
        mode = self._job_mode
        if mode == 'start':
            if self._ask('작업 시작', '10분 후 작업이 시작됩니다. 작업공간 내 사람 또는 파손 위험이 있는 물품이 없는지 확인해주세요.',
                         '확인', '취소', 'teal'):
                self.node.job_command('start')
        elif mode == 'cancel':
            self.node.job_command('cancel')

    def _update_job_btn(self, state, live):
        """관제 단계에 맞춰 시작/취소 버튼 모양 (관제가 /control_status를 보내고 있을 때만 활성)"""
        if live and self.conn.get('관제') and state == 'READY':
            mode, text, bg = 'start', '▶  시작', C['teal']
        elif live and self.conn.get('관제') and state in ('BEFORE', 'PREFLIGHT', 'STANDBY'):
            mode, text, bg = 'cancel', '✕  시작 취소', C['orange']
        else:
            mode, text, bg = None, '▶  시작', C['dim']
        if mode != self._job_mode:
            self._job_mode = mode
            self.job_btn.config(text=text, bg=bg, activebackground=bg,
                                state='normal' if mode else 'disabled')
        show_now = bool(live and self.conn.get('관제') and state == 'STANDBY')
        if show_now != self._now_shown:
            self._now_shown = show_now
            if show_now:
                self.now_btn.pack(side='right', padx=(8, 0), before=self.job_btn)
            else:
                self.now_btn.pack_forget()


def main(args=None):
    rclpy.init(args=args)
    q = queue.Queue()
    node = MonitorNode(q)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except (ExternalShutdownException, Exception):
            pass
    threading.Thread(target=spin, daemon=True).start()

    root = tk.Tk()
    MonitorGui(root, node, q)
    root.protocol('WM_DELETE_WINDOW', root.destroy)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
