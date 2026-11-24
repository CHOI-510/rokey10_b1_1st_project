# 진행 현황

관제가 관리합니다. 각 파트는 진행 상황을 관제에게 알려 주세요.

> 마지막 갱신: **11/24** (`de34e4f` 기준). 시스템 구조·실행 방법은 [`README.md`](../README.md)를 봅니다.

---

## 1. 파트별 상태

> 상태: `fake` → `구현 중` → `단독 테스트 완료` → `통합 완료`

| 파트 | 담당 | 상태 | 현재 작업 / 비고 |
| --- | --- | --- | --- |
| 관제 | 최은정 | 통합&nbsp;완료 | 상주 모드(READY · 웹/GUI 시작·취소·바로 시작) · 사전 점검 · 시작 대기 · 시작 설정 · 재고 검증 · TCP·툴 자동 확인 · 정지 감시 · 집기 실패 재스캔 · TOOL_SWEEP 측정존 복구 · **SAFE_OFF 원격 복구(`recover`)** · 관리자 GUI · launch 일원화(`box_bringup`). 남음: `progress_timeout_sec` 실제 값, robot-api 접근 제한(이슈 29), 웹 바로 시작 허용 여부 확정(이슈 30) |
| 탐지 | 김창균 | 통합&nbsp;완료 | Y 본탐색 뒤 X 확인 탐색, 반원 윗면 탐침, 충돌 정지점 집기 1회. 측정존에 순응·힘 제어로 내려놓은 뒤 WAYPOINT로 복귀. Goal 사이 탐색 재개 상태 유지. 남음: 측정존 내려놓기 접촉 민감도 실물 확인, 스캔공간 막힘 처리(이슈 22) |
| 판별 | 송현석 | 통합&nbsp;완료 | 높이·가로·세로 측정, 위상 정렬, 물성(C/D: 5→40 N 폭 변화, 0.8 mm 이상 말랑), 집기 재시도. 높이 측정 `h_floor_offset=-1.16 mm`, flip 허용 2 mm, 높이 허용 3 mm·가로/세로 2.5 mm. 측정존이 비었으면 `NOT_FOUND` → 관제 복구 |
| 적재 | 정수현 | 통합&nbsp;완료 | sync 집기, 층별 적재, 미분류 U, 시작 재고 실측(`inventory_check`). 접촉 3 N·절대 상한 25 N, TRAVEL_Z 수평 이동 100 mm/s. 남음: B·C 이동 특이점 경로 확인, `test_place_pick.py` 가짜 로봇 갱신(이슈 31) |
| DB·UI | 이승재 | 통합&nbsp;완료 | DB 노드 Supabase 모드(기존 `product`·`inventory`·`loc`·`log` 스키마, state 테이블 미사용) · LogSync + 재고 트리거 · 웹 대시보드(`box_ui/robot-ui`, 3D 로봇 뷰) · FastAPI(`box_ui/robot-api`, rosbridge·텔레메트리·교정값). 남음: `DB_UI_CONTRACT.md` 최신화(이슈 32) |

---

## 2. 결정 사항

| 날짜 | 결정 | 이유 |
| --- | --- | --- |
| 11/24 | **`recover` 경로 수정**: `JOB_COMMANDS`에 `recover` 추가(DB 노드 HTTP 경로에서 "알 수 없는 명령"으로 거절되던 버그), recover만 응답 대기 10 s(DB 노드) / 12 s(robot-api `ROSBRIDGE_TIMEOUT`·`BOX_API_TIMEOUT`), 복구 후 DB 상태 초기화 실패 시 경고 로그, `arm_monitor`가 `_stop_lock` 안에서 `_armed` 변경. 사용하지 않던 `box_place/common_SH.py` 삭제 | SAFE_OFF 해제 + STANDBY 확인이 최대 약 7초라 3 s 대기로는 성공해도 UI에 "응답 없음"이 뜸 |
| 11/24 | **README 전면 재작성 + `images/` 다이어그램 5종** (시스템 설계도 · 플로우 차트 · HW/SW 아키텍처 · 노드 구성도 · 작업 공간 배치도). 세부 표는 `<details>`로 접음 | 초기 틀에 덧붙이던 README가 최신 코드와 어긋나고 읽기 어려워짐 |
| 11/23 | **웹 폴더 정리**: `box_ui/my-next-app` → `box_ui/robot-ui`, 루트 `robot-api` → `box_ui/robot-api`. `box_ui/COLCON_IGNORE` 유지. 서버 설정은 `box_ui/robot-ui/.env.local`(DB 노드 `server_settings.py`가 읽음, `BOX_SERVER_ENV_FILE`로 지정 가능) | 웹 코드를 한 폴더로 모아 관리. 서버 키를 ROS 터미널마다 export하지 않아도 되게 |
| 11/22 | **SAFE_OFF 원격 복구 `recover`** (`box_comm/safe_off_recovery.py`): READY이고 작업이 없을 때만, 로봇 상태가 SAFE_OFF(3)일 때만 `set_robot_control(3)` → STANDBY 확인 후 정지 래치 해제. 이미 STANDBY면 래치만 해제, 그 밖의 안전 상태(비상정지 6, SAFE_STOP 5 등)는 펜던트로. 복구 중 새 정지 요청이 오면 취소하고 정지 재발행(`_stop_revision`) | 웹 비상정지 뒤 현장 PC로 가지 않고 재개. 하드웨어 비상정지는 원격으로 풀지 않음 |
| 11/22 | **웹 UI에서도 바로 시작(`start_now`) 허용**: 관제의 `source='ui'` 거절 제거 (11/18 "바로 시작은 현장 전용" 대체). rosbridge 경로로만 열려 있고 DB 노드 HTTP 경로(`REMOTE_JOB_COMMANDS`)에는 없음 | DB·UI 담당 변경. **관제 검토 중 — 이슈 30** |
| 11/22 | **Supabase 모드 = 기존 스키마 그대로** (`db_store:=supabase`): `product`(name, len, weight) · `inventory`(loc_id, product_name, cnt) · `loc`(id, coords, inventory_calibration) · `log`. **state 테이블 없음** — 실시간 단계·시각은 DB 노드 메모리와 `/control_status`. 재고는 구역별 절대값 PATCH, supabase SDK 대신 표준 `urllib` | 이미 운영 중인 테이블을 바꾸지 않고 연동. SDK 설치 의존 제거 |
| 11/21 | **로그 → Supabase → 트리거로 재고 갱신**: DB 노드가 `~/box_logs/*.jsonl` 원본을 유지하고 `LogSync`가 키워드 포함 줄만 `log`에 INSERT(중복 무시). `sql/log_inventory_trigger.sql`이 메시지에서 재고를 **절대값**으로 파싱해 `inventory` 갱신, 구역별 watermark(`inventory_log_positions`)로 과거 로그 재전송 무시. 파일 모드에서는 `log-sync` 컨테이너가 같은 일을 함 | 웹 화면이 꺼져도 기록이 계속되고, +1 방식이 아니라 중복 전송돼도 재고가 틀어지지 않음 |
| 11/21 | **로봇 텔레메트리·위치 API** (robot-api): `/robot/telemetry`는 rosbridge로 `get_current_posj/posx`만 읽음(움직이는 명령 없음), `/robot/locations`·`/robot/inventory/calibration`은 서버 키로 `loc`을 읽어 UI·`inventory_check`에 제공 | 브라우저에 서비스 키를 주지 않고 3D 뷰·교정값 제공 |
| 11/20 오후 | **박스별 재고 DB 기록 + 시작 팝업 재고 확인** (오전 "수동 재고 설정 폐기"를 대체): 관제가 박스마다 `UpdateInventory`, 작업 끝에 최종 스냅샷. 시작 재고 실측은 **선택** — launch `inventory_check:=false`(기본)면 DB/UI 재고를 그대로 쓰고, `true`면 A~D 실측. launch 시작 전에 DB HTTP(8765)·rosbridge(9090) 포트 사용 여부 확인(`runtime_guard`) | 실측은 시간이 걸려 시연 흐름에서는 선택으로. 웹이 실시간 재고를 보여 주려면 중간 기록이 필요 |
| 11/20 | **측정존 비우기 = 도구 쓸기 `TOOL_SWEEP`**: 판별 `NOT_FOUND`·판별/적재 `GRASP_FAILED` 뒤 관제가 고정 경로로 도구를 집어 측정존을 쓸고 반납. 성공 = 경로 완료, 잔류 물체는 다음 탐지에서 확인. 도구를 쥔 채 실패하면 자동 개방하지 않음 | 밀어내기 경로(`push_path`) 대신 실물에서 검증된 도구 경로. 판별이 측정존이 빈 것을 감지하면(`NOT_FOUND`) 같은 복구로 처리 |
| 11/20 | **판별 물성 기준**: 물성 측정 시작 힘 3 → 5 N, 말랑 기준 `soft_dw` 1.0 → 0.8 mm (알루미늄 0.5, 지우개 1.0~1.3) | C/D 실물 측정값 분포에 맞춤 |
| 11/20 | **virtual `sim_boxes` 중심 탐지 보정**: 가상 접촉 반경(25mm)과 중심 탐침 반경(50mm)이 가상 상면 반폭(20mm)과 맞지 않아 모든 탐침이 미접촉 처리되던 문제를 확인. 가상 상면 탐침 판정에만 반폭 40mm를 별도 적용. 실제 로봇 탐지 기준은 변경하지 않음 | 가상 박스를 찾고도 `반원에 유효 윗면 없음`으로 작업이 중단되는 시뮬레이션 불일치 수정 |
| 11/20 | **인터페이스 v0.7 / 재고 스냅샷 전달**: `/place`와 `/inventory_check` Goal에 `Inventory inventory` 추가. 관제가 각 호출 시점의 재고를 전달하고 Result의 실측 수량을 받아 메모리를 갱신. 시작 실측 후 초기 스냅샷을 저장하고, 작업 중에는 DB 재고를 갱신하지 않으며 DONE·중단 시 최종 스냅샷 저장. 작업 중단은 마지막 확인값을 저장하고 재측정 필요를 `reason`과 `/control_status.inventory_needs_recheck`에 표시. 새 작업 U는 0부터 시작. v0.4의 “Place Goal에서 inventory 제거” 결정은 대체됨 | 파트에 호출 당시 재고 문맥을 전달하고, 작업 결과를 완료·중단 시점에 반영 |
| 11/20 오전 (11/20 오후 대체) | **수동 재고 설정 폐기, 실측 및 종료 스냅샷으로 통일**: 시작 시 `/inventory_check`로 A~D를 실측하고 DB에 초기값 저장. 적재 중에는 관제 메모리와 관리자 GUI 상태만 갱신. 정상 완료(측정 공간 비움), 비상정지·오류 중단 때 최종 재고를 DB에 저장. 웹 UI 목표 흐름은 안전 확인 팝업 후 확인 시점부터 10분 대기·남은 시간 표시. 관리자 GUI의 바로 시작은 유지. 웹 UI 연동은 미착수 | 미측정 수동 입력 대신 작업 공간의 실물과 관제 상태를 기준으로 UI에 제공. 중간 DB 쓰기를 없애고 종료 결과를 한 번 반영 |
| 11/20 | **시작 설정은 카운트다운 뒤 실행**: 바로 시작 또는 대기 종료 후 그리퍼 최대 열기 → HOME → 그리퍼 닫기 → 재고 실측(옵션) → 스캔. PREFLIGHT에서는 그리퍼 점유만으로 거부하지 않으며 로봇 동작도 하지 않음. launch `inventory_check:=false`는 실측만 생략하고 DB 재고는 유지 | 경고 뒤 대기 중에 로봇이 먼저 움직이는 문제를 막고, 시작 직전에 초기화하도록 순서를 정함 |
| 11/20 | **공통 이동 높이·수평 속도**: `TRAVEL_Z=160 mm`, DB 측정존 XY 위 WAYPOINT를 공정 인계점으로 사용. TRAVEL_Z 이상에서 수평 이동할 때만 `TRAVEL_SPEED_MM_S=100 mm/s`; 스캔 선 사이 이동과 수직·접촉 이동에는 적용하지 않음 | 공정 간 인계 동작과 탐색 경로를 구분하고 안전 높이 이송 속도를 공통화 |
| 11/20 | **판별 높이 접촉 기준 보정**: DB 측정존 바닥 z=30.9는 유지하고, 높이 측정에만 `h_floor_offset=-1.16 mm` 적용. flip 허용 2 mm, 높이 판정 허용 3 mm, 가로·세로 2.5 mm | 바닥 기준을 DB·집기 동작과 섞지 않고 높이 측정 오차만 보정 |
| 11/20 | **탐지 완료 후 X 확인 탐색** 및 충돌 정지점 추가 집기 시도 적용 | 가로 본탐색에서 놓친 물품을 한 번 더 찾고, 충돌 정지 위치에서 집기 기회를 제공 |
| 11/19 (폐기 → 11/20 오후 시작 팝업 재고 확인으로 부활) | 시작 전 수동 재고 설정: 웹 팝업·관리자 GUI에서 A~D·U를 입력하고 `POST /inventory`로 반영하는 안 | 11/20에 로봇 시작 실측·최종 스냅샷 방식으로 대체. 구현 및 연동 대상으로 사용하지 않음 |
| 11/19 | **전체 실행은 launch 하나** (`box_bringup` 새 패키지): `system.launch.py mode:=real/virtual` 또는 `fake:=true`. 두산 bringup은 우리 `robot.launch.py`(수업 `m0609_rg2_bringup`과 같은 노드, OnRobot 그리퍼 드라이버·RViz 기본 끔)로 포함. 관제가 bringup을 켜고 끄지는 않음 | 최종 시연은 launch로. 관제가 재시작돼도 로봇 드라이버는 살아 있어야 해서 관제는 bringup 상태를 감지·표시만. 그리퍼 드라이버가 Modbus 접속을 잡아 매번 pkill 하던 단계 제거 |
| 11/19 (높이 보정은 11/20에 추가) | **측정존 바닥 z = 30.9 하나** (DB·`common` 기본값). 판별 sync 집기도 Goal 값을 씀. 높이 측정 접촉은 다음 날 별도 `h_floor_offset` 보정을 추가 | 30.9는 물리 바닥 좌표와 sync 집기 기준으로 유지. 측정된 접촉 기준 오차만 보정 |
| 11/19 | **구역 만재 → 미분류로 보냄** (`Place.Result placed_class` 추가, 적재 `('U', 정원)` 반환). 관제: 재고 그 구역 = 정원, U +1, `ZONE_FULL` WARN, 작업 계속 | 쥔 채 `NO_SLOT`으로 끝나면 사람이 빼야 하고 무인 운전이 멈춤 |
| 11/19 | **DB 저장 실패해도 작업 계속**: 재고·작업 상태 쓰기가 안 되면 DB 노드가 모아 뒀다가 다시 보냄(`DB_RETRY` 로그), 관제에는 성공 응답 | 와이파이가 잠깐 끊겨도 무인 운전이 멈추지 않게. 재고 스냅샷도 같은 재시도 경로로 저장 |
| 11/19 | **대기·카운트다운 중 관제 오류는 주의 로그만** (`CONTROL_WARN`): 터미널 키 입력 오류 등은 키 입력만 끄고 계속. 작업 중 관제 코드 오류는 그대로 `CONTROL_ERROR`(정지) | Ctrl+C가 아닌 뜬금없는 키 오류로 관제가 멈추면 안 됨. 비상정지는 UI·관리자 GUI·펜던트로 가능 |
| 11/18 | **파트 노드 죽음 감지 `PART_DIED`**: 작업 중 실행 중인 파트 Action 서버가 3초 넘게 안 보이면 `move_stop` + cancel → `ERROR : Part node stopped (place)`. 시간 값 없이 항상 켜짐 (`progress_timeout_sec`는 '살아 있는데 멈춤'용으로 따로, 기본 끔 유지) | Action은 무한 대기라 파트 노드가 죽으면 관제가 RUNNING으로 영원히 기다림. 관제를 끄자는 게 아니라 '작업 중 대기'를 끝내는 것 |
| 11/18 | **관제 코드 오류 복구 `CONTROL_ERROR`**: `run_job`에서 예상 못 한 예외 → 로봇 정지 + 파트 cancel → `ERROR : Control error — 예외` → READY. 관제 프로세스는 죽지 않음 | 상주 모드에서 관제가 죽으면 UI·로봇 상태 조회까지 같이 끊김 |
| 11/18 | **적재 수량 확인 불가**: 적재가 잰 수량이 1~6 밖이면 `place_interface`가 작업을 실패시키지 않고 `count = 0`으로 보냄 → 관제는 재고를 덮어쓰지 않고 `COUNT_MISMATCH` '적재 현황 확인 불가' | 박스는 이미 놓였으므로 작업을 끝낼 이유가 없음. 11/17 '0 방어'가 interface에서 먼저 막혀 실제로는 작업 ERROR가 되던 것을 바로잡음 |
| 11/18 | **적재 미완료 기록 자동 정리**: 다음 Goal에서 그리퍼가 비어 있으면 pending을 지우고 진행(WARN), 쥐고 있으면 `PLACE_FAILED`. 관제 사전 점검도 그리퍼가 무언가를 쥐고 있으면 FAIL | 매번 상태 파일을 `rm`하던 문제 (지우면 자리별 첫 접촉 Z 기준까지 사라짐). 적재는 매번 실제로 닿아 보고 수량을 재므로 막아야 할 경우는 '쥔 채 멈춤'뿐 |
| 11/18 | **사전 점검 결과를 이벤트 로그로** (`code PREFLIGHT`, WARN·FAIL 항목 + 통과/실패 요약) | 웹·관리자 GUI 로그 창에서 시작 안 된 이유를 보게 |
| 11/18 | **중복 정리**: 세 파트 Action 서버 → `box_comm/part_server.py` 하나 (파트 interface에는 입력·결과 검사만), 메시지 변환 → `box_comm/convert.py`, 실행 방식은 `start_algorithm` 하나(fake·test_*_motion도 바꿈, fake는 `use_robot=False`), `fake_database` 삭제 → `database_main` 파일 모드 + 시험 파라미터(`start_inventory`, `estop_at`), 판별 PARAMS·탐지 `GRIP_FORCE_N` 기본값을 `common`에서, 적재 설정 `mass_g` 삭제(안 씀), 모니터 GUI는 관제가 살아 있으면 로봇 상태를 직접 조회하지 않음, `scan_original` 실행 노드에서 뺌. 접촉 하강 루프(탐지 2·판별 3·적재 1)는 파트별 튜닝이라 그대로 | 같은 수정을 세 군데 해야 하던 곳 정리. 동작(값·흐름)은 바뀌지 않음 |
| 11/18 | **로그 번호 잠금**: `/event_log` seq를 메인·executor 스레드가 같이 올리던 것을 lock으로 | 같은 번호가 나가면 DB 노드가 중복으로 보고 한 줄을 버림 |
| 11/18 | **READY 시작 신호 순서**: 지난 시작 신호를 READY 발행 '전에' 지움 | READY를 보자마자 UI가 보낸 시작이 '수락' 응답만 받고 사라질 수 있었음 |
| 11/18 | **판별 → 적재 인수인계 상태 확정**: 판별 위상 정렬 후 L1(가장 긴 변) = 측정존 x, L2 = y, 가장 짧은 변 = 높이, 측정존 중앙. 적재는 집은 폭이 L1·L2 중 어느 변인지 로그, 둘 다 아니면 WARN | 판별 담당 요청(f8fdd3c 위상 정렬). 적재는 이 상태를 믿고 고정 자세로 집으므로 실물에서 어긋나면 바로 보이게 |
| 11/18 | **집은 뒤 0.5초 기다리고 15mm 천천히 들기** (`GRIPPER settle_after_grip_s`·`lift_slow_mm`·`lift_slow_vel`, 탐지·적재 공통) | 실물: 40N이 다 걸리기 전에 들면 박스가 핑거 사이로 빠짐 (적재에서 먼저 확인, 탐지도 같은 방식으로 집으므로 같이) |
| 11/18 | **관리자 모니터 GUI** (`box_control/monitor_gui.py`): 관제 실행 시 별도 프로세스로 같이 뜸(`-p gui:=false`로 끔), 보기 + 비상정지 버튼. 관제 상태는 새 토픽 `/control_status`(`std_msgs/String` JSON, transient_local, 임시) | DB·웹 UI 연동 전까지 관제 PC에서 단계·재고·파트 진행·로봇 상태를 한눈에 확인. 별도 프로세스라 GUI가 죽어도 작업은 계속. `box_interfaces`를 바꾸지 않으려고 JSON 문자열로 |
| 11/18 | **적재 측정존 집기 = sync 집기, 판별과 같은 설정** (`box_comm/sync_grip.py` + `common.SYNC_GRIP`, 판별 `rg2_sync_grip` 공용화): 100mm로 열고 핑거 끝을 바닥 + 2.6mm(판별 실물 값)에 둔 채 닫으면서 TCP를 올림. 층 판단은 집을 때 잰 'TCP − 박스 바닥'으로 보정. `pick_above_floor_mm` 삭제. 적재 하강 얹힘 감시는 안 함(판별이 측정존 중심에 정해진 위상으로 정렬) | 실물: RG2는 벌릴수록 핑거 끝이 올라가(열림↔닫힘 39mm) TCP 바닥 + 5에서 닫으면 A 60mm 면을 바닥 위 17mm(위 3mm)에서 물어 2번 놓침. 고정 z로는 '아래를 잡기'와 '빈손일 때 바닥 안 찍기'를 동시에 못 함. 적재는 항상 눕힌 박스라 판별 방식이 맞음(적재 담당 의견). 핑거 끝 높이는 판별·적재가 다를 이유가 없어 공용 값 하나 |
| 11/18 | **집기 기준 정리** (CONTRIBUTING 2-5-1 표): 잡힘 판정 공용(`gripper.check_grip`/`holding`: 비트 + 패드 간격 17~80mm + 들어 올린 뒤 폭 변화 3mm, 판별은 폭 범위 제외), 쥐는 힘 40N(`GRIPPER`, sync는 10N으로 닫음), 다시 집기 3번(탐지 각도, 판별·적재 같은 자리), 판별 가로 얹힘 → 바로 `GRASP_FAILED`, 판별·적재 최종 실패 → 측정존 비우기 → 재스캔(**밀어내기 경로가 없으면 fake: WARN 로그만, 대기 없음 — 스캔 중에 사람이 치움**) | DO 시절 탐지는 WebLogic 폭 조건으로 '한쪽 핑거만 걸려 멈춘 것'을 걸렀는데 Modbus 통합 때 비트 + 5mm만 남음. 판별은 범위 밖도 U로 재야 함. 판별 얹힘은 같은 자리로 다시 내려가도 같음. 밀어내기는 미구현 |
| 11/18 | **측정존 z = 바닥 Z로 통일, 집는 높이는 파트별**: DB `measure_zone` z는 측정존 바닥(판별 해석과 같음). 탐지는 작업공간에서 **윗면 − 15mm**(그리퍼 팁 길이, `GRIP_DEPTH_FROM_TOP_MM`)로 집고(원본 50% → 겹친 박스 대비) 높이 탐지 하한 Z0+40 → Z0+17, 측정존에는 닿을 때까지 내려놓기(하한 바닥 + 2). 적재는 **바닥 + 5mm**(`pick_above_floor_mm`)에서 집고 층 판단은 TCP Z − 5로(기존 첫 접촉 Z 기준 그대로 사용). 관제 측정존 확인도 바닥 + 5mm(`RECOVERY probe_above_floor_mm`). 판별의 '들어 올리면서 집기'는 길이 측정용이라 판별만 | 적재가 바닥 높이(30, 실측 바닥 30.9보다 0.9 아래)로 movel해 집어서 핑거 끝이 바닥을 눌러 안전 정지가 날 수 있었음. 탐지는 50% 높이를 잡으면 겹친 박스의 아래까지 걸림. 공유는 '바닥 위치' 사실 하나, 잡는 방식은 파트 몫(11/17 원칙과 같음) |
| 11/17 | **탐지 통합**: 담당 원본 `m0609_scan_final.py`(1파일)와 분리본(9파일)을 받아, 원본 기준으로 판별·적재와 같은 3파일 구조(`scan_algorithm`·`scan_config`·`scan_main`)로 통합. 원본의 로봇 연결·TCP 등록·인자·JSON 기록은 관제(box_comm)가 맡으므로 삭제. **Goal 한 번 = 물품 1개** + 탐색 커서(이어서 탐색). 그리퍼 DO → Modbus, 집기는 높이를 찾은 첫 지점에서 0/45/90° 3번(원본은 점마다 3번, 최대 12번), 측정존 내려놓기는 판별 방식(천천히 내려가 닿으면 놓기). 높이 탐지 같은 자리 3회 실패 → 스캔공간 막힘(임시 `SCAN_FAILED`). `DsrIO`에 `amovel`·`movejx`·`motion_state` 추가. 에뮬레이터 `sim_boxes`, `fake_run.sh` `SCAN_MAIN`. 원본은 `scan_original.py`로 보관 | 분리본은 관제 없이 혼자 돌기 위한 코드(ROS·TCP·정지)가 대부분이라 파일만 많아짐. 집기 각도는 팀 논의안(±30°)이 실물 시험 전이라 보류하고 원본 시험값(0/45/90°) 사용 |
| 11/17 | **그리퍼 세 파트 모두 Modbus**(`box_comm/gripper.py`, 판별 RG2 코드를 옮김). `robot.grip_open/grip_close/gripper_width/gripper`, 기존 `grip()` 이름 유지. Goal 동안만 접속(동시 접속 1개), DO 1·2는 끔. 설정 `common.GRIPPER`. `BOX_GRIPPER=fake`로 가짜 그리퍼. 실제 로봇은 OnRobot 드라이버를 꺼야 함 | 폭·잡힘 여부를 읽어 집기 실패를 감지하려고. DO는 미리 정한 잡기/놓기뿐이라 판별 폭 측정도 불가 |
| 11/17 | **집기 실패 처리**: 파트가 `GRIP_RETRY`(3)번 다시 집고 포기 → `GRASP_FAILED`. 파트 Goal은 거기서 끝. 판별·적재였으면 **관제가** 상태 `CLEARING`에서 직접 로봇을 움직여 측정존을 밀어내고 비었는지 확인(`box_control/recovery.py` = 관제 복구 동작 모음, 못 하면 `RECOVERY_FAILED` = 측정존 사용 불가 → 종료). 관제는 `GRASP_FAILED`면 공간 스캔부터 다시(`BOX_RETRY` 로그), 연속 `RECOVER_LIMIT`(3) 초과 → `RETRY_LIMIT` 종료. 탐지 기본 오류 코드 `GRASP_FAILED` → **`SCAN_FAILED`**(새 코드) | 다시 집는 방법은 파트마다 다르고(탐지 그리퍼 회전, 판별·적재 그대로), 측정존 비우기·흐름 되돌리기는 공통이라 관제가 맡음. 처음엔 파트 노드 안(box_comm)에서 비웠다가, 파트는 실패까지만·그 뒤는 관제로 경계를 정리(관제도 로봇을 움직이는 복구 동작이 앞으로 더 필요). 예상 못 한 오류가 재스캔으로 이어지지 않게 코드 분리 |
| 11/17 | **판별 연동**: 판별 코드 3개(`judge_main`, `judge_algorithm`, `rg2_sync_grip`) 저장소 반영, 판별이 직접 부르던 DSR_ROBOT2·Modbus를 `robot` 명령으로 교체(`amovel_rel`·`mwait`·`move_stop`·`force(ref)`·`joints()` 추가). 측정 알고리즘은 그대로 | 판별 단독 시험 때는 노드 namespace가 `dsr01`이라 급정지가 됐지만, `box_comm` 안에서는 서비스 이름이 안 맞아 접촉해도 하강이 안 멈추던 문제. 정지 확인·진행 알림도 빠져 있었음 |
| 11/17 | **탐지·판별도 같은 구조로**: 공용 어댑터 `box_comm/algorithm.py`(로봇 주입·실패 시 정지·`PartError` 변환), 세 interface에 `start_algorithm()`. 약속 `XxxAlgorithm(node, robot)` + `scan(poses, report)->bool` / `judge(specs, poses, report)->str` / `place(result_class, poses, report)->int`. 템플릿 3개 이 방식으로 변경. `DsrIO`는 첫 명령 때 서비스 준비 확인(bringup 없이 FAKE 가능), 두산 패키지가 없으면 `robot=None`. 예전 `start(Work)` 방식은 fake·test 노드용으로 유지 | 탐지·판별 실제 코드가 들어오기 전에 경계를 먼저 만들어 적재처럼 나중에 분리하는 일이 없게 |
| 11/17 | **적재 코드 분리 (담당자와 합의)**: 알고리즘 `box_place/place_algorithm.py`(적재 소유, 관제 코드 없음) / 로봇 명령·정지 확인 `box_comm/dsr_io.py` / 실패 시 정지·결과 변환 `place_interface.start_algorithm()`(관제). `place_main.py`는 진입점 한 줄, `place_working.py` 삭제 | 같은 700줄 파일을 주고받으며 working·main이 갈라지던 문제. 서로 다른 파일만 고치게 경계를 만듦. 이동 명령을 새로 넣어도 정지 확인이 자동으로 따라옴 |
| 11/17 | **`robot_monitor` 기본값 `off` → `dsr`**. bringup이 없거나 `dsr_msgs2`가 없으면 사전 점검 FAIL(예전엔 조용히 off로 바뀌어 WARN 통과). 로봇 없이 테스트할 때만 `-p robot_monitor:=off` (`fake_run.sh`는 원래 명시해서 영향 없음) | 시연에서 파라미터를 빠뜨려 정지 감지 없이 도는 것을 막기 위해 |
| 11/17 | **적재 수량 0 방어**: A~D 적재 성공인데 `count < 1`이면 재고를 덮어쓰지 않고 이전 값 유지(에러 로그 + `COUNT_MISMATCH`). 다음 같은 클래스 적재 때 측정값으로 바로잡힘 | 메시지 기본값 0 때문에 재고가 0으로 날아가는 것 방지 |
| 11/17 | 미분류 `U` 재고: DB가 A·B·C·D·U 다섯 개를 주고 U 초기값은 0. 관제는 적재 성공 후 U+1을 DB에 반영 (기존 구현 그대로, 문서로 확정) | 재고 반영은 적재 완료 후에만 |
| 11/17 | `NO_PROGRESS`(진행 없음)는 `move_stop` + cancel 후 **`ERROR`** 로 기록 (로봇 고장이 아니라 공정이 멈춘 것). 복구는 당분간 수동(박스 치우고 관제 재실행), 자동 리셋은 집기 실패 처리와 함께 설계 | 로봇은 정상, 사람이 원인 확인 |
| 11/16 | **적재 규칙 확정**: 구역은 클래스별 고정(A→A 구역). 1단(가장 안쪽)에 4층까지 → 다 차면 +y로 한 칸 앞 2단에 1층부터. x는 모든 단 동일. 구역당 단 수는 `STACK['max_tiers']`(기본 3) | 수량만 알면 위치가 정해지는 단순한 규칙 |
| 11/16 | **적재가 재고를 측정**: 매번 1단부터 박스를 쥔 채 내려가 닿은 z로 층 판단, 5층 높이면 놓지 않고 다음 단. 놓은 (단, 층) → 수량 `(단-1)*4 + 층` 을 관제에 반환 | 계산값이 아니라 실제 선반 상태를 재고로 씀 |
| 11/16 | **인터페이스 v0.4**: `PlaceZone.msg` 추가(`class_id`, 1단 `x`, `y`), `FixedPoses`에 `place_zones` 추가(DB 기준), Place Goal에서 `inventory` 제거, Place Result는 **`count`(측정 수량)만**. 단·층·z·단 간격·박스 높이는 적재만 가짐(적재 로그에만 남김) | 관제·DB는 1단 x·y와 수량만 알면 됨 |
| 11/16 | ~~재고 검증: 관제 기대값 우선~~ → **측정값 우선 (정책 a)**: 기대값(+1)과 다르면 측정값을 DB에 반영, 불일치는 경고 로그로 기록하고 작업 계속 | 실제 적재 상태가 기준. 선반·DB가 어긋나도 자동으로 바로잡힘 |
| 11/16 | 판별이 물품 위상을 항상 같게 맞춘 뒤 적재로 넘기는 것을 목표로 함 → 적재는 클래스별 박스 높이 하나로 층 판단 | z로 층을 판단하려면 놓는 방향이 같아야 함 (될지는 시험 필요) |
| 11/16 | **이벤트 로그는 토픽** `/event_log`(`LogEvent.msg`)로 관제가 보내기만 함(응답 없음, 누락 허용, QoS reliable + transient_local 100). 관제↔DB는 Service 3개(응답 필요) + Topic 1개 | 로그 때문에 작업이 막히지 않게. `ros2 topic echo`·`bag record`로 바로 확인 가능 |
| 11/16 | **파트 로그는 Feedback으로**: Feedback에 `log_level`·`log_msg` 추가, 파트는 `report.log(msg, level)`. 로그 칸이 채워진 Feedback만 관제가 `/event_log`로 넘김 (`code=PART_LOG`). 관제 이벤트: `JOB_START`·`BOX_DONE`·`COUNT_MISMATCH`·`JOB_STOPPED`·`JOB_CANCELED`·`JOB_DONE` | 허브 구조 유지(파트→DB 직접 없음), stage(진행 추적)와 로그를 분리 |
| 11/16 | **작업 상태 5개**: `BEFORE`·`RUNNING`·`DONE`·`ERROR`·`STOPPED` + `SetJobStatus.reason`. ERROR = 공정 오류(관제 판단, 로봇 정상), STOPPED = 로봇이 물리적으로 멈춤(비상정지 버튼, 두산 안전 정지·펜던트 비상정지, 연결 끊김). UI 표시 `{status} : {reason}` (예: `ERROR : Fail to pick`, `STOPPED : EMERGENCY STOP`). 이유 문구는 `common.ERROR_TEXT` 한 곳 | 사람이 할 일이 다름 (로그 확인 vs 현장 확인). status로 UI 색 구분 |
| 11/16 | 관제 `stop()` → **`abort_job()`** 이름 변경. 로봇을 멈추는 함수가 아니라 작업 종료 처리임(두산 `stop()`과 혼동 방지). 이제 DB에 ERROR/STOPPED + 이유를 기록 | 작업 중 멈추면 DB가 RUNNING으로 남던 문제 해결 |
| 11/16 | **정지 기능**: 비상정지 `/emergency_stop`(DB → 관제, `EmergencyStop.srv`), 관제가 두산 `move_stop`(모드 1, 3초 반복) + 진행 중 파트 Goal cancel. 로봇 상태는 `get_robot_state` 0.5초 주기 확인(`robot_monitor`: dsr / fake / off), 작업 중에만 정지로 처리. 진행 감시 `progress_timeout_sec`(기본 끔) → `ERROR : No progress` | 멈추게 하기와 알아차리기를 둘 다. 상태 토픽이 없는 컨트롤러 버전이라 주기 확인 |
| 11/16 | 파트는 cancel을 받으면 다음 `report()`/`report.check()`에서 `CANCELED`로 빠져나옴. 파트 노드는 MultiThreadedExecutor + Reentrant로 실행 중에도 cancel 수신, Goal은 한 번에 하나만 | `move_stop` 뒤 STANDBY로 돌아가 파트가 다음 동작을 보내는 문제 대응 |
| 11/16 | **적재 규칙 변경: 3층 × 2자리** (`STACK`), 다음 자리는 **−x 방향 80mm**(`place_config.json` `slot_dx_mm`, y 같음). 적재 실제 코드 `place_main`이 시작 시 `STACK`과 설정이 같은지 확인 | 적재 담당 실측 결과에 맞춤. 관제·DB·UI·적재가 같은 수량 해석 |
| 11/16 | **적재 실제 코드 통합**: `box_place/place_main.py` + `config/place_config.json` (entry `place_main`). 정지 대응(`report.check`, 하강 중 `report.keepalive`), 멈춘 뒤 `clear_pending` 복구. 지원 클래스 A/B/C (D·미분류는 수치·좌표가 나오면 추가) | 실제 로봇에서 검증된 힘 제어 적재를 시스템 약속에 맞춤. 동작 로직은 그대로 |
| 11/16 | **미분류 `U` (인터페이스 v0.5)**: 판별이 `U`를 내면 관제가 적재에 `U`로 보내고, 적재는 미분류 투하 위치(`place_config.json` `unclassified`, 경사로)에 놓기만 하고 `count = 0`. 미분류 수량은 관제가 세서(+1) DB에 보냄. `Inventory`에 `u` 추가. 미분류 위치는 DB가 아니라 적재 설정 | 쌓지 않으니 높이로 셀 필요 없음. 투하 자세까지 포함한 한 점이라 적재 설정에 둠 |
| 11/16 | **측정존 `[300, -160, 30]`** (z = 박스 파지 TCP 높이 = 적재 1층 첫 접촉 Z). 에뮬레이터 적재 시험은 `place_main -p sim_contact:=true`(힘 대신 가상 높이까지 movel), `fake_run.sh PLACE_MAIN=1 SIM=1` | 측정존 바닥 = 적재 구역 바닥 높이 가정. 에뮬레이터에 힘 센서 값이 없어서 |
| 11/16 | UI 적재 이미지는 수량 → `common.stack_layout()`(단별 층 수)으로 그림. DB에 박스별 위치를 저장하지 않음 | 수량에서 위치가 정해지므로 중복 저장 불필요 |
| 11/14 | 관제 중심 허브 구조. 관제↔파트 Action, 관제↔DB Service (11/16: 로그만 Topic 추가) | 파트 간 결합 제거, 통합 지점 단일화 |
| 11/14 | 인터페이스 v0.2 확정 (`box_interfaces`) | |
| 11/14 | 통신 코드를 `box_comm`으로 분리, 각 파트는 `run()`만 구현 | 인터페이스 변경 시 한 곳만 수정, Result 누락 방지 |
| 11/14 | 저장소 하나에 패키지 7개, Git은 `main` 하나 + 각자 자기 폴더만 수정 | 폴더가 나뉘어 충돌이 거의 없음, 초기 학습 부담 최소화 |
| 11/14 (대체됨) | ~~UI 범위: 상태·재고 조회 + 비상정지 버튼. UI에서 작업 시작은 하지 않음~~ → 시작·취소(11/18 DB 노드 HTTP, 웹 연결 11/20), 바로 시작·복구(11/22) | 원격 시작은 이번 범위에서 제외 |
| 11/14 | 에러 코드·`PartError`를 `box_comm/common.py`로 통합 | 코드 추가 시 한 곳만 수정, 파트마다 `PartError` 클래스가 달라지는 문제 방지 |
| 11/14 | `box_id`를 `<작업 시작 시각>_<순번>`으로 변경 | 관제 재실행 시 DB가 새 박스를 중복으로 보고 재고를 반영하지 않던 문제 해결 |
| 11/14 | 관제 상태 이름을 영어로 변경 (`SCANNING`, `SCAN_DONE` …), 로그에만 한글 병기 | DB status·Feedback stage와 표기 통일, 코드 호환성 |
| 11/14 | 스캔은 물품 유무(`found`)만 알리고, 작업완료 판단은 관제가 함 | 시작·완료 판단은 관제의 역할 |
| 11/14 | ~~**재고 검증 적용**: 관제 기대값 우선으로 DB 반영~~ → 11/16 측정값 우선으로 변경 | |
| 11/14 | **Feedback stage 필수화**, stage 목록을 `common.STAGES`로 확정, 관제가 마지막 stage·경과 시간 추적 | 지연·멈춤 위치 파악, 이후 safety 개입의 기반 |
| 11/14 | **시작 대기(STANDBY) 카운트다운**: 관제 실행 시 터미널에 경고 + 카운트다운, `q` 취소 / `s` 바로 시작 / 무응답 시 자동 시작. 기본 10초(`start_delay_sec`), 실제 운용 10분 상정 | 로봇 작동 전 작업공간에 사람·파손 위험 물품이 없는지 확인 |
| 11/14 | 비상정지는 **필수 구현 목표**. 안 되면 UI 버튼만 두고 "향후 추가 가능"으로 정리 | 안전 기능은 시연에서 반드시 설명해야 함 |
| 11/14 | **인터페이스 v0.3**: `FixedPoses.msg` 추가, `GetJobData` 응답에 `poses` 추가 | DB가 고정 좌표 사본을 관제에 전달 |
| 11/14 | ~~고정 좌표는 `common.py`가 원본, DB는 사본, 다르면 시작 안 함~~ → 아래 v0.3로 변경 | |
| 11/14 | **인터페이스 v0.3 (보강)**: 고정 좌표는 **DB가 기준**. 관제가 `GetJobData`로 받아 `/scan`·`/judge`·`/place` Goal에 매번 `poses`로 실어 보내고, 파트는 `self.poses`로 사용. `common.py` 값은 Goal에 좌표가 없을 때(단독 테스트)의 기본값. 관제는 DB 값으로 **무조건 덮어씀**(비교·경고 없음). DB에 좌표가 없으면 시작 안 함. (Goal에 `poses`를 넣는 변경까지 v0.3에 포함) | 좌표 관리를 DB 한 곳으로, 비교 후 종료하지 않고 DB 값으로 작업 |
| 11/14 | TCP·툴은 `common.py`에 정의, **파트 interface가 Goal마다 `run()` 직전에 확인·적용** (`box_comm/robot.py`). 파트는 자기 동작 안에서 다른 TCP 사용 가능 | 가상 모드에서 TCP 미등록으로 바닥 충돌한 문제 방지, 팀원 단독 테스트에도 적용 |
| 11/14 | 시작 전 점검은 `box_control/preflight.py`로 분리, `control.py`는 순서만 | 점검 항목이 늘어도 `control.py`가 비대해지지 않게 |
| 11/14 | 중단 방식 통일: 시작 전(데이터 실패·점검 실패·`q`)은 `cancel_start()` 하나, 작업 중은 `stop()` (11/16 `abort_job()`으로 이름 변경) | 경우마다 처리가 달라지지 않게 |
| 11/14 | 템플릿 3개가 `HOME_J`, `MEASURE_ZONE`을 `common.py`에서 가져다 쓰도록 수정 | 좌표를 파트 코드에 숫자로 적지 않게 |
| 11/14 | **관제 전용 파일 보호**: `box_comm/`, `box_interfaces/`, `*_template.py`는 commit 훅(팀원 commit 거부) + GitHub Actions(관제 외 push 시 실패 표시)로 보호 | 무료 public 저장소는 경로별 push 제한(push rulesets)이 안 돼서 실수 방지로 대신 |
| 11/14 | **템플릿 이름 변경**: `scan.py` 등 → `*_template.py` (관제 관리, 참고용, 수정 금지). 담당자는 템플릿을 복사해 자기 메인 코드를 만들고 `setup.py`에 진입점 추가. 관제는 템플릿만 업데이트하고, 담당자가 바뀐 부분을 자기 코드에 반영 | 템플릿 업데이트가 팀원 코드와 충돌하지 않게, 폴더 담당 규칙 유지 |

---

## 3. 남은 이슈

상태가 `완료`인 줄은 기록용으로 남겨 둡니다. 번호는 바꾸지 않습니다.

| # | 이슈 | 관련 | 상태 |
| --- | --- | --- | --- |
| 1 | **비상정지**: 관제 구현·실물 시험 완료(11/16, CLI·펜던트). 웹 대시보드·관리자 GUI 버튼 연결 완료(웹은 robot-api → rosbridge `/emergency_stop`). 정지 뒤 재개는 `recover`(11/22) | 관제, DB·UI | 완료 |
| 2 | `error_code`별 관제 처리 (재시도 / 건너뛰기 / 정지). 지금: `GRASP_FAILED`·판별 `NOT_FOUND`만 복구 후 재스캔(연속 3회), 나머지는 작업 종료 | 관제 | 현 정책 유지 |
| 3 | Action별 타임아웃: 전체 시간 제한 대신 진행 감시(`progress_timeout_sec`, 기본 0 = 끔)로 대체. 11/17 에뮬레이터 Feedback 최대 간격 약 17.5 s. **실물 전체 사이클 기준 값 정하기** | 관제 | 실물 측정 필요 |
| 4 | ~~미분류 물품 처리~~ → `U` (v0.5) | 판별, 적재 | 완료 |
| 5 | 측정값(W·D·H·무게)을 DB·UI까지 전달할지. 지금은 판별 로그(`PART_LOG`)에만 남음 | 판별, DB | 미정 (시연 범위 밖) |
| 6 | ~~위상 정보 판별 → 적재 전달~~ → 전달하지 않음. 측정존 인수인계 규칙(CONTRIBUTING 2-5-1)으로 고정 | 판별, 적재 | 완료 |
| 7 | 측정존 비우기: `recovery.TOOL_SWEEP` 도구 쓸기 경로 통합(11/20). 성공 = 경로 완료 | 관제 | 실물 통합 재확인 |
| 8 | 탐지·판별·적재 3개 프로세스의 두산 동시 연결 — 시험 도구·실제 코드로 가상·실물 확인 | 관제 | 완료 |
| 9 | ~~DB·UI 방식~~ → Supabase + 웹 대시보드(Next.js) + FastAPI/rosbridge, 현장용 관리자 GUI(Tk) | DB·UI | 완료 |
| 10 | 재고 검증(측정값 우선) 적용 완료. 불일치는 `COUNT_MISMATCH` 로그로 남고 `log` 테이블에 올라감(키워드 필터에 걸리는 경우). 차이가 클 때 중단할지는 미정 | 관제, DB | 현 정책 유지 |
| 11 | 집기 최종 실패: 탐지는 바로 재스캔, 판별·적재는 `TOOL_SWEEP` 후 재스캔, 연속 3회 상한(`RETRY_LIMIT`) | 관제 | 실물 통합 재확인 |
| 12 | Feedback 진행 감시: 개입 로직 완료, 파트 노드 종료는 `PART_DIED`로 항상 감지. `progress_timeout_sec` 값은 이슈 3과 같이 결정 | 관제, 파트 | 이슈 3과 함께 |
| 13 | ~~작업 통계 로그~~ → `/event_log` → JSONL → `log` 테이블, 대시보드 작업 로그 요약 | 관제, DB·UI | 완료 |
| 14 | TCP·툴 자동 확인 (`box_comm/robot.py`) — 가상·실물 확인 | 관제 | 완료 |
| 15 | ~~UI 작업 시작 버튼~~ → 웹 시작(안전 확인·재고 확인 팝업) · 취소 · 바로 시작 · 복구 | 관제, DB·UI | 완료 |
| 16 | 측정존 좌표: DB `loc`(`판별존` → `measure_zone`) `(300, −160)`, 바닥 z 30.9 | 관제, DB | 완료 |
| 17 / 17-1 | DB `get_fixed_poses()`·`place_zones` — Supabase 모드는 `loc.inventory_calibration` 1자리 signature XY 사용 | DB | 완료 |
| 18 | 적재 구역 재배치(11/19): A (480, −200) · B (630, −200) · C (635, −30) · D (635, 115), 2자리는 A·B −y 45 mm, C·D −y 30 mm. 미분류 `release_pose [793, −113, 186, 171, −117, −92]` | 적재, DB | 완료 |
| 19 | ~~접촉 감지 방식~~ → 순응제어 + 힘 제어 하강(접촉 3 N, 절대 상한 25 N) | 적재 | 완료 |
| 20 | 판별 → 적재 위상 고정: 판별 위상 정렬(11/18) + 적재 `_side_note` 확인 로그 | 판별, 적재 | 완료 (실물 로그로 확인) |
| 21 | 수량 측정의 한계: 앞 자리는 모두 찼다고 보고 셈. 안쪽 자리에서 박스를 빼면 실제보다 적게 셀 수 있음 | 적재, 관제 | 시연 범위에서는 허용 |
| 22 | **스캔공간 막힘 처리**: 같은 자리 높이 탐지 3회 실패 → 임시로 `SCAN_FAILED`(작업 종료, 사람이 치움). 후보: 건너뛰고 계속 / `SCAN_BLOCKED` 코드 — `scan_algorithm.py` `_on_scan_blocked` 한 곳 | 탐지, 관제 | 팀 결정 |
| 23 | 탐지 높이 설정: `SCAN_Z` 40 mm(11/19), 작업대 바닥 `TABLE_Z` 29.8 실측 반영 | 탐지 | 완료 |
| 24 | 판별 시뮬 모드 없음 → 에뮬레이터는 판별 fake | 판별, 관제 | 필요 시 |
| 25 | 탐지 집는 깊이: sync 집기, 핑거 끝 윗면 − 15 mm, 각도 0 → 90 → 45° | 탐지 | 완료 |
| 26 | 측정존 비우기 실제 구현 → 이슈 7과 같음 | 관제 | 이슈 7 |
| 27 | ~~측정존 바닥 z 하나로~~ → DB z 30.9 유지, 판별 높이만 `h_floor_offset` | 판별, 관제, DB | 완료 |
| 28 | ~~탐지 X 방향 확인 탐색~~ → 추가 | 탐지 | 완료 |
| 29 | **robot-api 접근 제한**: `compose.yaml`이 `8000:8000`(0.0.0.0)으로 열리고 rosbridge 경로에는 토큰 검사가 없음 → 같은 네트워크에서 `curl`로 `start_now`·`recover` 호출 가능. 후보: `127.0.0.1:8000:8000` 바인딩, 외부 공개(cloudflared) 시 토큰 | 관제, DB·UI | **시연 전 결정** |
| 30 | **웹 바로 시작 허용 여부**: 11/22 DB·UI 변경으로 웹에서 카운트다운 생략 가능. 유지하면 `common.py` 주석·`START_NOW` 로그 문구('현장 즉시 시작') 정리, 되돌리면 `control.py`에 `source='ui'` 거절 복구 | 관제, DB·UI | 팀 결정 |
| 31 | `box_place/test/test_place_pick.py`가 `place_algorithm`의 `move_to_waypoint`(11/20 공통 WAYPOINT)를 모르는 가짜 로봇이라 실패 | 적재 | 테스트 갱신 |
| 32 | `docs/DB_UI_CONTRACT.md`가 Supabase 모드(state 테이블 미사용, `log`·트리거)와 robot-api 경로를 반영하지 않음 | DB·UI | 문서 갱신 |

### 메모: 이슈 10 재고 검증 (v0.4, 측정값 우선)

```
적재 전 재고:  B = 3,  result_class = 'B'
기대값:        B = 4                       ← 적재 전 + 1
측정값:        적재 Result의 count (놓은 단·층으로 계산, 예: 1단 4층 → 4)
일치   → 그대로 DB 반영
불일치 → 측정값으로 DB 반영 + 경고 로그 "재고 불일치 HH:MM:SS box_id: 기대 4개, 측정 6개 (차이 +2) → 측정값으로 반영", 작업 계속
         다른 클래스 수량은 그대로
```

- 구현: `control.py`의 `expected_count()`, `verify_inventory()`. 작업 끝에 불일치 건수 요약 출력
- 테스트:
  - `ros2 run box_control fake_place --ros-args -p miscount_at:=2` (2번째 적재에서 수량을 일부러 +1 틀리게 측정)
  - `database_main -p store:=file -p start_inventory:="[0,5,0,0]"` + `fake_place` (11/18 `fake_database` 삭제) (선반은 비었는데 DB 재고가 남아 있는 상황 → 첫 B 적재에서 측정 1로 바로잡힘)
- 미분류는 `U`로 구현됨 (비교 없이 U+1, v0.5)
- 적재가 `count < 1`을 보내면 반영하지 않고 이전 값 유지 → 다음 같은 클래스 적재 때 바로잡힘 (11/17)
- 경고 로그를 DB에 남길지(UI 표시 여부)는 DB 담당과 협의

### 메모: 정지 기능 묶음 (#1·#3·#12) — 두산 패키지 확인 결과 (11/16, 구현 완료 — README '정지 기능' 참고)

시험 순서
1. 로봇 없이: fake 4개 + `control -p robot_monitor:=fake`
   - 작업 중 `ros2 param set /control fake_robot_state 5` → `STOPPED : Robot safety stop — robot_state=SAFE_STOP`
   - `database_main -p store:=file -p estop_at:=2` → 2번째 박스 뒤 UI 비상정지 흉내 (11/18 `fake_database` 삭제)
   - 카운트다운 중 `ros2 service call /emergency_stop ...` → 시작 안 함 (DB BEFORE 유지)
2. 에뮬레이터: `-p robot_monitor:=dsr` → 사전 점검에 `로봇 상태 STANDBY`, 작업 중 비상정지 시 `move_stop` 로그·로봇 멈춤 확인
   - `error`·`robot_disconnection` 토픽 이름을 `ros2 topic list`로 확인하고 다르면 `common.SAFETY` 수정
3. 실제 로봇: 펜던트 비상정지 → `STOPPED : Pendant emergency stop`, 해제 후 관제 재실행

(아래는 설계할 때 확인한 내용)

관제의 `abort_job()`은 작업 종료 처리일 뿐 로봇을 멈추지 않는다. 필요한 것은 두 가지:

**A. 관제가 로봇을 멈추기**
- `/dsr01/dsr_controller2/motion/move_stop` (`MoveStop`, `stop_mode` 0 빠른 정지(STO 없음) / 1 빠른 정지 / 2 부드러운 정지 / 3 HOLD). 두산이 별도 콜백 그룹으로 등록 → 파트가 `movel` 대기 중이어도 다른 노드에서 호출 가능
- `move_pause` / `move_resume`도 있음
- 주의: `move_stop` 후 로봇은 STANDBY로 돌아가므로 파트 `run()`이 **다음 movel을 그대로 보낸다** → 파트 Goal cancel + box_comm에서 cancel 시 다음 `report()`/로봇 명령 전에 중단하는 처리 필요

**B. 관제가 로봇 정지를 알아차리기**
- `RobotState.msg`는 정의만 있고 이 버전 컨트롤러는 토픽으로 발행하지 않음 → `/dsr01/dsr_controller2/system/get_robot_state`를 타이머로 주기 호출(예: 0.5초)
  - 3 SAFE_OFF, 5 SAFE_STOP(충돌 감지·보호 정지), 6 EMERGENCY_STOP(펜던트·하드웨어), 9 SAFE_STOP2, 10 SAFE_OFF2 → `STOPPED`
- 알람: 컨트롤러가 `error` 토픽(`RobotError`: level, group, code, msg1~3)을 발행 → `reason`에 덧붙이기. 연결 끊김은 `robot_disconnection` 토픽
  - 정확한 토픽 이름은 로봇 실행 중 `ros2 topic list | grep -i "error\|disconnection"`으로 확인 필요 (`/dsr01/error` 예상)
- 복구: `system/set_robot_control` (2 = SAFE_STOP 해제 → STANDBY). → 11/22 `recover`로 구현 (SAFE_OFF만 `robot_control=3`로 해제, 나머지는 펜던트)

### 메모: 이슈 12 Feedback stage 목록 (적용됨, `common.STAGES`)

| 파트 | stage 순서 |
| --- | --- |
| 탐지 | `space_scan` → `picking` → `moving` → `placing` |
| 판별 | `measuring_length1` → `measuring_length2` → `measuring_length3` → `measuring_weight` → (`measuring_materiality`) → `judging` |
| 적재 | `picking` → `moving` → `placing` |

- 판별의 길이 측정은 물품 방향에 따라 가로·세로·높이가 달라질 수 있어 번호로 구분
- 관제는 `io.progress(name)`으로 (마지막 stage, 경과 초)를 조회. 중단 로그에 자동 포함
- 템플릿 4개·fake 4개 모두 새 stage로 갱신

### 메모: 이슈 14 TCP 설정 (구현됨 — `box_comm/robot.py`)

- 문제: TCP는 로봇 컨트롤러에 **하나만** 활성화되는 전역 설정. 탐지·판별·적재 3개 프로세스가 같은 로봇을 번갈아 쓰므로, 한 파트가 바꾼 TCP가 다음 파트에 그대로 남음
- 안 A (파트 책임): 각 파트가 `run()` 시작마다 자기 TCP를 설정 — 규칙으로만 지켜짐
- 안 B (관제 책임): 관제가 Goal 보내기 직전에 확인 → 팀원 단독 테스트에는 적용 안 됨
- **채택 안 C**: 파트 interface가 Goal을 받으면 `run()` 직전에 확인·적용. 관제·단독 테스트 모두 적용, 관제는 로봇을 몰라도 됨
- 순서: 조회 → (다르면) 적용 → 조회 → (그래도 다르면) 수동 모드 → 등록 → 적용 → 자동 모드 → 조회. 최종 불일치면 `ROBOT_SETUP_FAILED`
- 11/17: 모드 전환·수동 모드 적용의 `success` 확인, 자동 모드 복귀는 실패해도 항상 시도(`_in_manual`), 두 오류를 함께 메시지에 남김
- 기어 프로젝트 방식(`subprocess`로 `ros2 service call`)과 달리 rclpy 서비스 클라이언트로 직접 호출 → 빠르고, 각 단계 결과를 코드로 확인
- 바닥 충돌 원인 추정: 가상 모드 에뮬레이터는 펜던트 설정을 공유하지 않아 `GripperDA_v1`이 미등록 → `set_tcp` 조용히 실패 → 플랜지 기준 이동 → 208.4mm 더 내려감

---

## 4. 작업 기록

최근 날짜가 위로 오게 씁니다.

### 11/24

- **관제 리뷰 (`e0416ea..e662b5b`, DB·UI 17커밋)**: `recover` DB 노드 경로 버그 수정, 타임아웃 조정, 사소한 정리 → `de34e4f` (위 결정 사항). ROS 모듈을 대체한 단위 시험 46개 + recover 타임아웃 라우팅 시험 통과
- **README 전면 재작성**: 양식(설계·플로우 차트 → OS → 장비 → 의존성 → 실행 순서) + 기능 상세·인터페이스·패키지·테스트·팀. `images/`에 다이어그램 5종. 낡은 내용 정정(웹 UI 미연동, 복구는 수동만, OnRobot `pkill`, 판별 무게 측정 등)
- **CONTRIBUTING·PROGRESS 최신화** (이 문서)
- 확인한 남은 문제: robot-api 접근 제한(이슈 29), 웹 바로 시작(이슈 30), `test_place_pick.py`(이슈 31), `DB_UI_CONTRACT.md`(이슈 32). FastAPI 테스트 38개 통과

### 11/21 ~ 11/23 (DB·UI)

- **11/21**: 대시보드(작업 상태·작업 로그·적재 현황), `log_sync.py`(JSONL → `log`), Supabase 트리거·SQL 시험(`sql/tests`, 빈 PostgreSQL 17에서 9개 시나리오), 3D 로봇 뷰(`robot-view/`), 로봇 실제 좌표 API(`telemetry.py`), DB 코드를 Supabase 스키마에 맞춤
- **11/22**: `db_store:=supabase` 모드(state 테이블 미사용, `urllib`), `inventory_check` 교정값 DB 조회(`calibration_store.py`, `loc.inventory_calibration`), SAFE_OFF 복구 `recover`(관제·DB 노드·API·UI), 웹 바로 시작 허용, M0609 URDF·RG2 메시로 3D 뷰 교체, RG2 추가 후 의존성 충돌 해결
- **11/23**: 로봇 연동(서버 설정 `server_settings.py`), 폴더 정리 `box_ui/robot-ui`·`box_ui/robot-api`, `box_ui/README.md`
- `scripts/run_real_robot.sh`: 이 프로젝트 install에서 실행되는지 확인 후 `mode:=real inventory_check:=true start_delay:=30.0 db_store:=file`로 실행 (`--check`는 확인만)

### 11/20

- **관제**: 시작 설정(그리퍼 열기 → HOME → 닫기)을 카운트다운 뒤로, launch `inventory_check` 선택 기능(기본 false) · 실측 이동 경로 최적화, 공통 인계 위치 `TRAVEL_Z` 160 · 수평 100 mm/s, 웹 시작 흐름 bringup 적용(박스별 재고 기록, 포트 점유 확인 `runtime_guard`), **TOOL_SWEEP 측정존 복구**(`recovery.py`, `test_recovery.py`·`test_control_recovery.py`) → `e0416ea` preview-version final
- **탐지**: Y 본탐색 뒤 X 확인 탐색 추가·구간 보정, 충돌 정지점 1회 집기(핑거 끝 Z 60 mm), 판별존 배치 검사 완화 → 제거, 쥠 폭 변화 허용, 측정존 배치 하강 여유
- **판별**: 높이 기준 보정(`h_floor_offset`), `flip_tol` 3 → 2 mm, 치수 허용 축별(높이 3 / 가로·세로 2.5 mm), 물성 시작 힘 5 N · 말랑 기준 0.8 mm, 높이 1차 하강이 바닥까지 가도 안 닿으면 `NOT_FOUND`(측정존 비었음)
- **적재**: 재고 점검 속도·불안정 기준·3회 반복·1초 정착, 미분류 구역 좌표 재수정, 힘 절대 상한 20 → 25 N, 접촉 판단 6 → 3 N, C 적재 각도, 힘 로그
- **DB·UI**: 웹 UI · FastAPI 첫 통합

### 11/19

- **관제**: launch 일원화(`box_bringup`: `system.launch.py`, `robot.launch.py` — OnRobot 드라이버·RViz 기본 끔), 만재 → 미분류, `PART_DIED`·관제 오류 복구, 바닥 z 30.9 통일, `box_ui` 폴더(`COLCON_IGNORE`), 적재 위치 변경 DB 반영, 탐색 기울기 15 → 25°, 시작 재고 실측 `/inventory_check` 추가(`inventory_check_algorithm.py`, `fake_inventory_check`), 통합 병합
- **탐지**: 탐색 알고리즘·재개 위치 수정, 쥠 판정 완화, 탐지 높이 40 mm · 그리퍼 축 변경, 물체 탐지 알고리즘 변경(반원 윗면 탐침), 판별존 배치 방식 수정
- **판별**: 대각 잡기 보정 단독 시험 `regrip_align_test`, 회피 최대 이탈 40 → 60 mm
- **적재**: 적재 위치 조정 · 파지 자세 재변경, 힘 제어 전 `movel`, 외력 감지 로그, 순응제어 잔류 외력 여유 1 mm
- **DB·UI**: Next.js 웹 UI 추가(PR #2)

### 11/18

- **저녁 리뷰 반영 (관제)**: 위 결정 사항 11/18 아래쪽 8줄 (PART_DIED, CONTROL_ERROR, 수량 확인 불가, 적재 미완료 자동 정리, 사전 점검 로그, 중복 정리, 로그 번호 잠금, READY 순서). ROS 없이 시험: 파트 서버 흐름(성공·실패·취소·반환값 검사·수량 0), 관제 시퀀스(정상·예외·수량 0), 로그 번호 8000줄 4스레드, PART_DIED, DB 시험 파라미터, 기존 테스트(`test_place_pick` +2, `test_scan_flow`, `test_judge_grip`) 통과. **실물·ROS 빌드 확인 필요** (`colcon build` 후 `./scripts/fake_run.sh`)
- 판별 물성(C/D)·정렬 실물 확인, D 적재 실물 확인 (A~D 구역 위치는 다시 정할 예정)
- **탐지 sync 집기**: 처음엔 기존 집기(힘 제어 접근 + 약하게 닫기)에 sync를 덧붙여 2단계로 넣었으나, 실물에서 ② 하강 중 J2 충돌 안전 정지 → 정쓰 지시로 기존 집기 로직을 걷어내고 **적재와 같은 흐름 하나로** 다시 씀(`_grip`: 위로 → 100mm 열기 → 윗면 − 25까지 위치 이동 → `close_with_lift` → 40N → 0.5초 → 천천히, `_sync_approach`). `_grip_approach`·`CENTER_DEPTH_FROM_TOP_MM`·`REGRIP_GAP_MM` 삭제, `SYNC_SLOW_FROM_MM` 15·`SYNC_SLOW_VEL` 30 추가. 시험 10개 통과
- **오후 통합**: 탐지(f014445 v120/a200 탐색·높이 탐지 설정)·판별(f8fdd3c 위상 정렬·물성 C/D·측정 속도) 받음 — 파일 겹침 없음, 아침 집기 기준(`_close_retry`, `SYNC_GRIP`, 폭 판정, 40N) 그대로 유지 확인
  - `box_comm/gripper.py` 폭 레지스터 275 부호 있게 읽기(닫힌 채 접속 시 패드 합 −6543mm, 판별 제보) + 패드 합 0~30mm 밖이면 오프셋 × 2. 판별 쪽 자체 보정은 그대로 둬도 무해
  - 탐지 `_grip`: 다 조인 뒤 0.5초 + 처음 15mm 천천히 (적재와 같은 `common.GRIPPER` 값)
  - 적재 `_side_note`: 집은 폭이 판별 정렬의 L1(x)·L2(y) 중 어느 변인지 로그, 아니면 WARN. `test_place_pick.py` 가짜 로봇에 `wait` 추가
- **관리자 모니터 GUI**: `monitor_gui.py`(새, tkinter), `control_interface.publish_status`·`/control_status`, `control.py`(단계·재고 변경·중단 시 발행, 관제 실행 시 GUI 같이 띄움, 끝나도 창 유지), `setup.py`(`monitor_gui` 실행 명령), `package.xml`(std_msgs·action_msgs·python3-tk)
- **실물 첫 통합 (탐지·판별·적재 실제 코드 + 관제)**: 한 사이클 성공 — 탐지 접촉(1번 선, dFy +2.05N) → 윗면 49.9mm → 측정존 → 판별 19.9 × 40.0 × 60.7 → A → 적재 A 1번 자리 1층 → 재고 A 1 → 다음 Goal 이어서 탐색(1번 선 Y−50). 사전 점검: 집에서 바꾼 discovery 설정 때문에 로봇 상태 응답 없음 FAIL → 실습실 설정으로 되돌려 해결
  - 적재 집기 2번 놓침(들다가) → 원인 RG2 핑거 끝 높이(위 결정 사항) → **sync 집기로 변경**: `box_comm/sync_grip.py`(새 파일, 판별 `rg2_sync_grip`의 핑거 끝 모델·닫힘 시간·`SyncGripper` 이동 + `DsrZPort`), `rg2_sync_grip.py`는 거기서 가져다 씀(판별 동작 그대로), `place_algorithm`(`_pick_widths`, `pick_from_measure_zone`, `_hold_off`로 층 판단·하강 하한·sim 보정), `place_config.json`(`pick_tip_above_floor_mm` 3, `sync_grip_force_n` 10)
  - 다른 실행 1회: 탐지가 측정존에 내려놓을 때 접촉 없이 하한 도달 → 판별 높이 −0.5mm → 사람이 정지 (이슈 25)
  - **잡힘 판정**: `common.GRIPPER` `box_width_range_mm`(17, 80)·`hold_width_tol_mm` 3, `gripper.check_grip`/`holding`, 탐지 `_grip`·`_holding`, 적재 `_holding`. 탐지 `GRIP_FORCE_N` 40
  - ROS 없이 시험: 적재 `box_place/test/test_place_pick.py` 4개(A 60/40, B 75/44, C 50/40 면 → 핑거 끝 바닥 위 3.0~4.3mm, 빈 측정존 3회 실패, 예전 설정 키 거부), 탐지 8개(한쪽 핑거 걸림: 범위 밖 12mm → 각도 재시도, 범위 안 30mm → 들어 올린 뒤 놓침 감지, 공기 운반 없음)
  - (같은 날 수정) 적재 여는 폭 '긴 변 + 15' → **100**, 핑거 끝 3 → **2.6**: 판별과 같은 방식·같은 값으로 통일, `common.SYNC_GRIP` 신설(판별 `PARAMS` 기본값도 여기서), `sync_grip.make_sync_gripper`·`GripMissed`, `place_config.json`의 `pick_tip_above_floor_mm`·`sync_grip_force_n` 삭제
  - **집기 기준 정리** (위 결정 사항): `GRIPPER grip_force_n` 40, `check_grip(check_range)`, 판별 `_close_retry`(가로·세로 같은 자리 3번)·`_check_holding`(들어 돌리거나 옮길 때)·가로 얹힘 `GRASP_FAILED`, 관제 `recovery`(push_path 없으면 fake 성공 + WARN)·`control`(`ZONE_CLEAR` WARN 기록)·`preflight` 문구
  - ROS 없이 시험: 탐지 8개, 적재 5개(100mm에서 각 면 집기, 40N 조임, 예전 설정 키 거부, 빈 측정존 3회 실패), 판별 4개(`box_judge/test/test_judge_grip.py`: 2번 실패 후 성공 = 같은 자리 3번 내려감, 3번 실패 → `GRASP_FAILED`, 얹힘 → 재시도 없이 `GRASP_FAILED`, 공용 기본값)
  - **실물 확인 필요**: 적재 집기(핑거 끝이 바닥을 긁지 않는지, 박스 아래쪽을 무는지), 층 판단 기준 Z가 30.19 근처로 나오는지, 판별 재시도
- **측정존 z 통일 + 집는 높이** (위 결정 사항): `scan_config`(`GRIP_DEPTH_FROM_TOP_MM` 15, `PLACE_MIN_ABOVE_FLOOR_MM` 2, `GRIP_HEIGHT_RATIO`·`PROBE_FLOOR_MIN_MM`·`PLACE_OVERTRAVEL_MM` 삭제), `scan_algorithm`(`calculate_grip_z`, `probe_floor_z`, 내려놓기 하한), `place_config.json`(`pick_above_floor_mm` 5), `place_algorithm`(측정존 집기 높이, 층 판단 TCP Z − 보정, sim 가상 접촉도 보정만큼 위, 설정 검사), `common`(`measure_zone` 설명, `RECOVERY probe_above_floor_mm`), `recovery`(측정존 확인 높이), DB 템플릿 주석, README·CONTRIBUTING 2-5-1
  - ROS 없이 시험: 탐지 흐름 6개 통과(낮은 박스 기준 30 → 10mm로 바꿈), 20mm 박스 → TCP 5에서 집고 측정존 35에 놓음 / 적재 sim A×4: 보정 0 → TCP Z 30.19·50.19·70.19·30.19, 수량 1~4(전과 같음), 보정 5 → 측정존 집기 35, TCP Z 35.19·55.19·75.19·35.19, 기준 Z·수량 같음
  - **에뮬레이터·실물 확인 필요**: `SCAN_MAIN=1 SIM=1 PLACE_MAIN=1`, 실물 적재 측정존 집기 높이

### 11/17

- **탐지 통합** (위 결정 사항)
  - 새 파일: `box_scan/scan_algorithm.py`(`ScanAlgorithm`, 원본 함수 대응: `scan_line`→`_scan_line`, `probe_contact`·`grip_force_approach`→`_force_down`, `pick_and_place`→`_find_and_grip`·`_grip`·`_carry_and_place`, `resume_scan`→`_go_resume`, `sequence`→`_scan` + 탐색 커서), `scan_config.py`(원본 설정·보정표 그대로), `scan_main.py`, `scan_original.py`(원본 보관), `test/test_scan_flow.py`(가짜 로봇·가짜 시계 6개). 수정: `dsr_io`(`amovel`·`movejx`·`motion_state`), `box_scan/setup.py`(`scan_main`·`scan_original`), `fake_run.sh`·`fake_stop.sh`(`SCAN_MAIN`, `SIM_BOXES`)
  - 원본에서 고친 것: CSV가 탐색·이송 구간이 한 파일로 섞이던 분리본 문제(매 구간 flush), 높이 4점 모두 실패 → Y−50 재개가 낮은 물체에서 끝없이 반복되던 원본 문제(같은 자리 3회 → 막힘)
  - **에뮬레이터 + 관제 통과**: 박스 2개 → 6번 선 → 이어서 9번 선 → 나머지 탐색 → `DONE` / 높이 30mm 박스 → `(1/3)` `(2/3)` → `스캔공간 막힘` ERROR → `ERROR : Fail to scan` / `BOX_FAKE_GRIP_FAILS=4` → 0·45·90° 실패 → 재스캔(1/3) → 이어서 탐색·집기 성공 → `DONE`(재스캔 1회) / **탐지 + 적재 실제 코드**(판별 fake): A·B → `DONE`, 박스 3개 A·A·C → A 1·2층(Z 30.19 / 50.19)·C 1층 → `DONE`
  - 주의: 에뮬레이터에서 `sim_boxes` 없이 `scan_main`을 돌리면 +Y 선 보정값(약 −10N) 때문에 바로 가짜 접촉 → 높이 탐지 Force 하강이 시간 초과(SCAN_FAILED). 적재가 B·C 구역으로 갈 때 특이점 알림 2-3205/3206
  - 집에서 bringup이 `Waiting for data on 'robot_description'`에서 멈춤 → 원인 `ROS_DISCOVERY_SERVER`(실습실 discovery 서버)가 `.bashrc`에 켜져 있었음. 실습실 밖에서는 끄고 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  - 남은 것: 실물 시험(`SCAN_MAIN=1`, SIM 없이), 이슈 22·23, 순응제어 램프(원본 0.3초 → `DsrIO.compliance_on` 0초)

- **그리퍼 Modbus 공용화 + 집기 실패 처리 + 판별 연동** (위 결정 사항)
  - 새 파일: `box_comm/gripper.py`(RG2·FakeRG2), `box_control/recovery.py`(관제 복구 동작, 측정존 비우기). 수정: `common`(GRIPPER·GRIP_RETRY·RECOVERY·RECOVER_LIMIT, 코드 `SCAN_FAILED`·`RECOVERY_FAILED`·`RETRY_LIMIT`, 로그 `BOX_RETRY`), `dsr_io`, `algorithm`, `control`(`on_part_error`), `preflight`(그리퍼·밀어내기 경로 점검), fake 3종(`grasp_fail_at`), 관제 파라미터 `clear_zone`, `fake_run.sh`(`BOX_GRIPPER`)
  - ROS 없이 시험 통과: 가짜 그리퍼 빈손 2회 → 3번째 잡힘 / 관제 측정존 비우기: 경로 없음 → RECOVERY_FAILED, 비움 성공 → 재스캔, 박스 남음 → RECOVERY_FAILED / 탐지 GRASP_FAILED는 비우기 없이 그대로 / CANCELED 유지 / 관제: 탐지·판별 1번씩 실패 → 재스캔 후 2개 완료(`DONE`), RECOVERY_FAILED → ERROR, 연속 4번 → `RETRY_LIMIT` / 판별 `RobotPort` → `robot` 명령 대응(J6 −91.1 → −1.1, 힘 base·tool, move_stop 1·2)
  - **에뮬레이터·실물 확인 필요**: fake 재스캔 시나리오, `PLACE_MAIN=1 SIM=1` A×4 재확인(그리퍼가 FakeRG2로 바뀜), 실물 Modbus 접속·적재 집기(DO → Modbus로 바뀜), 판별 실물 연동

- 관제 코드 점검·보완: `robot_monitor` 기본값 `dsr`(bringup·`dsr_msgs2` 없으면 사전 점검 FAIL), 적재 수량 0 방어(반영 안 함), `DONE` 상태 기록 실패 시 에러 로그
- `robot.py`: 두산 서비스 응답 `success` 확인(모드 전환·수동 모드 적용), `_in_manual()`로 자동 모드 복귀 보장 + 원래 오류 보존
- `robot_test.py`: 두산 서비스 오류가 traceback 대신 표에 FAIL로 남음, 정리 전에 기본 TCP·툴로 되돌린 뒤 테스트용 삭제(삭제 실패는 FAIL), 테스트용 TCP를 못 걸면 3번 건너뜀
- docstring 정리: `control.py`(파일 구성, 중단 경로, 정지 감시 파라미터, 낡은 TODO), `control_interface.py`(진행 감시·무한 대기 주의), `preflight.py`, `common.py`, README 정지 기능·재고 검증·TCP 절
- ROS 없이 로직 시험 통과(모드 복귀·재고 방어)
- **에뮬레이터 시험 통과** (가상 bringup, 수정본 빌드 후)
  - `robot_test` 전체 PASS: 1·2번 `TCP OK, TOOL OK`, 3번 테스트용 TCP·툴에서 **수동 모드 경로(`_in_manual`)로 기본값 복귀**, 정리(기본값 복귀 후 테스트용 삭제) PASS
  - fake 파트 + 관제 기본값(`robot_monitor=dsr`): 사전 점검 `로봇 상태 STANDBY`, 3개 처리 → `DONE`
  - `PLACE_MAIN=1 SIM=1`, A×4: 1자리 1·2·3층 (TCP Z 30.19 / 50.19 / 70.19) → 4번째 `A/1 3층 만재 → 파지 유지 후 상승` → 2자리 1층(수량 4), 재고 `A 4`, 적재 이미지 `A[3, 1]`, 불일치 0, `DONE`. 박스당 적재 약 30초(만재 후 다음 자리는 약 35초)
  - dsr 모드 비상정지: 4번째 박스 `place` picking 중 CLI `/emergency_stop` → `move_stop` 3초 반복 + Goal cancel → `STOPPED : EMERGENCY STOP — by cli`, 마지막 stage `picking` 기록, 재고 `A 3` 유지(미완료 박스 미반영). 다음 실행은 `clear_pending:=true`로 복구
  - `progress_timeout_sec:=60` 동안 오인 정지 없음 (Feedback 최대 간격 약 17.5초)
- 발견·수정: `robot_test.record()`가 한 줄에서 `info`/`error`를 바꿔 불러 rclpy `Logger severity cannot be changed between calls` → FAIL이 처음 나올 때 죽던 원래 버그. 줄을 나눠 수정
- 주의: 백그라운드로 띄운 fake 노드가 `ros2 run`만 죽고 남아 `/scan` 서버가 2개가 된 적 있음(`Ignoring unexpected goal response`) → `pkill -f "box_control.fake_"` 또는 `./scripts/fake_stop.sh`로 정리, `ros2 action info /scan`으로 서버 수 확인
- **적재 코드 분리**: v1(담당 원본 499줄) → v2(관제 통합) → v3(`place_working`, 담당 U 추가) → v4(`place_main`, 관제 7줄) 흐름을 정리하고 분리
  - `box_comm/dsr_io.py`(관제): 두산 서비스·DSR 호출, 명령 전 정지 확인, `force_z()` keepalive, `stop()`(힘·순응 해제 + move_stop)
  - `place_interface.start_algorithm()`(관제): `DsrIO` 주입, Goal마다 `bind(report)`, 실패 시 로봇 정지 + `PartError` 변환(CANCELED 유지)
  - `box_place/place_algorithm.py`(적재): v4 동작 그대로. 제거 — `_check`/`_keepalive`/`_ensure_robot`(robot.py와 중복)/`lock`(interface와 중복)/자체 `count_from_position`/예외 변환. 추가 — 설정에 새 클래스(D)가 생겨도 상태 파일 기준 칸 자동 생성(v4는 KeyError)
  - ROS 없이 가짜 로봇 시험 통과: sim A×4 → 1·2·3·4, U → 0, 7번째 → NO_SLOT + 정지, 집는 중 정지 → CANCELED + 정지 + 다음 Goal 거부(pending), 힘 제어 하강 경로 A×4, 미지원 D → PLACE_FAILED(움직이기 전이라 정지 없음). **에뮬레이터 확인 필요**
- **에뮬레이터 확인 (분리 후)**: `PLACE_MAIN=1 SIM=1` A×4 → 1·2·3층(Z 30.19/50.19/70.19) → 2자리 1층(수량 4), `A[3, 1]`, 불일치 0, `DONE` — 분리 전과 같음
- 실습실 네트워크에서 ros2 CLI(데몬)가 자꾸 멈춤 → `ROS_DOMAIN_ID=51`로 다른 팀과 분리해서 해결 (bringup 포함 모든 터미널, 통합 시 팀원 PC도 51). `fake_run.sh` 서버 대기에 5초 제한 + 데몬 강제 재시작 추가
- **탐지·판별 구조 통일**: `box_comm/algorithm.py`(공용 어댑터), scan/judge/place interface `start_algorithm()`, 템플릿 3개 알고리즘 방식으로. ROS 없이 시험: 템플릿 3개 robot=None으로 FAKE 동작(scan T,T,F / judge 판별 / place 1~4·U 0), 정지 → CANCELED, 일반 예외 → 파트 기본 코드, 적재 알고리즘 시험 재통과
- **발견·수정 (`DsrIO`)**: 위치·힘·정지 서비스 클라이언트를 DSR_ROBOT2 노드에 같이 만들었더니, TCP가 틀어져 `robot.py`가 수동 모드로 전환할 때 컨트롤러가 약 20초간 아무 서비스에도 응답하지 않음(다른 프로세스 포함). 예전 방식 `test_scan_motion`은 정상 → 원인 좁힘. 클라이언트를 전용 노드(`<이름>_io`) + 전용 executor로 옮겨 해결(`robot.py`와 같은 구조). 수정 후 `robot_test` 전체 PASS(scan_template), `PLACE_MAIN=1 SIM=1` A×4 결과 동일(`A[3, 1]`, DONE)
- **적재 규칙 단일화**: `place_config.json`에서 `max_layers`·`slot_count` 삭제, `place_algorithm`이 `common.STACK`을 그대로 씀(예전 설정에 키가 남아 있으면 같은 값일 때만 허용). DB·UI도 `common.stack_layout()`으로 같은 규칙 사용 → DB 저장·Goal 전달 불필요. 규칙 변경은 자리별 첫 접촉 Z 재측정이 따라오는 설계 변경이라 UI에서 바꾸지 않음
- 측정존 인수인계 정리: 스캔은 방향만 맞춰 측정존에 놓음 → 판별이 측정 + 정렬(중심·방향) → 적재는 같은 자세로 집기만. **결정**: 정렬 완료 상태는 숫자가 아니라 규칙 — 측정존 중심 x·y, 가장 긴 면 = 가로, 두 번째 = 세로, 가장 짧은 면 = 높이. 판별이 회전·정렬·중심 맞춤까지 하고, 적재는 이를 확정으로 보고 자기 `pick_orientation`으로 집음. 방향 값은 DB·관제로 전달하지 않음(A~D·U만). CONTRIBUTING 2-5-1에 약속 기록. 가로 = base X 확정. 물품 크기 A 60×40×20, B 75×29×25, C 50×40×20, D = A 또는 C 크기(물성으로 분류). 발견: `place_config.json` B `dimensions_mm`이 [75, 44, 25]로 표(29)와 다름(코드는 `height_mm`만 써서 동작 영향 없음, 적재 담당 수정 필요). 옆으로 선 박스(특히 B, 길이2·높이 차이 4mm)는 판별이 Rx·Ry 90° 회전으로 눕혀 규칙을 맞춤 → 적재는 항상 같은 상태를 받음. 남은 것: 적재 그리퍼 닫힘·적재 방향 재결정(보류)
- 남은 실제 로봇 확인: `robot_monitor=dsr` 기본값으로 실제 로봇 사전 점검, `progress_timeout_sec` 값, bringup 없이 실행 시 사전 점검 FAIL

### 11/16

- **관제 ↔ 적재 통합 확인 (실제 로봇)**: A·B·U 적재, 재고 불일치 → 측정값 반영 후 계속, 정지 시험(CLI 이동·적재 중, 펜던트 이동 중) 통과. 남은 실제 시험: C 구역, 2번 자리 넘어가기, `NO_SLOT`. 정지 후 미완료 기록 → `clear_pending` 복구 흐름 확인
- `fake_run.sh`가 두 번 사라진 원인: `rm -f ~/.ros/place_sim_state.json`과 실행 명령이 한 줄로 붙어 `rm`이 스크립트까지 지움 → `rm`은 따로 실행
- DB·UI 연동 가이드(Supabase) 작성: 테이블 7개 제안, DB 노드 약속, UI 버튼 2개(비상정지 / 원격 작업시작 + 팝업). 원격 시작은 관제 쪽 `JobCommand` 구현 필요
- **미분류 U 통합 (v0.5)**: `Inventory.u`, `common.UNCLASSIFIED`·`RESULT_CLASSES`·`INVENTORY_KEYS`, 판별·적재 interface가 `U` 허용(적재 U는 0만 허용), 관제 U → 재고 U +1(검증 없음), DB interface `U` 없으면 0, `place_main` = 적재 담당 `place_working`(미분류 투하 분기) + 설정 `unclassified`, fake 노드·템플릿 U 대응. ROS 없이 관제 흐름 시험 통과 (A, U, U, B → `U 2`), **에뮬레이터·실제 로봇 시험은 아직**
- **실제 로봇 A 1~3층 적재 성공** (`PLACE_MAIN=1`, 탐지·판별 fake, 박스 수동 배치): TCP Z 29.57 / 50.25 / 70.55 (기준 30.19 + 20mm씩, 오차 ±0.7mm 이내), 재고·불일치 0·`DONE`. 이동 중 펜던트 비상정지 → `STOPPED : Pendant emergency stop` 확인. 박스당 적재 약 44초
- 구역 좌표 실측 입력: A (480, −160), B (650, −160), C (580, 300)
- **에뮬레이터 적재 시험 통과** (`PLACE_MAIN=1 SIM=1`, A×4 + B×1): A 1자리 1·2·3층 → 만재 감지 후 2자리 1층(수량 4), B 1자리 1층, 재고 `A 4, B 1`, 적재 이미지 `A[3, 1] B[1]`, 불일치 0, `DONE`. 적재 중 CLI 비상정지 → `STOPPED : EMERGENCY STOP — by cli` 확인. 박스당 적재 약 33초(첫 박스 41초, HOME 이동). **실제 접촉 감지·실제 파지는 실제 로봇에서 확인 필요**
- 측정존 `[300, -160, 30]` 입력(`common.FIXED_POSES`, DB 템플릿). `place_main`에 `sim_contact`(에뮬레이터 전용, 상태 파일 분리), `fake_run.sh`에 `PLACE_MAIN=1`·`SIM=1`
- **적재 실제 코드 통합**: `place_main.py`·`place_config.json`을 `box_place`에 추가(entry `place_main`, 설정 설치), `STACK` 3층×2자리, 템플릿 −x 방향, `place_main`: 설정 경로(설치 폴더), `STACK` 일치 검사, 지원 클래스 검사(A/B/C), 모든 이동 전 `report.check()`, 하강 루프 `report.keepalive()`, `clear_pending`, 정지 실패 시에도 `CANCELED` 유지. `Reporter.keepalive()` 추가, 관제는 stage가 바뀔 때만 feedback 출력. ROS 없이 연결부 시험 통과, **실제 로봇 시험은 아직**
- **정지 기능**: `EmergencyStop.srv` + 관제 `/emergency_stop`, 관제 `request_stop()`(두산 `move_stop` 3초 반복 + 파트 Goal cancel), 로봇 상태 감시(`robot_monitor` dsr/fake/off, `get_robot_state` 0.5초), 두산 알람·연결 끊김 구독, 진행 감시(`progress_timeout_sec`), 사전 점검 '로봇 상태', 카운트다운 중 비상정지 → 시작 안 함. 파트 interface: cancel 수신(MultiThreaded + Reentrant), Goal 하나만, `report.check()`·`CANCELED`. DB `request_emergency_stop()`, `fake_database estop_at`. ROS 없이 로직 시험 통과, **ROS·에뮬레이터·실제 로봇 시험은 아직**
- **작업 상태 ERROR/STOPPED**: `SetJobStatus`에 `reason` 추가, `common.JOB_STATUS`·`STOP_CODES`·`ERROR_TEXT`·`status_for()`·`error_text()`, 관제 `stop()` → `abort_job()`(DB에 `ERROR : Fail to pick` 등 기록, `JOB_ERROR`/`JOB_STOPPED` 이벤트), DB interface·템플릿 `set_job_status(status, reason)`. STOPPED 감지 경로는 정지 기능 묶음에서
- **인터페이스 v0.4** (적재 규칙·재고 측정): `PlaceZone.msg` 추가, `FixedPoses.place_zones`, Place Goal에서 `inventory` 제거, Place Result `count`만
- `common.py`: `FIXED_POSES['place_zones']` 기본값(미설정), `STACK`(4층·3단), `count_from_position()`·`position_from_count()`·`stack_layout()` 추가
- `place_interface`: `run(result_class, report)` → 측정 수량(int) 반환, 범위 검사 후 Result로 변환
- 관제: 재고 검증을 **측정값 우선**으로 변경, 사전 점검에 적재 구역 A~D 확인 추가
- 적재 템플릿: 1단부터 내려가며 자리 찾기(5층 높이면 다음 단), z → 층 계산, FAKE 스택(`fake_start`)
- DB 템플릿: `get_fixed_poses()`에 `place_zones`, UI 갱신 로그에 `stack_layout` 표시. 옛 v0.3 이전 설명(비교 후 중단) 정리
- fake 노드: `fake_place`(`fake_start`, `miscount_at` 측정 오차), `fake_database`(`start_inventory`, 가짜 구역 좌표), `test_place_motion` 새 Result 형식
- **이벤트 로그**: `LogEvent.msg`, `/event_log` 토픽(관제 발행, DB 구독 → `on_event()`), Action 3개 Feedback에 `log_level`·`log_msg`, `common.Reporter`(`report(stage)` + `report.log()`), 관제 이벤트 6종, 템플릿·`fake_scan`에 `report.log()` 예시
- ROS 없이 로직 확인: 1단 꽉 참 → 2단, 3단 모두 참 → `NO_SLOT`, 구역 좌표 없음 → `PLACE_FAILED`, 측정값 반영·불일치 기록, 사전 점검 FAIL. **ROS 빌드·실행 시험은 아직**
- 세 파트 동시 연결 시험 도구: `test_judge_motion`, `test_place_motion` 추가, `test_scan_motion`에 `box_count` 파라미터 추가 (관제와 함께 돌릴 때 종료되도록)
- 템플릿 4개 + 관제: 로봇 없이 / 가상 로봇(robot_test 전체 PASS, test_scan_motion 동작, 전체 루프 + TCP 확인) 통과

### 11/14

- 관제 전용 파일 보호 추가: `.githooks/pre-commit`, `.github/workflows/protect-paths.yml`, CONTRIBUTING 1-2·3-1·3-6 수정
- 템플릿 이름 변경: `scan.py`·`judge.py`·`place.py`·`database.py` → `*_template.py`, 실행 진입점도 `scan_template` 등으로 변경. README·CONTRIBUTING 명령 수정, CONTRIBUTING에 메인 코드 만드는 법과 템플릿 업데이트 반영 방법 추가
- 인터페이스 v0.3에 포함: Action Goal 3개에 `poses` 추가, 관제가 DB 좌표로 무조건 덮어쓴 뒤 매 Goal에 전달, 파트는 `self.poses` 사용(템플릿은 Goal마다 받은 좌표를 로그로 남김). 사전 점검은 DB 좌표 유무·형식만 확인, 템플릿·`test_scan_motion` 수정
- **실제 로봇 테스트 통과** (`reapply_test:=false`): TCP는 펜던트 설정(`GripperDA_v1`)이 걸려 있었으나 **툴은 미적용(`''`) 상태였음** → 관제가 수동 모드 경로로 `Tool Weight` 적용. 이후 Goal은 `TCP OK, TOOL OK`. `test_scan_motion` 동작(HOME → ±50mm) 오류 없음, HOME posx z≈214.9 (가상 216.6과 일치)
- 실제 로봇도 자동 모드에서는 툴 적용이 바로 반영되지 않음 (수동 모드 경로로 적용됨)
- **가상 모드 `robot_test` 전체 통과**: TCP·툴 자동 적용, 같은 Goal 재전송 시 OK, 다른 TCP·툴에서 기본값 복귀, `run()` 안 `movej`/`movel` 정상(executor 충돌 없음). TCP 적용 후 HOME에서 posx z=216.6 (플랜지 기준이면 약 425 → TCP 208.4 반영 확인)
- 가상 모드에서는 자동 모드에서 `set_current_tcp/tool`이 반영되지 않음 → 기본값이 아닐 때는 항상 수동 모드 경로로 적용됨 (로그 문구 수정). 실제 로봇은 미확인
- 클래스 안에서 `DR_init.__dsr__node` 설정 시 파이썬 name mangling으로 두산 연결 실패 발견 → `box_comm.robot.connect_dsr()` 추가, 템플릿·CONTRIBUTING 예시 교체
- 관제 테스트 도구 추가: `test_scan_motion`(scan_interface + 실제 DSR 동작), `robot_test`(TCP·툴 자동 확인 PASS/FAIL 점검)
- 실제 bringup(가상 모드) 테스트에서 `Executor is already spinning` 발견 → 파트·DB 노드를 전용 executor로 spin, `robot.py`도 전용 executor 사용. (그대로 두면 팀원이 `run()`에서 `movej` 호출 시 같은 오류)
- `-p robot_setup:=on`이 YAML에서 bool로 읽혀 노드가 죽던 문제 수정
- 확인: `GetCurrentTcp`/`GetCurrentTool` 응답 필드는 `info`. 가상 모드 기본 상태는 TCP·툴 모두 `''`(미설정)
- 시작 전 중단 경로 통일(`cancel_start`), 템플릿에 공통 좌표 사용 예시 추가
- 인터페이스 v0.3 (`FixedPoses`), `common.py`에 TCP·툴·고정 좌표, `box_comm/robot.py`(TCP·툴 자동 확인), `box_control/preflight.py`(사전 점검) 추가
- 종료 시 Traceback(`ExternalShutdownException`)과 관제 종료 시 `Aborted` 정리
- 관제 시작 대기(STANDBY) 카운트다운 추가 (`q` 취소 / `s` 바로 시작, `start_delay_sec`)
- 재고 검증(관제 기대값 우선 + 경고 로그) 적용, Feedback stage 필수화·`common.STAGES`·관제 진행 추적 적용
- 판별 템플릿: 길이 측정 3회로 분리, 정렬 비교로 방향 무관 판별. fake_place에 `miscount_at` 테스트 옵션 추가
- 관제 상태 이름 영어화(`State` Enum), 로그에 한글 병기
- 재고 검증 방식(이슈 10), 집기 실패 처리(이슈 11), Feedback 감시와 stage 목록(이슈 12) 논의 내용 기록
- `box_comm/common.py` 추가 (에러 코드 통합), `box_id`에 작업 시작 시각 추가
- 실제 ROS 2 Jazzy에서 빌드·fake 전체 루프·단독 테스트·오류 경로(SERVER_DOWN, NO_SLOT) 확인
- 인터페이스 v0.2 기반 패키지 7개 구조 구성, GitHub 저장소 생성
- `box_comm` 통신 계층 + 파트별 템플릿(fake) + 관제 정상 경로 작성
- fake 4개로 관제 전체 루프(작업전 → 작업중 → 반복 → 작업완료) 동작 확인
