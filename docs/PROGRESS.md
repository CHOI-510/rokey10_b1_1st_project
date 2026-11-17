# 진행 현황

관제가 관리합니다. 각 파트는 진행 상황을 관제에게 알려 주세요.

---

## 1. 파트별 상태

> 상태: `fake` → `구현 중` → `단독 테스트 완료` → `통합 완료`

| 파트 | 담당 | 상태 | 현재 작업 / 비고 |
| --- | --- | --- | --- |
| 관제 | 최은정 | 정상 경로 + 재고 검증 + 진행 추적 + 사전 점검 + 시작 대기 + TCP·툴 자동 확인 + 정지 기능(비상정지·로봇 상태 감시·진행 감시) | 에러 정책(재시도·건너뛰기)·집기 실패 처리 미구현 |
| 탐지 | 김창균 | fake | |
| 판별 | 송현석 | fake | |
| 적재 | 정수현 | fake | v0.4 템플릿(1단부터 측정 적재)으로 메인 코드 작성 |
| DB·UI | 이승재 | fake | |

---

## 2. 결정 사항

| 날짜 | 결정 | 이유 |
| --- | --- | --- |
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
| 11/14 | UI 범위: 상태·재고 조회 + 비상정지 버튼. UI에서 작업 시작은 하지 않음 | 원격 시작은 이번 범위에서 제외 |
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

| # | 이슈 | 관련 | 상태 |
| --- | --- | --- | --- |
| 1 | **비상정지**: 관제 구현 완료, **실제 로봇 시험 완료(11/16)** — CLI 비상정지(이동·적재 중), 펜던트 비상정지(이동 중) → 그 자리에 정지, `STOPPED : EMERGENCY STOP — by cli` / `STOPPED : Pendant emergency stop`. 적재 `PLACE_FAILED`와 동시에 와도 정지가 우선 기록됨. 남은 것: DB·UI 버튼 연결(`request_emergency_stop('ui')`, 가이드 문서 전달), 펜던트 비상정지를 힘 제어 하강 중에 거는 시험(선택), 복구는 수동(펜던트 해제 → 그리퍼 확인 → `clear_pending`) | 관제, DB | DB·UI 연결 남음 |
| 2 | `error_code`별 관제 처리 (재시도 / 건너뛰기 / 정지), 실패 시 무한 루프 방지 | 관제 | 미정 |
| 3 | Action별 타임아웃과 cancel: cancel 경로는 완료(11/16). 전체 시간 제한 대신 진행 감시(`progress_timeout_sec`)로 대체. **11/17 에뮬레이터 측정**: Feedback 최대 간격 약 17.5초(첫 박스 `place` picking, HOME 이동 포함), 이후 약 13초(`picking`)·15초(만재 후 다음 자리 이동). 60초로 오인 정지 없음 → 실제 로봇 시간 재고 30~60초 사이로 확정 | 관제 | 실제 로봇 값 확인 |
| 4 | ~~미분류 물품 처리~~ → 11/16 `U`로 완료 (v0.5, 판별 U → 적재 투하 → 관제 U +1) | 판별, 적재 | 완료 |
| 5 | 측정값(W·D·H·무게)을 DB·UI까지 전달할지 | 판별, DB | 미정 |
| 6 | 위상(방향) 정보를 판별 → 적재로 전달할지 (옵션) | 판별, 적재 | 미정 |
| 7 | 측정존 비우기, 상태 확인·환경 세팅 (WBS 0-2, 0-3) 위치 | 관제 | 미정 |
| 8 | 탐지·판별·적재 3개 프로세스의 두산 로봇 동시 연결 — 시험 도구(`test_*_motion`)로 가상·실제 확인, 적재는 `place_main`으로 실제 확인 | 관제 | 완료 (실제 탐지·판별 코드로 재확인 필요) |
| 9 | DB 방식(SQLite 권장)과 UI 방식(웹 / PyQt) — PyQt를 노드 안에 띄우면 `box_comm` 수정 필요 | DB | 미정 |
| 10 | **재고 검증 (v0.4, 측정값 우선)**: 관제 쪽 적용 완료. 불일치 기록을 DB 작업 로그로 남길지(통계만 / 상세 기록), UI 표시 여부. 차이가 클 때(예: ±2 이상) 중단할지 | 관제, DB | DB·UI 담당과 협의 |
| 11 | **집기 최종 실패 처리**: 파트 내부 재시도 후에도 실패하면 `GRASP_FAILED`. 같은 박스를 계속 다시 찾는 무한 루프 방지 필요 (연속 실패 상한, 해당 위치 건너뛰기 여부) | 관제, 탐지 | 탐지 담당과 협의 |
| 12 | **Feedback 기반 진행 감시**: 개입 로직 완료(11/16, `progress_timeout_sec` 초과 → `move_stop` + cancel → `ERROR : No progress`). 파트가 `report()`를 충분히 자주 부르는지 확인 필요(11/17 에뮬레이터 `place_main`은 60초에서 문제 없음, 탐지·판별 실제 코드는 미확인). **Action은 무한 대기라 실제 운용에서는 `progress_timeout_sec`를 꼭 켤 것** (끄면 파트 노드가 죽었을 때 관제가 계속 기다림). 멈춘 뒤 대처(리셋 방식)는 미정 | 관제, 파트 | 시험 필요 |
| 13 | 작업 통계 로그 (작업완료 물품 수, 재시도 횟수 등) — `/event_log`로 기반 마련(`JOB_DONE` 요약, `PART_LOG`). DB 저장 형식·UI 로그 창 표시(level·code 필터) | 관제, DB | DB·UI 담당과 협의 |
| 14 | **TCP·툴 자동 확인** 구현·가상·실제 로봇 테스트 완료(`box_comm/robot.py`). 응답 필드 `info` 확인, 툴 이름 `Tool Weight` 확인, 가상·실제 모두 자동 모드 적용 불가 → 수동 모드 경로로 적용됨 | 관제 | 완료 |
| 16 | **측정존 좌표** `[300, -160, 30]` → `common.py` 기본값·DB 템플릿에 입력됨. 남은 것: **DB 실제 코드에 입력**, 탐지·판별도 z를 '박스 파지 TCP 높이'로 쓰는지 확인 | 관제, DB, 탐지, 판별, 적재 | DB 반영 필요 |
| 17 | DB: `get_fixed_poses()` 구현 (**필수**: 없으면 관제가 시작하지 않음). 인터페이스 v0.3라 전원 `git pull` 후 **다시 빌드** 필요 | DB, 전체 | 공지 필요 |
| 18 | **적재 실측값**: A/B/C는 `place_config.json`에 반영됨(높이, 자세, 첫 접촉 Z, 자리 간격 −80mm). 구역 1자리 x·y 실측: **A (480, −160), B (650, −160), C (580, 300)** → `common.py`·DB 템플릿 입력됨. 미분류: `unclassified.release_pose [670, -43, 144, 160, 180, 162]`, `approach_z 160` (적재 설정). 남은 것: **DB 실제 코드에 A/B/C 입력**, **D 수치·구역** | 적재, DB, 관제 | D 대기 |
| 19 | ~~접촉 감지 방식~~ → `place_main`에 구현(순응제어 + 목표 힘 −22N, 접촉 6N, 층 허용 오차 3mm). 실제 로봇 A 1~3층, B 1~2층 Z 오차 0.7mm 이내 | 적재 | 완료 |
| 20 | **판별 → 적재 위상 고정**: 판별이 항상 같은 방향으로 내려놓아야 박스 높이 하나로 층 판단 가능 (이슈 6과 연결) | 판별, 적재 | 시험 필요 |
| 21 | **수량 측정의 한계**: 앞 단은 모두 찼다고 보고 셈. 안쪽 단에서 박스를 빼면 빈자리부터 채우며 실제보다 적게 셀 수 있음. 모든 단이 차면(`NO_SLOT`) 박스를 쥔 채 끝남 → 측정존 복귀 등은 에러 정책(이슈 2)과 함께 | 적재, 관제 | 시연 범위에서는 허용 |
| 17-1 | DB: `get_fixed_poses()`에 `place_zones`(A~D 1단 x·y) 추가 (**필수**: 빠지면 사전 점검 FAIL). 인터페이스 v0.4라 전원 `git pull` 후 **다시 빌드** 필요, 적재 `run()` 형식 변경 | DB, 적재, 전체 | 공지 필요 |
| 15 | (옵션) **UI 작업 시작 버튼**: 누르면 시작 대기 팝업(실제 10분, 데모 10초) 후 자동 시작. 사실상 원격 시작이라 UI↔DB↔관제 경로가 필요 → 아이디어 단계 | 관제, DB | 아이디어 |

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
  - `fake_database -p start_inventory:="[0,5,0,0]"` + `fake_place` (선반은 비었는데 DB 재고가 남아 있는 상황 → 첫 B 적재에서 측정 1로 바로잡힘)
- 미분류는 `U`로 구현됨 (비교 없이 U+1, v0.5)
- 적재가 `count < 1`을 보내면 반영하지 않고 이전 값 유지 → 다음 같은 클래스 적재 때 바로잡힘 (11/17)
- 경고 로그를 DB에 남길지(UI 표시 여부)는 DB 담당과 협의

### 메모: 정지 기능 묶음 (#1·#3·#12) — 두산 패키지 확인 결과 (11/16, 구현 완료 — README '정지 기능' 참고)

시험 순서
1. 로봇 없이: fake 4개 + `control -p robot_monitor:=fake`
   - 작업 중 `ros2 param set /control fake_robot_state 5` → `STOPPED : Robot safety stop — robot_state=SAFE_STOP`
   - `fake_database -p estop_at:=2` → 2번째 박스 뒤 `STOPPED : EMERGENCY STOP — by fake_db`
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
- 복구: `system/set_robot_control` (2 = SAFE_STOP 해제 → STANDBY). 나중에 UI 복구 버튼 후보

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

### 11/17

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
