# 진행 현황

관제가 관리합니다. 각 파트는 진행 상황을 관제에게 알려 주세요.

---

## 1. 파트별 상태

> 상태: `fake` → `구현 중` → `단독 테스트 완료` → `통합 완료`

| 파트 | 담당 | 상태 | 현재 작업 / 비고 |
| --- | --- | --- | --- |
| 관제 | 최은정 | 정상 경로 + 재고 검증 + 진행 추적 + 사전 점검 + 시작 대기 + TCP·툴 자동 확인 | 에러 정책·타임아웃·비상정지 미구현 |
| 탐지 | 김창균 | fake | |
| 판별 | 송현석 | fake | |
| 적재 | 정수현 | fake | |
| DB·UI | 이승재 | fake | |

---

## 2. 결정 사항

| 날짜 | 결정 | 이유 |
| --- | --- | --- |
| 11/14 | 관제 중심 허브 구조. 관제↔파트 Action, 관제↔DB Service, 토픽 없음 | 파트 간 결합 제거, 통합 지점 단일화 |
| 11/14 | 인터페이스 v0.2 확정 (`box_interfaces`) | |
| 11/14 | 통신 코드를 `box_comm`으로 분리, 각 파트는 `run()`만 구현 | 인터페이스 변경 시 한 곳만 수정, Result 누락 방지 |
| 11/14 | 저장소 하나에 패키지 7개, Git은 `main` 하나 + 각자 자기 폴더만 수정 | 폴더가 나뉘어 충돌이 거의 없음, 초기 학습 부담 최소화 |
| 11/14 | UI 범위: 상태·재고 조회 + 비상정지 버튼. UI에서 작업 시작은 하지 않음 | 원격 시작은 이번 범위에서 제외 |
| 11/14 | 에러 코드·`PartError`를 `box_comm/common.py`로 통합 | 코드 추가 시 한 곳만 수정, 파트마다 `PartError` 클래스가 달라지는 문제 방지 |
| 11/14 | `box_id`를 `<작업 시작 시각>_<순번>`으로 변경 | 관제 재실행 시 DB가 새 박스를 중복으로 보고 재고를 반영하지 않던 문제 해결 |
| 11/14 | 관제 상태 이름을 영어로 변경 (`SCANNING`, `SCAN_DONE` …), 로그에만 한글 병기 | DB status·Feedback stage와 표기 통일, 코드 호환성 |
| 11/14 | 스캔은 물품 유무(`found`)만 알리고, 작업완료 판단은 관제가 함 | 시작·완료 판단은 관제의 역할 |
| 11/14 | **재고 검증 적용**: 관제 기대값 우선으로 DB 반영, 불일치는 관제 경고 로그로 기록하고 작업 계속 | 사이클은 멈추지 않고, 카운팅 문제는 사람이 확인 |
| 11/14 | **Feedback stage 필수화**, stage 목록을 `common.STAGES`로 확정, 관제가 마지막 stage·경과 시간 추적 | 지연·멈춤 위치 파악, 이후 safety 개입의 기반 |
| 11/14 | **시작 대기(STANDBY) 카운트다운**: 관제 실행 시 터미널에 경고 + 카운트다운, `q` 취소 / `s` 바로 시작 / 무응답 시 자동 시작. 기본 10초(`start_delay_sec`), 실제 운용 10분 상정 | 로봇 작동 전 작업공간에 사람·파손 위험 물품이 없는지 확인 |
| 11/14 | 비상정지는 **필수 구현 목표**. 안 되면 UI 버튼만 두고 "향후 추가 가능"으로 정리 | 안전 기능은 시연에서 반드시 설명해야 함 |
| 11/14 | **인터페이스 v0.3**: `FixedPoses.msg` 추가, `GetJobData` 응답에 `poses` 추가 | DB가 고정 좌표 사본을 관제에 전달 |
| 11/14 | ~~고정 좌표는 `common.py`가 원본, DB는 사본, 다르면 시작 안 함~~ → 아래 v0.3로 변경 | |
| 11/14 | **인터페이스 v0.3 (보강)**: 고정 좌표는 **DB가 기준**. 관제가 `GetJobData`로 받아 `/scan`·`/judge`·`/place` Goal에 매번 `poses`로 실어 보내고, 파트는 `self.poses`로 사용. `common.py` 값은 Goal에 좌표가 없을 때(단독 테스트)의 기본값. 관제는 DB 값으로 **무조건 덮어씀**(비교·경고 없음). DB에 좌표가 없으면 시작 안 함. (Goal에 `poses`를 넣는 변경까지 v0.3에 포함) | 좌표 관리를 DB 한 곳으로, 비교 후 종료하지 않고 DB 값으로 작업 |
| 11/14 | TCP·툴은 `common.py`에 정의, **파트 interface가 Goal마다 `run()` 직전에 확인·적용** (`box_comm/robot.py`). 파트는 자기 동작 안에서 다른 TCP 사용 가능 | 가상 모드에서 TCP 미등록으로 바닥 충돌한 문제 방지, 팀원 단독 테스트에도 적용 |
| 11/14 | 시작 전 점검은 `box_control/preflight.py`로 분리, `control.py`는 순서만 | 점검 항목이 늘어도 `control.py`가 비대해지지 않게 |
| 11/14 | 중단 방식 통일: 시작 전(데이터 실패·점검 실패·`q`)은 `cancel_start()` 하나, 작업 중은 `stop()` | 경우마다 처리가 달라지지 않게 |
| 11/14 | 템플릿 3개가 `HOME_J`, `MEASURE_ZONE`을 `common.py`에서 가져다 쓰도록 수정 | 좌표를 파트 코드에 숫자로 적지 않게 |

---

## 3. 남은 이슈

| # | 이슈 | 관련 | 상태 |
| --- | --- | --- | --- |
| 1 | **비상정지 (필수 목표)**: UI 버튼 → DB → 관제(`/emergency_stop`) → 두산 정지 서비스 경로 설계, `status`에 `STOPPED` 추가. 구현이 어려우면 UI 버튼만 두고 향후 추가 가능으로 정리 | 관제, DB | 정상 경로 이후 |
| 2 | `error_code`별 관제 처리 (재시도 / 건너뛰기 / 정지), 실패 시 무한 루프 방지 | 관제 | 미정 |
| 3 | Action별 타임아웃과 cancel | 관제 | 미정 |
| 4 | 미분류 물품 처리 (`result_class`에 `X` 추가 여부, 리젝트 존) | 판별, 적재 | 미정 |
| 5 | 측정값(W·D·H·무게)을 DB·UI까지 전달할지 | 판별, DB | 미정 |
| 6 | 위상(방향) 정보를 판별 → 적재로 전달할지 (옵션) | 판별, 적재 | 미정 |
| 7 | 측정존 비우기, 상태 확인·환경 세팅 (WBS 0-2, 0-3) 위치 | 관제 | 미정 |
| 8 | 탐지·판별·적재 3개 프로세스의 두산 로봇 동시 연결 확인 | 탐지, 판별, 적재 | 실제 로봇 연동 시 |
| 9 | DB 방식(SQLite 권장)과 UI 방식(웹 / PyQt) — PyQt를 노드 안에 띄우면 `box_comm` 수정 필요 | DB | 미정 |
| 10 | **재고 검증**: 관제 쪽 기능은 적용 완료. 불일치 기록을 DB 작업 로그로 남길지(통계만 / 상세 기록), UI 표시 여부 | 관제, DB | DB·UI 담당과 협의 |
| 11 | **집기 최종 실패 처리**: 파트 내부 재시도 후에도 실패하면 `GRASP_FAILED`. 같은 박스를 계속 다시 찾는 무한 루프 방지 필요 (연속 실패 상한, 해당 위치 건너뛰기 여부) | 관제, 탐지 | 탐지 담당과 협의 |
| 12 | **Feedback 기반 진행 감시**: stage 목록·필수화·관제 추적은 적용 완료. 일정 시간 소식이 없을 때의 **개입 로직**(cancel, 두산 정지) | 관제 | `common`의 safety 항목으로 추가 예정 |
| 13 | 작업 통계 로그 (작업완료 물품 수, 재시도 횟수 등) 형식 | 관제, DB | DB·UI 담당과 협의 |
| 14 | **TCP·툴 자동 확인** 구현·가상·실제 로봇 테스트 완료(`box_comm/robot.py`). 응답 필드 `info` 확인, 툴 이름 `Tool Weight` 확인, 가상·실제 모두 자동 모드 적용 불가 → 수동 모드 경로로 적용됨 | 관제 | 완료 |
| 16 | **측정존 좌표 실측** → **DB에 입력** (기준값). `common.py` 기본값도 같은 값으로 맞춰 두면 단독 테스트가 편함 | 관제, DB, 탐지, 판별, 적재 | 실측 필요 |
| 17 | DB: `get_fixed_poses()` 구현 (**필수**: 없으면 관제가 시작하지 않음). 인터페이스 v0.3라 전원 `git pull` 후 **다시 빌드** 필요 | DB, 전체 | 공지 필요 |
| 15 | (옵션) **UI 작업 시작 버튼**: 누르면 시작 대기 팝업(실제 10분, 데모 10초) 후 자동 시작. 사실상 원격 시작이라 UI↔DB↔관제 경로가 필요 → 아이디어 단계 | 관제, DB | 아이디어 |

### 메모: 이슈 10 재고 검증 (적용됨)

```
보낸 값:  inventory = {A:1, B:0, C:2, D:0},  result_class = 'C'
기대값:   {A:1, B:0, C:3, D:0}          ← result_class 칸만 +1
받은 값:  적재 Result의 inventory
일치   → 그대로 DB 반영
불일치 → 기대값으로 DB 반영 + 경고 로그 "HH:MM box_id 카운팅 오류, 확인 필요 (기대 …, 받음 …)", 작업 계속
```

- 구현: `control.py`의 `expected_inventory()`, `verify_inventory()`. 작업 끝에 불일치 건수 요약 출력
- 테스트: `ros2 run box_control fake_place --ros-args -p miscount_at:=2` (2번째 적재에서 일부러 틀린 재고 반환)
- 미분류(`X`)가 생기면 `result_class == 'X'`일 때만 변화 0을 허용 (`expected_inventory()`에 TODO)
- 경고 로그를 DB에 남길지(UI 표시 여부)는 DB 담당과 협의

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
- 기어 프로젝트 방식(`subprocess`로 `ros2 service call`)과 달리 rclpy 서비스 클라이언트로 직접 호출 → 빠르고, 각 단계 결과를 코드로 확인
- 바닥 충돌 원인 추정: 가상 모드 에뮬레이터는 펜던트 설정을 공유하지 않아 `GripperDA_v1`이 미등록 → `set_tcp` 조용히 실패 → 플랜지 기준 이동 → 208.4mm 더 내려감

---

## 4. 작업 기록

최근 날짜가 위로 오게 씁니다.

### 11/14

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
