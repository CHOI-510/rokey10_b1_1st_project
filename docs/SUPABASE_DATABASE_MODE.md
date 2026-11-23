# 현재 Supabase 스키마의 DB 노드

`db_store:=supabase`는 기존 테이블을 그대로 사용한다. 새 테이블이나 컬럼을 생성하지 않는다.

| 데이터 | 읽기/쓰기 |
| --- | --- |
| 분류와 물품 연결 | inventory.loc_id A~D, inventory.product_name → product.name |
| 물품 치수 | product.len의 가로,세로,높이(mm), 예: 60,40,20 |
| 무게 | product.weight(g); NULL/빈 값은 미측정 0 |
| 재고 | inventory.cnt 읽기 및 loc_id별 절대 수량 PATCH; 기존 standard/product_name 보존 |
| A~D 첫 자리 XY | loc.inventory_calibration의 1번 자리 signature.xy |
| 판별존 XYZ | loc.id=판별존 또는 measure_zone의 coords |
| U 투하 | 기존 적재 알고리즘의 전용 pose; loc의 빈 U.coords는 읽지 않음 |
| 작업 이력 | 기존 JSONL → LogSync → public.log |
| 실시간 단계/연결/시간 | ROS control_status와 DB 노드 메모리; state 테이블에 쓰지 않음 |

재고는 매번 실제 inventory를 조회한다. 이전 fake_db 재고를 사용하거나 기본 수량을 넣지 않는다.
A~D는 0~6, U는 0 이상 정수이며 누락/중복/잘못된 값은 오류로 처리한다.
여러 구역 PATCH는 각각의 REST 요청이므로 하나의 DB 트랜잭션이 아니다.
일부만 성공하면 기존 재시도 큐가 절대 수량을 재전송한다. +1 방식으로 중복 증가시키지 않는다.
JSONL log 트리거도 기존대로 작동한다. 로봇이 관측한 최신 재고가 서비스/로그 경로로 반영된다.

노드를 재시작하면 메모리의 실시간 단계·시간은 초기화되고 관제 재발행 상태로 갱신된다.
영구 작업 이력은 log에 남고 실제 재고는 inventory에 유지된다. state 테이블은 필요 없다.

서버 설정은 환경 변수 SUPABASE_URL/SUPABASE_SERVICE_KEY를 우선한다.
없으면 현재 GitHub 프로젝트의 box_ui/robot-ui/.env.local에서 URL과 서버 키를 읽는다.
BOX_SERVER_ENV_FILE로 서버 설정 파일을 명시할 수 있다. 파일은 셸로 실행하지 않고 허용된 키만 읽는다.
브라우저 publishable 키로 쓰지 않으며 키 값을 로그에 출력하지 않는다.
Python 표준 urllib를 사용하므로 별도 supabase SDK 설치가 필요 없다.

## 적용

GitHub 프로젝트의 box_database를 빌드하고 overlay를 source한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey10_b1_1st_project
colcon build --base-paths box_database --packages-select box_database
source install/local_setup.bash
ros2 launch box_bringup system.launch.py mode:=real bringup:=true inventory_check:=true start_delay:=30.0 db_store:=supabase
```

기존 launch를 먼저 종료한다. 이번 저장소 변경은 로봇 동작을 자동으로 시작하지 않는다.
`ros2 pkg prefix box_database`가 GitHub 프로젝트 install/box_database인지 확인한다.
기존 file 모드는 유지하며 file 모드의 초기 교정값은 로컬 ~/.ros/inventory_check_state.json을 사용한다.

## 검증 결과

DB 관련 단위 테스트 23개 통과. 운영 로봇과 분리된 ROS_DOMAIN_ID=217 및 HTTP 18765에서
설치된 DB 노드를 store=supabase로 실행해 health, inventory GET 및 GetJobData ROS 서비스 응답을 확인했다.
서버 키를 셸에서 export하지 않은 상태에서도 GitHub 프로젝트 서버 환경 파일을 정상 인식했다.
가로/세로/높이/무게 4개 규격과 A~D 교정 XY, 판별존 XYZ가 실제 DB 데이터로 반환됐다.
검증 중에는 inventory PATCH나 로봇 동작 명령을 보내지 않았고 테스트 노드는 종료했다.
재고 갱신의 값 검증·부분 실패 후 절대값 재시도·다른 컬럼 보존은 모의 REST 테스트로 검증했다.
