# loc의 초기 재고 교정값

`sql/loc_inventory_calibration.sql`을 Supabase SQL Editor에서 한 번 실행한다.
`loc.inventory_calibration` JSONB 컬럼을 만들고 원본 `inventory_check_state.json` version 2의
A~D 각 2자리 교정값과 signature를 그대로 저장한다. 기존 `coords`는 변경하지 않는다.
SQL에는 작업자가 제공한 실측값이 포함되며 최초 기본값이나 적재용 접촉 높이가 아니다.

실제 실행 흐름:

1. FastAPI가 서버의 서비스 키로 `loc(id, inventory_calibration)`을 읽는다.
2. `GET /robot/inventory/calibration`이 version 2 형식을 반환한다.
3. DB 교정값을 사용하는 real 모드의 `/inventory_check` 노드는 각 분류 Goal 시작 때 DB 교정값을 조회한다.
4. 모든 8자리의 signature를 Goal 좌표 및 현재 알고리즘 설정과 비교한 뒤 측정한다.
5. 조회 실패·누락·조건 불일치면 움직이기 전에 PLACE_FAILED로 종료한다. 파일로 자동 대체하지 않는다.

`db_store:=file`은 기존처럼 로컬 `~/.ros/inventory_check_state.json`을 읽는다.
이 모드에서는 교정값을 얻기 위해 FastAPI나 Supabase를 조회하지 않는다.
그 외 db_store 설정은 DB 교정값을 조회한다.
기본 API 주소는 `http://127.0.0.1:8000`이며 inventory_check 노드의
`inventory_calibration_api` 파라미터로 변경할 수 있다. 같은 호스트에서 실행하는 현재 구성을 기준으로 한다.
단독 실행의 `inventory_calibration_source` 기본값은 `file`이고 real launch는 위 db_store 선택을 따른다.
교정 모드는 기존과 같이 파일에 측정 결과를 저장한다. 이후 DB로 재등록해야 한다.

FastAPI 컨테이너는 `box_ui/robot-ui/.env.local`의 서버 키를 읽는다. UI에는 서비스 키를 전달하지 않는다.
API 수정은 Docker 이미지 재빌드/컨테이너 교체가 필요하다. ROS2 box_place, box_bringup은
프로젝트 overlay에 빌드한 다음 관제를 종료하고 해당 overlay로 재실행해야 적용된다.
로봇 시작/교정 동작은 배포 과정에서 자동으로 실행하지 않는다.

현재 Supabase loc의 A~D `coords`에는 자세 각도가 들어 있으므로 위치 XY로 전용하지 않는다.
현재 요청의 file 모드 Goal XY가 교정 signature와 같은지 검사한다. db_store를 supabase로
변경하려면 loc 위치 데이터와 판별존 ID 매핑도 별도로 확인해야 한다.

검증: API 단위 테스트 및 ROS2 교정 서명 불일치·DB 장애 시 측정 미실행 테스트.
실제 로봇 접촉 측정은 이 코드 검증에서 수행하지 않는다.

## 적재 위치의 원본

A~D 첫 자리 XY는 `loc.inventory_calibration['1'].signature.xy`를 사용한다.
`loc.coords`에 남은 자세 각도를 XY로 해석하지 않는다. 두 번째 자리 XY와 자세·높이·툴·힘 조건은
기존 초기 재고 점검의 8자리 signature 검증을 그대로 거친다.
판별존 `id='판별존'`은 내부 `measure_zone`으로 매핑하고 해당 행의 coords XYZ를 사용한다.
U는 별도 투하 pose를 사용하므로 DB 노드의 고정 XY 조회에서는 제외한다.
UI도 같은 교정 XY를 사용한다. UI의 loc 데이터는 서버 서비스 키를 보유한 FastAPI의
`GET /robot/locations`를 통해 읽으며, 브라우저에 서비스 키를 전달하거나 loc의 RLS 정책을 변경하지 않는다.

Supabase 모드는 이제 현재 product(name,len,weight), inventory, loc, log 구조에 맞춰 동작한다.
별도 state 테이블이나 Python SDK를 요구하지 않는다. 상세 계약은 SUPABASE_DATABASE_MODE.md를 참조한다.
