# DB 노드 로그 전송과 재고 갱신

흐름: ROS2 /event_log → DB 노드의 JSONL 기록 → LogSync 백그라운드 스레드의
Supabase log INSERT → Supabase 트리거의 inventory UPDATE → UI SELECT.
UI는 로컬 파일을 읽거나 로그를 INSERT하지 않는다. 페이지를 닫아도 DB 노드가 기록을 계속한다.

## 적용

1. Supabase SQL Editor에서 sql/log_inventory_trigger.sql을 실행한다.
2. DB 노드를 실행하는 서버 터미널 환경에 SUPABASE_URL, SUPABASE_SERVICE_KEY를 설정한다.
   Next.js .env.local에만 설정하면 Python DB 노드에는 전달되지 않는다.
3. GitHub 관리용 프로젝트의 box_database 패키지를 빌드하고 새 install/setup.bash를 source한 뒤
   DB 노드를 재시작한다. 기존 노드와 중복 실행하지 않는다.

DB 노드는 원본 JSONL을 유지하며 키워드가 포함된 메시지를 전송한다.
네트워크/DB 장애 시 재시도하고 재시작 시 기존 파일도 확인한다.
동일 timestamp/job_id는 재전송 시 중복 INSERT하지 않는다.
log.id는 DB identity가 생성하며 Python에서 보내지 않는다.

재고 UPDATE는 Python에서 직접 실행하지 않고 트리거만 담당한다.
초기 재고 점검 메시지의 A/B/C/D/U를 검증하여 다섯 행을 함께 갱신한다.
A~D 적재 완료 요약 및 `적재 → 수량 N` 상세 로그는 해당 구역을 N으로 덮어쓴다.
미분류 투하 완료 요약은 U를 명시된 값으로 덮어쓴다.
작업 완료 메시지의 전체 재고는 최종 스냅샷으로 반영한다.
집기·스캔·판별만 한 로그와 수량 없는 U 상세 동작 로그는 재고를 변경하지 않는다.
수량을 +1 증가시키지 않으므로 상세/요약이 모두 들어와도 이중 집계하지 않는다.
과거 로그의 재전송으로 재고가 되돌아가지 않도록 구역별 inventory_log_positions의 시각을 유지한다.
A 로그의 처리 시각이 B 로그 처리를 막지 않으며, 동일 구역은 더 최신 로그만 적용한다.
기존 트리거가 설치되어 있다면 변경된 sql/log_inventory_trigger.sql 전체를 다시 실행한다.
기존에 저장된 테스트 로그를 소급 재처리하지는 않는다.
SQL 구문·메시지 파싱 확인용 sql/test_inventory_log_parser.sql은 inventory를 수정하지 않는다.
PostgreSQL 17 임시 컨테이너에서 실제 트리거·파싱 검사를 실행해 통과했다.
`sql/tests/fixture.sql` → `sql/log_inventory_trigger.sql` → `sql/test_inventory_log_parser.sql` →
`sql/tests/scenarios.sql` 순서로 **빈 테스트 전용 DB에서만** 실행해 재현할 수 있다.
fixture는 테스트 역할·테이블을 만들므로 운영 Supabase에는 실행하지 않는다.
9개 시나리오: 초기값, 일반 이벤트 무변경, 이중 집계 방지, 구역별 역순 도착,
반복 적재·U, 과거 로그 무시, 최종값, 잘못된 수량 롤백, anon UPDATE 거절.
최종 검증 값은 A=5/B=3/C=3/D=2/U=1, 총 14개였다.
inventory에는 각 loc_id가 하나씩 있어야 한다. 자세한 조건은 SQL 파일을 참조한다.

서비스 키는 브라우저에 전달하거나 Git에 커밋하지 않는다.
웹의 로그 조회·스크롤은 기존 기능을 유지하고 inventory는 1초마다 조회한다.
로컬 테스트용 별도 프로젝트에는 자동 반영하지 않는다.
