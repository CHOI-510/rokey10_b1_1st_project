#!/usr/bin/env python3
"""
database_main.py — DB 노드 실행 진입점 (ros2 run box_database database_main)
================================================================
하는 일: box_database/database_work.py, 저장소: database_store.py, 설정: database_config.py
ROS 통신: box_comm/database_interface.py
테이블·화면·버튼 약속: docs/DB_UI_CONTRACT.md

실행
  # Supabase 없이 (파일 모드: ~/box_logs/fake_db/product.json, loc.json, state.json, state_updates.jsonl)
  ros2 run box_database database_main --ros-args -p store:=file
  # Supabase
  export SUPABASE_URL=https://kdvbiivjrdqvfwzomflj.supabase.co
  export SUPABASE_SERVICE_KEY=...        # service_role 키, 코드·git에 넣지 말 것
  ros2 run box_database database_main

혼자 시험 (관제·로봇 없이):  ./scripts/db_check.sh
관제와 같이 (fake 파트):     DB_MAIN=1 ./scripts/fake_run.sh
값 확인:                     curl -s localhost:8765/state | python3 -m json.tool
================================================================
"""
from box_comm.database_interface import start
from box_database.database_work import DatabaseWork


def main():
    start(DatabaseWork)


if __name__ == '__main__':
    main()
