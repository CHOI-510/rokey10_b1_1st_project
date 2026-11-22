#!/usr/bin/env bash
# Always use this checkout's built controller; never silently launch an old underlay.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$project_dir/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$workspace_dir/install/setup.bash" ]]; then
  source "$workspace_dir/install/setup.bash"
fi
if [[ ! -f "$project_dir/install/local_setup.bash" ]]; then
  echo "GitHub 프로젝트의 ROS2 패키지를 먼저 빌드하세요: $project_dir" >&2
  exit 1
fi
# local_setup prepends this overlay without replaying another workspace's chain.
source "$project_dir/install/local_setup.bash"
for package in box_comm box_control box_place box_database box_bringup; do
  actual_prefix="$(ros2 pkg prefix "$package")"
  expected_prefix="$project_dir/install/$package"
  if [[ "$actual_prefix" != "$expected_prefix" ]]; then
    echo "설치 경로 불일치: $package = $actual_prefix (기대: $expected_prefix)" >&2
    exit 1
  fi
  echo "$package: $actual_prefix"
done
if [[ "${1:-}" == "--check" ]]; then
  echo '경로 확인 완료. 로봇/관제를 실행하지 않았습니다.'
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo '사용법: bash scripts/run_real_robot.sh [--check]' >&2
  exit 1
fi
exec ros2 launch box_bringup system.launch.py mode:=real bringup:=true inventory_check:=true start_delay:=30.0 db_store:=file
