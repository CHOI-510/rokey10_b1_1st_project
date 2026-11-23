# 웹 UI와 API

```text
box_ui/
├── COLCON_IGNORE       # 웹 프로젝트는 ROS 2 패키지 탐색에서 제외
├── robot-ui/           # Next.js, 기존 my-next-app
└── robot-api/          # FastAPI와 Docker Compose
```

아래 명령은 프로젝트 루트에서 각각 다른 터미널로 실행합니다.

## API

`robot-ui/.env.local`에 기존 Supabase 설정이 있어야 합니다.

```bash
cd box_ui/robot-api
docker --context desktop-linux compose up -d --build
curl http://localhost:8000/health
```

Compose는 `../robot-ui/.env.local`과 프로젝트의 `../../box_database/box_database/log_sync.py`를 사용합니다.
폴더 이동 전 컨테이너가 남아 있어도 위 명령으로 새 경로 설정을 반영합니다.

## UI

```bash
cd box_ui/robot-ui
npm ci                # 최초 설치 시
npm run dev
```

[대시보드](http://localhost:3000/dashboard)를 엽니다. 운영 실행은 같은 디렉터리에서 `npm run build` 후 `npm start`를 사용합니다.
기존 `.env.local`은 `robot-ui` 아래에서 유지하며 Git에 올리지 않습니다.

ROS 2 관제·rosbridge 실행과 전체 연결 절차는 [API 실행 문서](robot-api/README.md)를 참고합니다.
DB 노드는 `box_ui/robot-ui/.env.local`에서 서버 설정을 찾습니다. 별도 설정 파일은 `BOX_SERVER_ENV_FILE`로 지정할 수 있습니다.
기존에 일반 `colcon build`로 설치한 DB 노드는 설정 탐색 코드가 반영되도록 `box_database`를 다시 빌드하고 해당 overlay를 source한 뒤 재시작합니다.
