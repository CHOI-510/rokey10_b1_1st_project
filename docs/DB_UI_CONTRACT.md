# DB·UI 약속 — 11/19

관제·DB 노드: 정쓰 · Supabase·웹 UI: 이승재

## 1. 원칙: 기록할 것만 DB에

| 정보 | 어디 | 남나 |
|---|---|---|
| 재고 A~D·미분류 | **Supabase `inventory`** | 영구 |
| 클래스 기준값, 좌표 | **Supabase `product`, `loc`** | 영구 (사람이 입력) |
| 작업 상태, 로봇 상태, 연결 | DB 노드 메모리 → `GET /state` | 휘발 (보여 주기만) |
| 이벤트 로그 (작업 이력 포함) | `~/box_logs/{job_id}.jsonl` | 영구 (파일) |
| 세부 진행 (스캔·판별·적재 단계, 파트 로그) | 관리자 GUI | 화면만 |

```
관제 ──ROS──▶ DB 노드 ──재고 바뀔 때만──▶ Supabase inventory
                │
                ├─ GET /state ◀── Next.js /api/state ◀── 브라우저 (1초마다)
                ├─ POST /estop·/start·/cancel·/inventory ◀── Next.js ◀── 버튼
                └─ ~/box_logs/*.jsonl ◀── Next.js /api/logs (최근 N줄)
```
모두 노트북 한 대. 버튼·상태는 Supabase를 거치지 않아서 인터넷이 끊겨도 동작해요.

## 2. Supabase 테이블 3개

```sql
drop table if exists state;          -- 예전 테이블 (11/19부터 안 씀)

create table if not exists product (
  class_id text primary key check (class_id in ('A','B','C','D')),
  name text not null default '',
  width_mm float8 not null, depth_mm float8 not null, height_mm float8 not null,
  weight_g float8 not null default 0           -- 0 = 미측정
);
create table if not exists loc (
  name text primary key check (name in ('A','B','C','D','measure_zone')),
  x float8 not null, y float8 not null,
  z float8                                     -- 측정존만 (바닥 z)
);
create table if not exists inventory (
  id int primary key default 1 check (id = 1),
  a int not null default 0, b int not null default 0, c int not null default 0,
  d int not null default 0, u int not null default 0,   -- u = 미분류
  updated_at timestamptz not null default now()
);

insert into product values
  ('A', '알루미늄 묶음 A', 60, 40, 20, 130), ('B', '젠가 묶음 B', 75, 29, 25, 0),
  ('C', '알루미늄 묶음 C', 50, 40, 20, 108), ('D', '지우개 D',    50, 40, 20, 55.3)
  on conflict (class_id) do nothing;
insert into loc values
  ('A', 480, -160, null), ('B', 650, -160, null), ('C', 580, 300, null), ('D', 300, 300, null),
  ('measure_zone', 300, -160, 30.9)            -- 측정존 바닥 30.9 (판별 실측, 11/18)
  on conflict (name) do nothing;
insert into inventory (id) values (1) on conflict (id) do nothing;

alter table product enable row level security;
alter table loc enable row level security;
alter table inventory enable row level security;
create policy "read" on product for select to anon using (true);
create policy "read" on loc for select to anon using (true);
create policy "read" on inventory for select to anon using (true);
```
- 쓰기는 DB 노드만 (`SUPABASE_SERVICE_KEY` 환경변수). 웹 코드·git에 **service_role 키 금지** (저장소 public).
- `loc`의 A~D·measure_zone 5행이 모두 있어야 관제가 시작해요. HOME 자세는 코드(`common.FIXED_POSES`)가 원본.
- A~D 구역 좌표는 바꿀 예정 → `loc` 행만 고치면 돼요.

## 3. `GET /state` — 화면 한 장에 필요한 것 전부

```json
{
  "status": "RUNNING",            // BEFORE · STARTING · RUNNING · DONE · ERROR · STOPPED
  "reason": "",                   // ERROR·STOPPED 이유, BEFORE면 시작 안 한 이유
  "start_at": null,               // STARTING 중 카운트다운 끝 시각 (ISO)
  "started_at": "…", "finished_at": null,
  "robot_ok": true, "robot_hint": "",
  "control_ok": true,             // 관제 신호를 5초 안에 받았나
  "db_ok": true,                  // false = Supabase 저장 실패, 다시 보내는 중 (작업은 계속)
  "inventory": {"a": 2, "b": 0, "c": 1, "d": 0, "u": 0},
  "capacity": 6, "at": "…"
}
```

| `status` | 화면 | 시작 | 취소 |
|---|---|---|---|
| `BEFORE` | 작업대기 | ○ | |
| `STARTING` | 시작준비 (점검 → 카운트다운 `start_at − 지금`) | | ○ |
| `RUNNING` | 작업중 | | (비상정지만) |
| `DONE` | 작업완료 | ○ | |
| `ERROR` | 오류 + `reason` | ○ | |
| `STOPPED` | **빨간불 비상정지** + `reason` | ○ | |

화면 판단 순서: `/state` 응답 없음 → "DB 노드 꺼짐" · `control_ok=false` → "관제 꺼짐" · `robot_ok=false` → 로봇 칸 빨강 + `robot_hint` · 그 외 `status`.

DONE·ERROR·STOPPED는 다음 시작까지 남아요. 단, 메모리 값이라 **DB 노드를 다시 켜면 `BEFORE`로 돌아가요** (마지막 결과는 로그 파일의 `JOB_*` 줄).

## 4. 버튼 (Next.js → `http://127.0.0.1:8765`)

| 웹 | DB 노드 | 언제 | 응답 |
|---|---|---|---|
| `GET /api/state` | `GET /state` | 1초마다 | 위 JSON |
| `POST /api/estop` | `POST /estop` | 언제든 | `{ok, message}` |
| `POST /api/start` | `POST /start` | 작업대기·완료·오류·정지 | `{ok, message}` |
| `POST /api/cancel` | `POST /cancel` | 시작준비 중 | `{ok, message}` |
| `POST /api/inventory` | `POST /inventory` `{a,b,c,d,u, source:'ui'}` | 시작 전 | `{ok, message, inventory}`, 범위 밖이면 400 |

```ts
// app/api/[cmd]/route.ts — estop · start · cancel · inventory 공통  (Next 15면 params가 Promise → `const { cmd } = await params`)
export async function POST(req: Request, { params }: { params: { cmd: string } }) {
  if (!['estop', 'start', 'cancel', 'inventory'].includes(params.cmd))
    return Response.json({ ok: false, message: '없는 명령' }, { status: 404 });
  const body = await req.json().catch(() => ({}));
  if (params.cmd !== 'estop' && process.env.START_PIN && body.pin !== process.env.START_PIN)
    return Response.json({ ok: false, message: 'PIN이 틀림' }, { status: 403 });
  const { pin, ...rest } = body;
  try {
    const r = await fetch(`http://127.0.0.1:8765/${params.cmd}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: params.cmd === 'inventory' ? JSON.stringify({ ...rest, source: 'ui' }) : undefined });
    return Response.json(await r.json(), { status: r.status });
  } catch {
    return Response.json({ ok: false, message: 'DB 노드 꺼짐 — 펜던트 비상정지 버튼을 쓰세요' });
  }
}

// app/api/state/route.ts
export const dynamic = 'force-dynamic';
export async function GET() {
  try { return Response.json(await (await fetch('http://127.0.0.1:8765/state')).json()); }
  catch { return Response.json({ ok: false }, { status: 503 }); }   // → 화면 "DB 노드 꺼짐"
}
```

팝업
- **시작**: "작업을 시작하시겠습니까?" + 재고 A·B·C·D·미분류 `−`/`+`(A~D 0~6) · [전부 0] → [시작]/[취소]. 재고를 바꿨으면 `/api/inventory`가 `ok`일 때만 `/api/start`. → 10분 카운트다운 후 작업.
- **비상정지**: "비상 정지를 하시겠습니까? 로봇이 현재 위치에서 정지합니다." → [정지]/[취소]. 작업 중·카운트다운 중·대기 중 언제든 `status = STOPPED`.
- **카운트다운 취소**: 시작준비 중 → `/api/cancel` → `status = BEFORE`, `reason = '시작 안 함: ui에서 취소'`.
- **바로 시작(카운트다운 생략)은 웹에 없음** — 현장 관리자 GUI 전용. DB 노드도 안 받아요(404).

## 5. 로그 (파일 → 웹)

- `~/box_logs/{job_id}.jsonl`, 한 줄 = `{seq, stamp, job_id, source, stage, level, code, box_id, class_id, expected, measured, message, received_at}`.
- 웹은 **결과 수준 줄만** 보여 줘요 (세부는 관리자 GUI):

```ts
// app/api/logs/route.ts — 가장 최근 파일에서 아래 code·ERROR만, 최신 N줄
import fs from 'fs'; import path from 'path';
export const dynamic = 'force-dynamic';
const SHOW = new Set(['JOB_START', 'JOB_DONE', 'JOB_ERROR', 'JOB_STOPPED', 'JOB_CANCELED',
                      'ZONE_FULL', 'INVENTORY_SET', 'START_NOW', 'UI_COMMAND', 'DB_RETRY']);
export async function GET(req: Request) {
  const n = Number(new URL(req.url).searchParams.get('limit') ?? 20);
  const dir = process.env.LOG_DIR ?? '/home/eunjeong/box_logs';
  if (!fs.existsSync(dir)) return Response.json([]);
  const files = fs.readdirSync(dir).filter(f => f.endsWith('.jsonl'))
    .map(f => ({ f, t: fs.statSync(path.join(dir, f)).mtimeMs })).sort((a, b) => b.t - a.t);
  if (!files.length) return Response.json([]);
  const rows = fs.readFileSync(path.join(dir, files[0].f), 'utf8').trim().split('\n')
    .map(l => JSON.parse(l)).filter(r => SHOW.has(r.code) || r.level === 'ERROR');
  return Response.json(rows.slice(-n).reverse());
}
```
- `message`는 그대로 표시. `ZONE_FULL`은 "A 구역을 비우세요" 안내, `START_NOW`는 "현장에서 바로 시작".

## 6. 실행·시험

```bash
# 파일 모드 (Supabase 없이): ~/box_logs/fake_db/{product,loc,inventory}.json
ros2 run box_database database_main --ros-args -p store:=file
./scripts/db_check.sh                                   # DB 노드 단독 시험

# Supabase
pip install supabase --break-system-packages
export SUPABASE_URL=https://kdvbiivjrdqvfwzomflj.supabase.co
export SUPABASE_SERVICE_KEY=...
python3 scripts/db_to_sql.py > seed.sql                 # 파일 값 → SQL (선택)
ros2 run box_database database_main                     # 키가 있으면 Supabase

# 전체 (fake 파트, 관제 상주)
RESIDENT=1 ./scripts/fake_run.sh
curl -s localhost:8765/state | python3 -m json.tool
curl -s -X POST localhost:8765/start        # / cancel / estop

# 웹
cd box_ui && npm run build && npm start
cloudflared tunnel --url http://localhost:3000
```
