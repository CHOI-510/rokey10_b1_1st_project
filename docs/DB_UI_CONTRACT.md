# DB·UI 연동 계약 (Supabase · DB HTTP API · 로그)

관제: 정쓰 · DB 노드(`database_main` 외): 정쓰 · Supabase 설정·웹 UI: 이승재

## 1. 전체 구조 (노트북 한 대)

```
[관제 노트북]
  ROS: bringup / 탐지·판별·적재 / 관제 control / DB 노드 database
  웹: Next.js API (next start) + cloudflared 터널
                                   
  관제 ──Service·토픽──▶ DB 노드 ──▶ Supabase state ──Realtime──▶ 웹 UI
  관제 ──/event_log──▶ DB 노드 ──▶ ~/box_logs/{job_id}.jsonl ──▶ Next.js /api/logs
  브라우저 버튼 ──▶ Next.js API ──▶ DB 노드 127.0.0.1:8765 ──▶ 관제
```

- ROS DB 노드는 파일/Supabase 저장과 로컬 HTTP API를 제공합니다. Next.js는 브라우저 요청을 이 API로 프록시해야 합니다. 브라우저에서 로봇 노트북의 `127.0.0.1`에 직접 요청하지 않습니다.
- Supabase는 **보여 주기 전용**이에요. 브라우저는 Supabase에서 읽기만 해요.
- 버튼은 Supabase를 거치지 않고 노트북 안에서 DB 노드로 바로 가요. 그래서 인터넷이 끊겨도 비상정지는 돼요.
- 로그는 DB에 쌓지 않고 파일로 남겨요.

## 2. Supabase 테이블 3개

### `product` — 고정, A~D 4행 (사람이 입력)

| 열 | 타입 | 설명 |
|---|---|---|
| `class_id` | text PK | `'A'`~`'D'` |
| `name` | text | 화면 표시 이름 (`'알루미늄 묶음 A'`) |
| `width_mm` | float8 not null | 가로 (로봇 base X 방향) |
| `depth_mm` | float8 not null | 세로 |
| `height_mm` | float8 not null | 높이 |
| `weight_g` | float8 not null default 0 | 0 = 미측정 (판별이 아직 무게를 안 씀) |

### `loc` — 고정, 위치 하나 = 1행 (사람이 입력)

| 열 | 타입 | 설명 |
|---|---|---|
| `name` | text PK | `'A'`·`'B'`·`'C'`·`'D'` = 적재 구역 1자리, `'measure_zone'` = 측정존 |
| `x`, `y` | float8 not null | mm, 로봇 base 기준 |
| `z` | float8 | 측정존만 (바닥 z). 적재 구역은 비움 (높이는 적재 파트가 가짐) |

- **A~D와 measure_zone 5행이 모두 있어야 관제가 시작해요.** D는 (300, 300) (11/18 입력).
- HOME 자세(관절 각도)는 DB에 넣지 않아요. 코드 `box_comm/common.py`의 `FIXED_POSES['home_j']`가 원본이에요.

### `state` — 가변, 1행 (id = 1). **DB 노드만 씀**, 화면은 이 한 줄만 구독

| 묶음 | 열 | 타입 | 언제 바뀌나 |
|---|---|---|---|
| 재고 | `a` `b` `c` `d` `u` | int | READY에서 UI가 설정하거나, 박스 처리 후 관제가 갱신. 작업 종료·중단 때 최종 스냅샷도 저장. `u` = 미분류 (구역이 꽉 차서 미분류로 보낸 것 포함) |
| 작업 결과 | `status` | text | `BEFORE` / `RUNNING` / `DONE` / `ERROR` / `STOPPED`. **다음 작업이 시작될 때까지 그대로 남음** |
| | `reason` | text | ERROR·STOPPED 이유. 작업 중단으로 물리 상태가 확정되지 않았으면 `재고 재측정 필요`를 포함. `DONE`은 정상 종료, `STOPPED`는 중단 당시 마지막 확인 재고 스냅샷 |
| | `started_at`, `finished_at` | timestamptz | RUNNING 될 때 / DONE·ERROR·STOPPED 될 때 |
| 지금 단계 | `stage` | text | 관제 단계 (아래 표). 수시로 바뀜 |
| | `job_id`, `box_id` | text | 지금 작업·박스 |
| | `box_seq` | int | 이번 작업에서 처리 끝난 박스 수 |
| | `start_at` | timestamptz | 카운트다운이 끝나는 시각 (`stage = STANDBY`일 때만, 아니면 비어 있음) |
| 로봇 | `robot_state` | text | 두산 상태 이름 (`STANDBY`, `MOVING`, `SAFE_STOP`, `EMERGENCY_STOP` …) |
| | `robot_ok` | bool | 작업 가능 (STANDBY·MOVING) |
| | `robot_connected` | bool | 로봇 상태 응답이 오고 있음 |
| | `robot_hint` | text | 사람이 할 일 한 줄 (정상이면 빈칸). 화면에 그대로 표시 |
| 연결 | `control_seen_at` | timestamptz | 관제 신호를 마지막으로 받은 시각 (관제가 2초마다 보냄) |
| | `heartbeat_at` | timestamptz | DB 노드가 2초마다 갱신 |
| | `updated_at` | timestamptz | 재고·결과가 바뀐 시각 |

`stage` 값 (관제 단계)

| 값 | 뜻 | 화면 |
|---|---|---|
| `READY` | 시작 버튼 기다림 (관제 상주 모드). 작업이 끝나면 여기로 돌아옴 | 시작 버튼 활성. 지난 결과는 `status`·`reason`으로 계속 표시 |
| `BEFORE`, `PREFLIGHT` | 데이터 받기·사전 점검 | "점검 중" |
| `STANDBY` | 카운트다운 | 확인 후 10분 대기. `start_at`까지 남은 시간 표시. 취소 버튼 활성 |
| `RUNNING` | 작업 시작 | |
| `START_SETTING` | 시작 직전 설정 | RUNNING 기록 후 그리퍼 열기 → HOME → 닫기. 기본 UI launch는 DB 재고를 사용하고 첫 스캔으로 진행 (`inventory_check:=true`면 실측 단계 추가) |
| `INVENTORY_CHECK` | 선택적 시작 재고 실측 | 별도 점검 서버를 실행하고 관제 점검 파라미터를 켠 경우 |
| `SCANNING`, `SCAN_DONE` | 탐지 | 단계 표시 "스캔" |
| `JUDGING`, `JUDGE_DONE` | 판별 | "판별" |
| `PLACING`, `PLACE_DONE` | 적재 | "적재" |
| `CLEARING` | 집기 실패 뒤 관제가 측정존을 비우는 중 | "복구 중" |
| `DONE`, `ERROR`, `STOPPED` | 끝남 (잠깐 보였다가 곧 `READY`) | `status`·`reason`으로 결과 표시 |

### 테이블 만들기 (SQL Editor에 그대로)

지금 `product`는 `name`이 PK이고 치수가 글자(`'60,40,20'`)라서 새로 만들어요. 값은 아래 insert에 다시 넣어 뒀어요.

```sql
drop table if exists product;
create table product (
  class_id text primary key check (class_id in ('A','B','C','D')),
  name text not null default '',
  width_mm float8 not null,
  depth_mm float8 not null,
  height_mm float8 not null,
  weight_g float8 not null default 0
);
insert into product values
  ('A', '알루미늄 묶음 A', 60, 40, 20, 130),
  ('B', '젠가 묶음 B',     75, 29, 25, 0),
  ('C', '알루미늄 묶음 C', 50, 40, 20, 108),
  ('D', '지우개 D',        50, 40, 20, 55.3);

drop table if exists loc;
create table loc (
  name text primary key check (name in ('A','B','C','D','measure_zone')),
  x float8 not null,
  y float8 not null,
  z float8
);
insert into loc values
  ('A', 480, -160, null),
  ('B', 650, -160, null),
  ('C', 580,  300, null),
  ('D', 300,  300, null),          -- 11/18 입력
  ('measure_zone', 300, -160, 30.9);   -- z = 측정존 바닥 (판별 실측, 11/18 30 → 30.9)

drop table if exists state;
create table state (
  id int primary key default 1 check (id = 1),
  a int not null default 0, b int not null default 0, c int not null default 0,
  d int not null default 0, u int not null default 0,
  status text not null default 'BEFORE',
  reason text not null default '',
  started_at timestamptz,
  finished_at timestamptz,
  stage text not null default '',
  job_id text not null default '',
  box_id text not null default '',
  box_seq int not null default 0,
  start_at timestamptz,
  robot_state text not null default '',
  robot_ok boolean not null default false,
  robot_connected boolean not null default false,
  robot_hint text not null default '',
  control_seen_at timestamptz,
  heartbeat_at timestamptz,
  updated_at timestamptz not null default now()
);
insert into state (id) values (1);

-- 권한: 브라우저(anon 키)는 읽기만. 쓰기는 DB 노드(service_role 키, RLS를 건너뜀)만.
alter table product enable row level security;
alter table loc     enable row level security;
alter table state   enable row level security;
create policy "read" on product for select to anon using (true);
create policy "read" on loc     for select to anon using (true);
create policy "read" on state   for select to anon using (true);

-- 실시간: state만
alter publication supabase_realtime add table state;
```

- `service_role` 키는 DB 노드 환경변수(`SUPABASE_SERVICE_KEY`)에만 넣어요. 웹 코드·git에는 **절대 넣지 마세요.** 저장소가 public이고 터널 주소는 누구나 들어올 수 있어요.
- 웹은 `anon` 키만 써요 (`NEXT_PUBLIC_SUPABASE_ANON_KEY`).

**판별 참고**: C와 D(지우개)는 치수가 같고, 판별이 물성(눌렀을 때 폭 변화)으로 가려요 (11/18 실물 확인). DB 노드는 항상 A→D 순서로 넘겨요.
**좌표 참고 (11/20)**: A~D 구역 위치는 다시 정할 예정이에요. 정하면 `loc` 행만 바꾸면 되고(적재 파트의 자리별 첫 접촉 Z는 적재 담당이 다시 잼). 측정존 바닥 z는 **30.9**(판별 실측)입니다. 판별의 높이 측정 접촉 기준에는 `h_floor_offset = -1.16 mm` 보정이 별도로 적용되지만, DB 좌표와 sync 집기 기준은 보정하지 않습니다. 공정 인계 WAYPOINT는 DB 측정존 x·y 위 절대 Z=160 mm입니다.

## 3. 화면이 판단하는 순서

`state` 한 줄로 아래 순서대로 판단해요 (위에서 걸리면 아래는 안 봄).

| 순서 | 조건 | 화면 |
|---|---|---|
| 1 | `지금 − heartbeat_at > 5초` | "DB 노드 꺼짐". 전부 회색, 버튼 비활성 (비상정지는 펜던트로) |
| 2 | `지금 − control_seen_at > 5초` | "관제 꺼짐". 로봇 상태 모름, 시작 비활성 |
| 3 | `robot_ok = false` | 로봇 칸 빨강 + `robot_hint` 그대로 표시. 시작 비활성 |
| 4 | `stage = 'READY'` | 시작 버튼 활성 |
| 5 | 그 외 | `stage`로 진행 단계, `status`·`reason`으로 결과 |

화면 구성

- **마지막 작업**: `status` · `box_seq`개 처리 · `finished_at` 끝남 · `reason`. 다음 시작 전까지 그대로 남아요.
- **경과 시간**: RUNNING 중에는 `지금 − started_at`, 끝났으면 `finished_at − started_at`.
- **적재 현황**: 구역당 정원 **6** (1자리 3층 × 2자리), 전체 24. 그림은 1자리 3층이 다 차면 2자리에 쌓아요.
  `layout = n => Array.from({length: Math.ceil(n/3)}, (_, i) => Math.min(3, n - 3*i))`
- **미분류**: `u` 숫자만 (쌓지 않음).
- **시각**: 상단 시계는 브라우저 시계로.

## 4. 버튼 (Next.js API 라우트 → DB 노드)

DB 노드가 노트북 안에서만 받는 창구를 열어요: `http://127.0.0.1:8765`

| 웹 라우트 | DB 노드로 | 응답 |
|---|---|---|
| `POST /api/estop` | `POST /estop` | `{ok, message}`. `ok=false`면 message(예: "관제 연결 안 됨 — 펜던트 비상정지 버튼을 쓰세요")를 그대로 띄움 |
| `POST /api/start` | `POST /start` | `{ok, message}`. 거절 이유 예: "준비 안 됨 (지금 관제 단계: RUNNING)" |
| `POST /api/cancel` | `POST /cancel` | `{ok, message}`. 카운트다운 취소. 작업이 시작된 뒤에는 거절 ("취소할 카운트다운 없음") → 그때는 비상정지 |
| 재고 조회 | `GET /inventory` | `{a, b, c, d, u, capacity}` |
| 재고 설정 | `POST /inventory` | `{a, b, c, d, u, source?}`. 관제가 READY일 때만. A~D는 `0..capacity`, U는 `0` 이상. 성공 시 DB 저장 후 관제에 변경 알림 |
| 상태 확인 | `GET /state` | 저장소 state 행에 최신 관제 stage·연결 시각을 병합해 반환 |
| 헬스 확인 | `GET /health` | `{ok, stage, store}` |

```ts
// app/api/estop/route.ts
export async function POST() {
  try {
    const r = await fetch('http://127.0.0.1:8765/estop', { method: 'POST' });
    return Response.json(await r.json());
  } catch {
    return Response.json({ ok: false, message: 'DB 노드 꺼짐 — 펜던트 비상정지 버튼을 쓰세요' });
  }
}

// app/api/start/route.ts — 터널 주소는 누구나 들어오므로 시작은 PIN 확인
export async function POST(req: Request) {
  const { pin } = await req.json().catch(() => ({}));
  if (process.env.START_PIN && pin !== process.env.START_PIN)
    return Response.json({ ok: false, message: 'PIN이 틀림' }, { status: 403 });
  try {
    const r = await fetch('http://127.0.0.1:8765/start', { method: 'POST' });
    return Response.json(await r.json());
  } catch {
    return Response.json({ ok: false, message: 'DB 노드 꺼짐' });
  }
}
```

버튼 동작

- **비상정지**: 팝업 "비상 정지를 하시겠습니까? 로봇이 현재 위치에서 정지합니다." [정지] → `/api/estop`, [취소] → 아무것도 안 보냄. 언제든 눌림.
  - 작업 중이면 관제가 작업을 멈추고 `status = STOPPED`, `reason = 'EMERGENCY STOP — by ui'`.
  - 카운트다운 중이면 시작하지 않고 `status = STOPPED`, reason 끝에 `(시작 전)`.
  - 대기(READY) 중이어도 `status = STOPPED`, reason 끝에 `(대기 중)`. 다음 시작 때 풀려요.
  - 화면은 **`status = STOPPED`면 작업 상태를 빨간불 "비상정지"**로 보여 주면 돼요 (버튼을 누른 화면뿐 아니라 다른 화면·관리자 GUI에도 똑같이 보임).
- **시작**: 시작 버튼 → 안전 확인 팝업(“10분 후 작업이 시작됩니다. 작업공간 내 사람 또는 파손 위험이 있는 물품이 없는지 확인해주세요.”) → [확인]에서 `/api/start` → 관제가 사전 점검 후 10분 카운트다운(로봇 정지) → 시작 설정에서 그리퍼 열기·HOME·닫기 → 재고 실측(옵션) → 스캔. [취소]는 팝업만 닫고 아무것도 보내지 않음.
  - 확인을 누른 시점부터 카운트다운이 시작되고, 대기 상태와 남은 시간을 표시합니다. 관리자 GUI에는 카운트다운을 건너뛰는 바로 시작 기능이 있으며 웹 UI에는 없습니다.
  - 재고 입력 UI는 READY에서 `POST /inventory`로 A~D와 U의 현재 재고를 저장합니다. DB 노드는 `inventory_changed` 내부 명령으로 관제 메모리를 새 값과 동기화합니다. 작업 중에는 수동 변경을 거부합니다.
  - 기본 UI launch는 `/inventory_check` 서버를 실행하지 않고, 작업 시작 때 DB 재고를 사용합니다. `inventory_check:=true`를 지정하면 실물 재고 실측을 수행합니다. 적재 결과는 박스 처리마다 DB에 저장하고, 완료·오류·정지 때 최종 스냅샷도 기록합니다.
  - 카운트다운 중에는 `start_at`으로 남은 시간을 보여 줘요.
  - **카운트다운 취소** 버튼 → `/api/cancel` → 관제가 작업을 시작하지 않고 `READY`로 돌아감. `status = BEFORE`, `reason = '시작 안 함: ui에서 취소'`.
  - 사전 점검에서 막히면 카운트다운 없이 `READY`로 돌아오고 `status = BEFORE`, `reason`에 이유가 남아요 (로그 창에도 FAIL 항목).
- `/api/cancel`은 `/api/start`와 같은 모양으로 만들면 돼요 (주소만 `/cancel`).
- **바로 시작(카운트다운 생략)은 웹에 없어요.** 현장 관리자 전용(관리자 GUI·관제 터미널)이에요. 현장에서는 작업공간을 직접 볼 수 있지만 원격 웹은 못 보기 때문이에요. DB 노드 창구도 이 명령(`start_now`)을 받지 않아요(404). 현장에서 바로 시작하면 로그에 `code: 'START_NOW'`(WARN)가 남으니, 웹 로그 창에 그대로 보여 주면 돼요.
- 버튼 기록은 로그 파일에 `code: 'UI_COMMAND'`로 남아요.

**관제 상주 모드 (11/18 구현)**: 관제를 켜 두면 `READY`에서 시작 명령을 기다려요. 작업이 끝나면(DONE·ERROR·STOPPED) 결과는 `status`에 남기고 다시 `READY`가 돼요. 대기 중에도 로봇 상태가 2초마다 올라가요.
관제가 작업 중에 죽었다가 다시 켜지면 DB 노드가 `RUNNING`으로 남은 작업을 `ERROR`로 정리해요.

## 5. 로그 (파일 → 화면)

- DB 노드가 `~/box_logs/{job_id}.jsonl`에 한 줄씩 붙여 써요 (한 줄 = JSON 하나, 시간순).
- 줄 형식: `{seq, stamp, job_id, source, stage, level, code, box_id, class_id, expected, measured, message, received_at}`
  - `source`: `control` / `scan` / `judge` / `place` / `ui` / `db` → 화면 태그 관제·탐지·판별·적재·UI·DB
  - `level`: `INFO` / `WARN` / `ERROR` → 색
  - `message`: 화면에 그대로 표시 (문구를 UI에서 만들지 않음)
  - `code`: 관제 `JOB_START`·`INVENTORY_CHECK_DONE`·`JOB_DONE`·`JOB_CANCELED`·`JOB_ERROR`·`JOB_STOPPED`·`BOX_DONE`·`BOX_RETRY`·`COUNT_MISMATCH`·`ZONE_CLEAR`·`STOP_REQUEST`·`ROBOT_ALARM`·`PREFLIGHT`·`JOB_COMMAND`·`START_NOW`·`CONTROL_ERROR`·`CONTROL_WARN`·`ZONE_FULL`, 파트 `PART_LOG`, DB 노드 `UI_COMMAND`·`INVENTORY_SET`·`DB_RETRY`.
  - **사전 점검**: WARN·FAIL 항목이 한 줄씩 `code = 'PREFLIGHT'`(FAIL은 `level = ERROR`)로 오고, 마지막에 '사전 점검 통과/실패' 한 줄이 와요 → 시작이 안 된 이유를 로그 창에서 바로 볼 수 있어요
  - **재고 확인 불가**: 적재가 수량을 못 재면 재고는 그대로 두고 `COUNT_MISMATCH` WARN '… 적재 현황 확인 불가 …'가 와요
  - **구역 꽉 참** (`ZONE_FULL`, WARN): 6개가 찬 구역의 박스는 미분류로 가요. 재고는 그 구역 6, `u` +1. "A 구역을 비우세요" 안내로 보여 주면 돼요
  - **DB 저장 실패** (`DB_RETRY`, source `db`): Supabase에 못 쓰면 DB 노드가 모아 뒀다가 다시 보내요. 작업은 멈추지 않아요. 다시 연결되면 INFO 한 줄

```ts
// app/api/logs/route.ts — 가장 최근 작업 파일의 마지막 N줄 (최신순)
import fs from 'fs'; import path from 'path';
export const dynamic = 'force-dynamic';
export async function GET(req: Request) {
  const n = Number(new URL(req.url).searchParams.get('limit') ?? 20);
  const dir = process.env.LOG_DIR ?? '/home/eunjeong/box_logs';
  if (!fs.existsSync(dir)) return Response.json([]);
  const files = fs.readdirSync(dir).filter(f => f.endsWith('.jsonl'))
    .map(f => ({ f, t: fs.statSync(path.join(dir, f)).mtimeMs })).sort((a, b) => b.t - a.t);
  if (!files.length) return Response.json([]);
  const lines = fs.readFileSync(path.join(dir, files[0].f), 'utf8').trim().split('\n');
  return Response.json(lines.slice(-n).reverse().map(l => JSON.parse(l)));
}
```

- 요약 숫자: 처리 = `state.box_seq`, 미분류 = `state.u`, 재시도 = 파일 안 `code = 'BOX_RETRY'` 줄 수, 오류 = `level = 'ERROR'` 줄 수.
- 화면은 1~2초마다 `/api/logs?limit=20`을 가져와요.
- 로그 화면 주의: 판별이 박스마다 측정 줄을 4~5개 남겨요. 기본은 WARN·ERROR와 `BOX_DONE`·`JOB_*`만 보여 주고, 나머지는 펼쳐 볼 수 있게 하면 깔끔해요.

## 6. 실행 순서 (노트북)

DB 노드 파일 (`box_database/box_database/`)

| 파일 | 내용 |
|---|---|
| `database_main.py` | 실행 진입점 (`ros2 run box_database database_main`) |
| `database_work.py` | 관제가 부르는 함수, 상태 동기화, 로그 파일, 버튼 창구 |
| `database_store.py` | 저장소: Supabase / 파일(JSON, DB 연동 전 확인용) |
| `database_config.py` | 환경 변수·주기·테이블 이름·fake 값 |
| `database_template.py` | 약속만 보여 주는 짧은 예시 (메모리, 참고용) |

**파일 모드 (DB 연동 전)**: Supabase 테이블 3개를 JSON 파일로 흉내 내요. 함수는 같고 저장소만 달라서, 나중에 Supabase로 바꿀 때 코드는 그대로예요.

| 파일 (`~/box_logs/fake_db/`) | Supabase 테이블 | 누가 씀 |
|---|---|---|
| `product.json` | `product` | 사람 (처음엔 기본값으로 만들어짐, 직접 고치면 다음 작업부터 반영) |
| `loc.json` | `loc` | 사람 (같음) |
| `state.json` | `state` 1행 | DB 노드 |
| `state_updates.jsonl` | (없음, 확인용) | DB 노드: `state`에 쓴 내용을 한 줄씩 — `{at, by, set}`. `by`: `service` = 관제 서비스(재고·작업 결과), `control_status` = 관제 단계·로봇. 2초마다 도는 heartbeat만 바뀐 줄은 뺌 |

Supabase로 옮길 때: 2장 SQL로 테이블을 만들고 → `python3 scripts/db_to_sql.py` 출력(파일 값 → insert 문)을 SQL Editor에 붙여 넣고 → 키를 export하고 `store`를 빼고 실행.

```bash
# Supabase 없이 (파일 모드)
ros2 run box_database database_main --ros-args -p store:=file

# Supabase
pip install supabase --break-system-packages
export SUPABASE_URL=https://kdvbiivjrdqvfwzomflj.supabase.co
export SUPABASE_SERVICE_KEY=...
ros2 run box_database database_main          # store:=auto (키가 있으면 Supabase, 없으면 파일)

# 웹
cd box_ui/robot-ui && npm run build && npm start        # 시연은 dev 말고 start
cloudflared tunnel --url http://localhost:3000 # 켤 때마다 주소가 바뀜 → 시연 직전 공유
```

## 7. 시험하는 법

| 무엇을 | 어떻게 | 기대 결과 |
|---|---|---|
| DB 노드만 (관제·로봇 없이) | 터미널 1: `database_main -p store:=file` / 터미널 2: `./scripts/db_check.sh` | 통과 N / 실패 0. 재고는 시험 전 값으로 되돌아감. 끝에 `state_updates.jsonl` 마지막 줄을 보여 줌 |
| 관제 + fake 파트 + 실제 DB 노드 | `./scripts/fake_run.sh` (11/18: DB는 항상 `database_main`, 기본 파일 모드 + 재고 0으로 시작. Supabase로 보려면 `DB_STORE=auto`) | 다른 터미널 `tail -f ~/box_logs/fake_db/state_updates.jsonl` 에 단계·재고·결과가 차례로 찍힘, `~/box_logs/{job_id}.jsonl`에 로그 |
| 비상정지 버튼 경로 | 위 상태에서 `curl -s -X POST localhost:8765/estop` | 관제가 `STOPPED : EMERGENCY STOP — by ui` |
| 화면만 (DB 노드 없이) | Supabase 대시보드에서 `state` 값을 손으로 바꿈 | 화면이 바로 바뀜. 로그는 `~/box_logs/test.jsonl`에 샘플 줄 |
| 재고 설정 | `curl -s -X POST localhost:8765/inventory -H 'Content-Type: application/json' -d '{"a":1,"b":0,"c":0,"d":0,"u":0}'` | READY에서 HTTP 200, 새 재고 저장 및 관제 메모리 갱신. READY 외에는 `ok=false`, 잘못된 값은 HTTP 400 |
| 시작·취소 버튼 | `RESIDENT=1 ./scripts/fake_run.sh` → 다른 터미널에서 `curl -s -X POST localhost:8765/start`, `.../cancel` | start: `{ok: true}` → STANDBY. cancel: READY로 돌아오고 `status=BEFORE`. 다시 start → 끝까지 → READY, `status=DONE` |
