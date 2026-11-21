-- Supabase SQL Editor: run as postgres. No browser UPDATE policy is required.
begin;

-- Private watermark prevents replayed old logs from overwriting newer inventory.
create table if not exists public.inventory_log_cursor (
  singleton boolean primary key default true check (singleton),
  applied_at timestamptz not null,
  job_id text
);
alter table public.inventory_log_cursor enable row level security;
revoke all on public.inventory_log_cursor from public, anon, authenticated;
insert into public.inventory_log_cursor(singleton, applied_at)
select true, greatest(now(), coalesce(max(l."timestamp"::timestamptz), '-infinity'::timestamptz))
from public.log l
on conflict (singleton) do nothing;

-- Browser reads are retained; logs and inventory are written by trusted server code.
revoke insert, update, delete on public.log from public, anon, authenticated;
revoke update on public.inventory from public, anon, authenticated;

create or replace function public.apply_initial_inventory_log()
returns trigger
language plpgsql
security definer
set search_path = ''
as $function$
declare
  payload text;
  snapshot jsonb;
  key text;
  previous_at timestamptz;
  event_at timestamptz;
  affected integer;
begin
  if new.message is null or new.message not like '시작 재고 점검 완료, DB 반영:%' then
    return new;
  end if;

  -- Only parse the dictionary payload, never execute the message as SQL.
  payload := btrim(substr(new.message, length('시작 재고 점검 완료, DB 반영:') + 1));
  snapshot := replace(payload, chr(39), '"')::jsonb;
  if jsonb_typeof(snapshot) is distinct from 'object' then
    raise exception '초기 재고 메시지는 A/B/C/D/U JSON 객체여야 합니다.';
  end if;
  foreach key in array array['A','B','C','D','U'] loop
    if not (snapshot ? key) or jsonb_typeof(snapshot -> key) is distinct from 'number'
       or (snapshot ->> key) !~ '^[0-9]+$' then
      raise exception '초기 재고 값이 올바르지 않습니다: %', key;
    end if;
  end loop;

  event_at := new."timestamp"::timestamptz;
  if event_at is null then
    raise exception '초기 재고 로그의 timestamp가 필요합니다.';
  end if;
  select c.applied_at into previous_at
  from public.inventory_log_cursor c where c.singleton = true for update;
  if not found then
    raise exception 'inventory_log_cursor 초기화가 필요합니다.';
  end if;
  if event_at <= previous_at then
    return new; -- Keep the log, but do not replay old inventory snapshots.
  end if;

  -- All five locations must map to exactly one inventory row.
  -- Lock in a stable order so all changes commit together.
  perform 1 from public.inventory i
  where i.loc_id in ('A','B','C','D','U') order by i.loc_id for update;
  foreach key in array array['A','B','C','D','U'] loop
    select count(*) into affected from public.inventory i where i.loc_id = key;
    if affected <> 1 then
      raise exception 'inventory.loc_id %는 정확히 1행이어야 합니다 (현재 %행).', key, affected;
    end if;
  end loop;

  update public.inventory i
  set cnt = snapshot ->> i.loc_id
  where i.loc_id in ('A','B','C','D','U');
  update public.inventory_log_cursor
  set applied_at = event_at, job_id = new.job_id
  where singleton = true;
  return new;
end;
$function$;

revoke all on function public.apply_initial_inventory_log() from public, anon, authenticated;
drop trigger if exists apply_initial_inventory_after_log on public.log;
create trigger apply_initial_inventory_after_log
  after insert on public.log
  for each row execute function public.apply_initial_inventory_log();

commit;
