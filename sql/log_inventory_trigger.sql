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

-- Each location has its own watermark: a late A log must not suppress a newer B log.
create table if not exists public.inventory_log_positions (
  loc_id text primary key check (loc_id in ('A','B','C','D','U')),
  applied_at timestamptz not null,
  job_id text
);
alter table public.inventory_log_positions enable row level security;
revoke all on public.inventory_log_positions from public, anon, authenticated;
insert into public.inventory_log_positions(loc_id, applied_at)
select zone, c.applied_at from unnest(array['A','B','C','D','U']) as zones(zone)
cross join public.inventory_log_cursor c where c.singleton = true
on conflict (loc_id) do nothing;

-- Absolute counts only: repeated PART_LOG/BOX_DONE records never increment twice.
create or replace function public.inventory_patch_from_log(message text)
returns jsonb language plpgsql immutable set search_path = ''
as $parser$
declare
  matched text[];
  payload text;
  snapshot jsonb;
  key text;
begin
  if message is null then return '{}'::jsonb; end if;
  if message like '시작 재고 점검 완료, DB 반영:%' then
    payload := btrim(substr(message, length('시작 재고 점검 완료, DB 반영:') + 1));
  elsif message like '작업 완료:%' then
    matched := regexp_match(message, '재고[[:space:]]*(\{[^}]*\})');
    if matched is null then return '{}'::jsonb; end if;
    payload := matched[1];
  end if;
  if payload is not null then
    snapshot := replace(payload, chr(39), '"')::jsonb;
    if jsonb_typeof(snapshot) is distinct from 'object' then
      raise exception '재고 스냅샷은 A/B/C/D/U 객체여야 합니다.';
    end if;
    foreach key in array array['A','B','C','D','U'] loop
      if not (snapshot ? key) or jsonb_typeof(snapshot -> key) is distinct from 'number'
         or (snapshot ->> key) !~ '^[0-9]+$' then
        raise exception '재고 값이 올바르지 않습니다: %', key;
      end if;
    end loop;
    return jsonb_build_object('A', snapshot->'A', 'B', snapshot->'B', 'C', snapshot->'C',
                              'D', snapshot->'D', 'U', snapshot->'U');
  end if;

  matched := regexp_match(message, '^([ABCD])[[:space:]]+적재 완료[[:space:]]*→[[:space:]]*([ABCD])[[:space:]]+([0-9]+)개');
  if matched is not null then
    if matched[1] <> matched[2] then raise exception '적재 구역이 일치하지 않습니다.'; end if;
    return jsonb_build_object(matched[1], matched[3]::bigint);
  end if;
  matched := regexp_match(message, '^([ABCD])[[:space:]].*적재[[:space:]]*→[[:space:]]*수량[[:space:]]+([0-9]+)([[:space:]]|$)');
  if matched is not null then return jsonb_build_object(matched[1], matched[2]::bigint); end if;
  matched := regexp_match(message, '^미분류 투하 완료[[:space:]]*→[[:space:]]*미분류[[:space:]]+([0-9]+)개');
  if matched is not null then return jsonb_build_object('U', matched[1]::bigint); end if;
  return '{}'::jsonb; -- Pick success, detection, judgment and movements do not change stock.
end;
$parser$;
revoke all on function public.inventory_patch_from_log(text) from public, anon, authenticated;

create or replace function public.apply_initial_inventory_log()
returns trigger language plpgsql security definer set search_path = ''
as $function$
declare
  snapshot jsonb;
  key text;
  previous_at timestamptz;
  event_at timestamptz;
  affected integer;
begin
  snapshot := public.inventory_patch_from_log(new.message);
  if snapshot = '{}'::jsonb then return new; end if;
  event_at := new."timestamp"::timestamptz;
  if event_at is null then raise exception '재고 로그의 timestamp가 필요합니다.'; end if;

  -- Serialize inventory events, including snapshots, in a consistent lock order.
  perform 1 from public.inventory_log_positions p order by p.loc_id for update;
  for key in select jsonb_object_keys(snapshot) order by 1 loop
    select p.applied_at into previous_at from public.inventory_log_positions p where p.loc_id = key;
    if not found then raise exception 'inventory_log_positions 초기화 필요: %', key; end if;
    if event_at <= previous_at then continue; end if;
    perform 1 from public.inventory i where i.loc_id = key for update;
    select count(*) into affected from public.inventory i where i.loc_id = key;
    if affected <> 1 then
      raise exception 'inventory.loc_id %는 정확히 1행이어야 합니다 (현재 %행).', key, affected;
    end if;
    update public.inventory i set cnt = snapshot ->> key where i.loc_id = key;
    update public.inventory_log_positions p
      set applied_at = event_at, job_id = new.job_id where p.loc_id = key;
  end loop;
  return new;
end;
$function$;

revoke all on function public.apply_initial_inventory_log() from public, anon, authenticated;
drop trigger if exists apply_initial_inventory_after_log on public.log;
create trigger apply_initial_inventory_after_log
  after insert on public.log
  for each row execute function public.apply_initial_inventory_log();

commit;
