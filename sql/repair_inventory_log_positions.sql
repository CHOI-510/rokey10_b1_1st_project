-- Run once in the Supabase SQL Editor when log INSERT fails with:
-- 42P01: relation "public.inventory_log_positions" does not exist.
-- The existing log trigger/function is retained. No log/inventory rows are deleted.
begin;
create table if not exists public.inventory_log_positions (
  loc_id text primary key check (loc_id in ('A','B','C','D','U')),
  applied_at timestamptz not null,
  job_id text
);
alter table public.inventory_log_positions enable row level security;
revoke all on public.inventory_log_positions from public, anon, authenticated;
-- Allow pending existing journals to establish the latest stock for each zone.
-- Existing watermark rows are never reset. The watcher processes recent files first.
insert into public.inventory_log_positions(loc_id, applied_at)
select zone, '-infinity'::timestamptz
from unnest(array['A','B','C','D','U']) as zones(zone)
on conflict (loc_id) do nothing;
notify pgrst, 'reload schema';
commit;
