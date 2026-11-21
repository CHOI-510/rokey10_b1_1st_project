create role anon;
create role authenticated;
create role service_role bypassrls;
create table public.inventory (loc_id text primary key, standard text, cnt text not null);
insert into public.inventory values ('A','규격 A','99'),('B','규격 B','99'),('C','규격 C','99'),('D','규격 D','99'),('U','미분류 U','99');
create table public.log(id bigint generated always as identity primary key, timestamp timestamptz not null, job_id text not null, message text not null, state text not null);
grant select on public.inventory,public.log to anon,authenticated,service_role;
grant insert on public.log to service_role;
grant usage,select on sequence public.log_id_seq to service_role;
