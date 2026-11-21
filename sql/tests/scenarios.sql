\set ON_ERROR_STOP on
begin;
set local role service_role;
insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '1 minute','fake-job',$m$시작 재고 점검 완료, DB 반영: {'A': 4, 'B': 2, 'C': 1, 'D': 2, 'U': 0}$m$,'작업중');
reset role;
do $$ begin assert (select jsonb_object_agg(loc_id,cnt::integer) from public.inventory)='{"A":4,"B":2,"C":1,"D":2,"U":0}'::jsonb; end $$;
\echo PASS initial snapshot via service_role INSERT

insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '2 minutes','fake-job','집기 성공 (1/3번째, 폭 60.3 mm)','적재중'),
 (now()+interval '3 minutes','fake-job','판별 결과 A','판별중');
do $$ begin assert (select cnt from public.inventory where loc_id='A')='4'; end $$;
\echo PASS pick and judgment do not increment inventory

insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '4 minutes','fake-job','A 2번 자리 2층 적재 → 수량 5 (TCP Z=40.36)','적재중'),
 (now()+interval '4 minutes 1 second','fake-job','A 적재 완료 → A 5개','작업중');
do $$ begin assert (select cnt from public.inventory where loc_id='A')='5'; end $$;
\echo PASS detail and summary do not double count A

insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '10 minutes','fake-job','B 적재 완료 → B 3개','작업중'),
 (now()+interval '6 minutes','fake-job','C 1번 자리 2층 적재 → 수량 2 (TCP Z=48.43)','적재중');
do $$ begin
 assert (select cnt from public.inventory where loc_id='B')='3';
 assert (select cnt from public.inventory where loc_id='C')='2';
end $$;
\echo PASS out-of-order events for different locations

insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '7 minutes','fake-job','C 적재 완료 → C 3개','작업중'),
 (now()+interval '8 minutes','fake-job','미분류 투하 완료 → 미분류 1개 (관제 집계)','작업중'),
 (now()+interval '8 minutes 1 second','fake-job','U 미분류 경사로 투하 동작 완료: TCP=[793.0]','적재중');
do $$ begin
 assert (select cnt from public.inventory where loc_id='C')='3';
 assert (select cnt from public.inventory where loc_id='U')='1';
end $$;
\echo PASS repeated C placement and unclassified U total

insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '1 minute','fake-job',$m$시작 재고 점검 완료, DB 반영: {'A': 4, 'B': 2, 'C': 1, 'D': 2, 'U': 0}$m$,'작업중'),
 (now()+interval '5 minutes','fake-job','B 적재 완료 → B 2개','작업중');
do $$ begin assert (select jsonb_object_agg(loc_id,cnt::integer) from public.inventory)='{"A":5,"B":3,"C":3,"D":2,"U":1}'::jsonb; end $$;
\echo PASS replayed and stale logs cannot rewind inventory

insert into public.log(timestamp,job_id,message,state) values
 (now()+interval '11 minutes','fake-job',$m$작업 완료: 5개 처리, 재고 {'A': 5, 'B': 3, 'C': 3, 'D': 2, 'U': 1}, 불일치 0건$m$,'작업완료');
do $$ begin assert (select sum(cnt::integer) from public.inventory)=14; end $$;
\echo PASS final inventory equals A5 B3 C3 D2 U1 total14

-- Invalid snapshots roll back the INSERT and all inventory changes.
do $$ declare before_count integer; rejected boolean:=false; begin
 select count(*) into before_count from public.log;
 begin
  insert into public.log(timestamp,job_id,message,state) values
   (now()+interval '12 minutes','fake-job',$m$시작 재고 점검 완료, DB 반영: {'A': -1, 'B': 2, 'C': 1, 'D': 2, 'U': 0}$m$,'작업중');
 exception when others then rejected:=true;
 end;
 assert rejected;
 assert (select count(*) from public.log)=before_count;
 assert (select cnt from public.inventory where loc_id='A')='5';
end $$;
\echo PASS invalid snapshot is rolled back

set local role anon;
do $$ declare denied boolean:=false; begin
 begin update public.inventory set cnt='999' where loc_id='A';
 exception when insufficient_privilege then denied:=true;
 end;
 assert denied;
end $$;
reset role;
\echo PASS browser anon cannot UPDATE inventory
select loc_id, cnt from public.inventory order by loc_id;
rollback;
