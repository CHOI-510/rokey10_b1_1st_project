-- Run after log_inventory_trigger.sql. Parser checks only; does not modify inventory/log.
do $test$
begin
  assert public.inventory_patch_from_log($m$시작 재고 점검 완료, DB 반영: {'A': 4, 'B': 2, 'C': 1, 'D': 2, 'U': 0}$m$)
    = '{"A":4,"B":2,"C":1,"D":2,"U":0}'::jsonb;
  assert public.inventory_patch_from_log('A 적재 완료 → A 5개') = '{"A":5}'::jsonb;
  assert public.inventory_patch_from_log('C 1번 자리 2층 적재 → 수량 2 (TCP Z=48.43)') = '{"C":2}'::jsonb;
  assert public.inventory_patch_from_log('B 1번 자리 3층 적재 → 수량 3 (TCP Z=62.11)') = '{"B":3}'::jsonb;
  assert public.inventory_patch_from_log('미분류 투하 완료 → 미분류 1개 (관제 집계)') = '{"U":1}'::jsonb;
  assert public.inventory_patch_from_log('집기 성공 (1/3번째, 폭 74.9 mm)') = '{}'::jsonb;
  assert public.inventory_patch_from_log('판별 결과 B') = '{}'::jsonb;
  assert public.inventory_patch_from_log('U 미분류 경사로 투하 동작 완료: TCP=[793.0]') = '{}'::jsonb;
  assert public.inventory_patch_from_log($m$작업 완료: 5개 처리, 재고 {'A': 5, 'B': 3, 'C': 3, 'D': 2, 'U': 1}, 불일치 0건$m$)
    = '{"A":5,"B":3,"C":3,"D":2,"U":1}'::jsonb;
end;
$test$;
