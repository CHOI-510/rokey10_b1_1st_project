#!/usr/bin/env python3
"""
db_to_sql.py — 파일 모드 테이블(product.json, loc.json, state.json의 재고)을 Supabase용 SQL로 출력
  python3 scripts/db_to_sql.py                       # 기본 폴더 ~/box_logs/fake_db
  python3 scripts/db_to_sql.py ~/box_logs/fake_db > seed.sql
  → 출력을 Supabase SQL Editor에 붙여 넣기 (테이블은 docs/DB_UI_CONTRACT.md의 create 문으로 먼저 만들 것)
  재고는 지금 값을 그대로 옮긴다 (state 행은 id=1 하나만, 나머지 열은 DB 노드가 채움)
"""
import json
import os
import sys


def q(v):
    if v is None:
        return 'null'
    if isinstance(v, (int, float)):
        return repr(float(v)) if isinstance(v, float) else str(v)
    return "'" + str(v).replace("'", "''") + "'"


def main():
    folder = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/box_logs/fake_db')
    with open(os.path.join(folder, 'product.json'), encoding='utf-8') as f:
        product = json.load(f)
    with open(os.path.join(folder, 'loc.json'), encoding='utf-8') as f:
        loc = json.load(f)
    state = {}
    try:
        with open(os.path.join(folder, 'state.json'), encoding='utf-8') as f:
            state = json.load(f)
    except FileNotFoundError:
        pass

    print(f'-- {folder} 에서 만든 SQL')
    print('delete from product;')
    print('insert into product (class_id, name, width_mm, depth_mm, height_mm, weight_g) values')
    print(',\n'.join(f"  ({q(r['class_id'])}, {q(r.get('name', ''))}, {q(r['width_mm'])}, {q(r['depth_mm'])}, "
                     f"{q(r['height_mm'])}, {q(r.get('weight_g') or 0.0)})"
                     for r in sorted(product, key=lambda r: r['class_id'])) + ';')
    print('delete from loc;')
    print('insert into loc (name, x, y, z) values')
    print(',\n'.join(f"  ({q(r['name'])}, {q(r['x'])}, {q(r['y'])}, {q(r.get('z'))})" for r in loc) + ';')
    inv = ', '.join(f'{k} = {int(state.get(k) or 0)}' for k in 'abcdu')
    print(f'insert into state (id) values (1) on conflict (id) do nothing;')
    print(f'update state set {inv} where id = 1;')


if __name__ == '__main__':
    main()
