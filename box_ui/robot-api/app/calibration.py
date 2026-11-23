"""Server-only read access to loc inventory calibration. No robot commands."""
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from fastapi import HTTPException


def read_locations():
    url = (os.getenv('SUPABASE_URL') or os.getenv('NEXT_PUBLIC_SUPABASE_URL', '')).rstrip('/')
    key = os.getenv('SUPABASE_SERVICE_KEY', '')
    if not url or not key:
        raise HTTPException(503, '교정값 조회용 Supabase URL·서버 서비스 키가 설정되지 않았습니다.')
    headers = {'apikey': key, 'Accept': 'application/json'}
    if not key.startswith('sb_secret_'):
        headers['Authorization'] = 'Bearer ' + key
    try:
        request = Request(url + '/rest/v1/loc?select=id,coords,inventory_calibration', headers=headers)
        with urlopen(request, timeout=3) as response:
            rows = json.load(response)
    except (HTTPError, URLError, OSError, ValueError) as error:
        raise HTTPException(503, 'loc 교정값 조회 실패: inventory_calibration 컬럼·읽기 권한·DB 연결을 확인하세요.') from error
    if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
        raise HTTPException(503, "loc 교정값 응답 형식이 올바르지 않습니다.")
    return rows


def read_calibration():
    rows = read_locations()
    references = {}
    for row in rows:
        category = row.get('id')
        if category not in ('A', 'B', 'C', 'D'):
            continue
        if category in references:
            raise HTTPException(503, 'loc 교정값에 중복 구역이 있습니다.')
        references[category] = row.get('inventory_calibration')
    for category in 'ABCD':
        slots = references.get(category)
        if not isinstance(slots, dict) or any(not isinstance(slots.get(str(tier)), dict) for tier in (1, 2)):
            raise HTTPException(503, f'loc[{category}]의 1·2번 자리 교정값이 필요합니다.')
    return {'version': 2, 'references': references}
