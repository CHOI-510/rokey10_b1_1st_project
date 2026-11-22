"""Fetch preserved loc calibration through FastAPI; no credentials in ROS/UI."""
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def read_calibration(api_url):
    try:
        request = Request(api_url.rstrip('/') + '/robot/inventory/calibration', headers={'Accept': 'application/json'})
        with urlopen(request, timeout=5) as response:
            value = json.load(response)
        if not isinstance(value, dict) or value.get('version') != 2 or not isinstance(value.get('references'), dict):
            raise ValueError('Invalid calibration response')
        return value
    except (HTTPError, URLError, OSError, ValueError) as error:
        raise RuntimeError('Supabase loc 교정값을 가져오지 못했습니다. FastAPI와 inventory_calibration 설정을 확인하세요.') from error
