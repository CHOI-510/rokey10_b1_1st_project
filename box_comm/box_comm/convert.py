#!/usr/bin/env python3
"""
convert.py — box_interfaces 메시지 ↔ 파이썬 값 변환 (관제·판별·DB 공용, 11/18)
================================================================
관리: 관제(정쓰). 예전에는 control_interface·database_interface·judge_interface가 같은 함수를 따로 가지고 있었다.

  inv_to_dict(Inventory) -> {'A'..'D', 'U'}
  dict_to_inv(dict) -> Inventory          A~D 필수, U는 없으면 0. 다른 키가 있으면 ValueError
  spec_to_dict(ClassSpec) -> dict / dict_to_spec(dict) -> ClassSpec
  dict_to_poses(dict|None) -> FixedPoses  None이거나 measure_zone이 없으면 available=False
                                          home_j가 없으면 common.FIXED_POSES (11/18: HOME은 코드가 원본)
  (FixedPoses → dict는 common.poses_from_msg — 파트 interface가 rclpy 없이도 쓰도록 common에 있음)
================================================================
"""

from box_interfaces.msg import ClassSpec, FixedPoses, Inventory, PlaceZone

from box_comm.common import FIXED_POSES, INVENTORY_KEYS, UNCLASSIFIED, VALID_CLASSES


def inv_to_dict(inv: Inventory) -> dict:
    return {'A': inv.a, 'B': inv.b, 'C': inv.c, 'D': inv.d, 'U': inv.u}


def dict_to_inv(d: dict) -> Inventory:
    keys = set(d.keys())
    if not set(VALID_CLASSES) <= keys or not keys <= set(INVENTORY_KEYS):
        raise ValueError(f'inventory는 {VALID_CLASSES} 키(+ 선택 {UNCLASSIFIED})를 가져야 함 (받은 값: {d!r})')
    return Inventory(a=int(d['A']), b=int(d['B']), c=int(d['C']), d=int(d['D']),
                     u=int(d.get(UNCLASSIFIED, 0)))


def spec_to_dict(s) -> dict:
    return {'class_id': s.class_id, 'width_mm': float(s.width_mm), 'depth_mm': float(s.depth_mm),
            'height_mm': float(s.height_mm), 'weight_g': float(s.weight_g)}


def dict_to_spec(d: dict) -> ClassSpec:
    return ClassSpec(class_id=str(d['class_id']), width_mm=float(d['width_mm']),
                     depth_mm=float(d['depth_mm']), height_mm=float(d['height_mm']),
                     weight_g=float(d['weight_g']))


def dict_to_poses(d) -> FixedPoses:
    """{'home_j': [6], 'measure_zone': [3], 'place_zones': {'A': [x, y], ...}} → FixedPoses.
    place_zones에서 None인 클래스는 빠진다 (→ 관제 사전 점검 FAIL)."""
    msg = FixedPoses()
    if not d or d.get('measure_zone') is None:
        msg.available = False
        return msg
    home = [float(v) for v in (d.get('home_j') or FIXED_POSES['home_j'])]
    zone = [float(v) for v in d['measure_zone']]
    if len(home) != 6 or len(zone) != 3:
        raise ValueError(f'고정 좌표 길이 오류: home_j {len(home)}개(6), measure_zone {len(zone)}개(3)')
    zones = []
    for c, xy in (d.get('place_zones') or {}).items():
        if xy is None:
            continue
        if c not in VALID_CLASSES or len(xy) != 2:
            raise ValueError(f'적재 구역 형식 오류: {c}={xy} (클래스 {VALID_CLASSES}, [x, y] 2개)')
        zones.append(PlaceZone(class_id=c, x=float(xy[0]), y=float(xy[1])))
    msg.available = True
    msg.home_j = home
    msg.measure_zone = zone
    msg.place_zones = zones
    return msg
