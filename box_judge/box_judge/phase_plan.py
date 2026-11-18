#!/usr/bin/env python3
"""
phase_plan.py — 위상 정렬 계획 (기하 계산만, ROS · 로봇 없음)
================================================================
  목표: 가장 긴 변(L1)과 두 번째 변(L2)으로 된 면이 바닥, L1 = 로봇 x, L2 = 로봇 y.

  경우 (L1 >= L2 >= L3)
    H = L3 (또는 H - L3 <= flip_tol)  넘기지 않음: 세운 그리퍼로 짧은 수평 변을 잡아 J6 로 돌려 놓기만
    그 밖                             넘기기: '지금 세로(H)도 L3 도 아닌 변'을 클램핑해 그 축으로 90 deg
                                       (+tilt 로 기울여 잡고 tilt - 90 으로 넘긴다, 공중 수직축 회전 없음)

  핑거 모델 (2026-11-17 기운 자세 실측 맞춤, 잔차 0.18 mm)
    수직 동작은 '책상 30.9 + 링크 모델' 로 맞지만 실제 책상은 base z 약 7.2, 핑거 끝은 링크 모델보다 약 23.7 mm 앞이다
    (두 오차가 수직에서는 상쇄). 기운 자세에서는 상쇄되지 않아 아래 실측값을 쓴다.
      e(raw)   = TCP 에서 핑거 끝(중심)까지 툴 z 거리 = tip_L + (tip_h(raw) - tip_h(tip_raw))
      hang(th) = TCP 에서 그리퍼 최저점까지 z = e cos(th) + tip_b |sin(th)| + tip_y0 sin(th)

  rokey1 phase_align_test.py (실물 검증판) 의 plan() 을 원래 TCP 회전(pivot tcp, 11/18 확인) 기준으로 줄인 것.
================================================================
"""

import math

from box_comm.sync_grip import tip_h_mm

PAD_MAX = 110.0 - 10.0 - 0.5      # 패드 간격 최대 (원시 110 - 패드 10 - 명령 여유 0.5)


class PlanFailed(Exception):
    pass


# ---------------------------------------------------------------- 벡터 · 회전
def add(u, v):
    return [u[0] + v[0], u[1] + v[1], u[2] + v[2]]


def sub(u, v):
    return [u[0] - v[0], u[1] - v[1], u[2] - v[2]]


def mul(u, k):
    return [u[0] * k, u[1] * k, u[2] * k]


def dot(u, v):
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def cross(u, v):
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def tr(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def col(A, j):
    return [A[0][j], A[1][j], A[2][j]]


def rotx(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rotz(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def zyz_mat(A, B, C):
    a, b, c = (math.radians(v) for v in (A, B, C))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
            [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
            [-sb * cc, sb * sc, cb]]


def mat_zyz(R):
    """회전 행렬 -> 두산 ZYZ (A, B, C) [deg]."""
    cb = max(-1.0, min(1.0, R[2][2]))
    B = math.degrees(math.acos(cb))
    if math.sqrt(max(0.0, 1.0 - cb * cb)) > 1e-6:
        return math.degrees(math.atan2(R[1][2], R[0][2])), B, math.degrees(math.atan2(R[2][1], -R[2][0]))
    if cb > 0:
        return math.degrees(math.atan2(R[1][0], R[0][0])), B, 0.0
    return math.degrees(math.atan2(-R[0][1], -R[0][0])), B, 0.0


def rot_err_deg(Ra, Rb):
    d = mm(tr(Ra), Rb)
    return math.degrees(math.acos(max(-1.0, min(1.0, (d[0][0] + d[1][1] + d[2][2] - 1.0) / 2.0))))


def down_frame(xhat):
    """툴 z 가 아래(-Z), 툴 x 가 xhat(수평) 인 자세."""
    z = [0.0, 0.0, -1.0]
    y = cross(z, xhat)
    return [[xhat[0], y[0], z[0]], [xhat[1], y[1], z[1]], [xhat[2], y[2], z[2]]]


def yaw_deg(v):
    return math.degrees(math.atan2(v[1], v[0]))


def unit_yaw(d):
    return [math.cos(math.radians(d)), math.sin(math.radians(d)), 0.0]


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap90(a):
    return (a + 90.0) % 180.0 - 90.0


# ---------------------------------------------------------------- 핑거 모델
class FingerModel:
    def __init__(self, table_z, tip_L, tip_raw, tip_b, tip_y0):
        self.S0, self.L, self.raw_m, self.b, self.y0 = table_z, tip_L, tip_raw, tip_b, tip_y0

    def e(self, raw):
        return self.L + (tip_h_mm(raw) - tip_h_mm(self.raw_m))

    def hang(self, th, raw):
        s, c = math.sin(math.radians(th)), math.cos(math.radians(th))
        return self.e(raw) * c + self.b * abs(s) + self.y0 * s


# ---------------------------------------------------------------- 계획
def plan(cfg, dims, psi_d, center_xy, model, pad_total, cur_x_yaw=None):
    """dims = {'H','W','D'} [mm], psi_d = D 변 방위각 [deg] (W 는 +90), center_xy = 물체 중심 (측정존).
    cfg: tilts, tilt_sign, flip_tol, margin, tip_past, grip_edge, grid, lift_extra, approach, obj_clear, body_clear,
         open_margin, cal_gain, cal_offset. 반환 dict (case = 'yaw' / 'flip')."""
    H, W, D = dims['H'], dims['W'], dims['D']
    L = sorted([H, W, D], reverse=True)
    S0 = model.S0
    u = {'D': unit_yaw(psi_d), 'W': unit_yaw(psi_d + 90.0)}
    raw_of = lambda w: w * cfg['cal_gain'] + cfg['cal_offset'] + pad_total  # noqa: E731
    out = {'L': L, 'dims': dims}

    if H - L[2] <= cfg['flip_tol']:
        opts = []
        for short, long_ in (('D', 'W'), ('W', 'D')):
            if dims[short] > dims[long_] + cfg['flip_tol']:
                continue
            opts.append((abs(wrap90(90.0 - yaw_deg(u[short]))), short, wrap90(90.0 - yaw_deg(u[short]))))
        _, short, dpsi = min(opts)
        out.update({'case': 'yaw', 'clamp': short, 'clamp_mm': dims[short], 'clamp_yaw': yaw_deg(u[short]),
                    'dpsi': dpsi, 'final_h': H})
        return out

    s_name = 'D' if D <= W else 'W'
    a_name = 'W' if s_name == 'D' else 'D'
    A, S = dims[a_name], dims[s_name]
    if A + cfg['open_margin'] > PAD_MAX:
        raise PlanFailed(f'클램핑할 변 {a_name} {A:.1f} mm + 여유가 최대 패드 간격 {PAD_MAX} 초과')
    cands = [u[a_name], mul(u[a_name], -1.0)]
    if cur_x_yaw is not None:
        cands.sort(key=lambda v: abs(wrap180(yaw_deg(v) - cur_x_yaw)))
    xhat = cands[0]
    Rd = down_frame(xhat)
    c = [center_xy[0], center_xy[1], S0 + H / 2.0]
    corners = [add(add(add(c, mul(u['D'], i * D / 2)), mul(u['W'], j * W / 2)), [0.0, 0.0, k * H / 2])
               for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
    e_s = u[s_name]
    raw_grip = raw_of(A)
    raw_open = raw_of(A + cfg['open_margin'])
    sg = cfg['tilt_sign']
    target_a = 0.0 if A >= H else 90.0                    # 넘긴 뒤 수평으로 남는 A 와 H 중 긴 쪽이 x
    dpsi = wrap90(target_a - yaw_deg(xhat))

    def frange(lo, hi, step):
        if hi < lo:
            return [0.5 * (lo + hi)]
        return [lo + i * step for i in range(int(math.floor((hi - lo) / step + 1e-9)) + 1)]

    s_list = frange(-S / 2 + cfg['grip_edge'], S / 2 - cfg['grip_edge'], cfg['grid'])
    v_list = frange(cfg['grip_edge'], H - cfg['grip_edge'], cfg['grid'])
    best = None
    for th1 in cfg['tilts']:
        t_init, t_fin = sg * th1, sg * (th1 - 90.0)
        R1, R2 = mm(Rd, rotx(t_init)), mm(Rd, rotx(t_fin))
        z1, y1 = col(R1, 2), col(R1, 1)
        hang_i, hang_f = model.hang(t_init, raw_grip), model.hang(t_fin, raw_grip)
        for s_g in s_list:
            for v_g in v_list:
                G = add(add(c, mul(e_s, s_g)), [0.0, 0.0, v_g - H / 2])
                T = add(G, mul(z1, cfg['tip_past']))
                P = sub(sub(T, mul(z1, model.e(raw_grip))), mul(y1, model.y0))
                ci = P[2] - hang_i - S0
                if ci < cfg['margin'] or max(dot(sub(T, k), z1) for k in corners) > cfg['body_clear']:
                    continue
                kt = [mv(tr(R1), sub(k, P)) for k in corners]
                pt = mv(tr(R1), sub(c, P))
                k2 = [mv(R2, k) for k in kt]
                p2 = mv(R2, pt)
                place = [center_xy[0] - p2[0], center_xy[1] - p2[1], S0 - min(k[2] for k in k2)]
                cf = place[2] - hang_f - S0
                if cf < cfg['margin']:
                    continue
                key = (round(math.hypot(s_g, v_g - H / 2)), -min(ci, cf), abs(th1 - 45.0))
                if best is not None and key >= best['key']:
                    continue
                need = 0.0                                  # 0 ~ 90 deg 회전 내내 물체 · 핑거 최저점이 책상 위 margin 이상
                for i in range(31):
                    Rp = mm(Rd, rotx(sg * (th1 - 3.0 * i)))
                    oz = P[2] + min(mv(Rp, k)[2] for k in kt) - S0
                    gz = P[2] - model.hang(sg * (th1 - 3.0 * i), raw_grip) - S0
                    need = max(need, cfg['margin'] - min(oz, gz))
                best = {'key': key, 'tilt': th1, 't_init': t_init, 't_fin': t_fin, 'P': P, 'R1': R1, 'R2': R2,
                        'place': place, 'ci': ci, 'cf': cf, 'lift': need + cfg['lift_extra'], 'off': key[0]}
    if best is None:
        raise PlanFailed('조건을 만족하는 잡는 점 · 기울기가 없음 (물체가 너무 작거나 핑거 여유 부족)')
    z1 = col(best['R1'], 2)
    need_a = (S0 + H + cfg['obj_clear'] + model.hang(best['t_init'], raw_open) - best['P'][2]) / max(-z1[2], 0.2)
    ap_d = max(cfg['approach'], need_a)
    z2 = col(best['R2'], 2)
    need_r = (S0 + S + cfg['obj_clear'] + model.hang(best['t_fin'], raw_open) - best['place'][2]) / max(-z2[2], 0.2)
    out.update({'case': 'flip', 'clamp': a_name, 'clamp_mm': A, 'side_mm': S, 'xhat': xhat, 'dpsi': dpsi,
                'final_h': S, 'approach': ap_d, 'retreat': max(cfg['approach'], need_r),
                'P_start': sub(best['P'], mul(z1, ap_d)), **best})
    return out


def describe(p):
    L = p['L']
    head = f'L1 {L[0]:.1f} >= L2 {L[1]:.1f} >= L3 {L[2]:.1f}'
    if p['case'] == 'yaw':
        return f'{head} | 넘기지 않음: {p["clamp"]} {p["clamp_mm"]:.1f} mm 잡고 수직축 {p["dpsi"]:+.1f} deg'
    return (f'{head} | 넘기기: 클램핑 {p["clamp"]} {p["clamp_mm"]:.1f} mm, 기울기 {p["t_init"]:+.0f} -> {p["t_fin"]:+.0f}, '
            f'핑거 여유 {p["ci"]:.1f} / {p["cf"]:.1f} mm, 들어 올림 {p["lift"]:.1f}, 넘긴 뒤 수직축 {p["dpsi"]:+.1f} deg')
