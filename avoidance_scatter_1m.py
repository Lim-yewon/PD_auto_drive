#!/usr/bin/env python3
"""
avoidance_scatter_1m.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
기동 시작점을 1m 간격으로 쪼갠 회피 성공/실패 산점도

  X축  : 속도 (m/s)
  Y축  : 핸들 조향각 SW (°)  — 성공 발생 구간 5–35°
  마커 : 초록 ● = SUCCESS   빨간 × = CRASH / CLIFF
  패널 : straight_dist 1m – 61m  (61개)

사용법
  python3 avoidance_scatter_1m.py           # 전체 61 패널
  python3 avoidance_scatter_1m.py --regen   # 캐시 무시하고 재시뮬
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import math, csv, os, sys, time, multiprocessing
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

BASE    = os.path.dirname(os.path.abspath(__file__))
CACHE   = os.path.join(BASE, 'avoidance_1m_cache.csv')
OUT_PNG = os.path.join(BASE, 'avoidance_scatter_1m.png')

# ══════════════════════════════════════════════════════
#  물리 상수 — index.html 완전 동일
# ══════════════════════════════════════════════════════
WHEELBASE, MAX_STEER_RAD = 2.5, math.pi / 6
COLL_RADIUS, CLIFF_X     = 3.0, 7.5
TARGET_THETA             = math.pi / 2
STEERING_RATIO           = 15
PHYS_DT                  = 0.02
EVADE_DIST               = 30.0
PD_KP, PD_KD             = 0.5, 0.05
PD_VREF                  = 60.0 / 3.6
OBS_X, OBS_Z             = 0.0, -62.0
MAX_TRAVEL               = 700.0

# ══════════════════════════════════════════════════════
#  파라미터 공간
# ══════════════════════════════════════════════════════
STEER_VALS = list(range(5, 91, 5))       # 5 ~ 90°  (우회전만)
SPEED_VALS = list(range(10, 130, 10))    # 10 ~ 120 km/h
DIST_VALS  = list(range(1, 62))          # 1 ~ 61m  (1m 간격)

# 그래프에 표시할 조향각 범위 (성공 발생 구간)
STEER_PLOT_MIN, STEER_PLOT_MAX = 5, 35


# ══════════════════════════════════════════════════════
#  단일 시뮬레이션 (avoidance_dataset.py 물리 로직 동일)
# ══════════════════════════════════════════════════════
def simulate(args):
    """(steer_sw, straight_dist, speed_kmh) → (steer, dist, speed_kmh, success)"""
    steer_sw, straight_dist, speed_kmh = args
    v = speed_kmh / 3.6
    if v < 0.1:
        return (steer_sw, straight_dist, speed_kmh, 0)

    x, z, theta = 0.0, 0.0, TARGET_THETA
    odist = phase_start = 0.0
    phase = 'STRAIGHT'
    prev_pd = prev_lane = 0.0
    passed_obs = False

    raw_e = -(steer_sw / STEERING_RATIO) * math.pi / 180.0
    delta_e = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, raw_e))

    max_steps = int(MAX_TRAVEL / (v * PHYS_DT)) + 200

    for _ in range(max_steps):
        # 충돌·낭떠러지 검사
        dobs = math.sqrt((x - OBS_X)**2 + (z - OBS_Z)**2)
        if not passed_obs and z <= OBS_Z:
            passed_obs = True
        if dobs < COLL_RADIUS or abs(x) > CLIFF_X:
            return (steer_sw, straight_dist, speed_kmh, 0)

        # 단계별 조향
        if phase == 'STRAIGHT':
            delta = 0.0
            if odist >= straight_dist:
                phase = 'EVADE'; phase_start = odist

        elif phase == 'EVADE':
            delta = delta_e
            if odist - phase_start >= EVADE_DIST:
                phase = 'PD_RETURN'; phase_start = odist
                prev_pd = (theta - TARGET_THETA) * (180 / math.pi)

        elif phase == 'PD_RETURN':
            err  = (theta - TARGET_THETA) * (180 / math.pi)
            er   = (err - prev_pd) / PHYS_DT
            sc   = PD_VREF / max(v, 1.0)
            spy  = max(-30., min(30., -err * PD_KP * sc - er * PD_KD * sc))
            sg   = -spy; prev_pd = err
            delta = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, -sg * math.pi / 180.))
            if abs(err) < 1.0 and abs(sg) < 0.5:
                phase = 'LANE_RETURN'; phase_start = odist
                la = max(10., v * 2.)
                prev_lane = (theta - (math.pi/2 + math.atan2(x, la))) * (180/math.pi)

        elif phase == 'LANE_RETURN':
            la   = max(10., v * 2.)
            dt   = math.pi / 2 + math.atan2(x, la)
            he   = (theta - dt) * (180 / math.pi)
            sc   = PD_VREF / max(v, 1.0)
            er   = (he - prev_lane) / PHYS_DT
            sg   = max(-30., min(30., he * PD_KP * sc + er * PD_KD * sc))
            prev_lane = he
            delta = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, -sg * math.pi / 180.))
            if abs(x) < 0.3 and abs((theta - TARGET_THETA) * (180/math.pi)) < 0.5 \
               and passed_obs:
                return (steer_sw, straight_dist, speed_kmh, 1)

        theta += (v / WHEELBASE) * math.tan(delta) * PHYS_DT
        x     += v * math.cos(theta) * PHYS_DT
        z     -= v * math.sin(theta) * PHYS_DT
        odist += v * PHYS_DT

    return (steer_sw, straight_dist, speed_kmh, 0)


# ══════════════════════════════════════════════════════
#  데이터 로드 (캐시 우선)
# ══════════════════════════════════════════════════════
def load_or_run(regen=False):
    if not regen and os.path.exists(CACHE):
        print(f"캐시 로드: {CACHE}")
        results = {}
        with open(CACHE, encoding='utf-8') as f:
            for r in csv.DictReader(f):
                key = (int(r['steer_sw']), int(r['dist']), int(r['speed_kmh']))
                results[key] = int(r['success'])
        print(f"  {len(results):,}개 로드 완료")
        return results

    # 새로 시뮬레이션
    args = [(s, d, sp)
            for d  in DIST_VALS
            for sp in SPEED_VALS
            for s  in STEER_VALS]
    total = len(args)
    n_w   = min(multiprocessing.cpu_count(), 8)
    print(f"시뮬레이션: {total:,}회  (워커 {n_w}개, DT={PHYS_DT}s)")
    t0 = time.time()

    raw = []
    with multiprocessing.Pool(n_w) as pool:
        for r in pool.imap_unordered(simulate, args, chunksize=300):
            raw.append(r)
            n = len(raw)
            if n % 4000 == 0:
                print(f"  {n:>6,}/{total:,}  {n/total*100:.1f}%")

    print(f"  완료 ({time.time()-t0:.1f}s)")

    # CSV 캐시 저장
    with open(CACHE, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['steer_sw', 'dist', 'speed_kmh', 'success'])
        w.writerows(raw)
    print(f"  캐시 저장: {CACHE}")

    return {(s, d, sp): ok for s, d, sp, ok in raw}


# ══════════════════════════════════════════════════════
#  그래프 생성
# ══════════════════════════════════════════════════════
def make_plot(results):
    steer_show = [s for s in STEER_VALS
                  if STEER_PLOT_MIN <= s <= STEER_PLOT_MAX]
    speeds_ms  = [round(sp / 3.6, 1) for sp in SPEED_VALS]

    NCOLS  = 8
    dists  = DIST_VALS          # 61개
    NROWS  = math.ceil(len(dists) / NCOLS)   # 8

    # 각 패널 크기 (인치)
    PW, PH = 3.8, 3.2
    FIG_W  = NCOLS * PW + 0.6
    FIG_H  = NROWS * PH + 1.8   # 상단 제목 + 하단 범례 여유

    fig = plt.figure(figsize=(FIG_W, FIG_H))
    fig.patch.set_facecolor('#ffffff')

    # suptitle 공간: top=0.97, 서브플롯은 그 아래부터
    TOP    = 0.962
    BOTTOM = 0.038
    LEFT   = 0.035
    RIGHT  = 0.998
    HSPACE = 0.58   # 제목-그래프 충돌 방지 핵심
    WSPACE = 0.08

    axes = []
    for i in range(NROWS * NCOLS):
        r, c = divmod(i, NCOLS)
        ax = fig.add_subplot(NROWS, NCOLS, i + 1)
        axes.append(ax)

    fig.subplots_adjust(top=TOP, bottom=BOTTOM, left=LEFT, right=RIGHT,
                        hspace=HSPACE, wspace=WSPACE)

    # ── 최대 성공률 계산 (색상 강도용) ──────────────────
    max_rate = 0.0
    for d in dists:
        tot = len(steer_show) * len(SPEED_VALS)
        ok  = sum(results.get((s, d, sp), 0)
                  for s in steer_show for sp in SPEED_VALS)
        if tot > 0:
            max_rate = max(max_rate, ok / tot * 100)

    # ── 서브플롯 그리기 ───────────────────────────────────
    for idx, d in enumerate(dists):
        ax  = axes[idx]
        row = idx // NCOLS
        col = idx  % NCOLS

        ok_x, ok_y, ng_x, ng_y = [], [], [], []
        for sp in SPEED_VALS:
            sp_ms = round(sp / 3.6, 1)
            for s in steer_show:
                v = results.get((s, d, sp), 0)
                if v == 1:
                    ok_x.append(sp_ms); ok_y.append(s)
                else:
                    ng_x.append(sp_ms); ng_y.append(s)

        # 실패 먼저, 성공 위에 덮기
        if ng_x:
            ax.scatter(ng_x, ng_y, c='#e74c3c', s=18, marker='x',
                       linewidths=1.1, zorder=2, alpha=0.65, rasterized=True)
        if ok_x:
            ax.scatter(ok_x, ok_y, c='#27ae60', s=20, marker='o',
                       zorder=3, alpha=0.88,
                       edgecolors='#166a25', linewidths=0.3, rasterized=True)

        # 성공률
        n_ok = len(ok_x)
        n_tot = n_ok + len(ng_x)
        rate  = n_ok / n_tot * 100 if n_tot else 0
        remaining = OBS_Z * -1 - d   # = 62 - d

        # 패널 배경: 성공률이 높을수록 연한 초록
        if rate > 0:
            alpha_bg = min(rate / max_rate * 0.18, 0.18)
            ax.set_facecolor(
                (0.85 + (1-alpha_bg)*0.15,
                 0.95 + alpha_bg*0.05,
                 0.85 + (1-alpha_bg)*0.15))
        else:
            ax.set_facecolor('#f5f5f7')

        # 성공 0이면 회색, 있으면 진한 초록
        rate_color = '#27ae60' if rate >= 10 else ('#e67e22' if rate > 0 else '#aaaaaa')

        # ── 제목 (패널 위) ──────────────────────────────
        ax.set_title(
            f'Start {d}m  |  {remaining}m left  |  {rate:.0f}%',
            fontsize=7.2, fontweight='bold', pad=4,
            color='#222222')

        # ── 축 설정 ──────────────────────────────────────
        ax.set_xlim(speeds_ms[0] - 0.4, speeds_ms[-1] + 0.4)
        ax.set_ylim(STEER_PLOT_MIN - 1.5, STEER_PLOT_MAX + 1.5)

        # 눈금: 좌측 열만 Y, 하단 행만 X 표시
        if col == 0:
            ax.set_ylabel('SW°', fontsize=6.5, labelpad=2)
            ax.set_yticks([5, 10, 15, 20, 25, 30, 35])
            ax.tick_params(axis='y', labelsize=5.5)
        else:
            ax.set_yticks([])

        if row == NROWS - 1:
            ax.set_xlabel('m/s', fontsize=6.5, labelpad=2)
            ax.set_xticks(speeds_ms[::2])
            ax.tick_params(axis='x', labelsize=5.5, rotation=30)
        else:
            ax.set_xticks([])

        ax.grid(True, ls=':', lw=0.35, color='#bbbbbb', alpha=0.9)
        for sp_obj in ax.spines.values():
            sp_obj.set_linewidth(0.5)
            sp_obj.set_edgecolor('#cccccc')

        # 성공 건수 (우상단 작은 텍스트)
        ax.text(0.97, 0.97, f'{n_ok}', transform=ax.transAxes,
                fontsize=6.2, va='top', ha='right',
                color=rate_color, fontweight='bold')

    # ── 빈 패널 숨기기 ────────────────────────────────────
    for idx in range(len(dists), len(axes)):
        axes[idx].set_visible(False)

    # ── 전체 범례 (하단 중앙) ─────────────────────────────
    legend_els = [
        Line2D([0], [0], marker='o', color='w',
               markerfacecolor='#27ae60', markeredgecolor='#166a25',
               markersize=9, label='Success'),
        Line2D([0], [0], marker='x', color='#e74c3c',
               markersize=9, markeredgewidth=1.4, label='Crash / Cliff'),
    ]
    fig.legend(handles=legend_els, loc='lower center', ncol=2,
               fontsize=10, framealpha=0.95, edgecolor='#cccccc',
               bbox_to_anchor=(0.5, 0.002))

    # ── 전체 제목 (최상단) ────────────────────────────────
    fig.suptitle(
        'Avoidance Scatter Plot  ·  Every 1m Evade-Start Interval  (1m – 61m)\n'
        'Obstacle: 62m ahead  |  Steer SW 5–35°  |  Speed 10–120 km/h  |  SR = 15  |  EVADE = 30m\n'
        'Panel title: start_dist  |  remaining_dist  |  success rate(%)',
        fontsize=11.5, fontweight='bold',
        y=0.998, va='top',
        color='#111111')

    # ── 저장 ──────────────────────────────────────────────
    print(f"PNG 저장 중... ({FIG_W:.0f}\" × {FIG_H:.0f}\")")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches='tight',
                facecolor='white', metadata={'Title': 'Avoidance 1m Scatter'})
    plt.close(fig)
    print(f"저장 완료: {OUT_PNG}")
    print(f"  파일 크기: {os.path.getsize(OUT_PNG)/1024:.0f} KB")


# ══════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════
if __name__ == '__main__':
    multiprocessing.freeze_support()

    regen   = '--regen' in sys.argv
    results = load_or_run(regen=regen)
    make_plot(results)
