#!/usr/bin/env python3
"""
avoidance_scatter_plot.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
회피 성공/실패 산점도 — 기동 시작점별 서브플롯

X축 : 속도 (m/s)
Y축 : 핸들 조향각 SW (°)
마커: 초록 ● = 성공 (SUCCESS)
      빨강 × = 실패 (충돌/낭떠러지)

각 서브플롯 = 회피 기동 시작점(m) 하나씩

사용법
  python3 avoidance_scatter_plot.py          # 기본 (6 패널)
  python3 avoidance_scatter_plot.py --all    # 전체 거리 (7 패널)
  python3 avoidance_scatter_plot.py --right  # 양(+) 조향만
  python3 avoidance_scatter_plot.py --both   # 좌우 모두
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import csv, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ══════════════════════════════════════════════════════
#  설정값  ── 여기만 바꾸면 됩니다
# ══════════════════════════════════════════════════════
BASE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.join(BASE, 'avoidance_dataset.csv')
OBS_DIST = 62          # 장애물 거리 (m) — JS DEFAULT_OBS

# 기본 표시 거리 목록 (straight_dist_m 값)
# 성공이 실제로 발생하는 구간: 5~35m
DEFAULT_DISTS = [10, 15, 20, 25, 30, 35]
ALL_DISTS     = [5, 10, 15, 20, 25, 30, 35]

# 스캐터에 표시할 조향각 범위 (SW°) — 실제 성공 발생 구간
STEER_MIN = 5
STEER_MAX = 35   # 35° 이상은 전부 낭떠러지로 데이터 패턴 없음

NCOLS = 3

# 마커 스타일 (참고 이미지와 동일)
S_COLOR = '#2ecc40'   # 성공 초록
F_COLOR = '#e74c3c'   # 실패 빨강
S_MARKER, F_MARKER = 'o', 'x'
MARKER_SIZE = 60
LINE_WIDTH  = 1.8
# ══════════════════════════════════════════════════════

# ── 인자 파싱 ──────────────────────────────────────────
use_all   = '--all'   in sys.argv
use_both  = '--both'  in sys.argv   # 좌우 모두 표시
show_dist = ALL_DISTS if use_all else DEFAULT_DISTS

# ── 데이터 로드 ────────────────────────────────────────
print("데이터 로드 중...")
rows = []
with open(SRC, encoding='utf-8') as f:
    for r in csv.DictReader(f):
        steer = int(r['steer_deg_sw'])
        dist  = int(r['straight_dist_m'])
        spd   = round(float(r['speed_kmh']) / 3.6, 2)  # km/h → m/s

        # 조향각 필터
        if use_both:
            if steer == 0:
                continue
            steer = abs(steer)     # 좌우 같이 보면 절대값으로 통합
        else:
            if steer <= 0:
                continue           # 기본: 우회전(+) 방향만

        if steer < STEER_MIN or steer > STEER_MAX:
            continue               # 범위 밖은 패턴 없으므로 제외
        if dist not in show_dist:
            continue

        rows.append({
            'speed': spd,
            'steer': steer,
            'dist':  dist,
            'ok':    int(r['avoidance_success']),
        })

print(f"  필터링 후: {len(rows):,}개 데이터 포인트")

# ── 속도·조향각 눈금 계산 ──────────────────────────────
all_speeds = sorted(set(r['speed'] for r in rows))
all_steers = sorted(set(r['steer'] for r in rows))

speed_min, speed_max = min(all_speeds) - 0.3, max(all_speeds) + 0.3
steer_lo,  steer_hi  = STEER_MIN - 1,          STEER_MAX + 1

# ── 서브플롯 레이아웃 ──────────────────────────────────
nrows = (len(show_dist) + NCOLS - 1) // NCOLS
fig, axes = plt.subplots(nrows, NCOLS,
                          figsize=(5.8 * NCOLS, 5.2 * nrows),
                          constrained_layout=True)
axes = np.array(axes).flatten()
fig.patch.set_facecolor('#ffffff')

for idx, d in enumerate(show_dist):
    ax = axes[idx]

    sub  = [r for r in rows if r['dist'] == d]
    ok_r = [r for r in sub if r['ok'] == 1]
    ng_r = [r for r in sub if r['ok'] == 0]

    # ── 산점도 ────────────────────────────────────────
    if ng_r:
        ax.scatter([r['speed'] for r in ng_r],
                   [r['steer'] for r in ng_r],
                   c=F_COLOR, s=MARKER_SIZE, marker=F_MARKER,
                   linewidths=LINE_WIDTH, zorder=3, alpha=0.80,
                   label='Crash / Cliff')
    if ok_r:
        ax.scatter([r['speed'] for r in ok_r],
                   [r['steer'] for r in ok_r],
                   c=S_COLOR, s=MARKER_SIZE, marker=S_MARKER,
                   zorder=4, alpha=0.90, edgecolors='#1a8a28',
                   linewidths=0.5, label='Success')

    # ── 성공 경계선 추정 (속도별 최소 성공 조향각) ────────
    boundary = {}
    for sp in all_speeds:
        sp_ok = [r['steer'] for r in ok_r if r['speed'] == sp]
        if sp_ok:
            boundary[sp] = min(sp_ok)

    if len(boundary) >= 2:
        bx = sorted(boundary.keys())
        by = [boundary[s] for s in bx]
        ax.step(bx, by, where='mid',
                color='#ff8800', lw=2.0, ls='--',
                zorder=5, alpha=0.85, label='Min steer boundary')

    # ── 서브플롯 꾸미기 ──────────────────────────────────
    remaining = OBS_DIST - d
    n_ok, n_total = len(ok_r), len(sub)
    rate = n_ok / n_total * 100 if n_total else 0

    ax.set_title(f'Start: {d} m   (→ {remaining} m to obstacle)',
                 fontsize=11, fontweight='bold', pad=9)
    ax.set_xlabel('Speed (m/s)', fontsize=10)
    ax.set_ylabel('Steer Angle SW (deg)', fontsize=10)

    ax.set_xlim(speed_min, speed_max)
    ax.set_ylim(steer_lo, steer_hi)
    ax.set_xticks(all_speeds[::2])          # 2개마다 눈금
    ax.set_yticks(all_steers)
    ax.tick_params(labelsize=8)

    ax.set_facecolor('#f7f9fc')
    ax.grid(True, ls='--', lw=0.6, color='#cccccc', alpha=0.8)

    # 성공률 박스
    ax.text(0.02, 0.97,
            f'Success: {n_ok} / {n_total}  ({rate:.0f}%)',
            transform=ax.transAxes, fontsize=8.5, va='top',
            color='#1a6a20', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.35',
                      facecolor='#e6ffe6', edgecolor='#88cc88', alpha=0.9))

    ax.legend(fontsize=8, loc='lower right',
              framealpha=0.9, edgecolor='#cccccc')

# 빈 서브플롯 숨김
for idx in range(len(show_dist), len(axes)):
    axes[idx].set_visible(False)

# ── 전체 범례 + 제목 ──────────────────────────────────
legend_els = [
    Line2D([0],[0], marker='o', color='w', markerfacecolor=S_COLOR,
           markersize=9, markeredgecolor='#1a8a28', label='Success'),
    Line2D([0],[0], marker='x', color=F_COLOR,
           markersize=9, markeredgewidth=LINE_WIDTH, label='Crash / Cliff'),
    Line2D([0],[0], color='#ff8800', lw=2, ls='--',
           label='Min steer boundary (estimated)'),
]
fig.legend(handles=legend_els, loc='lower center',
           ncol=3, fontsize=10, framealpha=0.95,
           bbox_to_anchor=(0.5, -0.04))

direction_note = '(both directions, abs value)' if use_both else '(right turn only)'
fig.suptitle(
    f'Avoidance Success vs Crash  ·  Speed × Steer Angle  {direction_note}\n'
    f'Obstacle at {OBS_DIST} m ahead  |  '
    f'Steer range shown: {STEER_MIN}–{STEER_MAX}° SW  |  SR = 15',
    fontsize=12, fontweight='bold', y=1.01)

# ── 저장 ──────────────────────────────────────────────
out_name = 'avoidance_scatter_all.png' if use_all else 'avoidance_scatter.png'
out_path = os.path.join(BASE, out_name)
plt.savefig(out_path, dpi=160, bbox_inches='tight', facecolor='white')
plt.close()
print(f"저장 완료: {out_path}")


# ── 텍스트 요약 출력 ──────────────────────────────────
print()
print("┌─ 기동 시작점별 성공률 요약 " + "─" * 35)
print(f"│  {'시작점':>6}  {'남은거리':>8}  {'총수':>5}  "
      f"{'성공':>5}  {'성공률':>7}  {'최적 조향각'}")
print("├" + "─" * 62)
all_rows_full = []
with open(SRC, encoding='utf-8') as f:
    for r in csv.DictReader(f):
        if int(r['steer_deg_sw']) > 0:
            all_rows_full.append(r)
for d in show_dist:
    sub = [r for r in all_rows_full if int(r['straight_dist_m']) == d]
    ok  = [r for r in sub if int(r['avoidance_success']) == 1]
    rate = len(ok)/len(sub)*100 if sub else 0
    best = sorted(ok, key=lambda r: float(r['min_clearance_m']),
                  reverse=True)[:1]
    best_str = (f"{best[0]['steer_deg_sw']}° SW @ "
                f"{float(best[0]['speed_kmh']):.0f} km/h") if best else 'N/A'
    print(f"│  {d:>4}m   {OBS_DIST-d:>6}m남음  "
          f"{len(sub):>5}  {len(ok):>5}  {rate:>6.1f}%  {best_str}")
print("└" + "─" * 62)
