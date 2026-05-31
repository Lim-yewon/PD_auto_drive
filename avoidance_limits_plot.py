#!/usr/bin/env python3
"""
avoidance_limits_plot.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
회피 기동 한계점(임계점) 분석 그래프 생성기

avoidance_dataset.csv 를 읽어 다음 4종 한계 그래프를 생성:

  Plot 1  속도별 "최소 필요 조향각" 곡선
            — 각 (속도, 기동거리)에서 성공하는 최소 |SW각|
  Plot 2  속도별 "최대 허용 기동 시작점" 곡선
            — 각 (속도, 조향각)에서 성공하는 최대 기동시작점
  Plot 3  성공/실패 경계 등고선 히트맵
            — steer × start_dist 공간에서 속도별 50% 경계선
  Plot 4  파라미터 3개를 모두 담은 3D 산점도 (성공=초록, 실패=빨강)

출력: avoidance_limits.png / avoidance_limits.csv
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
from collections import defaultdict

BASE  = os.path.dirname(os.path.abspath(__file__))
SRC   = os.path.join(BASE, 'avoidance_dataset.csv')
OUT_P = os.path.join(BASE, 'avoidance_limits.png')
OUT_C = os.path.join(BASE, 'avoidance_limits.csv')

# ── 데이터 로드 ──────────────────────────────────────────────
print("데이터 로드 중...")
rows = []
with open(SRC, encoding='utf-8') as f:
    for r in csv.DictReader(f):
        rows.append({
            'speed':   int(r['speed_kmh']),
            'steer':   int(r['steer_deg_sw']),
            'dist':    int(r['straight_dist_m']),
            'success': int(r['avoidance_success']),
            'outcome': r['outcome'],
            'clearance': float(r['min_clearance_m']),
        })

speeds = sorted(set(r['speed']  for r in rows))
steers = sorted(set(r['steer']  for r in rows))
dists  = sorted(set(r['dist']   for r in rows))
abs_steers = sorted(set(abs(s) for s in steers if s > 0))

print(f"  {len(rows):,}행  |  속도 {len(speeds)}종  "
      f"|  조향 {len(steers)}종  |  거리 {len(dists)}종")


# ══════════════════════════════════════════════════════════════
#  한계값 계산
# ══════════════════════════════════════════════════════════════

# 1. 최소 필요 조향각: 각 (speed, dist) → min |steer| s.t. success
#    (양측 모두 체크 — 양수 방향 최솟값)
min_steer = {}   # (speed, dist) → min |steer_sw| or None
for sp in speeds:
    for d in dists:
        ok_steers = [abs(r['steer']) for r in rows
                     if r['speed'] == sp and r['dist'] == d
                     and r['success'] == 1 and r['steer'] != 0]
        min_steer[(sp, d)] = min(ok_steers) if ok_steers else None

# 2. 최대 허용 기동 시작점: 각 (speed, |steer|) → max dist s.t. success
max_dist = {}    # (speed, abs_steer) → max dist or None
for sp in speeds:
    for ast in abs_steers:
        ok_dists = [r['dist'] for r in rows
                    if r['speed'] == sp and abs(r['steer']) == ast
                    and r['success'] == 1]
        max_dist[(sp, ast)] = max(ok_dists) if ok_dists else None

# 3. 성공률 매트릭스: (steer, dist) → 속도별 성공률
# 모든 속도 합산 및 속도별 분리
def success_rate_matrix(rows_sub, steer_list, dist_list):
    mat = np.full((len(steer_list), len(dist_list)), np.nan)
    for si, st in enumerate(steer_list):
        for di, d in enumerate(dist_list):
            g = [r for r in rows_sub if r['steer'] == st and r['dist'] == d]
            if g:
                mat[si, di] = sum(r['success'] for r in g) / len(g) * 100
    return mat

# 속도 경계선용: 각 속도에서 50% 성공 경계
boundary_lines = {}   # speed → list of (steer_idx, dist_idx) boundary
for sp in speeds:
    sub  = [r for r in rows if r['speed'] == sp]
    mat  = success_rate_matrix(sub, steers, dists)
    boundary_lines[sp] = mat


# ══════════════════════════════════════════════════════════════
#  한계값 CSV 저장
# ══════════════════════════════════════════════════════════════
limit_rows = []
for sp in speeds:
    for d in dists:
        ms = min_steer.get((sp, d))
        limit_rows.append({
            'speed_kmh':            sp,
            'straight_dist_m':      d,
            'min_steer_sw_deg':     ms if ms is not None else 'N/A',
            'min_steer_wheel_deg':  round(ms / 15, 2) if ms is not None else 'N/A',
            'avoidable':            1 if ms is not None else 0,
        })
for sp in speeds:
    for ast in abs_steers:
        md = max_dist.get((sp, ast))
        limit_rows.append({
            'speed_kmh':            sp,
            'straight_dist_m':      'N/A',
            'min_steer_sw_deg':     ast,
            'min_steer_wheel_deg':  round(ast / 15, 2),
            'avoidable':            1 if md is not None else 0,
        })

with open(OUT_C, 'w', newline='', encoding='utf-8') as f:
    w = csv.DictWriter(f, fieldnames=limit_rows[0].keys())
    w.writeheader()
    w.writerows(limit_rows)
print(f"  CSV: {OUT_C}")


# ══════════════════════════════════════════════════════════════
#  그래프 스타일
# ══════════════════════════════════════════════════════════════
BG      = '#0a0f1e'
PANEL   = '#0f1828'
GRID_C  = '#1a2a40'
TICK_C  = '#7aaccc'
LABEL_C = '#a0c8e8'
TITLE_C = '#d0e8ff'

SUCCESS_CM = LinearSegmentedColormap.from_list(
    'sc', ['#1a0a2e', '#1a3070', '#1a6ade', '#00c8a0', '#3dffaa'], N=256)

SPEED_PALETTE = {
    10:  '#3355ff', 20: '#3399ff', 30: '#00d4ff',
    40:  '#3dffaa', 50: '#aaff44', 60: '#ffdd00',
    70:  '#ffaa00', 80: '#ff7700', 90: '#ff5500',
    100: '#ff3333', 110:'#ff1166', 120:'#ff00aa',
}

def style_ax(ax, title, xlabel, ylabel):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TITLE_C, fontsize=10, fontweight='bold', pad=7)
    ax.set_xlabel(xlabel, color=LABEL_C, fontsize=8)
    ax.set_ylabel(ylabel, color=LABEL_C, fontsize=8)
    ax.tick_params(colors=TICK_C, labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID_C)
    ax.grid(color=GRID_C, lw=0.5, alpha=0.7)


# ══════════════════════════════════════════════════════════════
#  Figure
# ══════════════════════════════════════════════════════════════
fig = plt.figure(figsize=(18, 14))
fig.patch.set_facecolor(BG)
gs  = gridspec.GridSpec(2, 2, figure=fig,
                        hspace=0.38, wspace=0.30,
                        top=0.91, bottom=0.06, left=0.07, right=0.97)

ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1])
ax3 = fig.add_subplot(gs[1, 0])
ax4 = fig.add_subplot(gs[1, 1])


# ── Plot 1: 속도별 최소 필요 조향각  ─────────────────────────
# X: 기동 시작점(m), Y: 최소 |SW 조향각|(°), 선=속도
style_ax(ax1,
         'Min Required Steer Angle  [Success Threshold]',
         'Evade Start Distance (m)',
         'Min |Steer SW| needed (deg)')

sel_speeds = speeds[::2]  # 격한 선 줄이기
for sp in sel_speeds:
    ys = [min_steer.get((sp, d)) for d in dists]
    # None → nan
    ys_plot = [y if y is not None else np.nan for y in ys]
    col = SPEED_PALETTE.get(sp, '#888888')
    ax1.plot(dists, ys_plot, color=col, lw=2.0, label=f'{sp} km/h',
             marker='o', markersize=3, alpha=0.9)

ax1.set_xlim(dists[0] - 2, dists[-1] + 2)
ax1.set_ylim(0, 95)
ax1.axhline(90, color='#ff4444', lw=1.0, ls=':', alpha=0.6, label='Max SW (90°)')
ax1.axvline(59, color='#ffcc00', lw=1.0, ls='--', alpha=0.5, label='Obs boundary 59m')
ax1.legend(fontsize=6.5, loc='upper left', ncol=2,
           facecolor='#0a1428', edgecolor=GRID_C, labelcolor=TICK_C)


# ── Plot 2: 속도별 최대 허용 기동 시작점  ────────────────────
# X: 속도(km/h), Y: 최대 start_dist, 선=조향각
style_ax(ax2,
         'Max Allowed Evade Start  [by Steer Angle]',
         'Speed (km/h)',
         'Max Start Distance (m)')

sel_steers = [5, 10, 15, 20, 30, 45, 60, 75, 90]
steer_colors = ['#3dffaa','#00d4ff','#2a7ade','#aa44ff',
                '#ffaa00','#ff6644','#ff3366','#ff0088','#cc0055']
for ast, col in zip(sel_steers, steer_colors):
    if ast not in abs_steers:
        continue
    ys = [max_dist.get((sp, ast)) for sp in speeds]
    ys_plot = [y if y is not None else np.nan for y in ys]
    ax2.plot(speeds, ys_plot, color=col, lw=2.0, label=f'±{ast}° SW',
             marker='s', markersize=3.5, alpha=0.9)

ax2.axhline(59, color='#ffcc00', lw=1.0, ls='--', alpha=0.5,
            label='Obs boundary 59m')
ax2.set_xlim(speeds[0] - 3, speeds[-1] + 3)
ax2.set_ylim(0, dists[-1] + 5)
ax2.legend(fontsize=6.5, loc='upper right', ncol=2,
           facecolor='#0a1428', edgecolor=GRID_C, labelcolor=TICK_C)


# ── Plot 3: 성공/실패 경계 등고선 히트맵 + 속도별 50% 경계선 ─
style_ax(ax3,
         'Success Boundary Contour  [all speeds]',
         'Evade Start Distance (m)',
         'Steer SW (deg)')

# 전체 속도 합산 성공률 매트릭스
all_mat = success_rate_matrix(rows, steers, dists)
im = ax3.imshow(all_mat, aspect='auto', origin='lower',
                cmap=SUCCESS_CM, vmin=0, vmax=100,
                extent=[dists[0], dists[-1], steers[0], steers[-1]])
cb = plt.colorbar(im, ax=ax3, fraction=0.04, pad=0.02)
cb.set_label('Success Rate (%)', color=LABEL_C, fontsize=7)
cb.ax.tick_params(colors=TICK_C, labelsize=6)

# 속도별 50% 경계선 (contour)
steer_arr = np.array(steers, dtype=float)
dist_arr  = np.array(dists,  dtype=float)
D, S      = np.meshgrid(dist_arr, steer_arr)  # shape (n_steer, n_dist)

for sp in sel_speeds:
    sub  = [r for r in rows if r['speed'] == sp]
    mat  = success_rate_matrix(sub, steers, dists)
    # contour at 50%
    try:
        cs = ax3.contour(D, S, mat, levels=[50],
                         colors=[SPEED_PALETTE.get(sp, '#888')],
                         linewidths=1.5, alpha=0.9)
        ax3.clabel(cs, fmt=f'{sp}', fontsize=6,
                   colors=[SPEED_PALETTE.get(sp, '#888')])
    except Exception:
        pass

ax3.axhline(0,  color='#ffffff', lw=0.5, ls='--', alpha=0.2)
ax3.axvline(59, color='#ffcc00', lw=1.0, ls='--', alpha=0.5)
ax3.set_xlim(dists[0], dists[-1])
ax3.set_ylim(steers[0], steers[-1])

from matplotlib.lines import Line2D
legend_els = [Line2D([0],[0], color=SPEED_PALETTE.get(sp,'#888'),
                     lw=1.5, label=f'{sp} km/h')
              for sp in sel_speeds]
ax3.legend(handles=legend_els, fontsize=6, loc='upper right', ncol=2,
           facecolor='#0a1428', edgecolor=GRID_C, labelcolor=TICK_C,
           title='50% boundary', title_fontsize=6)


# ── Plot 4: 최소 조향각 히트맵 (속도 × 기동거리)  ────────────
style_ax(ax4,
         'Min Required Steer SW (deg)  [heat = harder to avoid]',
         'Evade Start Distance (m)',
         'Speed (km/h)')

mat4 = np.full((len(speeds), len(dists)), np.nan)
for si, sp in enumerate(speeds):
    for di, d in enumerate(dists):
        ms = min_steer.get((sp, d))
        mat4[si, di] = ms if ms is not None else 95   # 95° = impossible marker

# 불가능 구간(95)은 별도 색으로
cmap4 = LinearSegmentedColormap.from_list(
    'limit', ['#001a10','#00aa44','#aaff00','#ffcc00','#ff4400','#440000'], N=256)
im4 = ax4.imshow(mat4, aspect='auto', origin='lower',
                 cmap=cmap4, vmin=0, vmax=95,
                 extent=[dists[0], dists[-1], speeds[0], speeds[-1]])
cb4 = plt.colorbar(im4, ax=ax4, fraction=0.04, pad=0.02)
cb4.set_label('Min |Steer SW| (deg) — bright = harder', color=LABEL_C, fontsize=7)
cb4.ax.tick_params(colors=TICK_C, labelsize=6)

# 불가능 구간 표시 (95로 채워진 셀)
impossible = np.where(mat4 >= 94, 1.0, np.nan)
ax4.imshow(impossible, aspect='auto', origin='lower',
           cmap='Reds', vmin=0, vmax=1, alpha=0.55,
           extent=[dists[0], dists[-1], speeds[0], speeds[-1]])

ax4.axvline(59, color='#ffcc00', lw=1.0, ls='--', alpha=0.6,
            label='Obs boundary 59m')
ax4.set_xlim(dists[0], dists[-1])
ax4.set_ylim(speeds[0], speeds[-1])
ax4.legend(fontsize=7, facecolor='#0a1428', edgecolor=GRID_C, labelcolor=TICK_C)


# ── 제목 ──────────────────────────────────────────────────────
fig.suptitle(
    'Autonomous Driving Simulator — Avoidance Limit Analysis\n'
    'Obstacle: x=0, z=−62 m  |  '
    'Coll.radius=3.0 m  |  Cliff |x|>7.5 m  |  SR=15',
    fontsize=13, fontweight='bold', color=TITLE_C, y=0.96)

plt.savefig(OUT_P, dpi=160, bbox_inches='tight', facecolor=BG)
plt.close(fig)
print(f"  그래프: {OUT_P}")
print("완료!")
