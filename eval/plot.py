"""
Per-sequence per-control-point accuracy plots.

One figure per sequence:
  Top strip : GNSS quality colour per control point
  Bottom    : 3D error bars per method (offline SLAM + standalone RTK)
"""

import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.ticker as mticker
from pathlib import Path

from coords  import convert_extracted_points_to_utm, convert_gnss_to_utm
from readers import read_tum_trajectory, read_ground_truth, find_nearest_pose, find_gnss_at_timestamps

ROOT     = Path(__file__).resolve().parent.parent
TRAJ_DIR = ROOT / 'trajectories'
GT_DIR   = ROOT / 'ground_truth'
DATA_DIR = ROOT / 'data'

METHOD_COLORS = ['#2166AC', '#4DAC26', '#F4A582', '#762A83', '#1B7837']
RTK_COLOR     = '#D7191C'

GNSS_COLORS = {
    'Fix':    '#1B9E77',
    'Float':  '#E6AB02',
    'DGPS':   '#D95F02',
    'Single': '#CC2222',
    'No':     '#888888',
}


def _gnss_type(pos_type):
    return {4: 'Fix', 5: 'Float', 2: 'DGPS', 1: 'Single'}.get(int(pos_type), 'No')


def _slam_errors(method, seq_name, gt_pts):
    """Return {gt_id: error_3d} for a SLAM method's offline trajectory."""
    gt_by_id = {p['id']: p for p in gt_pts}
    ts_rows  = [{'point_id': p['id'], 'timestamp': p['timestamp']} for p in gt_pts]
    dpath    = TRAJ_DIR / method / seq_name
    traj_path = dpath / 'traj_offline.txt'
    if not traj_path.exists() or not (dpath / 'enu_origin.json').exists():
        return {}
    origin   = json.loads((dpath / 'enu_origin.json').read_text())
    traj_ts, traj_pos = read_tum_trajectory(str(traj_path))
    poses    = find_nearest_pose(traj_ts, traj_pos, [r['timestamp'] for r in ts_rows])
    poses_utm = convert_extracted_points_to_utm(poses, origin)
    result = {}
    for pose, pu in zip(poses, poses_utm):
        pid = ts_rows[pose['index']]['point_id']
        if pid in gt_by_id:
            result[pid] = float(np.linalg.norm(pu['position'] - gt_by_id[pid]['position']))
    return result


def _rtk_errors(seq_name, gt_pts):
    """Return {gt_id: {error, gnss_type}} from RTK GNSS measurements."""
    gnss_path = DATA_DIR / seq_name / 'rtk.txt'
    if not gnss_path.exists():
        return {}
    gt_by_id = {p['id']: p for p in gt_pts}
    ts_rows  = [{'point_id': p['id'], 'timestamp': p['timestamp']} for p in gt_pts]
    gnss_raw   = np.loadtxt(str(gnss_path))
    gnss_found = find_gnss_at_timestamps(gnss_raw, [r['timestamp'] for r in ts_rows])
    gnss_utm   = convert_gnss_to_utm(gnss_found)
    result = {}
    for cp in gnss_utm:
        pid = ts_rows[cp['index']]['point_id']
        if pid not in gt_by_id:
            continue
        gpos = np.array([cp['gnss_utm_easting'], cp['gnss_utm_northing'], cp['gnss_utm_height']])
        result[pid] = {
            'error':     float(np.linalg.norm(gpos - gt_by_id[pid]['position'])),
            'gnss_type': _gnss_type(cp.get('gnss_pos_type', -1)),
        }
    return result


def plot_sequence(seq_name, slam_methods, has_rtk, out_dir):
    gt_pts   = read_ground_truth(str(GT_DIR / f'{seq_name}.csv'))
    slam_data = {m: _slam_errors(m, seq_name, gt_pts) for m in slam_methods}
    rtk_data  = _rtk_errors(seq_name, gt_pts) if has_rtk else {}

    gt_ids = [p['id'] for p in gt_pts
              if any(p['id'] in d for d in slam_data.values()) or p['id'] in rtk_data]
    n = len(gt_ids)
    if n == 0:
        return

    n_bars  = len(slam_methods) + (1 if has_rtk else 0)
    bw      = min(0.7 / n_bars, 0.22)
    offsets = np.linspace(-(n_bars - 1) / 2 * bw, (n_bars - 1) / 2 * bw, n_bars)
    x       = np.arange(n)

    matplotlib.rcParams.update({
        'font.family': 'serif', 'font.size': 8.5,
        'axes.linewidth': 0.6, 'xtick.major.width': 0.5, 'ytick.major.width': 0.5,
    })

    fig = plt.figure(figsize=(max(7.0, n * 0.38), 3.4))
    gs  = fig.add_gridspec(2, 1, height_ratios=[0.10, 1.0],
                           hspace=0.06, left=0.08, right=0.97, top=0.91, bottom=0.18)
    ax_strip = fig.add_subplot(gs[0])
    ax_err   = fig.add_subplot(gs[1], sharex=ax_strip)

    # GNSS quality strip
    for xi, gid in zip(x, gt_ids):
        c = GNSS_COLORS.get(rtk_data[gid]['gnss_type'], '#888888') if gid in rtk_data else '#888888'
        ax_strip.bar(xi, 1.0, width=1.0, color=c, linewidth=0)
    ax_strip.set_ylim(0, 1)
    ax_strip.set_yticks([])
    ax_strip.set_ylabel('GNSS', fontsize=6.5, labelpad=6)
    for spine in ax_strip.spines.values():
        spine.set_visible(False)
    ax_strip.tick_params(bottom=False, labelbottom=False)

    # Error bars — SLAM methods
    method_handles = []
    for i, (method, color) in enumerate(zip(slam_methods, METHOD_COLORS)):
        errs  = [slam_data[method].get(gid, 0.0) for gid in gt_ids]
        label = method.replace('_', '-').upper()
        ax_err.bar(x + offsets[i], errs, bw, color=color, alpha=0.88, linewidth=0)
        method_handles.append(mpatches.Patch(color=color, alpha=0.88, label=label))

    # Error bars — RTK (clipped)
    CLIP = 2.0
    if has_rtk:
        rtk_errs = [rtk_data[gid]['error'] if gid in rtk_data else 0.0 for gid in gt_ids]
        rtk_disp = [min(e, CLIP) for e in rtk_errs]
        ax_err.bar(x + offsets[-1], rtk_disp, bw, color=RTK_COLOR, alpha=0.80, linewidth=0)
        for xi, val in zip(x, rtk_errs):
            if val > CLIP:
                ax_err.annotate('', xy=(xi + offsets[-1], CLIP * 1.02),
                                xytext=(xi + offsets[-1], CLIP * 0.84),
                                arrowprops=dict(arrowstyle='->', color=RTK_COLOR,
                                                lw=1.1, mutation_scale=7))
        rtk_max = max(rtk_errs)
        method_handles.append(mpatches.Patch(color=RTK_COLOR, alpha=0.80,
                                             label=f'RTK standalone (max {rtk_max:.1f} m)'))

    # 10 cm reference line
    ax_err.axhline(0.10, color='#555555', lw=0.7, ls='--', dashes=(4, 3))
    ax_err.text(n - 0.5, 0.105, '10 cm', va='bottom', ha='right', fontsize=6.5, color='#555555')

    y_max = CLIP * 1.08 if has_rtk else None
    ax_err.set_xlim(-0.5, n - 0.5)
    ax_err.set_ylim(0, y_max)
    ax_err.set_xticks(x)
    ax_err.set_xticklabels(gt_ids, rotation=50, ha='right', fontsize=6.0)
    ax_err.set_ylabel('3D Error (m)', labelpad=4)
    ax_err.yaxis.set_minor_locator(mticker.MultipleLocator(0.1))
    ax_err.grid(axis='y', which='major', lw=0.4, ls=':', color='#cccccc')
    ax_err.spines['top'].set_visible(False)
    ax_err.spines['right'].set_visible(False)

    # Legends
    gnss_handles = [mpatches.Patch(color=GNSS_COLORS[k], label=k)
                    for k in ['Fix', 'Float', 'DGPS', 'Single']]
    ax_strip.legend(handles=gnss_handles, loc='upper right', fontsize=6.0,
                    frameon=True, framealpha=0.9, edgecolor='#cccccc',
                    ncol=4, borderpad=0.3, handlelength=0.8,
                    bbox_to_anchor=(1.0, 1.6))
    ax_err.legend(handles=method_handles, loc='upper left', fontsize=7.0,
                  frameon=True, framealpha=0.9, edgecolor='#cccccc',
                  handlelength=1.2, borderpad=0.4)

    p = seq_name.rsplit('_seq', 1)
    title = f"{p[0].replace('_', ' ').title()} Seq. {p[1]}" if len(p) == 2 else seq_name
    fig.suptitle(f'Absolute positioning accuracy — {title}', fontsize=9, y=0.98)

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f'{seq_name}.pdf'
    fig.savefig(path, dpi=300, bbox_inches='tight')
    print(f'  → {path}')
    plt.close(fig)


def plot_all(slam_methods, all_seqs, has_rtk):
    out_dir = ROOT / 'results' / 'figures'
    for seq in all_seqs:
        plot_sequence(seq, slam_methods, has_rtk, out_dir)
