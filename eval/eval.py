#!/usr/bin/env python3
"""
Batch SLAM/RTK evaluation.

To add a new method, drop its sequences under trajectories/<method>/<seq>/ with:
  traj_offline.txt   (required)
  traj_online.txt    (optional)
  enu_origin.json    [lat, lon, alt]

Usage:
    python eval.py [--method M] [--seq S]
"""

import json
import argparse
import numpy as np
from pathlib import Path

from coords  import convert_extracted_points_to_utm, convert_gnss_to_utm
from readers import (read_tum_trajectory, read_ground_truth,
                     find_nearest_pose, find_gnss_at_timestamps)
from metrics import absolute_errors, se3_errors, gnss_errors

ROOT     = Path(__file__).resolve().parent.parent
TRAJ_DIR = ROOT / 'trajectories'
GT_DIR   = ROOT / 'ground_truth'
DATA_DIR = ROOT / 'data'


def _seq_config(n):
    return {'gt': GT_DIR / f'{n}.csv', 'gnss_gt': DATA_DIR / n / 'rtk.txt'}


def discover():
    slam_methods, all_seqs = set(), set()
    for mdir in sorted(TRAJ_DIR.iterdir()):
        if mdir.is_dir() and not mdir.name.startswith('.'):
            for sdir in sorted(mdir.iterdir()):
                if sdir.is_dir() and (sdir/'traj_offline.txt').exists():
                    slam_methods.add(mdir.name)
                    all_seqs.add(sdir.name)
    slam_methods, all_seqs = sorted(slam_methods), sorted(all_seqs)
    return slam_methods, all_seqs, any((DATA_DIR/s/'rtk.txt').exists() for s in all_seqs)


def check_inputs(method, seq_name):
    cfg   = _seq_config(seq_name)
    if method == 'rtk':
        miss = [] if cfg['gnss_gt'].exists() else ['gnss_gt']
    else:
        dpath = TRAJ_DIR / method / seq_name
        miss = [f for f in ['traj_offline.txt', 'enu_origin.json'] if not (dpath / f).exists()]
    if not cfg['gt'].exists():
        miss.append(cfg['gt'].name)
    return miss


def evaluate_trajectory(traj_path, ts_rows, gt_by_id, enu_origin):
    if not Path(traj_path).exists(): return None
    traj_ts, traj_pos = read_tum_trajectory(traj_path)
    poses = find_nearest_pose(traj_ts, traj_pos, [r['timestamp'] for r in ts_rows])
    if not poses: return None
    valid_rows = [ts_rows[p['index']] for p in poses]
    poses_utm  = convert_extracted_points_to_utm(poses, enu_origin)
    assocs = [{'gt_idx': row['point_id'], 'extracted_point': pu, 'gt_point': gt_by_id[row['point_id']]}
              for row, pu in zip(valid_rows, poses_utm) if row['point_id'] in gt_by_id]
    return assocs or None


def evaluate_gnss(gnss_path, ts_rows, gt_by_id):
    gnss_raw   = np.loadtxt(gnss_path)
    gnss_found = find_gnss_at_timestamps(gnss_raw, [r['timestamp'] for r in ts_rows])
    gnss_utm   = convert_gnss_to_utm(gnss_found)
    assocs = []
    for cp in gnss_utm:
        pid = ts_rows[cp['index']]['point_id']
        if pid not in gt_by_id: continue
        gpos = np.array([cp['gnss_utm_easting'], cp['gnss_utm_northing'], cp['gnss_utm_height']])
        assocs.append({'gt_idx': pid, 'gnss_point': cp, 'gt_point': gt_by_id[pid],
                       'gnss_utm': tuple(gpos), 'gt_utm': gt_by_id[pid]['position']})
    return assocs or None


def _run_sequence(method, seq_name):
    cfg      = _seq_config(seq_name)
    gt_pts   = read_ground_truth(str(cfg['gt']))
    ts_rows  = [{'point_id': p['id'], 'timestamp': p['timestamp']} for p in gt_pts]
    gt_by_id = {p['id']: p for p in gt_pts}

    if method != 'rtk':
        dpath  = TRAJ_DIR / method / seq_name
        origin = json.loads((dpath / 'enu_origin.json').read_text())
        assocs = evaluate_trajectory(str(dpath/'traj_offline.txt'), ts_rows, gt_by_id, origin)
        if assocs is None:
            return [('offline', None, 'trajectory evaluation failed')]
        ae, se  = absolute_errors(assocs), se3_errors(assocs)
        m       = {'n_pts': len(assocs), **ae, **(se or {})}
        results = [('offline', m, None)]
        if se:
            results.append(('offline_rel', m, None))
        on = evaluate_trajectory(str(dpath / 'traj_online.txt'), ts_rows, gt_by_id, origin)
        if on:
            results.append(('online', {'n_pts': len(on), **absolute_errors(on)}, None))
        return results
    else:
        assocs = evaluate_gnss(str(cfg['gnss_gt']), ts_rows, gt_by_id)
        if assocs is None:
            return [('rtk', None, 'no GNSS results')]
        return [('rtk', {'n_pts': len(assocs), **gnss_errors(assocs)}, None)]


def run_one(method, seq_name):
    miss = check_inputs(method, seq_name)
    if miss:
        return [('offline', None, f"missing: {', '.join(miss)}")]
    try:
        return _run_sequence(method, seq_name)
    except Exception as e:
        return [('offline', None, f'FAILED: {e}')]


_fmt = lambda v: '   —  ' if v is None else f'{v:.3f} '
_gap = lambda off, se3: f'{(off-se3)/off*100:.0f}%' if off and se3 and off > 0 else '  — '


def print_table(rows):
    data    = {(m, s, v): mt for m, v, s, mt in rows}
    cols    = [(m, m.replace('_', '-').upper()) for m in sorted({m for m, v, _, _ in rows if v != 'rtk'})]
    seqs    = sorted({s for _, _, s, _ in rows})
    has_rtk = any(v == 'rtk' for _, v, _, _ in rows)
    g       = lambda m, k: m.get(k) if m else None

    SLAM_HDR = f"  {'Online':>7}  {'Offline':>7}  {'SE3':>7}  {'Gap':>4}"
    SLAM_UNT = f"  {'[m]':>7}  {'[m]':>7}  {'[m]':>7}  {'[%]':>4}"
    print()
    h1 = f"{'':24}" + ''.join(f"  {lbl:^30}" for _, lbl in cols) + (f"  {'RTK':^7}" if has_rtk else '')
    h2 = f"{'Scene':<15} {'Seq.':<9}" + SLAM_HDR * len(cols) + (f"  {'Standalone':>10}" if has_rtk else '')
    h3 = f"{'':24}" + SLAM_UNT * len(cols) + (f"  {'[m]':>10}" if has_rtk else '')
    print(h1, h2, h3, '─' * len(h3), sep='\n')

    for seq in seqs:
        p = seq.rsplit('_seq', 1)
        scene = p[0].replace('_', ' ').title() if len(p) == 2 else seq
        slbl  = f'Seq. {p[1]}' if len(p) == 2 else ''
        line  = f"{scene:<15} {slbl:<9}"
        for method, _ in cols:
            on_v  = g(data.get((method, seq, 'online')),      'rmse_3d')
            off_v = g(data.get((method, seq, 'offline')),     'rmse_3d')
            se3_v = g(data.get((method, seq, 'offline_rel')), 'rmse_3d_rel')
            line += f"  {_fmt(on_v):>7}  {_fmt(off_v):>7}  {_fmt(se3_v):>7}  {_gap(off_v, se3_v):>4}"
        if has_rtk:
            line += f"  {_fmt(g(data.get(('rtk', seq, 'rtk')), 'rmse_3d')):>10}"
        print(line)
    print()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--method')
    p.add_argument('--seq')
    p.add_argument('--plot', action='store_true', help='Save per-sequence accuracy figures')
    args = p.parse_args()

    slam_methods, all_seqs, has_rtk = discover()
    methods   = [args.method] if args.method else slam_methods + (['rtk'] if has_rtk else [])
    sequences = [args.seq]    if args.seq    else all_seqs

    rows = []
    for method in methods:
        for seq_name in sequences:
            results = run_one(method, seq_name)
            rows.extend((method, v, seq_name, m) for v, m, e in results if m is not None)

    if rows:
        print_table(rows)

    if args.plot:
        from plot import plot_all
        plot_seqs = [args.seq] if args.seq else all_seqs
        plot_all(slam_methods, plot_seqs, has_rtk)


if __name__ == '__main__':
    main()
