"""File reading and nearest-neighbour timestamp matching."""

import csv
import numpy as np


def read_tum_trajectory(path):
    """TUM format: timestamp x y z qx qy qz qw"""
    timestamps, positions = [], []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 8 and not line.startswith('#'):
                timestamps.append(float(parts[0]))
                positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(timestamps), np.array(positions)


def read_ground_truth(path):
    """
    CSV with header: point_id, easting, northing, height, env, timestamp.
    Returns list of {id, position, env, timestamp}.
    """
    pts = []
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            pts.append({
                'id':        row['point_id'],
                'position':  np.array([float(row['easting']), float(row['northing']), float(row['height'])]),
                'env':       row.get('env', 'outdoor'),
                'timestamp': float(row['timestamp']),
            })
    return pts


def find_nearest_pose(traj_ts, traj_pos, target_ts, max_dt=2.0):
    """One nearest pose per target timestamp, within max_dt seconds."""
    result = []
    for i, t in enumerate(target_ts):
        diffs = np.abs(traj_ts - t)
        idx = np.argmin(diffs)
        if diffs[idx] <= max_dt:
            result.append({'index': i, 'position': traj_pos[idx].copy()})
    return result


def find_gnss_at_timestamps(gnss_data, timestamps, tol=2.0):
    """Match each timestamp to the nearest GNSS row within tol seconds."""
    gnss_ts = gnss_data[:, 0]
    result = []
    for i, t in enumerate(timestamps):
        diffs = np.abs(gnss_ts - t)
        idx = np.argmin(diffs)
        if diffs[idx] <= tol:
            row = gnss_data[idx]
            result.append({
                'index':         i,
                'gnss_lat':      float(row[1]),
                'gnss_lon':      float(row[2]),
                'gnss_height':   float(row[3]),
                'gnss_pos_type': int(row[4])   if gnss_data.shape[1] >= 5 else -1,
                'gnss_blh_std':  float(row[5]) if gnss_data.shape[1] >= 6 else -1.0,
            })
    return result
