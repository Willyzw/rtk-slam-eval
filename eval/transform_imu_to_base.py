#!/usr/bin/env python3
"""
Transform a TUM-format trajectory from IMU frame to base-center frame.

For each pose (p, R) in the trajectory, the base-center position is:
    p_base = p + R @ t_imu_base

where t_imu_base is the translation from IMU to base center in the IMU frame.
The rotation is unchanged.

Usage:
    python transform_imu_to_base.py traj_imu.txt traj_base.txt
"""

import argparse
import numpy as np
from scipy.spatial.transform import Rotation


# Default IMU → base-center offset (metres, in IMU frame), from calib.yaml
t_imu_base = np.array([-0.073, -0.023, -0.172])


def transform_imu_to_base(traj: np.ndarray) -> np.ndarray:
    """
    traj  : (N, 8) array — TUM format: timestamp x y z qx qy qz qw
    return: (N, 8) array with positions shifted to base center
    """
    out = traj.copy()
    for i in range(len(traj)):
        R = Rotation.from_quat(traj[i, 4:8]).as_matrix()
        out[i, 1:4] = traj[i, 1:4] + R @ t_imu_base
    return out


def main():
    p = argparse.ArgumentParser(description='Transform TUM trajectory: IMU frame → base center')
    p.add_argument('input',  help='Input TUM trajectory (IMU frame)')
    p.add_argument('output', help='Output TUM trajectory (base-center frame)')
    args = p.parse_args()

    traj = np.loadtxt(args.input)
    if traj.ndim == 1:
        traj = traj[np.newaxis, :]
    if traj.shape[1] != 8:
        p.error(f'Expected 8 columns (TUM format), got {traj.shape[1]}')

    traj_base  = transform_imu_to_base(traj)

    np.savetxt(args.output, traj_base,
               fmt=['%.9f'] + ['%.6f'] * 3 + ['%.9f'] * 4)
    print(f'Saved {len(traj_base)} poses → {args.output}')


if __name__ == '__main__':
    main()
