"""Error metrics: absolute 3D RMSE and SE3-aligned relative RMSE."""

import math
import numpy as np


def _rmse(vals):
    return math.sqrt(sum(v ** 2 for v in vals) / len(vals))


def absolute_errors(associations):
    """RMSE of 3D absolute distance between extracted_point and gt_point."""
    errors = [float(np.linalg.norm(
        np.array(a['extracted_point']['position']) - np.array(a['gt_point']['position'])
    )) for a in associations]
    return {'rmse_3d': _rmse(errors)}


def se3_errors(associations):
    """
    RMSE after SVD rigid-body (SE3) alignment of extracted → GT.
    Returns {'rmse_3d_rel': float}, or None if fewer than 3 points.
    """
    if len(associations) < 3:
        return None
    src = np.array([a['extracted_point']['position'] for a in associations])
    dst = np.array([a['gt_point']['position']        for a in associations])
    sc, dc = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - sc).T @ (dst - dc))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = dc - R @ sc
    errors = [float(np.linalg.norm(
        R @ np.array(a['extracted_point']['position']) + t - np.array(a['gt_point']['position'])
    )) for a in associations]
    return {'rmse_3d_rel': _rmse(errors)}


def gnss_errors(associations):
    """RMSE of 3D absolute distance between gnss_utm and gt_utm."""
    errors = [float(np.linalg.norm(np.array(a['gnss_utm']) - np.array(a['gt_utm'])))
              for a in associations]
    return {'rmse_3d': _rmse(errors)}
