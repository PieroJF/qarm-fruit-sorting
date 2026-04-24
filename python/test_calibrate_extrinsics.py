"""Unit tests for solve_survey1_extrinsics — synthetic pinhole round-trip.

Builds a known camera pose, projects the chessboard inner-corner grid,
feeds the 2D points back into solve_survey1_extrinsics, asserts
reconstruction error is submillimetre / subdegree.
"""
from __future__ import annotations
import os
import sys

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from calibrate_extrinsics import solve_survey1_extrinsics


_SQUARE_MM = 30.0
_INNER_COLS = 7
_INNER_ROWS = 5


def _chess_corners_in_base(origin_base_m: np.ndarray) -> np.ndarray:
    """Return (N, 3) inner-corner coords in base frame, metres."""
    pts = []
    for j in range(_INNER_ROWS):
        for i in range(_INNER_COLS):
            pts.append([
                origin_base_m[0] + i * _SQUARE_MM / 1000.0,
                origin_base_m[1] + j * _SQUARE_MM / 1000.0,
                origin_base_m[2],
            ])
    return np.asarray(pts, dtype=np.float64)


def _project(pts3d_base, R_cam_in_base, C_cam_in_base, K):
    """Project base-frame 3D points to image pixels under pinhole.
    R_cam_in_base: rotation of camera frame in base frame.
    C_cam_in_base: camera optical centre in base frame.
    """
    R_base_to_cam = R_cam_in_base.T
    pts_cam = (R_base_to_cam @ (pts3d_base.T - C_cam_in_base.reshape(3, 1))).T
    u = K[0, 0] * pts_cam[:, 0] / pts_cam[:, 2] + K[0, 2]
    v = K[1, 1] * pts_cam[:, 1] / pts_cam[:, 2] + K[1, 2]
    return np.stack([u, v], axis=1)


def test_solve_recovers_pose_within_tolerance():
    K = np.array([
        [380.0, 0.0, 320.0],
        [0.0, 380.0, 240.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)

    # Known camera pose: above and slightly in front of table,
    # tilted forward by 15° and offset to the right by 5 cm.
    tilt = np.deg2rad(15.0)
    R_cam_in_base = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(np.pi + tilt), -np.sin(np.pi + tilt)],
        [0.0, np.sin(np.pi + tilt),  np.cos(np.pi + tilt)],
    ])
    C_cam_in_base = np.array([0.40, 0.05, 0.33])

    origin = np.array([0.30, 0.00, 0.00])
    corners_3d = _chess_corners_in_base(origin)
    corners_2d = _project(corners_3d, R_cam_in_base, C_cam_in_base, K)

    R_rec, C_rec, rms = solve_survey1_extrinsics(
        corners_2d=corners_2d.astype(np.float32),
        corners_3d_base=corners_3d.astype(np.float32),
        K=K, dist_coeffs=dist,
    )

    # Reprojection residual should be tiny on noise-free synthetic data
    assert rms < 0.1  # pixels
    # Camera centre recovered within 1 mm
    np.testing.assert_allclose(C_rec, C_cam_in_base, atol=1e-3)
    # Rotation columns recovered within ~0.5°
    for k in range(3):
        cos_err = np.dot(R_rec[:, k], R_cam_in_base[:, k])
        assert cos_err > np.cos(np.deg2rad(0.5))
