"""Unit tests for calibrate_closed_loop. Hardware-free — uses synthetic
correspondences generated from a known ground-truth transform."""
import os
import sys
import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from calibrate_closed_loop import solve_extrinsics


def _random_rotation(rng):
    # Random unit quaternion -> rotation matrix
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),   1 - 2*(x*x + y*y)],
    ])


def test_solve_recovers_known_transform():
    """Given synthetic pixel<->base correspondences generated from a known
    T_ugreen_to_base and known intrinsics, solve_extrinsics must recover
    the transform within 1 cm / 1 deg."""
    rng = np.random.default_rng(42)
    K = np.array([[800., 0., 640.], [0., 800., 360.], [0., 0., 1.]])
    dist = np.zeros(5)

    # Known transform: base -> ugreen camera frame
    R_true = _random_rotation(rng)
    t_true = np.array([0.3, -0.2, 0.8])
    T_base_to_cam = np.eye(4)
    T_base_to_cam[:3, :3] = R_true
    T_base_to_cam[:3, 3] = t_true

    # Synthesize 10 base-frame points in a spread
    base_pts = rng.uniform(low=[0.1, -0.3, 0.0], high=[0.5, 0.3, 0.2],
                            size=(10, 3))
    # Project them through the known extrinsics + intrinsics
    pix = []
    for p in base_pts:
        p_cam = T_base_to_cam @ np.array([*p, 1.0])
        u = K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2]
        v = K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2]
        pix.append((u, v))
    pix = np.array(pix, dtype=np.float64)

    T_cam_to_base, rms = solve_extrinsics(base_pts, pix, K, dist)
    # Expected: T_cam_to_base == inverse(T_base_to_cam)
    T_expect = np.linalg.inv(T_base_to_cam)
    err_t = np.linalg.norm(T_cam_to_base[:3, 3] - T_expect[:3, 3])
    R_err = T_cam_to_base[:3, :3] @ T_expect[:3, :3].T
    angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
    assert err_t < 0.01, f"translation err {err_t * 1000:.1f} mm > 10 mm"
    assert angle < np.deg2rad(1.0), f"rotation err {np.rad2deg(angle):.2f} deg"
    assert rms < 1e-3, f"synthetic RMS {rms:.4f} m unexpectedly large"


if __name__ == "__main__":
    fails = 0
    for t in [test_solve_recovers_known_transform]:
        try:
            t()
            print(f"  [OK]   {t.__name__}")
        except Exception as ex:
            print(f"  [FAIL] {t.__name__}: {ex}")
            fails += 1
    sys.exit(fails)
