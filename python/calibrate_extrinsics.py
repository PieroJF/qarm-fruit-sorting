"""SolvePnP wrapper to recover D415 pose in base frame at survey1.
See docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md §3.1.
"""
from __future__ import annotations
import numpy as np
import cv2


def solve_survey1_extrinsics(corners_2d: np.ndarray,
                              corners_3d_base: np.ndarray,
                              K: np.ndarray,
                              dist_coeffs: np.ndarray):
    """Return (R_cam_in_base, C_cam_in_base_m, reproj_rms_px).

    Parameters
    ----------
    corners_2d : (N, 2) float32, pixel coords of chessboard inner corners
                 in the survey1 frame.
    corners_3d_base : (N, 3) float32, same corners in the robot base frame
                      (metres), derived from chess_origin_in_base_m +
                      per-square spacing.
    K : (3, 3) camera intrinsics matrix.
    dist_coeffs : (5,) or (4,) lens distortion coefficients.

    Raises
    ------
    RuntimeError if solvePnP fails or reprojection RMS > 5 px.
    """
    ok, rvec, tvec = cv2.solvePnP(
        objectPoints=corners_3d_base.reshape(-1, 1, 3).astype(np.float32),
        imagePoints=corners_2d.reshape(-1, 1, 2).astype(np.float32),
        cameraMatrix=K.astype(np.float64),
        distCoeffs=dist_coeffs.astype(np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("cv2.solvePnP failed")

    R_base_to_cam, _ = cv2.Rodrigues(rvec)     # rotates base -> cam
    R_cam_in_base = R_base_to_cam.T            # rotates cam  -> base
    # solvePnP returns (rvec, tvec) such that X_cam = R_base_to_cam @ X_base
    # + tvec; set X_cam=0 and solve for the camera centre in base frame.
    C_cam_in_base = (-R_base_to_cam.T @ tvec).reshape(3)

    # Reprojection RMS for diagnostics / gate
    proj, _ = cv2.projectPoints(
        corners_3d_base.reshape(-1, 1, 3).astype(np.float32),
        rvec, tvec, K.astype(np.float64), dist_coeffs.astype(np.float64))
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - corners_2d.reshape(-1, 2), axis=1)
    rms = float(np.sqrt(np.mean(err ** 2)))

    if rms > 5.0:
        raise RuntimeError(
            f"solvePnP reprojection RMS {rms:.2f} px > 5 px; "
            "calibration rejected")

    return R_cam_in_base, C_cam_in_base, rms
