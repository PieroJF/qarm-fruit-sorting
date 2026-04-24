"""
Session chessboard calibration CLI.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.1.

Two entry points:

  run_calibration_core(...)
    Pure-function orchestrator that takes already-captured inputs (touched
    TCP, survey joints, one frame, intrinsics) and writes session_cal.json.
    Unit-tested offline.

  main()
    Interactive CLI that collects the inputs from hardware + operator:
      1. Jog-touch the chessboard origin corner.
      2. Move arm to teach_points['survey1'].
      3. Capture a D415 frame.
      4. Call run_calibration_core.
"""
from __future__ import annotations
import argparse
import datetime as _dt
import json
import os
import sys
import time

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from session_cal import SessionCal
from homography_solver import solve_homography, camera_height_from_homography

_INNER_COLS = 7
_INNER_ROWS = 5
_SQUARE_MM = 30.0


def _chess_world_pts() -> np.ndarray:
    """Return the 24 inner-corner world coords in the chessboard's own
    frame (origin = top-left inner corner, +X = columns, +Y = rows).
    Shape (N, 2) float32, units: mm."""
    return np.array(
        [[i * _SQUARE_MM, j * _SQUARE_MM]
         for j in range(_INNER_ROWS)
         for i in range(_INNER_COLS)],
        dtype=np.float32,
    )


def _find_chessboard_corners(frame_bgr: np.ndarray) -> np.ndarray:
    """Return subpixel-refined inner corners, shape (N, 2) float32.
    Raises RuntimeError if findChessboardCorners fails."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, (_INNER_COLS, _INNER_ROWS),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        raise RuntimeError(
            f"findChessboardCorners failed - check framing, lighting, "
            f"and that the full {_INNER_COLS}x{_INNER_ROWS} inner-corner "
            f"pattern is visible.")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return refined.reshape(-1, 2).astype(np.float32)


def run_calibration_core(
    touched_tcp: np.ndarray,
    survey_joints: np.ndarray,
    captured_frame: np.ndarray,
    d415_intrinsics: dict,
    out_path: str,
) -> SessionCal:
    """Given inputs from the interactive CLI (or tests), solve the
    homography, derive camera height, build a SessionCal, and save it."""
    # Always save the captured frame so chessboard-detection failures can
    # be debugged visually without re-jogging Phase 1.
    logs_dir = os.path.join(os.path.dirname(_HERE), "logs")
    try:
        os.makedirs(logs_dir, exist_ok=True)
        debug_path = os.path.join(logs_dir, "calibration_latest.png")
        cv2.imwrite(debug_path, captured_frame)
        print(f"  [debug] survey frame saved to {debug_path}")
    except Exception as ex:
        print(f"  [warn] could not save debug frame: {ex}")
    image_pts = _find_chessboard_corners(captured_frame)
    world_pts = _chess_world_pts()
    H_pixel_to_chess, _rms_mm = solve_homography(image_pts, world_pts)
    # Reprojection RMS in the pixel domain (spec gate: < 2 px).
    H_inv = np.linalg.inv(H_pixel_to_chess)
    proj_px = cv2.perspectiveTransform(
        world_pts.reshape(-1, 1, 2).astype(np.float32), H_inv
    ).reshape(-1, 2)
    err_px = np.linalg.norm(proj_px - image_pts, axis=1)
    rms_px_in_pixels = float(np.sqrt(np.mean(err_px ** 2)))

    cam_h_mm = camera_height_from_homography(
        H_pixel_to_chess,
        fx=d415_intrinsics["fx"], fy=d415_intrinsics["fy"],
        cx=d415_intrinsics["cx"], cy=d415_intrinsics["cy"],
    )

    # --- solvePnP: recover camera pose in base frame at survey1 ---
    # Ray-plane projection (fruit_detector.pixel_to_base_frame) uses this
    # instead of the nadir-pinhole parallax approximation. Refuses to
    # persist calibration if reprojection RMS > 5 px.
    from calibrate_extrinsics import solve_survey1_extrinsics

    K_matrix = np.array([
        [d415_intrinsics["fx"], 0.0, d415_intrinsics["cx"]],
        [0.0, d415_intrinsics["fy"], d415_intrinsics["cy"]],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    # D415 colour stream is rectified by the SDK; zero distortion.
    dist_coeffs = np.zeros(5, dtype=np.float64)

    # corners_3d_base: same grid ordering as _chess_world_pts() (row-major
    # over rows then cols), offset into base frame by touched_tcp.
    origin_base = np.asarray(touched_tcp, dtype=np.float64)
    corners_3d_base = np.zeros((_INNER_ROWS * _INNER_COLS, 3), dtype=np.float32)
    k = 0
    for j in range(_INNER_ROWS):
        for i in range(_INNER_COLS):
            corners_3d_base[k] = [
                origin_base[0] + i * _SQUARE_MM / 1000.0,
                origin_base[1] + j * _SQUARE_MM / 1000.0,
                origin_base[2],
            ]
            k += 1

    try:
        R_cam, C_cam, pnp_rms = solve_survey1_extrinsics(
            corners_2d=image_pts.astype(np.float32),
            corners_3d_base=corners_3d_base,
            K=K_matrix,
            dist_coeffs=dist_coeffs,
        )
        print(f"  solvePnP: RMS = {pnp_rms:.2f} px, "
              f"camera at base XYZ = {C_cam.round(3).tolist()} m")
        cam_extrinsics_survey1 = {
            "R_cam_in_base": R_cam.tolist(),
            "C_cam_in_base_m": C_cam.tolist(),
            "reproj_rms_px": pnp_rms,
        }
    except RuntimeError as ex:
        print(f"  solvePnP REJECTED: {ex}")
        cam_extrinsics_survey1 = None

    cal = SessionCal(
        timestamp=_dt.datetime.now().isoformat(timespec="seconds"),
        chess_origin_in_base_m=np.asarray(touched_tcp, dtype=float),
        h_pixel_to_chess_mm=H_pixel_to_chess,
        survey_pose_joints_rad=np.asarray(survey_joints, dtype=float),
        chess_pattern={
            "cols": _INNER_COLS + 1, "rows": _INNER_ROWS + 1,
            "square_mm": _SQUARE_MM,
            "inner_cols": _INNER_COLS, "inner_rows": _INNER_ROWS,
        },
        d415_intrinsics=dict(d415_intrinsics),
        homography_reproj_rms_px=rms_px_in_pixels,
        camera_height_above_table_m=cam_h_mm / 1000.0,
        image_size=(captured_frame.shape[1], captured_frame.shape[0]),
        cam_extrinsics_survey1=cam_extrinsics_survey1,
    )
    cal.save(out_path)
    print(f"\n  session_cal.json written to {out_path}")
    print(f"  chessboard corners found: {image_pts.shape[0]}")
    print(f"  reprojection RMS: {rms_px_in_pixels:.3f} px "
          f"(gate: < 2.0 px)")
    print(f"  camera height above table: {cam_h_mm/1000:.3f} m")
    if rms_px_in_pixels >= 2.0:
        print("  [WARN] RMS exceeds 2 px gate - reteach survey1 or "
              "check chessboard flatness.")
    return cal


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", default=os.path.join(
        os.path.dirname(_HERE), "session_cal.json"))
    parser.add_argument("--no-touch", action="store_true",
        help="Skip the jog-touch step; reuse last session_cal.json's "
             "chess_origin_in_base_m.")
    args = parser.parse_args(argv)

    # Import hardware lazily so tests don't load them.
    from qarm_driver import QArmDriver
    from camera import QArmCamera
    from touch_probe import jog_and_capture, capture_tcp
    from calibrate_closed_loop import slow_move_to_joints  # existing helper

    driver = QArmDriver()
    driver.connect()
    time.sleep(0.3)

    # Cache location for the Phase-1 touched TCP, so iterating on Phase-2
    # camera/framing issues doesn't require re-jogging every time.
    touch_cache = os.path.join(os.path.dirname(_HERE), "logs",
                                "last_touched_tcp.json")

    try:
        # --- Phase 1: origin registration
        if args.no_touch:
            # Priority: session_cal.json -> touch cache -> fail
            if os.path.exists(args.out):
                prior = SessionCal.load(args.out)
                touched_tcp = prior.chess_origin_in_base_m
                print(f"  reusing chess_origin_in_base_m from "
                      f"{args.out}: {touched_tcp}")
            elif os.path.exists(touch_cache):
                with open(touch_cache) as f:
                    touched_tcp = np.asarray(json.load(f)["tcp"], dtype=float)
                print(f"  reusing touched_tcp from cache "
                      f"{touch_cache}: {touched_tcp}")
            else:
                sys.exit(f"--no-touch but neither {args.out} nor "
                         f"{touch_cache} exists")
        else:
            print("Phase 1: Place chessboard flat on table. Jog gripper "
                  "tip to the TOP-LEFT INNER CORNER. Press ENTER to "
                  "confirm, ESC to abort.")
            touched_tcp = jog_and_capture(driver)
            print(f"  origin TCP = {touched_tcp}")
            # Cache so Phase 2 retries don't require re-jogging.
            try:
                os.makedirs(os.path.dirname(touch_cache), exist_ok=True)
                with open(touch_cache, "w") as f:
                    json.dump({"tcp": list(touched_tcp)}, f)
                print(f"  [cache] saved to {touch_cache} "
                      f"(reuse via --no-touch)")
            except Exception as ex:
                print(f"  [warn] could not cache touched_tcp: {ex}")

        # --- Phase 2: homography capture
        tp_path = os.path.join(os.path.dirname(_HERE), "teach_points.json")
        with open(tp_path) as f:
            tp = json.load(f)
        if "survey1" not in tp or tp["survey1"].get("joints_rad") is None:
            sys.exit("teach_points.json: survey1 is missing or unset - "
                     "teach it first (D4 lab step).")
        survey_joints = np.asarray(
            tp["survey1"]["joints_rad"], dtype=float)

        print("\nPhase 2: moving arm to survey1 ...")
        slow_move_to_joints(driver, survey_joints,
                             tp["survey1"].get("gripper", 0.10))

        cam = QArmCamera()
        cam.open()
        try:
            # Warm up until the sensor actually delivers non-empty frames.
            # cam.read() returns the buffer regardless of frame validity,
            # so a fixed-count warm-up can silently yield all-zero frames.
            # Match the preflight pattern: poll until mean(color) > 5.
            deadline = time.time() + 10.0
            while time.time() < deadline:
                try:
                    c, _ = cam.read()
                except Exception:
                    continue
                if c.mean() > 5:
                    break
            else:
                raise RuntimeError(
                    "camera warm-up timeout - no valid frame after 10 s. "
                    "Unplug/replug D415 or power-cycle the QArm.")
            frames = []
            for _ in range(5):
                c, _ = cam.read()
                frames.append(c.copy())
            captured = np.median(np.stack(frames), axis=0).astype(np.uint8)
            if captured.mean() < 5:
                raise RuntimeError(
                    f"captured frame is mostly black (mean={captured.mean():.1f}). "
                    "D415 lost stream mid-capture; retry.")
            intr = {
                "fx": float(cam.intrinsics["fx"]),
                "fy": float(cam.intrinsics["fy"]),
                "cx": float(cam.intrinsics["cx"]),
                "cy": float(cam.intrinsics["cy"]),
            }
        finally:
            cam.close()

        # --- Phase 3: solve + write
        run_calibration_core(
            touched_tcp=touched_tcp,
            survey_joints=survey_joints,
            captured_frame=captured,
            d415_intrinsics=intr,
            out_path=args.out,
        )
    finally:
        try: driver.card.close()
        except Exception: pass
        driver._connected = False


if __name__ == "__main__":
    main()
