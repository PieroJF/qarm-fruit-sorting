"""Closed-loop UGreen-driven hand-eye calibration.

Workflow (hardware session):
  1. Load teach points and UGreen intrinsics.
  2. Capture baseline frame (arm out of UGreen view — user moves arm to
     homeplace0 / pickhome0 first; baseline is whatever the UGreen sees).
  3. For each of N calibration labels (cal_*):
       - Move arm to that teach point.
       - Capture UGreen frame.
       - tcp_from_diff -> pixel (u, v).
       - Pair with the teach point xyz (base-frame ground truth).
  4. Solve PnP (cv2.solvePnP) for T_base_to_cam, then invert for
     T_cam_to_base. Write calibration_ugreen.json with intrinsics +
     transform + residuals.

The resulting T_cam_to_base lets us convert any UGreen pixel that lies
on the known table plane (z ~= 0.13 m) to a base-frame XY — independent
cross-check of the D415 calibration.
"""
import json
import os
import sys
import time
import numpy as np
import cv2

from qarm_driver import QArmDriver
from ugreen_tracker import capture, tcp_from_diff, save_baseline, load_baseline
from ugreen_intrinsics import load_intrinsics

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
POINTS_FILE = os.path.join(REPO, "teach_points.json")
CALIB_OUT = os.path.join(REPO, "calibration_ugreen.json")
BASELINE_PATH = os.path.join(REPO, "logs", "ugreen_baseline.png")
CAPTURE_DIR = os.path.join(REPO, "logs", "closed_loop")


def solve_extrinsics(base_pts, pixels, K, dist):
    """Solve cv2.solvePnP for T_cam_to_base given N correspondences.

    Parameters
    ----------
    base_pts : (N, 3) float — object points in the robot base frame.
    pixels   : (N, 2) float — image points in the UGreen pixel frame.
    K        : (3, 3) float — camera intrinsic matrix (pixels).
    dist     : (5,)   float — distortion coefficients.

    Returns (T_cam_to_base (4,4), rms_residual_m)
    """
    if len(base_pts) < 4:
        raise ValueError("need >= 4 correspondences")
    ok, rvec, tvec = cv2.solvePnP(
        base_pts.astype(np.float64).reshape(-1, 1, 3),
        pixels.astype(np.float64).reshape(-1, 1, 2),
        K.astype(np.float64), dist.astype(np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("cv2.solvePnP returned False")
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.flatten()
    # R,t give base -> cam. Invert for cam -> base.
    T_b2c = np.eye(4); T_b2c[:3, :3] = R; T_b2c[:3, 3] = t
    T_c2b = np.linalg.inv(T_b2c)

    # Compute residuals in base frame: project each pixel back to its
    # base-frame point and compare.
    reproj_pix, _ = cv2.projectPoints(
        base_pts.reshape(-1, 1, 3), rvec, tvec, K, dist)
    err_px = np.linalg.norm(pixels - reproj_pix.reshape(-1, 2), axis=1)
    # Convert to approx metres at the typical fruit depth (0.8 m) for
    # intuition; this is reported in the log, not used by the solver.
    rms_px = float(np.sqrt((err_px ** 2).mean()))
    # Approx metres per pixel at 0.8 m depth, K[0,0] ~ 800 => ~1 mm/pix
    rms_m = rms_px * 0.8 / float(K[0, 0])
    return T_c2b, rms_m


def slow_move_to_joints(q, target, grip, seconds=3.0, steps=200):
    cj, cg = q.read_all()
    cj = np.array(cj, dtype=float); cg = float(cg)
    for i in range(1, steps + 1):
        a = i / steps
        s = 3 * a * a - 2 * a * a * a
        q.set_joints_and_gripper(
            cj + s * (target - cj),
            cg + s * (grip - cg))
        time.sleep(seconds / steps)
    for _ in range(20):
        q.set_joints_and_gripper(target, grip)
        time.sleep(0.05)


def run_calibration(labels, baseline_path=BASELINE_PATH,
                     capture_dir=CAPTURE_DIR):
    """Execute the closed-loop calibration against the labels in order.
    Returns the result dict and writes calibration_ugreen.json."""
    os.makedirs(capture_dir, exist_ok=True)

    with open(POINTS_FILE) as f:
        pts = json.load(f)
    intr, dist = load_intrinsics()
    K = np.array([[intr['fx'], 0, intr['cx']],
                  [0, intr['fy'], intr['cy']],
                  [0, 0, 1]], dtype=np.float64)

    baseline = load_baseline(baseline_path)

    q = QArmDriver(); q.connect(); time.sleep(0.3)
    for _ in range(5):
        try:
            q.read_all(); break
        except Exception:
            time.sleep(0.3)

    base_pts, pixels, used_labels = [], [], []
    try:
        for lbl in labels:
            if lbl not in pts:
                print(f"  [skip] {lbl} missing from teach_points"); continue
            target = np.array(pts[lbl]['joints_rad'], dtype=float)
            grip = float(pts[lbl].get('gripper', 0.15))
            print(f"\n-> {lbl}")
            slow_move_to_joints(q, target, grip)
            frame = capture()
            out = os.path.join(capture_dir, f"{lbl}.png")
            cv2.imwrite(out, frame)
            tcp = tcp_from_diff(frame, baseline)
            if tcp is None:
                print(f"    no TCP detected; skipping")
                continue
            base_pts.append(pts[lbl]['xyz_m'])
            pixels.append(list(tcp))
            used_labels.append(lbl)
            print(f"    pixel={tcp}  base={np.round(pts[lbl]['xyz_m'],3)}")
    finally:
        try: q.card.close()
        except Exception: pass
        q._connected = False

    if len(used_labels) < 4:
        raise RuntimeError(f"only {len(used_labels)} correspondences, need >= 4")

    base_pts = np.array(base_pts, dtype=np.float64)
    pixels = np.array(pixels, dtype=np.float64)
    T_c2b, rms_m = solve_extrinsics(base_pts, pixels, K, dist)

    out = {
        'timestamp': __import__('datetime').datetime.now()
            .isoformat(timespec='seconds'),
        'source': 'calibrate_closed_loop.py',
        'camera': 'UGreen',
        'intrinsics': intr,
        'dist': list(map(float, dist.flatten())),
        'n_points': int(len(used_labels)),
        'labels': used_labels,
        'rms_px_to_m_approx': float(rms_m),
        'rms_mm_approx': float(rms_m * 1000),
        'T_cam_to_base': T_c2b.tolist(),
    }
    with open(CALIB_OUT, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"\nRMS approx: {rms_m * 1000:.1f} mm  (target < 15)")
    print(f"wrote {CALIB_OUT}")
    return out


if __name__ == "__main__":
    default_labels = [f"cal_{i:02d}" for i in range(1, 9)]
    labels = sys.argv[1:] if len(sys.argv) > 1 else default_labels
    run_calibration(labels)
