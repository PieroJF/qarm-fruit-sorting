"""
Hand-eye calibration for the RealSense D415 mounted near the QArm workspace.

Workflow:
  1. You first use teach_points.py to record 6+ points labeled cal_01..cal_NN.
     Each point has joints + xyz in the robot base frame. Place the gripper
     tip at a visible location on the table (e.g., a small marker) and save.
  2. Run this script. For every cal_* point in teach_points.json:
       - Captures one RGB + depth frame from the D415.
       - Shows the RGB frame and asks you to click the same marker location.
       - Reads depth at that pixel, back-projects to camera frame.
  3. Solves the rigid transform T_cam_to_base (4x4) via the Umeyama method
     (closed-form SVD, no RANSAC needed since the points are hand-picked).
  4. Writes calibration.json with the 4x4 matrix, residual errors, and
     intrinsics used.

Notes:
  - Requires cv2 (for click capture) and the camera opens / streams via the
    Quanser SDK Python bindings.
  - At least 3 non-collinear points are required; 6+ is recommended for a
    robust fit.
  - The solver assumes rigid transform (R, t). No scale, no shear.
  - If depth at the clicked pixel is zero/invalid, the script falls back
    to the median depth in a 5x5 patch around the click; if that is still
    invalid the point is skipped.
"""

import json
import os
import sys
import time
from datetime import datetime

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python (cv2) is required for the click UI.")
    sys.exit(1)

from camera import QArmCamera


POINTS_FILE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "teach_points.json")
)
CALIB_FILE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "calibration.json")
)

DEPTH_PATCH = 5  # pixels: half-size of the fallback depth window


def load_cal_points():
    if not os.path.exists(POINTS_FILE):
        print(f"ERROR: {POINTS_FILE} not found. Run teach_points.py first.")
        sys.exit(1)
    with open(POINTS_FILE, "r") as f:
        all_pts = json.load(f)
    cal = {k: v for k, v in all_pts.items() if k.startswith("cal_")}
    if len(cal) < 3:
        print(f"ERROR: need at least 3 cal_* points, got {len(cal)}.")
        print("Use teach_points.py to record cal_01, cal_02, cal_03, ...")
        sys.exit(1)
    print(f"Loaded {len(cal)} calibration points: {sorted(cal.keys())}")
    return cal


class FrameClicker:
    def __init__(self, window_name="calibration"):
        self.window_name = window_name
        self.click_xy = None

    def _cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_xy = (x, y)

    def pick(self, img, title_overlay):
        self.click_xy = None
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self._cb)
        while True:
            disp = img.copy()
            cv2.putText(disp, title_overlay, (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(disp, "LMB: pick   s: skip   q: abort",
                        (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (200, 200, 200), 1)
            if self.click_xy is not None:
                cv2.circle(disp, self.click_xy, 6, (0, 255, 0), 2)
            cv2.imshow(self.window_name, disp)
            k = cv2.waitKey(20) & 0xFF
            if k == ord("q"):
                cv2.destroyWindow(self.window_name)
                return "abort"
            if k == ord("s"):
                cv2.destroyWindow(self.window_name)
                return "skip"
            if k in (13, 32) and self.click_xy is not None:
                cv2.destroyWindow(self.window_name)
                return self.click_xy


def depth_at(depth_img, u, v, patch=DEPTH_PATCH):
    """Return depth in metres at (u,v). Falls back to patch median if
    the exact pixel is zero."""
    h, w = depth_img.shape[:2]
    if 0 <= v < h and 0 <= u < w:
        d = int(depth_img[v, u])
        if d > 0:
            return d / 1000.0
    u0, u1 = max(0, u - patch), min(w, u + patch + 1)
    v0, v1 = max(0, v - patch), min(h, v + patch + 1)
    window = depth_img[v0:v1, u0:u1]
    valid = window[window > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0


def pixel_to_camera(u, v, depth_m, intr):
    Z = depth_m
    X = (u - intr["cx"]) * Z / intr["fx"]
    Y = (v - intr["cy"]) * Z / intr["fy"]
    return np.array([X, Y, Z], dtype=float)


def umeyama(src, dst):
    """Closed-form rigid transform R, t such that dst ≈ R @ src + t.
    Umeyama 1991 without scale. src, dst are N×3."""
    assert src.shape == dst.shape and src.shape[1] == 3
    n = src.shape[0]
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    S = src - mu_src
    D = dst - mu_dst
    H = S.T @ D / n
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    W = np.eye(3)
    W[2, 2] = d  # reflection-safe
    R = Vt.T @ W @ U.T
    t = mu_dst - R @ mu_src
    return R, t


def residuals(src, dst, R, t):
    pred = (R @ src.T).T + t
    err = pred - dst
    per_pt = np.linalg.norm(err, axis=1)
    return per_pt


def main():
    cal = load_cal_points()

    cam = QArmCamera()
    cam.open()
    intr = cam.intrinsics
    print(f"Camera intrinsics: fx={intr['fx']:.1f} fy={intr['fy']:.1f} "
          f"cx={intr['cx']:.1f} cy={intr['cy']:.1f}")
    try:
        # Warmup
        for _ in range(10):
            try:
                color, depth = cam.read()
                if color.mean() > 5:
                    break
            except Exception:
                pass
            time.sleep(0.05)

        clicker = FrameClicker()
        src_pts = []  # camera frame (X,Y,Z) meters
        dst_pts = []  # robot base frame (x,y,z) meters
        used_labels = []

        for label in sorted(cal.keys()):
            pt = cal[label]
            world_xyz = np.array(pt["xyz_m"], dtype=float)
            # Capture a fresh frame right before each click
            try:
                color, depth = cam.read()
            except Exception as e:
                print(f"  [{label}] camera read failed: {e}  (skipped)")
                continue

            overlay = f"{label}  base={world_xyz.round(3)}"
            print(f"\n>> {label}: click the marker in the RGB window  "
                  f"(base xyz = {world_xyz.round(3)})")
            result = clicker.pick(color, overlay)
            if result == "abort":
                print("  user aborted"); break
            if result == "skip":
                print(f"  skipped {label}")
                continue

            u, v = result
            d_m = depth_at(depth, u, v)
            if d_m is None or d_m <= 0:
                print(f"  [{label}] no valid depth at ({u},{v})  (skipped)")
                continue

            p_cam = pixel_to_camera(u, v, d_m, intr)
            print(f"  pixel=({u},{v})  depth={d_m:.3f}m  "
                  f"p_cam={p_cam.round(3)}")
            src_pts.append(p_cam)
            dst_pts.append(world_xyz)
            used_labels.append(label)

        cv2.destroyAllWindows()
    finally:
        cam.close()

    if len(src_pts) < 3:
        print(f"\nERROR: need at least 3 valid pairs, got {len(src_pts)}.")
        sys.exit(1)

    src = np.array(src_pts)
    dst = np.array(dst_pts)
    R, t = umeyama(src, dst)
    res = residuals(src, dst, R, t)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    print("\n=== Calibration result ===")
    for lbl, r_mm in zip(used_labels, res * 1000):
        print(f"  {lbl:8s}  residual = {r_mm:6.1f} mm")
    print(f"  mean   residual : {res.mean() * 1000:.1f} mm")
    print(f"  max    residual : {res.max() * 1000:.1f} mm")
    print(f"  RMS    residual : "
          f"{np.sqrt((res ** 2).mean()) * 1000:.1f} mm")
    print("  T_cam_to_base (4x4):")
    for row in T:
        print("    [" + ", ".join(f"{v:+.4f}" for v in row) + "]")

    out = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "intrinsics": intr,
        "n_points": int(len(src_pts)),
        "labels": used_labels,
        "residuals_mm": [float(r * 1000) for r in res],
        "mean_residual_mm": float(res.mean() * 1000),
        "max_residual_mm": float(res.max() * 1000),
        "rms_residual_mm": float(np.sqrt((res ** 2).mean()) * 1000),
        "T_cam_to_base": T.tolist(),
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[ok] wrote {CALIB_FILE}")

    if out["max_residual_mm"] > 15:
        print("\n[warn] max residual > 15 mm — consider redoing calibration "
              "with more points or better marker visibility.")


if __name__ == "__main__":
    main()
