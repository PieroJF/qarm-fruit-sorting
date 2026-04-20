#!/usr/bin/env python3
"""
Master script: Full autonomous fruit sorting pipeline.

Loads all configuration from teach_points.json (basket positions, home) and
calibration.json (T_cam_to_base), connects QArm + D415 camera, detects fruits,
and runs the complete pick-and-place FSM.

Usage:
    C:\\Python313\\python.exe main_final.py [--no-camera] [--dry-run]
                                            [--pick-only] [--calib FILE]
                                            [--detect-pose LABEL]

Flags:
    --no-camera        Skip live camera detection, use manual fruit list instead
    --dry-run          Print the plan but don't move the arm
    --pick-only        Just detect, pick up and lift each fruit (no basket sorting)
    --calib FILE       Path to calibration JSON (default: ../calibration.json)
    --detect-pose LBL  Teach-point label to move to before detecting fruits
                       (default: pickhome1; must match the pose where the
                       chosen calibration was derived — camera is arm-mounted
                       so T_cam_to_base is pose-dependent).
"""

import json
import os
import sys
import time

import numpy as np

from qarm_driver import QArmDriver
from qarm_kinematics import forward_kinematics, inverse_kinematics
from sorting_controller import FruitSortingController
from camera import QArmCamera
from fruit_detector import detect_fruits, draw_detections, detection_depth_mm

try:
    import cv2
except ImportError:
    cv2 = None

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TEACH_FILE = os.path.join(BASE_DIR, "teach_points.json")
CALIB_FILE = os.path.join(BASE_DIR, "calibration.json")
DETECT_SAVE = os.path.join(BASE_DIR, "figures", "detection_result.png")
DEFAULT_DETECT_POSE = "pickhome1"


def get_arg(flag, default=None):
    """Pull `--flag VALUE` from sys.argv, returning VALUE or default."""
    if flag in sys.argv:
        i = sys.argv.index(flag)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default


def slow_move_to_joints(qarm, target_joints, target_grip, seconds=3.0,
                        steps=200):
    """Smoothstep interpolation from current pose to target_joints. Holds
    briefly at end so position-mode PID settles. Avoids the snap-to-target
    that qarm.home() used in its default path."""
    cj, cg = qarm.read_all()
    cj = np.array(cj, dtype=float)
    cg = float(cg)
    for i in range(1, steps + 1):
        a = i / steps
        s = 3 * a * a - 2 * a * a * a
        qarm.set_joints_and_gripper(
            cj + s * (target_joints - cj),
            cg + s * (target_grip - cg),
        )
        time.sleep(seconds / steps)
    for _ in range(20):
        qarm.set_joints_and_gripper(target_joints, target_grip)
        time.sleep(0.05)


def load_teach_points():
    if not os.path.exists(TEACH_FILE):
        print(f"[warn] {TEACH_FILE} not found — using hardcoded defaults")
        return None
    with open(TEACH_FILE) as f:
        pts = json.load(f)
    print(f"Loaded {len(pts)} teach points from {TEACH_FILE}")
    return pts


def load_calibration(path=CALIB_FILE):
    if not os.path.exists(path):
        print(f"[warn] {path} not found — T_cam_to_base = identity")
        return np.eye(4)
    with open(path) as f:
        cal = json.load(f)
    T = np.array(cal["T_cam_to_base"])
    rms = cal.get("rms_residual_mm", "?")
    pose = cal.get("pose_label", "unspecified")
    print(f"Loaded calibration from {os.path.basename(path)}: "
          f"{cal['n_points']} points, RMS={rms} mm, pose={pose}")
    return T


def extract_baskets(pts):
    baskets = {}
    mapping = {
        "basket_a": "strawberry",
        "basket_b": "banana",
        "basket_c": "tomato",
    }
    for label, fruit_type in mapping.items():
        if label in pts:
            baskets[fruit_type] = np.array(pts[label]["xyz_m"])
            print(f"  basket {fruit_type:12s} = {baskets[fruit_type].round(3)}")
    return baskets


def extract_home(pts):
    if "home" in pts:
        h = np.array(pts["home"]["xyz_m"])
        print(f"  home = {h.round(3)}")
        return h
    return None


def detect_fruits_live(cam, T_cam_to_base):
    print("\nCapturing frame for fruit detection...")
    # Warmup — wait for a non-black color frame AND reasonable depth coverage.
    # The D415 takes ~2-3 s to start producing usable depth; the previous
    # bound of "color.mean() > 5" was often satisfied before depth was ready
    # and the first detect_fruits call saw 0% valid depth.
    deadline = time.time() + 10.0
    color = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.zeros((720, 1280), dtype=np.uint16)
    while time.time() < deadline:
        try:
            color, depth = cam.read()
        except Exception:
            pass
        if color.mean() > 5 and (depth > 0).mean() > 0.10:
            break
        time.sleep(0.1)
    # A few extra frames so auto-exposure has fully settled
    for _ in range(10):
        try:
            color, depth = cam.read()
        except Exception:
            pass
        time.sleep(0.05)
    print(f"  frame: {color.shape}, depth valid: {(depth > 0).mean():.0%}")

    dets = detect_fruits(color, depth)
    print(f"  detected {len(dets)} fruits:")

    if cv2 is not None and dets:
        annotated = draw_detections(color, dets)
        os.makedirs(os.path.dirname(DETECT_SAVE), exist_ok=True)
        cv2.imwrite(DETECT_SAVE, annotated)
        print(f"  saved annotated image to {DETECT_SAVE}")

    positions = []
    types = []
    for det in dets:
        row, col = det.centroid
        d_mm = detection_depth_mm(det, depth)
        if d_mm is None:
            print(f"    skip {det.fruit_type} at ({row},{col}): "
                  f"no plausible depth in blob")
            continue

        p_world = cam.pixel_to_world(row, col, d_mm, T_cam_to_base)
        positions.append(p_world)
        types.append(det.fruit_type)
        print(f"    {det.fruit_type:12s}  pixel=({row:3d},{col:3d})  "
              f"depth={d_mm:.0f}mm  world={p_world.round(3)}")

    return positions, types


def main():
    flags = set(a for a in sys.argv[1:] if not a.startswith("--")
                or a in ("--no-camera", "--dry-run", "--pick-only"))
    use_camera = "--no-camera" not in flags
    dry_run = "--dry-run" in flags
    pick_only = "--pick-only" in flags
    calib_path = get_arg("--calib", CALIB_FILE)
    detect_pose = get_arg("--detect-pose", DEFAULT_DETECT_POSE)

    mode_label = "PICK & LIFT" if pick_only else "FULL SORT"
    print("=" * 60)
    print(f"  QArm Fruit Sorting — {mode_label}")
    print("=" * 60)
    print(f"  calibration   : {calib_path}")
    print(f"  detect pose   : {detect_pose}")

    # --- Load config ---
    pts = load_teach_points()
    T_cam_to_base = load_calibration(calib_path)

    baskets = {}
    home_pos = None
    safe_z = None

    if pts:
        baskets = extract_baskets(pts)
        home_pos = extract_home(pts)
        if "safe_z" in pts:
            safe_z = pts["safe_z"]["xyz_m"][2]
            print(f"  safe_z = {safe_z:.3f} m")

    # Defaults for anything not in teach_points
    if not baskets.get("strawberry"):
        baskets["strawberry"] = np.array([0.30, -0.20, 0.05])
    if not baskets.get("banana"):
        baskets["banana"] = np.array([-0.30, -0.20, 0.05])
    if not baskets.get("tomato"):
        baskets["tomato"] = np.array([0.00, -0.35, 0.05])

    # --- Connect hardware ---
    print("\nConnecting to QArm...")
    qarm = QArmDriver()
    try:
        qarm.connect()
    except RuntimeError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    cam = None
    if use_camera:
        print("Opening D415 camera...")
        try:
            cam = QArmCamera()
            cam.open()
        except Exception as e:
            print(f"[warn] camera failed: {e} — falling back to manual mode")
            cam = None
            use_camera = False

    try:
        # --- Move to detection pose (NOT joints=0) ---
        # Camera is arm-mounted — T_cam_to_base is only valid at the pose
        # where the calibration was derived. Detecting from joints=0 while
        # using a calibration captured at pickhome1 produces wildly wrong
        # world coordinates.
        if pts and detect_pose in pts:
            tp = pts[detect_pose]
            target = np.array(tp["joints_rad"], dtype=float)
            tg = float(tp.get("gripper", 0.15))
            print(f"\nMoving to detection pose '{detect_pose}' (3 s)...")
            slow_move_to_joints(qarm, target, tg, seconds=3.0)
        else:
            print(f"\n[warn] detection pose '{detect_pose}' not in "
                  f"teach_points — homing to joints=0 instead "
                  f"(calibration may be invalid here)")
            qarm.home(duration=5.0, steps=250)
        time.sleep(0.5)

        # --- Detect fruits ---
        if use_camera and cam is not None:
            positions, types = detect_fruits_live(cam, T_cam_to_base)
            cam.close()
            cam = None
        else:
            print("\nUsing manual fruit list (no camera)...")
            positions = [
                np.array([0.20, 0.30, 0.02]),
                np.array([0.15, 0.25, 0.02]),
                np.array([-0.10, 0.35, 0.02]),
            ]
            types = ["strawberry", "tomato", "banana"]

        if not positions:
            print("\nNo fruits detected! Check camera/lighting/HSV thresholds.")
            return

        # --- Summary ---
        print(f"\n{'=' * 60}")
        print(f"  SORT PLAN: {len(positions)} fruits")
        print(f"{'=' * 60}")
        counts = {}
        for i, (p, t) in enumerate(zip(positions, types)):
            basket_pos = baskets.get(t, baskets["tomato"])
            counts[t] = counts.get(t, 0) + 1
            print(f"  {i+1:2d}. {t:12s} at {p.round(3)}  -> basket at {basket_pos.round(3)}")
        print(f"\n  Totals: {counts}")
        print(f"  Baskets: " + ", ".join(
            f"{k}={v.round(2)}" for k, v in baskets.items()))

        if dry_run:
            print("\n[dry-run] Skipping execution.")
            return

        input("\nPress ENTER to start sorting (arm will move)...")

        # --- Configure controller ---
        controller = FruitSortingController(qarm, pick_only=pick_only)
        controller.BASKETS = baskets
        if home_pos is not None:
            controller.HOME_POS = home_pos
        if safe_z is not None:
            controller.SAFE_Z = safe_z

        controller.set_fruit_positions(positions, types)

        # --- Run ---
        # Drop the old 500 Hz (dt=0.002) passthrough — controller default
        # (100 Hz) is sufficient for QArm motion bandwidth and frees CPU.
        t0 = time.time()
        controller.run_autonomous()
        elapsed = time.time() - t0

        # --- Stats ---
        print(f"\n{'=' * 60}")
        print(f"  RESULTS")
        print(f"{'=' * 60}")
        print(f"  Fruits sorted: {controller.sorted_count}/{len(positions)}")
        print(f"  Total time:    {elapsed:.1f} s")
        print(f"  Avg per fruit: {elapsed / max(1, controller.sorted_count):.1f} s")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
    finally:
        print("\nReturning home and disconnecting...")
        try:
            qarm.home(duration=3.0)
        except Exception:
            pass
        try:
            qarm.disconnect()
        except Exception:
            pass
        if cam is not None:
            try:
                cam.close()
            except Exception:
                pass
        print("Done.")


if __name__ == "__main__":
    main()
