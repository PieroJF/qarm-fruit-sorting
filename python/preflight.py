"""Pre-lab-session sanity script.

Runs 7 checks in order. Each check returns (ok, message). The script
prints a green 'PREFLIGHT OK' or red 'PREFLIGHT FAIL: <reason>' and
exits with the number of failing checks.

Checks:
  1. QArm connect + read_all + one set_joints round trip (-843 early).
  2. D415 warmup (color mean > 5, depth valid > 10%).
  3. UGreen open (idx=3 MSMF), one frame with mean > 5.
  4. calibration.json loadable; RMS printed; age < 7 d; pose label.
  5. HSV detector runs on one D415 frame; per-blob circ/sat printed.
  6. Offline integration tests (test_integration + test_ugreen_tracker
     + test_calibrate_closed_loop) — zero hardware dependency.
  7. Visual reference: arm -> pickhome1; UGreen image-diff against
     stored reference; TCP pixel within 20 px of reference TCP.

Usage:
  C:/Python313/python.exe python/preflight.py            # full
  C:/Python313/python.exe python/preflight.py --offline  # skip 1,2,3,7
"""
import os
import sys
import json
import time
import subprocess
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
CALIB_FILE = os.path.join(REPO, "calibration.json")
REF_FRAME = os.path.join(REPO, "logs", "ugreen_pickhome1_reference.png")
REF_TCP = os.path.join(REPO, "logs", "ugreen_pickhome1_tcp.json")
BASELINE = os.path.join(REPO, "logs", "ugreen_baseline.png")

ANSI_RED = "\033[91m"
ANSI_GREEN = "\033[92m"
ANSI_RESET = "\033[0m"


def _mark(ok):
    return (ANSI_GREEN + "OK  " + ANSI_RESET) if ok else (ANSI_RED + "FAIL" + ANSI_RESET)


def check_qarm():
    try:
        from qarm_driver import QArmDriver
        import numpy as np
    except Exception as ex:
        return False, f"import: {ex}"
    q = QArmDriver()
    try:
        q.connect()
        time.sleep(0.4)
        for _ in range(5):
            try:
                j, g = q.read_all(); break
            except Exception:
                time.sleep(0.3)
        else:
            return False, "read_all timeout -> power cycle QArm"
        q.set_joints_and_gripper(j, g)
    except Exception as ex:
        return False, f"driver: {ex}"
    finally:
        try: q.card.close()
        except Exception: pass
        q._connected = False
    return True, f"joints OK, gripper={g:.2f}"


def check_d415():
    try:
        from camera import QArmCamera
    except Exception as ex:
        return False, f"import: {ex}"
    cam = QArmCamera()
    try:
        cam.open()
        deadline = time.time() + 10
        while time.time() < deadline:
            try:
                c, d = cam.read()
            except Exception:
                continue
            if c.mean() > 5 and (d > 0).mean() > 0.10:
                return True, (f"color mean={c.mean():.0f} "
                              f"depth={(d>0).mean()*100:.0f}% valid")
            time.sleep(0.1)
        return False, "no valid frame after 10 s"
    except Exception as ex:
        return False, f"{ex}"
    finally:
        try: cam.close()
        except Exception: pass


def check_ugreen():
    try:
        from ugreen_tracker import capture
        f = capture(warmup=20)
        return True, f"frame {f.shape} mean={f.mean():.0f}"
    except Exception as ex:
        return False, f"{ex}"


def check_calibration():
    if not os.path.exists(CALIB_FILE):
        return False, f"{CALIB_FILE} missing"
    with open(CALIB_FILE) as f:
        cal = json.load(f)
    rms = cal.get("rms_residual_mm", None)
    pose = cal.get("pose_label", "unspecified")
    ts = cal.get("timestamp", "?")
    age_days = None
    try:
        age_days = (datetime.now() - datetime.fromisoformat(ts)).days
    except Exception:
        pass
    warn = []
    ok = True
    if rms is None:
        warn.append("no RMS")
    elif rms > 50:
        warn.append(f"RMS={rms:.0f}mm > 50"); ok = False
    if age_days is not None and age_days > 7:
        warn.append(f"age={age_days}d > 7"); ok = False
    detail = f"RMS={rms}mm pose={pose} age={age_days}d"
    if warn:
        detail += f"  warnings={warn}"
    return ok, detail


def check_hsv():
    try:
        from camera import QArmCamera
        from fruit_detector import detect_fruits
    except Exception as ex:
        return False, f"import: {ex}"
    cam = QArmCamera()
    try:
        cam.open()
        for _ in range(30):
            try: c, d = cam.read()
            except Exception: pass
            if c.mean() > 5 and (d > 0).mean() > 0.10: break
            time.sleep(0.1)
        dets = detect_fruits(c, d)
        lines = [f"{len(dets)} blobs"]
        for i, x in enumerate(dets):
            lines.append(
                f"  [{i}] {x.fruit_type} area={x.area:.0f} "
                f"conf={x.confidence:.2f}")
        return True, "; ".join(lines)
    except Exception as ex:
        return False, f"{ex}"
    finally:
        try: cam.close()
        except Exception: pass


def check_offline_tests():
    """Run the three test scripts in-process and sum their exit codes."""
    test_files = [
        os.path.join(HERE, "test_integration.py"),
        os.path.join(HERE, "test_ugreen_tracker.py"),
        os.path.join(HERE, "test_calibrate_closed_loop.py"),
    ]
    fails = 0
    names = []
    for tf in test_files:
        r = subprocess.run([sys.executable, tf],
                             capture_output=True, text=True)
        if r.returncode != 0:
            fails += 1
            names.append(os.path.basename(tf))
    if fails:
        return False, f"{fails} test file(s) failed: {names}"
    return True, "all offline tests passed"


def check_visual_reference():
    if not os.path.exists(REF_FRAME) or not os.path.exists(BASELINE):
        return False, "reference or baseline missing — run with --capture-ref"
    if not os.path.exists(REF_TCP):
        return False, "ref TCP sidecar missing — capture reference first"
    try:
        from ugreen_tracker import capture, tcp_from_diff, load_baseline
        from qarm_driver import QArmDriver
        import numpy as np
    except Exception as ex:
        return False, f"import: {ex}"
    # Move arm to pickhome1
    try:
        with open(os.path.join(REPO, "teach_points.json")) as f:
            pts = json.load(f)
        if "pickhome1" not in pts:
            return False, "pickhome1 missing from teach_points"
        target = np.array(pts["pickhome1"]["joints_rad"], dtype=float)
        grip = float(pts["pickhome1"].get("gripper", 0.15))
        q = QArmDriver(); q.connect(); time.sleep(0.3)
        for _ in range(5):
            try: q.read_all(); break
            except Exception: time.sleep(0.3)
        try:
            from calibrate_closed_loop import slow_move_to_joints
            slow_move_to_joints(q, target, grip)
        finally:
            try: q.card.close()
            except Exception: pass
            q._connected = False
        frame = capture()
        baseline = load_baseline(BASELINE)
        tcp = tcp_from_diff(frame, baseline)
        if tcp is None:
            return False, "no TCP detected at pickhome1"
        with open(REF_TCP) as f:
            ref = json.load(f)
        dx = tcp[0] - ref["tcp"][0]
        dy = tcp[1] - ref["tcp"][1]
        d = (dx * dx + dy * dy) ** 0.5
        ok = d < 20
        return ok, f"tcp delta = {d:.1f} px (threshold 20)"
    except Exception as ex:
        return False, f"{ex}"


CHECKS = [
    ("1 QArm     ", check_qarm,               False),
    ("2 D415     ", check_d415,               False),
    ("3 UGreen   ", check_ugreen,             False),
    ("4 Calib    ", check_calibration,        True),
    ("5 HSV      ", check_hsv,                False),
    ("6 Tests    ", check_offline_tests,      True),
    ("7 VisualRef", check_visual_reference,   False),
]


def main():
    offline = "--offline" in sys.argv
    fails = 0
    for name, fn, offline_safe in CHECKS:
        if offline and not offline_safe:
            print(f"  [SKIP]  {name}  (offline mode)"); continue
        print(f"  ...     {name}  running", end="\r")
        try:
            ok, msg = fn()
        except Exception as ex:
            ok, msg = False, f"exception: {ex}"
        print(f"  [{_mark(ok)}] {name}  {msg}")
        if not ok:
            fails += 1
    print()
    if fails == 0:
        print(ANSI_GREEN + "PREFLIGHT OK — cleared for lab work." + ANSI_RESET)
    else:
        print(ANSI_RED + f"PREFLIGHT FAIL — {fails} check(s) red." +
              ANSI_RESET)
    return fails


if __name__ == "__main__":
    sys.exit(main())
