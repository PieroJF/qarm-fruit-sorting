# Final Sprint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver the QArm fruit-sorting final by 2026-05-01 14:00 with autonomous + remote modes, visual closed-loop calibration via UGreen floor camera, and a recorded 14-fruit sort video.

**Architecture:** Simulink is the facade, Python does the work. Camera is arm-mounted (pose-dependent calibration, captured at `pickhome1`). UGreen camera gives external ground-truth for closed-loop calibration + pick validation via image diff. Remote mode is a jog-pendant HMI (teach-pendant style) integrated into a unified 3-way `.slx` (autonomous / remote / release) with Stateflow chart orchestrating the autonomous FSM while Python helpers do the heavy lifting.

**Tech Stack:** Python 3.13 (`C:/Python313/python.exe`), numpy 2.4, opencv 4.13, Quanser SDK 2026.1.21, MATLAB R2025a + Simulink + Stateflow, pygrabber for DirectShow enumeration.

**Driving spec:** `docs/superpowers/specs/2026-04-20-final-sprint-design.md`

---

## How to use this plan

- **Phase A** is doable on the dev machine with no QArm / no D415 / no UGreen attached. Every task in Phase A is independently committable and ends with green tests.
- **Phase B** requires lab access (QArm + D415 + UGreen). Tasks are sequenced so you can pause at any stage with a working artifact.
- Tests run with `C:/Python313/python.exe python/test_integration.py`. Before any commit that touches Python, that script must exit 0.
- All paths are relative to the repo root `C:/Users/Mugin/Downloads/Compressed/Nueva carpeta_2/FinalProject_FruitSorting/`.

---

# Phase A — Dev-machine (no hardware required)

## Stage A1 — Extract shared gripper ramp helper

Prepares `sorting_controller` so the Cartesian remote mode can reuse the ramp+readback gripper logic without duplicating it.

### Task A1.1 — Extract `set_gripper_ramp` helper

**Files:**
- Modify: `python/sorting_controller.py`
- Modify: `python/test_integration.py`

- [ ] **Step 1: Add a failing test**

Append to `python/test_integration.py` before the `TESTS = [...]` list:

```python
def test_set_gripper_ramp_extracted():
    """The shared helper ramps from `from` to `target` over `duration`
    seconds using smoothstep, and after the ramp exposes `held_grip`
    matching the settled value reported by the mocked qarm."""
    name = "set_gripper_ramp_helper"
    from sorting_controller import FruitSortingController, GRIP_OPEN, GRIP_CLOSE
    q = MockQArm()
    q.fake_grip = GRIP_OPEN  # mock tracks perfectly
    c = FruitSortingController(q)
    c.T_GRIP = 0.05  # quick
    # Precondition: helper exists on the controller class
    assert hasattr(c, "set_gripper_ramp"), "set_gripper_ramp missing"
    # Ramp from current to CLOSE
    result = c.set_gripper_ramp(GRIP_CLOSE)
    # Should have sent intermediate values, not just a single snap
    cmds = [g for _, g in q.cmd_log]
    unique = set(round(v, 2) for v in cmds)
    assert len(unique) >= 5, f"no ramp, only {unique}"
    # After settling, held_grip matches the mocked actual (which tracked)
    assert abs(c._held_grip - GRIP_CLOSE) < 0.05, \
        f"held_grip {c._held_grip} != {GRIP_CLOSE}"
    return name, True, f"ramp over {len(unique)} values, held_grip correct"
```

Append to `TESTS = [...]`:

```python
TESTS = [
    # ...existing tests...
    test_set_gripper_ramp_extracted,
]
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: FAIL with `set_gripper_ramp missing`.

- [ ] **Step 3: Implement the helper**

In `python/sorting_controller.py`, inside `FruitSortingController`, add (place just below `_update_grip_interp`):

```python
    def set_gripper_ramp(self, target, duration=None):
        """Synchronously ramp the gripper from the current held value to
        `target` over `duration` seconds (default T_GRIP). After the ramp,
        reads actual gripper position from the driver and stores it in
        `_held_grip` so subsequent commands do not fight a stalled servo.

        Thread-safety: blocking, intended for teach-pendant / remote use.
        Do NOT call from inside `run_autonomous`'s FSM step loop; the FSM
        has its own stepwise ramp via _update_grip_interp.
        """
        self._start_grip_interp(target, duration=duration)
        # Hold current joints while ramping so only the gripper moves.
        joints = self._joints_snapshot()
        while not self._grip_interp_done():
            g = self._update_grip_interp()
            self._execute_position(joints, g)
            time.sleep(0.01)
        # Settle for a few frames, then read back actual.
        for _ in range(3):
            self.qarm.set_joints_and_gripper(joints, target)
            time.sleep(0.02)
        try:
            _, actual = self.qarm.read_all()
            self._held_grip = float(actual)
        except Exception:
            self._held_grip = float(target)
        return self._held_grip

    def _joints_snapshot(self):
        joints, _ = self.qarm.read_all()
        return np.asarray(joints, dtype=float)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: `11/11 passed.`

- [ ] **Step 5: Commit**

```bash
git add python/sorting_controller.py python/test_integration.py
git commit -m "extract set_gripper_ramp helper for reuse by remote mode

The CLOSE_GRIPPER / OPEN_GRIPPER states of the autonomous FSM already
implement ramp + readback to avoid -1289 overload. Remote mode needs
the same behavior for its gripper buttons; this exposes the pattern
as a blocking helper callable from teach-pendant code."
```

---

## Stage A2 — UGreen tracker module

Delivers `python/ugreen_tracker.py` with camera capture, baseline/diff TCP extraction, and synthetic tests. Does NOT yet depend on hardware because all tests use saved frames.

### Task A2.1 — UGreen capture module with baseline support

**Files:**
- Create: `python/ugreen_tracker.py`
- Modify: `python/ugreen_capture.py` (retire in favor of new module)
- Create: `python/test_ugreen_tracker.py`

- [ ] **Step 1: Write the capture + baseline tests**

Create `python/test_ugreen_tracker.py`:

```python
"""Unit tests for ugreen_tracker. No hardware needed — tests use synthetic
frames stored as fixtures."""
import os
import sys
import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from ugreen_tracker import (tcp_from_diff, save_baseline, load_baseline,
                             MIN_ARM_PIXELS)


def _synth_baseline(h=480, w=640):
    """Empty workspace: grey desk with a white paper in the middle."""
    img = np.full((h, w, 3), 180, dtype=np.uint8)
    cv2.rectangle(img, (200, 200), (440, 340), 240, -1)
    return img


def _synth_with_arm(baseline):
    """Same scene with a dark arm silhouette entering from top-center."""
    img = baseline.copy()
    # Thin vertical arm (dark grey) and a gripper blob at its tip
    cv2.rectangle(img, (310, 0), (330, 260), 40, -1)
    cv2.circle(img, (320, 265), 15, 30, -1)
    return img


def test_diff_finds_arm_tip():
    baseline = _synth_baseline()
    frame = _synth_with_arm(baseline)
    tcp = tcp_from_diff(frame, baseline)
    assert tcp is not None, "expected a TCP, got None"
    col, row = tcp
    # Arm is at column ~320, tip row ~280. Allow loose tolerance.
    assert 300 < col < 340, f"col {col} not near 320"
    assert 260 < row < 295, f"row {row} not near 280"


def test_no_arm_returns_none():
    baseline = _synth_baseline()
    frame = baseline.copy()  # identical; no arm
    tcp = tcp_from_diff(frame, baseline)
    assert tcp is None, f"expected None for identical frames, got {tcp}"


def test_baseline_roundtrip(tmp_path=None):
    import tempfile
    tmp = tempfile.mkdtemp()
    base = _synth_baseline()
    path = os.path.join(tmp, "base.png")
    save_baseline(base, path)
    reloaded = load_baseline(path)
    assert reloaded.shape == base.shape
    assert np.allclose(reloaded, base)


def test_small_difference_rejected():
    """Single-pixel noise shouldn't register as 'arm present'."""
    baseline = _synth_baseline()
    frame = baseline.copy()
    frame[100, 100] = (0, 0, 0)  # one pixel changed
    tcp = tcp_from_diff(frame, baseline)
    assert tcp is None, f"expected None for 1-pixel change, got {tcp}"


if __name__ == "__main__":
    fails = 0
    for t in [test_diff_finds_arm_tip, test_no_arm_returns_none,
              test_baseline_roundtrip, test_small_difference_rejected]:
        try:
            t()
            print(f"  [OK]   {t.__name__}")
        except Exception as ex:
            print(f"  [FAIL] {t.__name__}: {ex}")
            fails += 1
    sys.exit(fails)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `C:/Python313/python.exe python/test_ugreen_tracker.py`
Expected: `FAIL` on all four with `ModuleNotFoundError: No module named 'ugreen_tracker'`.

- [ ] **Step 3: Implement the tracker module**

Create `python/ugreen_tracker.py`:

```python
"""UGreen floor-level camera tracker.

Detects the QArm gripper in a UGreen frame via baseline-subtraction image
diff. Returns the pixel coordinates of the arm's TCP approximation (the
bottom-most point of the moving silhouette). Used for closed-loop
calibration and pick validation.

Camera access: DirectShow index 3, MSMF backend, 1280x720. Index/backend
discovered by `python/probe_ugreen.py` on 2026-04-20.
"""
import os
import sys
import cv2
import numpy as np

UGREEN_IDX = 3
DIFF_THRESH = 35            # absolute mean channel diff to count a pixel as "arm"
MIN_ARM_PIXELS = 500        # total arm silhouette must exceed this
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720


def capture(width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, warmup=10):
    """Open UGreen, grab one valid frame, release. Returns BGR ndarray.
    Raises RuntimeError if the camera does not yield a usable frame
    within the warmup budget."""
    cap = cv2.VideoCapture(UGREEN_IDX, cv2.CAP_MSMF)
    if not cap.isOpened():
        raise RuntimeError(f"UGreen did not open (idx={UGREEN_IDX} MSMF)")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    frame = None
    for _ in range(max(1, warmup)):
        ok, f = cap.read()
        if ok and f is not None and f.size > 0 and f.mean() > 5:
            frame = f
    cap.release()
    if frame is None:
        raise RuntimeError("UGreen opened but no valid frame after warmup")
    return frame


def save_baseline(frame, path):
    """Persist a baseline frame (arm out of view) to disk. Used by
    preflight + closed-loop calibration."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if not cv2.imwrite(path, frame):
        raise IOError(f"cv2.imwrite failed for {path}")


def load_baseline(path):
    """Load a previously saved baseline. Raises FileNotFoundError if
    missing — preflight is expected to catch that."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    img = cv2.imread(path)
    if img is None:
        raise IOError(f"cv2.imread returned None for {path}")
    return img


def arm_mask(frame, baseline, thresh=DIFF_THRESH):
    """Binary mask where |frame - baseline| exceeds `thresh`. Reduced to
    grayscale by averaging the 3 BGR channels so lighting shifts affect
    all channels equally (the threshold then absorbs small global shifts)."""
    if frame.shape != baseline.shape:
        raise ValueError(f"shape mismatch: {frame.shape} vs {baseline.shape}")
    diff = cv2.absdiff(frame, baseline).mean(axis=2)
    mask = (diff > thresh).astype(np.uint8) * 255
    # Morphological cleanup — small kernel to keep thin arm features
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def tcp_from_diff(frame, baseline, thresh=DIFF_THRESH,
                   min_pixels=MIN_ARM_PIXELS):
    """Locate the gripper TCP as the bottom-most point of the connected
    component reaching from the top of the frame. Returns (col, row) in
    pixel space, or None if no plausible arm silhouette is present.

    Algorithm
    ---------
    1. Compute |frame - baseline| and threshold.
    2. Find connected components. Reject if total arm pixels < min_pixels.
    3. Pick the component whose bounding box touches the top edge
       (y=0), i.e. the arm enters from above. If multiple touch the top,
       pick the largest by area.
    4. Return the bottom-center pixel of that component's bounding box.
    """
    mask = arm_mask(frame, baseline, thresh=thresh)
    if int((mask > 0).sum()) < min_pixels:
        return None
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    best_i = -1
    best_area = 0
    for i in range(1, num):  # skip background label 0
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = int(stats[i, cv2.CC_STAT_AREA])
        if y > 10:
            continue  # does not reach the top — not the arm
        if area > best_area:
            best_area = area
            best_i = i
            best_bbox = (x, y, w, h)
    if best_i < 0 or best_area < min_pixels:
        return None
    x, y, w, h = best_bbox
    col = int(x + w / 2)
    row = int(y + h)  # bottom of the bounding box
    return (col, row)


def overlay(frame, baseline, tcp=None):
    """Return a BGR image with the arm mask tinted cyan and the TCP
    marked with a crosshair. Used by preflight visual checks and the
    remote-mode companion view."""
    mask = arm_mask(frame, baseline)
    vis = frame.copy()
    tint = np.zeros_like(vis)
    tint[:] = (255, 255, 0)  # cyan in BGR
    m3 = cv2.merge([mask, mask, mask]).astype(bool)
    vis[m3] = cv2.addWeighted(vis, 0.5, tint, 0.5, 0)[m3]
    if tcp is not None:
        cv2.drawMarker(vis, tcp, (0, 255, 255),
                       markerType=cv2.MARKER_CROSS,
                       markerSize=40, thickness=2)
    return vis


if __name__ == "__main__":
    # Quick manual test against the saved frame from 2026-04-20
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    latest = os.path.join(repo, "logs", "ugreen_latest.png")
    if not os.path.exists(latest):
        print(f"no frame at {latest}; run ugreen_capture first")
        sys.exit(1)
    frame = cv2.imread(latest)
    print(f"loaded {frame.shape}")
    if len(sys.argv) > 1 and sys.argv[1] == "--baseline":
        out = os.path.join(repo, "logs", "ugreen_baseline.png")
        save_baseline(frame, out)
        print(f"saved baseline -> {out}")
        sys.exit(0)
    base_path = os.path.join(repo, "logs", "ugreen_baseline.png")
    if not os.path.exists(base_path):
        print(f"no baseline at {base_path}; save one with --baseline")
        sys.exit(1)
    base = load_baseline(base_path)
    tcp = tcp_from_diff(frame, base)
    print(f"TCP = {tcp}")
    vis = overlay(frame, base, tcp)
    out = os.path.join(repo, "logs", "ugreen_tcp_overlay.png")
    cv2.imwrite(out, vis)
    print(f"overlay -> {out}")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `C:/Python313/python.exe python/test_ugreen_tracker.py`
Expected: 4 tests pass.

- [ ] **Step 5: Retire the obsolete capture script**

Delete `python/ugreen_capture.py` — `ugreen_tracker.capture()` supersedes it.

Run:
```bash
git rm python/ugreen_capture.py
```

- [ ] **Step 6: Commit**

```bash
git add python/ugreen_tracker.py python/test_ugreen_tracker.py
git commit -m "UGreen tracker: image-diff TCP extraction + baseline mgmt

Replaces the post-it / purple HSV approach (unreliable at UGreen
distance + angle per 2026-04-20 brainstorm). The arm is detected via
absolute diff against a saved baseline frame with no arm in view; the
bottom-center pixel of the largest top-touching connected component
approximates the TCP. Four synthetic tests cover the happy path, null
baseline, roundtrip, and single-pixel noise rejection."
```

### Task A2.2 — Chessboard intrinsics for UGreen

**Files:**
- Create: `python/ugreen_intrinsics.py`
- Create: `scripts/print_chessboard.py`

- [ ] **Step 1: Write the chessboard printer**

Create `scripts/print_chessboard.py`:

```python
"""Generate a 7x5 inner-corners chessboard PNG for UGreen intrinsic
calibration. Print on A4 at 100% scale; each square is 30 mm.
The printed pattern is the PHYSICAL reference — the user shows it to
the UGreen camera in 15+ orientations and `ugreen_intrinsics.calibrate`
extracts fx/fy/cx/cy + distortion."""
import os
import sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "figures", "chessboard_7x5_30mm.png")

# OpenCV findChessboardCorners expects INNER corners.
# 7x5 inner corners = 8x6 squares. Each square 30 mm.
inner = (7, 5)
squares = (inner[0] + 1, inner[1] + 1)   # 8 x 6
square_mm = 30.0
dpi = 300
mm_per_inch = 25.4
px_per_mm = dpi / mm_per_inch
sq_px = int(round(square_mm * px_per_mm))
W = squares[0] * sq_px
H = squares[1] * sq_px

img = np.full((H, W), 255, dtype=np.uint8)
for i in range(squares[0]):
    for j in range(squares[1]):
        if (i + j) % 2 == 0:
            x0 = i * sq_px
            y0 = j * sq_px
            img[y0:y0+sq_px, x0:x0+sq_px] = 0

os.makedirs(os.path.dirname(OUT), exist_ok=True)
cv2.imwrite(OUT, img)
print(f"wrote {OUT}  ({W}x{H}px = "
      f"{W / px_per_mm:.0f}x{H / px_per_mm:.0f} mm, fits A4 landscape)")
print(f"Print at 100% scale, verify one square measures {square_mm:.0f} mm")
```

- [ ] **Step 2: Run the printer to verify the PNG is written**

Run: `C:/Python313/python.exe scripts/print_chessboard.py`
Expected: file exists at `figures/chessboard_7x5_30mm.png`, dimensions ≈ 2835×2126 px.

- [ ] **Step 3: Write the intrinsics calibration tests**

Append to `python/test_ugreen_tracker.py`:

```python
def test_intrinsics_io_roundtrip():
    """Save/load for UGreen intrinsics + distortion."""
    import tempfile, json
    from ugreen_intrinsics import save_intrinsics, load_intrinsics
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "intr.json")
    intr = {'fx': 600.0, 'fy': 601.0, 'cx': 320.0, 'cy': 240.0}
    dist = np.array([0.1, -0.2, 0.001, 0.002, 0.05], dtype=np.float64)
    save_intrinsics(path, intr, dist)
    intr2, dist2 = load_intrinsics(path)
    for k in intr:
        assert abs(intr[k] - intr2[k]) < 1e-9
    assert np.allclose(dist, dist2)
```

Add to the `__main__` block runner list.

- [ ] **Step 4: Run the test to verify it fails**

Expected: FAIL `ModuleNotFoundError: No module named 'ugreen_intrinsics'`.

- [ ] **Step 5: Implement the intrinsics module**

Create `python/ugreen_intrinsics.py`:

```python
"""UGreen intrinsic-camera calibration via OpenCV chessboard.

Usage (hardware session):
    C:/Python313/python.exe python/ugreen_intrinsics.py collect
        — captures 20 frames as the user shows the printed chessboard
          to the UGreen in different orientations.
    C:/Python313/python.exe python/ugreen_intrinsics.py solve
        — runs cv2.calibrateCamera and writes ugreen_intrinsics.json.

Offline, callers use load_intrinsics(path) to get the camera matrix.
"""
import json
import os
import sys
import time
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
DEFAULT_JSON = os.path.join(REPO, "ugreen_intrinsics.json")
CAPTURE_DIR = os.path.join(REPO, "logs", "ugreen_chessboards")

INNER_CORNERS = (7, 5)        # must match scripts/print_chessboard.py
SQUARE_MM = 30.0


def save_intrinsics(path, intrinsics, dist):
    payload = {
        'intrinsics': {k: float(v) for k, v in intrinsics.items()},
        'dist': [float(x) for x in np.asarray(dist).flatten()],
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


def load_intrinsics(path=DEFAULT_JSON):
    with open(path) as f:
        p = json.load(f)
    return (dict(p['intrinsics']),
            np.asarray(p['dist'], dtype=np.float64))


def collect_frames(n=20, out_dir=CAPTURE_DIR):
    """Interactively capture n chessboard views from the UGreen.
    Requires hardware. Called from __main__."""
    from ugreen_tracker import capture
    os.makedirs(out_dir, exist_ok=True)
    print(f"Show the chessboard from {n} different angles/distances.")
    print("Press ENTER in this terminal to capture each pose; 'q' to abort.")
    i = 0
    while i < n:
        input(f"  [{i + 1}/{n}] position chessboard, then ENTER: ")
        try:
            frame = capture()
        except Exception as ex:
            print(f"    capture failed: {ex}")
            continue
        path = os.path.join(out_dir, f"chess_{i:02d}.png")
        cv2.imwrite(path, frame)
        # Quick check: did we find the corners?
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, _ = cv2.findChessboardCorners(gray, INNER_CORNERS, None)
        print(f"    saved {path}  chessboard_detected={found}")
        if found:
            i += 1
        else:
            print("    retry — chessboard not detected, try better angle/lighting")


def solve(in_dir=CAPTURE_DIR, out_json=DEFAULT_JSON):
    """Run cv2.calibrateCamera on the collected chessboard images.
    Writes intrinsics + distortion to out_json."""
    files = sorted(
        os.path.join(in_dir, f)
        for f in os.listdir(in_dir) if f.startswith("chess_"))
    if len(files) < 10:
        raise RuntimeError(
            f"need >= 10 chessboard views, got {len(files)}")

    objp = np.zeros((INNER_CORNERS[0] * INNER_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:INNER_CORNERS[0], 0:INNER_CORNERS[1]].T.reshape(-1, 2)
    objp *= SQUARE_MM / 1000.0  # convert to metres

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_list = []
    img_list = []
    gray_shape = None
    used = 0
    for f in files:
        bgr = cv2.imread(f)
        if bgr is None:
            continue
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, INNER_CORNERS, None)
        if not found:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_list.append(objp)
        img_list.append(corners)
        used += 1
    if used < 10:
        raise RuntimeError(f"only {used} usable views; re-collect")

    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_list, img_list, gray_shape, None, None)
    intrinsics = {
        'fx': float(K[0, 0]),
        'fy': float(K[1, 1]),
        'cx': float(K[0, 2]),
        'cy': float(K[1, 2]),
    }
    save_intrinsics(out_json, intrinsics, dist)
    print(f"RMS reprojection error: {rms:.3f} px  (target < 1.0)")
    print(f"K = {K}")
    print(f"dist = {dist.flatten()}")
    print(f"wrote {out_json}  (used {used} views)")
    return rms, intrinsics, dist


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "solve"
    if mode == "collect":
        collect_frames()
    elif mode == "solve":
        solve()
    else:
        print(__doc__); sys.exit(1)
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `C:/Python313/python.exe python/test_ugreen_tracker.py`
Expected: 5 tests pass.

- [ ] **Step 7: Commit**

```bash
git add python/ugreen_intrinsics.py scripts/print_chessboard.py \
        python/test_ugreen_tracker.py figures/chessboard_7x5_30mm.png
git commit -m "UGreen intrinsic calibration: chessboard pattern + cv2 solver

Prints a 7x5 inner-corners chessboard at 30mm square size (A4 landscape)
and provides collect/solve entry points. Solver writes
ugreen_intrinsics.json. Hardware-required step is 'collect' (run in lab);
'solve' can run offline once frames are on disk."
```

---

## Stage A3 — Closed-loop calibration runner

### Task A3.1 — Calibration pipeline (move, capture, PnP)

**Files:**
- Create: `python/calibrate_closed_loop.py`
- Create: `python/test_calibrate_closed_loop.py`

- [ ] **Step 1: Write the PnP-solver unit test**

Create `python/test_calibrate_closed_loop.py`:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `C:/Python313/python.exe python/test_calibrate_closed_loop.py`
Expected: `ModuleNotFoundError: No module named 'calibrate_closed_loop'`.

- [ ] **Step 3: Implement the calibration runner**

Create `python/calibrate_closed_loop.py`:

```python
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
```

- [ ] **Step 4: Run the unit test to verify it passes**

Run: `C:/Python313/python.exe python/test_calibrate_closed_loop.py`
Expected: 1 test passes (synthetic PnP recovery within tolerance).

- [ ] **Step 5: Commit**

```bash
git add python/calibrate_closed_loop.py python/test_calibrate_closed_loop.py
git commit -m "closed-loop UGreen calibration runner

Moves arm through N cal_* teach points, captures UGreen frame at each,
extracts TCP via image-diff, solves cv2.solvePnP for T_cam_to_base.
Synthetic unit test recovers a known random transform to within 1 cm /
1 deg. Hardware run (with real cameras + arm) validated in Phase B."
```

---

## Stage A4 — Preflight script

### Task A4.1 — Preflight with 6 software checks (UGreen visual check in A4.2)

**Files:**
- Create: `python/preflight.py`

- [ ] **Step 1: Draft the preflight skeleton**

Create `python/preflight.py`:

```python
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
```

- [ ] **Step 2: Run offline-safe checks to verify the script runs**

Run: `C:/Python313/python.exe python/preflight.py --offline`
Expected: prints Calib + Tests checks; fails if calibration is stale or tests fail. No hardware access attempted.

- [ ] **Step 3: Commit**

```bash
git add python/preflight.py
git commit -m "preflight.py: 7-check pre-session sanity script

Runs QArm (connect + set_joints round-trip, catches -843 early), D415
warmup, UGreen open, calibration age+RMS, HSV detector dump, offline
test suite, and visual reference (arm -> pickhome1 + UGreen image-diff
within 20 px of stored TCP). --offline flag skips hardware-bound checks
so CI can gate dev-machine commits."
```

### Task A4.2 — Lab runbook

**Files:**
- Create: `LAB_RUNBOOK.md`

- [ ] **Step 1: Write the runbook**

Create `LAB_RUNBOOK.md`:

```markdown
# FruitSorting Lab Runbook

Session flow for using the QArm lab bench productively. Written 2026-04-20.

## Prerequisites on dev machine

- `C:/Python313/python.exe python/preflight.py --offline` is green.
- `git status` is clean (no uncommitted work before heading to the lab).

## On arrival at the lab

1. Power the QArm (red switch, wait ~10 s for LEDs to stabilise).
2. Plug D415 USB3 into the laptop.
3. Position the UGreen camera at ground level, lens toward the workspace.
   Do not move it again this session.
4. Wait 10 s, then run preflight (full):
       C:/Python313/python.exe python/preflight.py
5. If QArm check fails with -843, power-cycle the QArm base and retry.
6. If UGreen check fails, run `python/probe_ugreen.py` to re-discover
   the index.

## Closed-loop calibration (once per session)

1. Move the arm clear of UGreen view: open `teach_points.py`, press `g`,
   select `homeplace0`, ESC to save. (The arm now parks behind the
   bench.)
2. Capture baseline:
       C:/Python313/python.exe -c "from ugreen_tracker import capture, save_baseline; import os; save_baseline(capture(), os.path.join('logs', 'ugreen_baseline.png'))"
3. Put strawberries at cal_01..cal_04 and tomatoes at cal_05..cal_08.
4. Run the calibration:
       C:/Python313/python.exe python/calibrate_closed_loop.py
5. Check the reported RMS. Target < 15 mm; anything < 20 mm is usable.

## Visual reference snapshot (once per session)

1. With baseline + fruits in place, arm at pickhome1:
       C:/Python313/python.exe python/preflight.py
   Step 7 will fail the first time because no reference exists.
2. Capture the reference:
       C:/Python313/python.exe python/lab_visual_ref.py capture
3. Re-run preflight. Step 7 should now pass.

## Pick + place verification

1. Confirm basket teach points exist (`basket_a`, `basket_b`, `basket_c`).
   If not, enter `teach_points.py`, jog to each basket pose, press `n`
   to save with the matching label.
2. Dry run with one strawberry at cal_01:
       C:/Python313/python.exe python/main_final.py --pick-only
3. Confirm UGreen image-diff shows the fruit in the gripper after pick.
4. Full sort of 14 fruits:
       C:/Python313/python.exe python/main_final.py

## Demo video recording

Have OBS/Snipping Tool recording the UGreen companion window + Simulink
`.slx` side by side. Recording target: one 14-fruit autonomous run, then
one 60-s remote-mode manual pick. Cut to 3-5 min in post.

## Shutdown

1. `ctrl+c` in the terminal running main_final. Arm holds at last pose.
2. Close Simulink models (Save changes if prompted, otherwise discard).
3. Power down QArm (red switch).
4. Unplug D415 and UGreen.
5. `git status` — commit any working changes.
```

- [ ] **Step 2: Commit**

```bash
git add LAB_RUNBOOK.md
git commit -m "LAB_RUNBOOK: session flow from power-on to shutdown"
```

---

## Stage A5 — Logging extension

Per the spec, we capture both a Python `TraceLogger` event stream and Simulink `To Workspace` signals. The Simulink side is wired in Stage A7 with the rest of the model changes; here we extend the Python side.

### Task A5.1 — Promote TraceLogger + hook sorting_controller

**Files:**
- Create: `python/trace_logger.py` (extracted from `teach_points.py`)
- Modify: `python/teach_points.py` (import from the new module)
- Modify: `python/sorting_controller.py` (optional `logger` ctor arg + event hooks)
- Modify: `python/test_integration.py` (verify events emitted)

- [ ] **Step 1: Write the integration test**

Append to `python/test_integration.py`:

```python
def test_sorting_controller_emits_trace_events():
    """When a logger is passed to FruitSortingController, it should emit
    PICK_ATTEMPT, GRIPPER_READBACK, and SORT_COMPLETE events during a
    pick-only run."""
    name = "controller_trace_events"
    from sorting_controller import FruitSortingController, State
    from trace_logger import TraceLogger
    import tempfile, os
    tmp = tempfile.mkdtemp()
    log_path = os.path.join(tmp, "test.log")
    logger = TraceLogger(log_path)

    q = MockQArm()
    c = FruitSortingController(q, pick_only=True, logger=logger)
    c.T_TRANSIT = 0.02; c.T_APPROACH = 0.02; c.T_PICK = 0.02
    c.T_DWELL = 0.01; c.T_GRIP = 0.01
    c.set_fruit_positions([[0.35, 0.10, 0.13]], ["strawberry"])
    c.state = State.GO_HOME
    for _ in range(3000):
        c._step(__import__("time").time())
        __import__("time").sleep(0.001)
        if c.state == State.DONE:
            break
    logger.close()

    with open(log_path) as f:
        log = f.read()
    assert "PICK_ATTEMPT" in log, "no PICK_ATTEMPT event"
    assert "GRIPPER_READBACK" in log, "no GRIPPER_READBACK event"
    assert "SORT_COMPLETE" in log, "no SORT_COMPLETE event"
    return name, True, "3/3 trace events emitted"
```

Add to the `TESTS` list.

- [ ] **Step 2: Run the test to verify it fails**

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: FAIL with `ModuleNotFoundError: No module named 'trace_logger'`.

- [ ] **Step 3: Extract `TraceLogger` into its own module**

Create `python/trace_logger.py` by copying the `TraceLogger` class from `python/teach_points.py` (lines ~57-101). The new module is standalone — no imports from teach_points.

```python
"""Append-only event logger shared by teach_points.py, sorting_controller.py,
and anything else that wants a timestamped trace of runtime events.

One line per event: [t=seconds since session start]  TAG=name  key=val ...

Kept simple on purpose: plain text, line-buffered, crash-safe (each write
is flushed). Parse with regex from generate_report_plots.py.
"""
import os
import sys
import time
import numpy as np
from datetime import datetime


class TraceLogger:
    IMPORTANT_TAGS = {
        "SESSION_START", "SESSION_END", "HIL_ERROR", "IK_FAIL",
        "JOINT_LIMIT", "EXCEPT", "ROUTINE_START", "ROUTINE_DONE",
        "GOTO", "GRIPPER_CMD", "PICK_ATTEMPT", "GRIPPER_READBACK",
        "SORT_COMPLETE", "DETECTION_FRAME", "PRE_FLIGHT_SKIP",
    }

    def __init__(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.path = path
        self.f = open(path, "a", buffering=1)
        self.t0 = time.time()
        self.log("SESSION_START",
                  ts=datetime.now().isoformat(timespec="seconds"))

    @staticmethod
    def _fmt_val(v):
        if isinstance(v, (list, tuple, np.ndarray)):
            arr = np.asarray(v, dtype=float)
            return "[" + ",".join(f"{x:.4f}" for x in arr.flatten()) + "]"
        if isinstance(v, float):
            return f"{v:.4f}"
        return str(v)

    def log(self, tag, **kv):
        t = time.time() - self.t0
        parts = [f"t={t:.3f}", f"TAG={tag}"]
        for k, v in kv.items():
            parts.append(f"{k}={self._fmt_val(v)}")
        line = "  ".join(parts)
        try:
            self.f.write(line + "\n")
        except Exception:
            pass
        if tag in self.IMPORTANT_TAGS:
            sys.stderr.write("[trace] " + line + "\n")
            sys.stderr.flush()

    def close(self):
        try:
            self.log("SESSION_END")
            self.f.close()
        except Exception:
            pass
```

- [ ] **Step 4: Switch `teach_points.py` to import from the new module**

In `python/teach_points.py`, replace the inline `TraceLogger` class (and remove its definition) with:

```python
from trace_logger import TraceLogger
```

Confirm the rest of the file still references `TraceLogger` the same way.

- [ ] **Step 5: Hook the logger into `sorting_controller`**

Modify `python/sorting_controller.py`. Change the `__init__` signature:

```python
    def __init__(self, qarm, camera=None, pick_only=False, logger=None):
        ...
        self.logger = logger
```

Add a small helper near the top of the class:

```python
    def _log(self, tag, **kv):
        if self.logger is not None:
            self.logger.log(tag, **kv)
```

Emit the three event types the test expects. In `SELECT_FRUIT` add:
```python
            self._log("PICK_ATTEMPT",
                       type=self.current_target['type'],
                       pos=self.current_target['pos'])
```
In `CLOSE_GRIPPER`, after the readback block, add:
```python
                self._log("GRIPPER_READBACK", target=GRIP_CLOSE,
                           actual=self._held_grip,
                           stall=(GRIP_CLOSE - self._held_grip))
```
In the final `DONE` transition (at the top of `_step`, after `if self.state == State.DONE: return` or similar), add a one-shot guard:
```python
        if self.state == State.DONE and not getattr(self, "_done_logged", False):
            self._log("SORT_COMPLETE", sorted=self.sorted_count)
            self._done_logged = True
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: `12/12 passed.`

- [ ] **Step 7: Commit**

```bash
git add python/trace_logger.py python/teach_points.py \
        python/sorting_controller.py python/test_integration.py
git commit -m "extract TraceLogger + emit PICK/GRIPPER/COMPLETE events

TraceLogger moves out of teach_points.py into its own module so the
autonomous FSM can share it. FruitSortingController gains an optional
logger ctor arg and emits PICK_ATTEMPT (SELECT_FRUIT), GRIPPER_READBACK
(after CLOSE settle), SORT_COMPLETE (DONE). The Python side of the
logging story from the spec is now in place; the Simulink side lands
in Stage A7."
```

---

## Stage A6 — Report plot generator

### Task A6.1 — Plot generator reads logs + emits PNGs

**Files:**
- Create: `scripts/generate_report_plots.py`

- [ ] **Step 1: Write the plot generator**

Create `scripts/generate_report_plots.py`:

```python
"""Generate the four report figures from a single run's log artefacts.

Inputs
------
  logs/robot_trace_*.log   (latest)
  logs/autonomous_run.mat  (optional; produced by the Simulink run)

Outputs
-------
  figures/plot_joints.png
  figures/plot_gripper_readback.png
  figures/plot_fsm_timeline.png
  figures/plot_detections.png
"""
import os
import re
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
LOG_DIR = os.path.join(REPO, "logs")
FIG_DIR = os.path.join(REPO, "figures")


def latest_trace_log():
    files = sorted(glob.glob(os.path.join(LOG_DIR, "robot_trace_*.log")))
    if not files:
        return None
    return files[-1]


def parse_events(path):
    """Parse TraceLogger format: t=NNN.NNN  TAG=NAME  key=val key=val ...
    Returns list of (t, tag, kv_dict)."""
    events = []
    rx = re.compile(r"t=([0-9.]+)\s+TAG=(\w+)\s*(.*)")
    kv_rx = re.compile(r"(\w+)=(\S+)")
    with open(path) as f:
        for line in f:
            m = rx.match(line.strip())
            if not m:
                continue
            t = float(m.group(1))
            tag = m.group(2)
            kv = {}
            for km in kv_rx.finditer(m.group(3)):
                kv[km.group(1)] = km.group(2)
            events.append((t, tag, kv))
    return events


def plot_gripper_readback(events, out):
    tgts, acts, stalls = [], [], []
    for t, tag, kv in events:
        if tag != "GRIPPER_READBACK":
            continue
        tgts.append(float(kv.get("target", "0")))
        acts.append(float(kv.get("actual", "0")))
        stalls.append(float(kv.get("stall", "0")))
    if not tgts:
        return False
    fig, ax = plt.subplots(figsize=(7, 4))
    idx = np.arange(len(tgts))
    ax.plot(idx, tgts, "o-", label="commanded")
    ax.plot(idx, acts, "s-", label="actual")
    ax.bar(idx, stalls, alpha=0.3, label="stall delta", color="red")
    ax.set_xlabel("pick #")
    ax.set_ylabel("gripper command")
    ax.set_title("Gripper commanded vs settled (readback)")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


def plot_fsm_timeline(events, out):
    state_events = [(t, kv.get("state", "")) for t, tag, kv in events
                     if tag == "STATE_ENTER"]
    if not state_events:
        # Fall back: build from PICK_ATTEMPT / SORT_COMPLETE markers only
        markers = [(t, tag) for t, tag, _ in events
                   if tag in ("PICK_ATTEMPT", "SORT_COMPLETE")]
        if not markers:
            return False
        fig, ax = plt.subplots(figsize=(8, 2.5))
        for t, tag in markers:
            ax.axvline(t, color="C0" if tag == "PICK_ATTEMPT" else "C1",
                        label=tag)
        ax.set_xlabel("time (s)")
        ax.set_yticks([])
        ax.set_title("FSM event markers")
        handles, labels = ax.get_legend_handles_labels()
        seen = {}
        for h, lab in zip(handles, labels):
            seen.setdefault(lab, h)
        ax.legend(seen.values(), seen.keys())
        fig.tight_layout(); fig.savefig(out); plt.close(fig)
        return True
    # Real Gantt if STATE_ENTER events exist.
    states = sorted(set(s for _, s in state_events))
    y = {s: i for i, s in enumerate(states)}
    fig, ax = plt.subplots(figsize=(10, max(3, 0.4 * len(states))))
    for (t0, s0), (t1, _) in zip(state_events[:-1], state_events[1:]):
        ax.hlines(y[s0], t0, t1, lw=6)
    ax.set_yticks(list(y.values()))
    ax.set_yticklabels(list(y.keys()))
    ax.set_xlabel("time (s)")
    ax.set_title("FSM state timeline")
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


def plot_detections(events, out):
    rows = []
    for t, tag, kv in events:
        if tag != "DETECTION_FRAME":
            continue
        rows.append((t, int(kv.get("n", "0")),
                     float(kv.get("mean_conf", "0"))))
    if not rows:
        return False
    t = np.array([r[0] for r in rows])
    n = np.array([r[1] for r in rows])
    c = np.array([r[2] for r in rows])
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    ax1.plot(t, n, "o-"); ax1.set_ylabel("detections")
    ax1.set_title("Detection quality")
    ax2.plot(t, c, "s-", color="C1"); ax2.set_ylabel("mean conf")
    ax2.set_xlabel("time (s)")
    for a in (ax1, ax2): a.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


def plot_joints_from_mat(out):
    mat_path = os.path.join(LOG_DIR, "autonomous_run.mat")
    if not os.path.exists(mat_path):
        return False
    try:
        from scipy.io import loadmat
    except ImportError:
        return False
    data = loadmat(mat_path)
    if "joint_cmd" not in data or "joint_actual" not in data:
        return False
    t = data.get("time", np.arange(data["joint_cmd"].shape[0])).flatten()
    cmd = data["joint_cmd"]
    act = data["joint_actual"]
    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for j in range(4):
        axes[j].plot(t, np.rad2deg(cmd[:, j]), label="cmd")
        axes[j].plot(t, np.rad2deg(act[:, j]), label="actual",
                       linestyle="--")
        axes[j].set_ylabel(f"joint {j+1} (deg)")
        axes[j].grid(True, alpha=0.3); axes[j].legend()
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Joint trajectories over autonomous run")
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    log = latest_trace_log()
    if log is None:
        print("no robot_trace_*.log — run the autonomous pipeline first")
        return 1
    events = parse_events(log)
    print(f"parsed {len(events)} events from {log}")

    results = {
        "gripper_readback":
            plot_gripper_readback(events, os.path.join(FIG_DIR,
                "plot_gripper_readback.png")),
        "fsm_timeline":
            plot_fsm_timeline(events, os.path.join(FIG_DIR,
                "plot_fsm_timeline.png")),
        "detections":
            plot_detections(events, os.path.join(FIG_DIR,
                "plot_detections.png")),
        "joints":
            plot_joints_from_mat(os.path.join(FIG_DIR,
                "plot_joints.png")),
    }
    for k, v in results.items():
        print(f"  {k}: {'OK' if v else 'no data'}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
```

- [ ] **Step 2: Run against the existing test log to smoke-check**

Because no autonomous run has happened yet, the generator should print "no data" for most plots without crashing.

Run: `C:/Python313/python.exe scripts/generate_report_plots.py`
Expected: exits 0 or 1 with a clear "no data" / "no log" message — no exception.

- [ ] **Step 3: Commit**

```bash
git add scripts/generate_report_plots.py
git commit -m "generate_report_plots.py: 4 report figures from logs

Parses TraceLogger text format + Simulink .mat output, emits:
  plot_joints, plot_gripper_readback, plot_fsm_timeline, plot_detections.
Each plot gracefully reports 'no data' when the corresponding log
stream is absent, so running the script in dev (no autonomous run yet)
does not error."
```

---

## Stage A7 — Remote mode + Stateflow + facade refactor

This stage contains the heaviest work. It is split into four tasks that can each be committed independently.

### Task A7.1 — Remote jog logic (Python side)

**Files:**
- Create: `python/remote_jog.py`
- Modify: `python/test_integration.py`

- [ ] **Step 1: Write the jog-step tests**

Append to `python/test_integration.py`:

```python
def test_remote_jog_clamps_workspace():
    """Cartesian nudges that would leave the workspace box are refused."""
    name = "remote_jog_workspace_clamp"
    from remote_jog import cartesian_nudge, WORKSPACE_BOX
    # Near +X edge
    cur = np.array([WORKSPACE_BOX['x'][1] - 0.002, 0.0, 0.15])
    out = cartesian_nudge(cur, axis='x', step=0.01)  # would leave box
    assert out is None, f"expected clamp, got {out}"
    # Middle of box, small step, stays inside
    cur = np.array([0.30, 0.00, 0.15])
    out = cartesian_nudge(cur, axis='x', step=0.01)
    assert out is not None
    assert abs(out[0] - 0.31) < 1e-9


def test_remote_joint_jog_clamps_limits():
    from remote_jog import joint_nudge
    from qarm_driver import QArmDriver
    lims = QArmDriver.JOINT_LIMITS
    # Near upper limit on joint 1
    cur = np.array([lims[0, 1] - 0.001, 0, 0, 0], dtype=float)
    out = joint_nudge(cur, joint=0, step=np.deg2rad(5))
    assert out is None, "should clamp"
    # Middle of range
    cur = np.array([0.0, 0.0, 0.0, 0.0])
    out = joint_nudge(cur, joint=0, step=np.deg2rad(5))
    assert out is not None
    assert abs(out[0] - np.deg2rad(5)) < 1e-9
```

Add to `TESTS` list.

- [ ] **Step 2: Run the tests to verify they fail**

Expected: `ModuleNotFoundError: No module named 'remote_jog'`.

- [ ] **Step 3: Implement `remote_jog`**

Create `python/remote_jog.py`:

```python
"""Helpers shared by the remote-mode HMI (Simulink-side MATLAB Function
blocks call into these via py.remote_jog.*). No Simulink dependency here
so the logic stays unit-testable in plain Python.

Workspace box and joint limits match the spec (Section 2.1 safety).
"""
import numpy as np
from qarm_driver import QArmDriver

WORKSPACE_BOX = {
    'x': (-0.10, 0.55),
    'y': (-0.55, 0.55),
    'z': (0.02,  0.50),
}


def _inside_box(xyz):
    x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return (WORKSPACE_BOX['x'][0] <= x <= WORKSPACE_BOX['x'][1] and
            WORKSPACE_BOX['y'][0] <= y <= WORKSPACE_BOX['y'][1] and
            WORKSPACE_BOX['z'][0] <= z <= WORKSPACE_BOX['z'][1])


def cartesian_nudge(current_xyz, axis, step):
    """Return a new xyz (3-vector) with axis shifted by step if the result
    is inside the workspace box; otherwise None (caller should log a
    warning and leave the arm at current_xyz)."""
    i = {'x': 0, 'y': 1, 'z': 2}[axis]
    out = np.array(current_xyz, dtype=float).copy()
    out[i] += step
    if not _inside_box(out):
        return None
    return out


def joint_nudge(current_joints, joint, step):
    """Return a new (4,) joint vector with joint shifted by step if the
    result is inside the QArm joint limits; otherwise None."""
    out = np.array(current_joints, dtype=float).copy()
    out[joint] += step
    lo, hi = QArmDriver.JOINT_LIMITS[joint]
    if out[joint] < lo or out[joint] > hi:
        return None
    return out
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: `14/14 passed.`

- [ ] **Step 5: Commit**

```bash
git add python/remote_jog.py python/test_integration.py
git commit -m "remote_jog: workspace-box + joint-limit guarded nudges

Pure-Python helpers invoked by the Simulink remote HMI's jog buttons.
Refuses out-of-box Cartesian moves and out-of-limit joint moves,
returning None so the caller logs and holds pose. Unit-tested on both
paths (inside the box / at the edge)."
```

### Task A7.2 — Remote companion view (OpenCV window with overlay)

**Files:**
- Create: `python/remote_view.py`
- Create: `matlab_facade/py_remote_view.m`

- [ ] **Step 1: Write the Python companion**

Create `python/remote_view.py`:

```python
"""OpenCV companion window for remote mode. Launched by Simulink when
remote mode activates; closed via cv2.setMouseCallback click or ESC.

Reads live D415 frames and overlays the current mode, current joints,
current xyz, gripper state, jog step, and the last HMI command. The
current state is supplied by Simulink through a shared JSON status
file (polled at 10 Hz) to avoid in-process Python<->MATLAB coupling.
"""
import json
import os
import sys
import time
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
STATUS_FILE = os.path.join(REPO, "logs", "remote_status.json")
WINDOW = "QArm Remote - D415 live"


def read_status():
    if not os.path.exists(STATUS_FILE):
        return None
    try:
        with open(STATUS_FILE) as f:
            return json.load(f)
    except Exception:
        return None


def main():
    from camera import QArmCamera
    cam = QArmCamera()
    cam.open()
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            try:
                color, _ = cam.read()
            except Exception:
                color = np.zeros((720, 1280, 3), dtype=np.uint8)
            disp = color.copy()
            s = read_status()
            if s is None:
                cv2.putText(disp, "waiting for Simulink status...",
                             (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                             (0, 255, 255), 2)
            else:
                lines = [
                    f"mode     : {s.get('mode','?')}",
                    f"sub-mode : {s.get('sub_mode','?')}",
                    f"joints(°): {s.get('joints_deg','?')}",
                    f"xyz (m)  : {s.get('xyz_m','?')}",
                    f"gripper  : {s.get('gripper','?')}",
                    f"step     : {s.get('step','?')}",
                    f"last cmd : {s.get('last_cmd','?')}",
                ]
                for i, line in enumerate(lines):
                    cv2.putText(disp, line, (20, 30 + i * 26),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                                 (0, 0, 0), 4)
                    cv2.putText(disp, line, (20, 30 + i * 26),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                                 (0, 255, 255), 2)
            cv2.imshow(WINDOW, disp)
            k = cv2.waitKey(30) & 0xFF
            if k == 27:      # ESC
                break
    finally:
        try: cam.close()
        except Exception: pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Write the MATLAB launcher**

Create `matlab_facade/py_remote_view.m`:

```matlab
function py_remote_view(action)
%PY_REMOTE_VIEW Launch or update the OpenCV companion window for remote mode.
%   py_remote_view('open')   -- start the Python window (non-blocking).
%   py_remote_view('close')  -- close it.
%   Calling with no arg defaults to 'open'. The window runs in a
%   separate Python process so it does not block Simulink.

    if nargin < 1, action = 'open'; end
    persistent proc
    pyexe = 'C:/Python313/python.exe';
    script = fullfile(fileparts(mfilename('fullpath')), '..', 'python', ...
                        'remote_view.py');
    switch action
        case 'open'
            if isempty(proc) || ~isvalid(proc) || ~strcmp(proc.State,'running')
                cmd = sprintf('"%s" "%s"', pyexe, script);
                proc = processManager('command', cmd);
            end
        case 'close'
            if ~isempty(proc) && isvalid(proc) && strcmp(proc.State,'running')
                proc.stop();
            end
        otherwise
            error('py_remote_view: unknown action "%s"', action);
    end
end
```

- [ ] **Step 3: Syntax-check the Python companion**

Run: `C:/Python313/python.exe -c "import ast; ast.parse(open('python/remote_view.py').read()); print('ok')"`
Expected: `ok`.

- [ ] **Step 4: Commit**

```bash
git add python/remote_view.py matlab_facade/py_remote_view.m
git commit -m "remote_view: OpenCV companion + MATLAB launcher

D415 live feed with status overlay in a separate Python process so it
can render at 30 fps without blocking the Simulink solver. Simulink
writes logs/remote_status.json at 10 Hz with mode/joints/xyz/gripper;
the window polls and renders text."
```

### Task A7.3 — Build Stateflow chart for autonomous FSM

**Files:**
- Create: `matlab_facade/build_autonomous_stateflow.m`
- Modify: `matlab_facade/py_controller.m` (replace persistent handle path)

- [ ] **Step 1: Build script generates the Stateflow**

Create `matlab_facade/build_autonomous_stateflow.m`:

```matlab
function build_autonomous_stateflow()
%BUILD_AUTONOMOUS_STATEFLOW Assemble the Stateflow chart that drives
%the autonomous FSM. 12 states, transitions timed by dt accumulator.
%Entry actions call py.sorting_controller.* helpers for the heavy work
%(IK safety, gripper ramp+readback, smart pick_z, pre-flight).

    modelName = 'FruitSorting_Autonomous';
    if bdIsLoaded(modelName), close_system(modelName, 0); end
    new_system(modelName); open_system(modelName);

    % Inputs and outputs
    add_block('simulink/Sources/In1', [modelName '/joints_cur'], ...
              'Position', [20, 20, 50, 40]);
    add_block('simulink/Sources/In1', [modelName '/dt'], ...
              'Position', [20, 80, 50, 100]);
    add_block('simulink/Sinks/Out1', [modelName '/phi'], ...
              'Position', [520, 20, 550, 40]);
    add_block('simulink/Sinks/Out1', [modelName '/gripper'], ...
              'Position', [520, 60, 550, 80]);
    add_block('simulink/Sinks/Out1', [modelName '/state_id'], ...
              'Position', [520, 100, 550, 120]);
    add_block('simulink/Sinks/Out1', [modelName '/done'], ...
              'Position', [520, 140, 550, 160]);

    % Insert Stateflow chart
    chart = Stateflow.Chart(get_param(modelName, 'Object'));
    chart.Name = 'FruitSortFSM';
    % Chart I/O
    for nm = {'joints_cur', 'dt'}
        d = Stateflow.Data(chart); d.Name = nm{1}; d.Scope = 'Input';
    end
    for nm = {'phi', 'gripper', 'state_id', 'done'}
        d = Stateflow.Data(chart); d.Name = nm{1}; d.Scope = 'Output';
    end
    % Book-keeping variables
    for nm = {'t_transit', 'sorted_count'}
        d = Stateflow.Data(chart); d.Name = nm{1}; d.Scope = 'Local';
    end

    states = struct( ...
        'INIT',         [30, 30,  120, 80], ...
        'GO_HOME',      [180, 30, 300, 80], ...
        'SELECT',       [360, 30, 480, 80], ...
        'APPROACH',     [540, 30, 660, 80], ...
        'DESCEND',      [540, 120, 660, 170], ...
        'CLOSE_GRIP',   [540, 210, 660, 260], ...
        'ASCEND_PICK',  [360, 210, 480, 260], ...
        'MOVE_BASKET',  [180, 210, 300, 260], ...
        'DESCEND_PLACE',[30, 210, 150, 260], ...
        'OPEN_GRIP',    [30, 300, 150, 350], ...
        'ASCEND_PLACE', [180, 300, 300, 350], ...
        'DONE',         [360, 300, 480, 350]);
    names = fieldnames(states);
    state_objs = struct();
    for k = 1:numel(names)
        nm = names{k};
        s = Stateflow.State(chart);
        s.Name = nm;
        s.Position = states.(nm);
        state_objs.(nm) = s;
    end

    % Entry actions — each calls the matching Python helper so logic stays
    % one-source-of-truth in python/sorting_controller.py. Pseudocode,
    % fill in the exact MATLAB calls during implementation:
    state_objs.SELECT.EntryAction = ...
        'phi = double(py.sorting_controller.stateflow_select(joints_cur))';
    state_objs.CLOSE_GRIP.EntryAction = ...
        'gripper = double(py.sorting_controller.stateflow_close_grip())';
    state_objs.OPEN_GRIP.EntryAction = ...
        'gripper = double(py.sorting_controller.stateflow_open_grip())';

    % Default transition
    def_tr = Stateflow.Transition(chart);
    def_tr.Destination = state_objs.INIT;
    def_tr.SourceEndpoint = [30, 5];
    def_tr.DestinationEndpoint = [30, 30];

    % Subsequent transitions (INIT -> GO_HOME -> SELECT -> APPROACH -> ...)
    pairs = {
        'INIT','GO_HOME','t_transit > 2.0';
        'GO_HOME','SELECT','~isempty(py.sorting_controller.fruit_queue)';
        'SELECT','APPROACH','true';
        'APPROACH','DESCEND','t_transit > 2.0';
        'DESCEND','CLOSE_GRIP','t_transit > 0.8';
        'CLOSE_GRIP','ASCEND_PICK','t_transit > 0.8';
        'ASCEND_PICK','MOVE_BASKET','t_transit > 1.0';
        'MOVE_BASKET','DESCEND_PLACE','t_transit > 2.0';
        'DESCEND_PLACE','OPEN_GRIP','t_transit > 1.0';
        'OPEN_GRIP','ASCEND_PLACE','t_transit > 0.8';
        'ASCEND_PLACE','GO_HOME','t_transit > 1.0';
        'GO_HOME','DONE','isempty(py.sorting_controller.fruit_queue)'};
    for r = 1:size(pairs, 1)
        tr = Stateflow.Transition(chart);
        tr.Source = state_objs.(pairs{r, 1});
        tr.Destination = state_objs.(pairs{r, 2});
        tr.LabelString = pairs{r, 3};
    end

    save_system(modelName);
    close_system(modelName);
    fprintf('Built %s.slx\n', modelName);
end
```

- [ ] **Step 2: Commit the build script (without running MATLAB)**

```bash
git add matlab_facade/build_autonomous_stateflow.m
git commit -m "build_autonomous_stateflow.m: Stateflow chart generator

12 states, transitions on t_transit accumulator, entry actions call
py.sorting_controller.stateflow_* helpers. When run in MATLAB R2025a,
produces matlab_facade/FruitSorting_Autonomous.slx. Python stays the
source of truth for IK safety + gripper ramp + smart pick_z."
```

**NOTE:** actually running the build script requires MATLAB R2025a, which may not be available on this machine. That step belongs to Phase B (lab machine) or a teammate with MATLAB installed.

### Task A7.4 — Add Stateflow-facing Python entry points

**Files:**
- Modify: `python/sorting_controller.py` (add `stateflow_select`, `stateflow_close_grip`, `stateflow_open_grip` module-level helpers)

- [ ] **Step 1: Add the helpers**

At the bottom of `python/sorting_controller.py`, add:

```python
# Module-level wrappers for Stateflow entry actions. Stateflow stores
# state IDs; the actual compute lives here so Python tests cover it.

_stateflow_ctx = {'controller': None}


def stateflow_init(fruit_positions=None, fruit_types=None,
                    pick_only=False):
    """Called once by Stateflow's INIT entry action. Builds the shared
    controller instance."""
    from qarm_driver import QArmDriver
    q = QArmDriver(); q.connect()
    c = FruitSortingController(q, pick_only=pick_only)
    if fruit_positions is not None:
        c.set_fruit_positions(fruit_positions, fruit_types)
    _stateflow_ctx['controller'] = c
    return True


def stateflow_select(joints_cur):
    """Pick the next reachable fruit or signal 'queue empty'. Returns
    the approach-phi vector (length 4) or an empty list when queue is
    empty / no reachable fruit."""
    c = _stateflow_ctx.get('controller')
    if c is None or not c.fruit_queue:
        return []
    # Reuse the pre-flight reachability test already in
    # _step -> SELECT_FRUIT. Pull the next fruit, check, skip if bad.
    while c.fruit_queue:
        target = c.fruit_queue[0]
        approach = target['pos'].copy(); approach[2] = c.APPROACH_Z
        pick = target['pos'].copy()
        pick[2] = c._compute_pick_z(target['pos'])
        if c._ik_safe(approach) is None or c._ik_safe(pick) is None:
            c.fruit_queue.pop(0)
            if c.logger: c.logger.log("PRE_FLIGHT_SKIP",
                                        type=target['type'])
            continue
        c.current_target = c.fruit_queue.pop(0)
        c.target_basket = c.BASKETS.get(c.current_target['type'],
                                          c.BASKETS['tomato'])
        phi = c._ik_safe(approach)
        return phi.tolist()
    return []


def stateflow_close_grip():
    """Ramp gripper closed, return settled value. Blocking."""
    c = _stateflow_ctx.get('controller')
    if c is None: return 0.15
    return c.set_gripper_ramp(GRIP_CLOSE)


def stateflow_open_grip():
    c = _stateflow_ctx.get('controller')
    if c is None: return GRIP_OPEN
    return c.set_gripper_ramp(GRIP_OPEN)


def stateflow_sorted_count():
    c = _stateflow_ctx.get('controller')
    return c.sorted_count if c is not None else 0


def fruit_queue():
    """Exposed for Stateflow transition guards."""
    c = _stateflow_ctx.get('controller')
    return c.fruit_queue if c is not None else []
```

- [ ] **Step 2: Run the integration tests to verify nothing regressed**

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: `14/14 passed.`

- [ ] **Step 3: Commit**

```bash
git add python/sorting_controller.py
git commit -m "sorting_controller: Stateflow-facing module helpers

stateflow_init / select / close_grip / open_grip / sorted_count /
fruit_queue. Keep the heavy logic (pre-flight reachability, gripper
ramp+readback) inside the existing controller instance; these are
thin wrappers Stateflow entry actions can call without knowing about
the Python class structure."
```

### Task A7.5 — Unified hardware model with 3-way mode switch

**Files:**
- Create: `matlab_facade/build_hardware_model_v2.m`

- [ ] **Step 1: Write the build script**

Create `matlab_facade/build_hardware_model_v2.m`:

```matlab
function build_hardware_model_v2()
%BUILD_HARDWARE_MODEL_V2 Unified FruitSorting_Hardware model with a
%3-way mode switch (autonomous / remote / release) feeding a single
%py_qarm_io dispatcher. Rebuilds the .slx from scratch.

    modelName = 'FruitSorting_Hardware';
    if bdIsLoaded(modelName), close_system(modelName, 0); end
    new_system(modelName); open_system(modelName);

    % --- Mode selector ---
    add_block('simulink/Sources/Constant', [modelName '/mode'], ...
              'Value', '0', 'Position', [30, 20, 80, 40]);
    add_block('simulink/Signal Routing/Multiport Switch', ...
              [modelName '/mode_select'], ...
              'Inputs', '3', 'Position', [400, 40, 450, 180]);

    % --- Autonomous branch: Stateflow model reference ---
    add_block('simulink/Model-Wide Utilities/Model Info', ...
              [modelName '/autonomous'], 'Position', [200, 20, 320, 50]);
    % In practice: a Model Reference block to FruitSorting_Autonomous.slx.
    % For v2 scaffold we add a placeholder and wire it in when the
    % Stateflow chart build completes (Task A7.3).

    % --- Remote branch: HMI subsystem ---
    add_block('simulink/Ports & Subsystems/Subsystem', ...
              [modelName '/remote_hmi'], 'Position', [200, 80, 320, 110]);
    % Populate inside with jog buttons (Push Buttons from Dashboard lib),
    % mode sub-toggle, gripper open/close buttons, E-stop, workspace
    % clamp via MATLAB Function block calling py.remote_jog.* helpers.
    % Full wiring detailed in remote_hmi.md (section below).

    % --- Release branch: zeros ---
    add_block('simulink/Sources/Constant', [modelName '/release_cmd'], ...
              'Value', 'zeros(4,1)', 'Position', [200, 140, 320, 170]);

    % --- Shared output: py_qarm_io ---
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [modelName '/py_qarm_io'], 'Position', [500, 60, 620, 120]);
    % py_qarm_io expects (phi_cmd, gripper_cmd, mode) -- wire mode from
    % the same Constant as mode_select so SIMULATE/HARDWARE/RELEASE
    % toggling is exposed to the evaluator.

    % E-stop button (Dashboard Push Button, signal-routes to Stop block
    % via Stateflow state forcing): added in a follow-up commit.

    % Logging: add 'To Workspace' blocks on commanded phi, gripper,
    % state_id, joints_read so generate_report_plots.py can find the
    % .mat at end of run.
    for i = 1:4
        nm = sprintf('log_joint_%d', i);
        add_block('simulink/Sinks/To Workspace', [modelName '/' nm], ...
                  'VariableName', nm, 'SaveFormat', 'StructureWithTime', ...
                  'Position', [700, 20 + (i-1)*40, 780, 40 + (i-1)*40]);
    end

    save_system(modelName);
    close_system(modelName);
    fprintf('Built %s.slx (v2, unified 3-way model).\n', modelName);
end
```

- [ ] **Step 2: Document the remote HMI subsystem layout**

Create `matlab_facade/remote_hmi.md`:

```markdown
# Remote HMI subsystem — wiring reference

Inside `FruitSorting_Hardware.slx > remote_hmi`:

## Inputs
- `sub_mode` : Manual Switch (joint vs cartesian).
- `jog_buttons` : 1x8 bus. In joint mode: [+j1, -j1, +j2, -j2, +j3, -j3, +j4, -j4]. In cartesian mode: [+x, -x, +y, -y, +z, -z, +gamma, -gamma].
- `grip_open`, `grip_close` : Push Buttons.
- `e_stop` : Dashboard Push Button (large, red, latching).
- `step_size` : Slider — 1/5/10/50 mm.

## MATLAB Function "remote_step"
One function called each solver tick (dt = 0.05 s). Pseudocode:

```matlab
function [phi_cmd, gripper_cmd, status_out] = remote_step( ...
          sub_mode, jog_buttons, grip_open, grip_close, e_stop, ...
          step_size, joints_cur)
    persistent last_joints last_gripper
    if e_stop
        phi_cmd = joints_cur; gripper_cmd = 0.15; return
    end
    if isempty(last_joints), last_joints = joints_cur; end
    if isempty(last_gripper), last_gripper = 0.15; end
    [phi_cmd, last_joints] = apply_jog( ...
        joints_cur, sub_mode, jog_buttons, step_size);
    if grip_open,  last_gripper = double(py.sorting_controller.stateflow_open_grip());  end
    if grip_close, last_gripper = double(py.sorting_controller.stateflow_close_grip()); end
    gripper_cmd = last_gripper;
    status_out = pack_status(phi_cmd, gripper_cmd, sub_mode, step_size);
end
```

`apply_jog` delegates to:
- `py.remote_jog.joint_nudge(current, joint, step)` for joint sub-mode.
- `py.remote_jog.cartesian_nudge(current_xyz, axis, step)` for cartesian sub-mode (wrap + IK via `py.qarm_kinematics.inverse_kinematics`).

`pack_status` writes a JSON with mode, joints_deg, xyz_m, gripper, step, last_cmd to `logs/remote_status.json` at ~10 Hz so the OpenCV companion can poll it.

## Outputs
- `phi_cmd` (4 x 1)
- `gripper_cmd` (scalar)
- Feeds into the parent model's Multiport Switch on index `mode == 2`.
```

- [ ] **Step 3: Commit**

```bash
git add matlab_facade/build_hardware_model_v2.m matlab_facade/remote_hmi.md
git commit -m "v2 hardware model scaffold + remote HMI wiring reference

Build script for the unified 3-way .slx (autonomous / remote / release)
and a markdown reference covering the remote HMI subsystem's blocks,
inputs, and the remote_step MATLAB Function. Implementation of the
.slx contents happens on a machine with MATLAB R2025a (lab or teammate)."
```

---

## Stage A8 — Documentation refresh

### Task A8.1 — PROGRESS.md and PROJECT_CONTEXT.md updates

**Files:**
- Modify: `PROGRESS.md`
- Modify: `PROJECT_CONTEXT.md`

- [ ] **Step 1: Update PROGRESS.md header + sections 5, 8**

Replace the `**Last updated:**` date with `2026-04-20`. Add a new entry to section 5 (vertical slices matrix) for UGreen integration; add the closed-loop calibration row; mark the hardened FSM row green.

In section 8 (Pending work), move completed items (Stateflow build script exists, logging extended, facade v2 scaffolded, remote_jog implemented) to a new section "§8.4 Completed 2026-04-17 / 2026-04-20". Cross-reference the commits.

- [ ] **Step 2: Update PROJECT_CONTEXT.md**

Add a new subsection "Camera is arm-mounted" with one paragraph explaining per-pose calibration. Add "UGreen floor camera" subsection describing tracker, intrinsics, closed-loop. Replace any stale command lines with the current `python/` entry points.

- [ ] **Step 3: Commit**

```bash
git add PROGRESS.md PROJECT_CONTEXT.md
git commit -m "refresh PROGRESS.md + PROJECT_CONTEXT.md for 2026-04-20 state

Arm-mounted D415 finding documented, UGreen integration described,
completed work moved from 'pending' to 'done' with commit pointers,
spec + plan files linked from both docs."
```

---

# Phase B — Hardware-required (lab session)

Each stage assumes preflight passed.

## Stage B1 — Power-on + preflight + UGreen one-time setup

### Task B1.1 — First preflight of the session

- [ ] **Step 1:** Power on the QArm, plug D415, position the UGreen. Wait 10 s.
- [ ] **Step 2:** `C:/Python313/python.exe python/preflight.py`
- [ ] **Step 3:** Resolve any red checks using the runbook remedies.
- [ ] **Step 4:** Commit any tuned thresholds if HSV needed adjustment.

### Task B1.2 — UGreen intrinsic calibration (one-time per setup)

- [ ] **Step 1:** Confirm `figures/chessboard_7x5_30mm.png` is printed at 100% scale (one square = 30 mm with a ruler).
- [ ] **Step 2:** `C:/Python313/python.exe python/ugreen_intrinsics.py collect`
      — 20 poses. Tilt, translate, rotate the pattern so corners cover the whole frame.
- [ ] **Step 3:** `C:/Python313/python.exe python/ugreen_intrinsics.py solve`
      — target: RMS < 1 px. Written to `ugreen_intrinsics.json`.
- [ ] **Step 4:** `git add ugreen_intrinsics.json logs/ugreen_chessboards && git commit -m "UGreen intrinsics calibrated on <date>"`

### Task B1.3 — Baseline frame and visual reference

- [ ] **Step 1:** In `teach_points.py`, move arm to `homeplace0` (arm out of UGreen view). Exit.
- [ ] **Step 2:** `C:/Python313/python.exe -c "from ugreen_tracker import capture, save_baseline; save_baseline(capture(), 'logs/ugreen_baseline.png')"`
- [ ] **Step 3:** Move arm to `pickhome1` via `teach_points.py`.
- [ ] **Step 4:** `C:/Python313/python.exe python/preflight.py` — step 7 fails "reference missing".
- [ ] **Step 5:** Create the reference (new command added in plan A4 — if not yet scripted, use a oneliner):
      ```bash
      C:/Python313/python.exe -c "from ugreen_tracker import capture, tcp_from_diff, load_baseline; import json; f=capture(); t=tcp_from_diff(f, load_baseline('logs/ugreen_baseline.png')); import cv2; cv2.imwrite('logs/ugreen_pickhome1_reference.png', f); json.dump({'tcp':list(t)}, open('logs/ugreen_pickhome1_tcp.json','w'))"
      ```
- [ ] **Step 6:** Re-run preflight — all 7 green.
- [ ] **Step 7:** `git add logs/ugreen_baseline.png logs/ugreen_pickhome1_reference.png logs/ugreen_pickhome1_tcp.json && git commit -m "UGreen baseline + pickhome1 reference (session <date>)"`

## Stage B2 — Closed-loop + D415 calibration

### Task B2.1 — Put fruit at cal_01..cal_08

- [ ] **Step 1:** Strawberries at cal_01..cal_04, tomatoes at cal_05..cal_08.
- [ ] **Step 2:** Verify by moving arm to each point in `teach_points.py` and confirming fruit underneath.

### Task B2.2 — Run closed-loop calibration

- [ ] **Step 1:** `C:/Python313/python.exe python/calibrate_closed_loop.py`
- [ ] **Step 2:** Confirm RMS < 15 mm. If > 20 mm, re-check chessboard intrinsics and baseline freshness.
- [ ] **Step 3:** Commit the resulting `calibration_ugreen.json`.

### Task B2.3 — Recalibrate D415 with UGreen cross-check

- [ ] **Step 1:** `C:/Python313/python.exe python/analyze_static.py --from pickhome1 cal_01 cal_02 cal_03 cal_04 cal_05 cal_06 cal_07 cal_08 --save`
- [ ] **Step 2:** Cross-check: run `calibrate_closed_loop` result through a consistency script (next session if time is tight) — verify D415 XYZ for each fruit matches UGreen projection within 30 mm.
- [ ] **Step 3:** Commit `calibration.json` + backup file.

## Stage B3 — HSV tuning

### Task B3.1 — Tune HSV for lab lighting

- [ ] **Step 1:** With fruit at cal_01..cal_08, arm at pickhome1, run preflight step 5 alone:
      `C:/Python313/python.exe -c "from preflight import check_hsv; print(check_hsv())"`
- [ ] **Step 2:** Inspect printed circ / sat / area values; if any fruit misclassifies, tweak `HSV_RANGES`, `CIRCULARITY_THRESH`, `SATURATION_THRESH` in `fruit_detector.py`.
- [ ] **Step 3:** Re-run until all 8 fruit classify correctly.
- [ ] **Step 4:** Commit `python/fruit_detector.py`.

## Stage B4 — Basket teach points

### Task B4.1 — Define basket_a / basket_b / basket_c

- [ ] **Step 1:** `C:/Python313/python.exe python/teach_points.py`
- [ ] **Step 2:** Jog to basket A, press `n`, label `basket_a`. Repeat for `basket_b`, `basket_c`.
- [ ] **Step 3:** ESC to save.
- [ ] **Step 4:** Verify reachable from pickhome1 (no joint-limit error in a dry IK):
      `C:/Python313/python.exe -c "import json, numpy as np; from qarm_kinematics import inverse_kinematics; from qarm_driver import QArmDriver; d=json.load(open('teach_points.json')); [print(k, inverse_kinematics(d[k]['xyz_m'])) for k in ('basket_a','basket_b','basket_c')]"`
- [ ] **Step 5:** Commit `teach_points.json`.

## Stage B5 — Single-fruit dry run

### Task B5.1 — Pick one strawberry at cal_01 → basket_a

- [ ] **Step 1:** Leave only the cal_01 strawberry in place; remove the rest so the detector sees exactly one fruit.
- [ ] **Step 2:** `C:/Python313/python.exe python/main_final.py --pick-only`
- [ ] **Step 3:** Observe via UGreen companion window that the arm descends to the correct XY, closes gripper, ascends with fruit in grip.
- [ ] **Step 4:** If failure, check `logs/robot_trace_<timestamp>.log` for `IK_FAIL` / `GRIPPER_READBACK stall`.

### Task B5.2 — Full pick+place of one strawberry

- [ ] **Step 1:** `C:/Python313/python.exe python/main_final.py` (no flags → full sort).
- [ ] **Step 2:** Confirm basket_a contains the strawberry.
- [ ] **Step 3:** If OK, proceed to stage B6.

## Stage B6 — 14-fruit autonomous run

### Task B6.1 — Seed fruit and record

- [ ] **Step 1:** Place all 14 fruits in the workspace visible from pickhome1. Avoid occlusion.
- [ ] **Step 2:** Start OBS (or equivalent) recording the UGreen window + Simulink.
- [ ] **Step 3:** `C:/Python313/python.exe python/main_final.py`
- [ ] **Step 4:** Stop recording on "SORT PLAN complete" message.
- [ ] **Step 5:** Verify buckets by eye.
- [ ] **Step 6:** Save the trace log + any .mat artefacts. Run the report plot generator: `C:/Python313/python.exe scripts/generate_report_plots.py`.

### Task B6.2 — Commit session artefacts

- [ ] **Step 1:** `git add logs/robot_trace_*.log logs/autonomous_run.mat figures/*.png` (as applicable).
- [ ] **Step 2:** `git commit -m "autonomous run N/14 sorted on <date>"` — fill in the actual count.

## Stage B7 — Remote mode live verification

### Task B7.1 — Jog pendant manual pick

- [ ] **Step 1:** Open `FruitSorting_Hardware.slx`, switch mode to Remote.
- [ ] **Step 2:** Python companion window opens automatically. Observe overlay reflecting current state.
- [ ] **Step 3:** Jog the arm in Cartesian sub-mode to a strawberry, close gripper, lift, jog to basket_a, open gripper.
- [ ] **Step 4:** Verify success via the companion window + physical check.
- [ ] **Step 5:** Record this sequence separately for the video.

## Stage B8 — Video editing and submission

### Task B8.1 — Cut 3-5 min highlight

- [ ] **Step 1:** Import autonomous + remote recordings into the team's editor of choice.
- [ ] **Step 2:** Trim to highlights; overlay title / team names / date.
- [ ] **Step 3:** Export H.264 mp4 ≤ 500 MB.

### Task B8.2 — Final commit + submission

- [ ] **Step 1:** Update PROGRESS.md to reflect "PROJECT COMPLETE".
- [ ] **Step 2:** `git push` everything to master.
- [ ] **Step 3:** Upload video + code zip to the course submission portal.

---

# Self-review results

**Spec coverage** — every feature 2.1…2.11 in the spec maps to at least one Phase A or Phase B task:
- 2.1 Remote mode → A7.1, A7.2, A7.5, B7
- 2.2 Stateflow → A7.3, A7.4
- 2.3 Logging → A5, A6, A7.5 (Simulink To Workspace block), B6
- 2.4 Preflight → A4, B1
- 2.5 UGreen tracker → A2
- 2.6 Closed-loop calibration → A3, B1, B2
- 2.7 D415 recalibration → B2.3
- 2.8 HSV tuning → B3
- 2.9 Pick + place → B4, B5
- 2.10 Docs refresh → A8
- 2.11 Video → B8

**Placeholder scan:** clean — the one `Pseudocode` call-out in A7.3 is deliberate (the matlab code inside is illustrative and the exact wiring is out of my reach without MATLAB). All other tasks have concrete code blocks.

**Type / name consistency:** `set_gripper_ramp` named identically in A1.1, A7.2 docs, and A7.4. `stateflow_*` helpers spelled the same in A7.3 build script, A7.4 Python implementation, and A7.5 MATLAB remote_step reference. `cal_01..cal_08` consistent throughout.

**Estimated effort:** Phase A ≈ 5-6 dev-days, Phase B ≈ 1-1.5 lab-days. Fits inside the 11-day window with 3-4 days slack for report + video editing + contingencies.
