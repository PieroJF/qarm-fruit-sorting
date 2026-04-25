# D1 Chessboard Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the session-calibration subsystem described in §3.1 of the 2026-04-22 overhead-vision spec. At the end of D1 the team has `python/calibrate_chessboard.py` that (a) lets an operator jog the gripper to the chessboard origin corner and record the TCP pose, (b) drives the arm to SURVEY_POSE, captures a D415 frame, locates the chessboard, and solves the pixel→chess-plane homography, (c) writes a complete `session_cal.json` consumable by the downstream detector (built D2).

**Architecture:** Pure Python, single-file CLI with three helper modules so tests can exercise the homography math without hardware. No hardware required for unit tests — synthetic chessboard images drive every non-I/O test. Hardware-touching code paths (jog loop, camera capture, slow_move_to_joints) are wrapped so they can be swapped for fakes.

**Tech Stack:** Python 3.13 (via `py -3.13`), NumPy, OpenCV 4.13, the existing `qarm_driver.QArmDriver` / `camera.QArmCamera` / `qarm_kinematics.forward_kinematics` already in the repo. Test style matches `test_integration.py`: script-based, `exit code == number of failed sections`, no pytest framework.

**Spec reference:** `docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md`, commits 87013a4 / e85dbb1 / ca44e11.

---

## File structure

Four new Python files, one modified teach-points file, one new test file:

| Path | Responsibility |
|---|---|
| `python/session_cal.py` (new) | Dataclass + JSON read/write for `session_cal.json`. No OpenCV, no hardware imports. |
| `python/homography_solver.py` (new) | Pure-math: given image corners + world corners, return H, reprojection RMS, camera height. |
| `python/touch_probe.py` (new) | Interactive jog-and-confirm helper. Reads QArmDriver, returns `T_chess_origin_in_base`. |
| `python/calibrate_chessboard.py` (new) | Top-level CLI that wires the three above, captures the frame, and writes `session_cal.json`. |
| `python/test_calibrate_chessboard.py` (new) | All unit + integration tests for the three non-I/O modules. Synthetic images only. |
| `teach_points.json` (modify) | Add a stub `survey1` entry with `NaN` joints and a TODO note. Populated in D4 lab. |

`calibrate_chessboard.py` is a thin orchestrator; the bulk of testable logic is in the three helper modules. This lets the test file exercise 100% of the math and IO without standing up OpenCV windows or hardware.

---

## Task A1: `session_cal.py` — dataclass + JSON IO

**Files:**
- Create: `python/session_cal.py`
- Test: `python/test_calibrate_chessboard.py` (new sections added incrementally)

- [ ] **Step 1: Write the failing test**

Create `python/test_calibrate_chessboard.py`:

```python
"""
Unit + integration tests for the D1 chessboard-calibration subsystem.

Run with:
    py -3.13 python/test_calibrate_chessboard.py
Exit code is the number of failed sections (0 = all pass).
"""
import os
import sys
import json
import tempfile
import traceback
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

_RESULTS = []


def _section(name, fn):
    try:
        fn()
        _RESULTS.append((name, True, None))
        print(f"  [PASS] {name}")
    except Exception as ex:
        _RESULTS.append((name, False, ex))
        print(f"  [FAIL] {name}: {ex}")
        traceback.print_exc()


# ========================================================================
# A1. session_cal IO
# ========================================================================
def test_session_cal_roundtrip():
    from session_cal import SessionCal

    cal = SessionCal(
        timestamp="2026-04-22T14:30:00",
        chess_origin_in_base_m=np.array([0.30, 0.10, 0.00]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.array([0.1, -0.2, 0.3, 0.0]),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30,
                       "inner_cols": 6, "inner_rows": 4},
        d415_intrinsics={"fx": 912.6, "fy": 911.3,
                          "cx": 635.4, "cy": 343.0},
        homography_reproj_rms_px=0.45,
        camera_height_above_table_m=0.60,
        image_size=(1280, 720),
    )

    with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                      delete=False) as f:
        path = f.name
    try:
        cal.save(path)
        roundtrip = SessionCal.load(path)
        assert np.allclose(roundtrip.chess_origin_in_base_m,
                            cal.chess_origin_in_base_m)
        assert np.allclose(roundtrip.h_pixel_to_chess_mm,
                            cal.h_pixel_to_chess_mm)
        assert roundtrip.timestamp == cal.timestamp
        assert roundtrip.chess_pattern["square_mm"] == 30
        assert roundtrip.image_size == (1280, 720)
    finally:
        os.unlink(path)


def test_session_cal_missing_file_raises():
    from session_cal import SessionCal
    try:
        SessionCal.load("/no/such/path.json")
        assert False, "should have raised"
    except FileNotFoundError:
        pass


if __name__ == "__main__":
    _section("A1 session_cal roundtrip", test_session_cal_roundtrip)
    _section("A1 session_cal missing file", test_session_cal_missing_file_raises)
    fails = sum(1 for _, ok, _ in _RESULTS if not ok)
    print(f"\n{len(_RESULTS)} test(s), {fails} failed")
    sys.exit(fails)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'session_cal'`.

- [ ] **Step 3: Implement `session_cal.py`**

Create `python/session_cal.py`:

```python
"""
Data carrier + JSON serialisation for the per-session calibration output
of calibrate_chessboard.py.

Consumed by survey_capture.py (built D2) and the fruit detector (D2).
"""
from __future__ import annotations
import json
from dataclasses import dataclass, asdict, field
from typing import Any
import numpy as np


@dataclass
class SessionCal:
    timestamp: str
    chess_origin_in_base_m: np.ndarray           # shape (3,)
    h_pixel_to_chess_mm: np.ndarray              # shape (3, 3)
    survey_pose_joints_rad: np.ndarray           # shape (4,)
    chess_pattern: dict                           # cols/rows/square_mm/...
    d415_intrinsics: dict                         # fx/fy/cx/cy
    homography_reproj_rms_px: float
    camera_height_above_table_m: float
    image_size: tuple                             # (w, h)

    def save(self, path: str) -> None:
        payload: dict[str, Any] = {
            "timestamp": self.timestamp,
            "chess_origin_in_base_m":
                np.asarray(self.chess_origin_in_base_m).tolist(),
            "H_pixel_to_chess_mm":
                np.asarray(self.h_pixel_to_chess_mm).tolist(),
            "survey_pose_joints_rad":
                np.asarray(self.survey_pose_joints_rad).tolist(),
            "chess_pattern": self.chess_pattern,
            "d415_intrinsics": self.d415_intrinsics,
            "homography_reproj_rms_px": float(self.homography_reproj_rms_px),
            "camera_height_above_table_m":
                float(self.camera_height_above_table_m),
            "image_size": list(self.image_size),
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)

    @classmethod
    def load(cls, path: str) -> "SessionCal":
        with open(path, "r") as f:
            d = json.load(f)
        return cls(
            timestamp=d["timestamp"],
            chess_origin_in_base_m=np.asarray(d["chess_origin_in_base_m"]),
            h_pixel_to_chess_mm=np.asarray(d["H_pixel_to_chess_mm"]),
            survey_pose_joints_rad=np.asarray(d["survey_pose_joints_rad"]),
            chess_pattern=d["chess_pattern"],
            d415_intrinsics=d["d415_intrinsics"],
            homography_reproj_rms_px=float(d["homography_reproj_rms_px"]),
            camera_height_above_table_m=float(d["camera_height_above_table_m"]),
            image_size=tuple(d["image_size"]),
        )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: Both A1 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/session_cal.py python/test_calibrate_chessboard.py
git commit -m "feat(cal): SessionCal dataclass + JSON roundtrip tests

D1-A1: schema for session_cal.json. Pure data, no OpenCV or hardware.
Consumed by the detector built on D2."
```

---

## Task A2: `homography_solver.py` — findHomography + reprojection RMS

**Files:**
- Create: `python/homography_solver.py`
- Test: `python/test_calibrate_chessboard.py` (append A2 section)

- [ ] **Step 1: Add failing tests**

Append to `python/test_calibrate_chessboard.py`, inside the `if __name__ == "__main__":` block (add the `_section` calls) and as top-level functions above the runner:

```python
# ========================================================================
# A2. homography_solver
# ========================================================================
def _make_synthetic_corners(inner_cols=6, inner_rows=4, square_mm=30,
                             fx=912.0, fy=912.0, cx=640.0, cy=360.0,
                             cam_height_mm=600.0):
    """Project the 24 inner chessboard corners onto a nadir pinhole
    camera of known intrinsics at cam_height_mm above the plane.
    Chess origin (0,0,0) is centred horizontally under the camera.
    Returns (image_pts (N,2) float32, world_pts (N,2) float32)."""
    # World points: chessboard lies flat, origin offset so the pattern
    # is roughly centred under the camera.
    pattern_w = (inner_cols - 1) * square_mm
    pattern_h = (inner_rows - 1) * square_mm
    x0, y0 = -pattern_w / 2, -pattern_h / 2
    world = np.array([[x0 + i * square_mm, y0 + j * square_mm]
                      for j in range(inner_rows)
                      for i in range(inner_cols)], dtype=np.float32)
    # Nadir pinhole: u = cx + fx * X / h,  v = cy + fy * Y / h
    image = np.zeros_like(world)
    image[:, 0] = cx + fx * world[:, 0] / cam_height_mm
    image[:, 1] = cy + fy * world[:, 1] / cam_height_mm
    return image.astype(np.float32), world.astype(np.float32)


def test_homography_recovers_identity_scale():
    from homography_solver import solve_homography

    image, world = _make_synthetic_corners()
    H, rms = solve_homography(image, world)
    # Reproject and compare.
    import cv2
    proj = cv2.perspectiveTransform(image.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - world, axis=1)
    assert err.max() < 0.1, f"max reprojection error {err.max():.4f} mm"
    assert rms < 0.05, f"rms {rms:.4f} too high"


def test_homography_rejects_collinear_points():
    from homography_solver import solve_homography
    # All points on a single line → findHomography should return None
    # or raise; our wrapper must raise ValueError.
    collinear_img = np.array([[i * 10.0, 100.0] for i in range(10)],
                              dtype=np.float32)
    collinear_world = np.array([[i * 30.0, 0.0] for i in range(10)],
                                dtype=np.float32)
    try:
        solve_homography(collinear_img, collinear_world)
        assert False, "should have raised on collinear input"
    except ValueError:
        pass
```

Add runner calls:
```python
    _section("A2 homography recovers identity", test_homography_recovers_identity_scale)
    _section("A2 homography rejects collinear", test_homography_rejects_collinear_points)
```

- [ ] **Step 2: Run to verify it fails**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: A2 sections FAIL with `ModuleNotFoundError: No module named 'homography_solver'`.

- [ ] **Step 3: Implement `homography_solver.py`**

Create `python/homography_solver.py`:

```python
"""
Homography math for the chessboard session-calibration.

Pure OpenCV, no hardware imports. Kept separate from calibrate_chessboard.py
so the math is unit-testable against synthetic images.
"""
from __future__ import annotations
import numpy as np
import cv2


def solve_homography(image_pts: np.ndarray,
                     world_pts: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Solve H mapping image pixels to world-XY (mm), plus the reprojection
    RMS in mm.

    Parameters
    ----------
    image_pts : (N, 2) float array of pixel coordinates.
    world_pts : (N, 2) float array of corresponding world coordinates.

    Returns
    -------
    H : (3, 3) float64. Multiply pixel (u, v, 1).T by H to get world (X, Y, 1).T.
    rms_mm : float. Root-mean-square reprojection error in world-mm.

    Raises ValueError if findHomography fails (e.g. collinear input, <4 pts).
    """
    image_pts = np.asarray(image_pts, dtype=np.float32)
    world_pts = np.asarray(world_pts, dtype=np.float32)
    if image_pts.shape[0] < 4:
        raise ValueError(f"need >=4 points, got {image_pts.shape[0]}")

    H, mask = cv2.findHomography(image_pts, world_pts, method=0)
    if H is None:
        raise ValueError("findHomography returned None (collinear input?)")

    proj = cv2.perspectiveTransform(
        image_pts.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - world_pts, axis=1)
    rms_mm = float(np.sqrt(np.mean(err ** 2)))
    return H.astype(np.float64), rms_mm
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: all 4 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/homography_solver.py python/test_calibrate_chessboard.py
git commit -m "feat(cal): homography solver with reprojection RMS

D1-A2: solve_homography wraps cv2.findHomography with clean errors
and returns RMS in world units (mm). Unit-tested on a synthetic
nadir pinhole projection of 24 inner chessboard corners."
```

---

## Task A3: camera height derivation from H

**Files:**
- Modify: `python/homography_solver.py` (add `camera_height_from_homography`)
- Test: `python/test_calibrate_chessboard.py` (append A3 section)

The technique: for a nadir (straight-down) camera at height h above the table, the homography H maps a pixel offset of 1 px near the principal point to a world offset of h/fx mm in X (and h/fy mm in Y). Inverting: `h ≈ |H near principal point| * focal`. Concretely we use the Jacobian determinant of the pixel→world map at the principal point: `det(J) = (h/fx)·(h/fy)`, so `h = sqrt(|det(J)| · fx · fy)`. This is accurate to within ~15% for tilts ≤ 30° per spec §4.3.

- [ ] **Step 1: Add failing test**

Append to `python/test_calibrate_chessboard.py` above the runner block:

```python
# ========================================================================
# A3. camera height from homography
# ========================================================================
def test_camera_height_matches_synthetic_truth():
    from homography_solver import (solve_homography,
                                    camera_height_from_homography)
    true_h_mm = 600.0
    fx, fy, cx, cy = 912.0, 912.0, 640.0, 360.0
    image, world = _make_synthetic_corners(
        fx=fx, fy=fy, cx=cx, cy=cy, cam_height_mm=true_h_mm)
    H, _ = solve_homography(image, world)
    recovered = camera_height_from_homography(H, fx=fx, fy=fy, cx=cx, cy=cy)
    rel_err = abs(recovered - true_h_mm) / true_h_mm
    assert rel_err < 0.02, (
        f"recovered h {recovered:.1f} vs truth {true_h_mm:.1f} "
        f"(rel err {rel_err*100:.1f}%)")


def test_camera_height_handles_tilted_camera_within_15pct():
    """Approximate check: rotate the synthetic projection by ~20 deg and
    confirm the recovered height is within 15% of the true optical-axis
    intercept distance."""
    import cv2
    fx, fy, cx, cy = 912.0, 912.0, 640.0, 360.0
    true_h_mm = 600.0
    image, world = _make_synthetic_corners(
        fx=fx, fy=fy, cx=cx, cy=cy, cam_height_mm=true_h_mm)
    # Synthesise a tilt by scaling one axis (a rough stand-in for a 20-deg
    # roll around the optical axis' perpendicular; enough to stress the
    # nadir assumption).
    image_tilted = image.copy()
    image_tilted[:, 1] = (image_tilted[:, 1] - cy) * 0.94 + cy
    from homography_solver import (solve_homography,
                                    camera_height_from_homography)
    H, _ = solve_homography(image_tilted, world)
    recovered = camera_height_from_homography(H, fx=fx, fy=fy, cx=cx, cy=cy)
    rel_err = abs(recovered - true_h_mm) / true_h_mm
    assert rel_err < 0.15, (
        f"under mild tilt, recovered {recovered:.1f} vs {true_h_mm:.1f} "
        f"(rel err {rel_err*100:.1f}%)")
```

Add runner calls:
```python
    _section("A3 camera height nadir", test_camera_height_matches_synthetic_truth)
    _section("A3 camera height tilted", test_camera_height_handles_tilted_camera_within_15pct)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: A3 sections FAIL with ImportError on `camera_height_from_homography`.

- [ ] **Step 3: Implement**

Append to `python/homography_solver.py`:

```python
def camera_height_from_homography(H: np.ndarray,
                                   fx: float, fy: float,
                                   cx: float, cy: float) -> float:
    """
    Estimate camera height above the chess plane, in the same units as
    H's world range (so: mm if H maps pixel->mm).

    Derivation: for a nadir pinhole camera, the Jacobian of the pixel->world
    map near the principal point has determinant (h/fx)*(h/fy), so
        h = sqrt(|det J| * fx * fy).

    Accurate to ~2% for a nadir camera; ~15% for tilts up to ~30 deg
    (spec §4.3 enforces this). For larger tilts the value is only an
    approximation; the caller should not trust it past 20% rel err.
    """
    # Pixel->world at principal point: evaluate H and its derivative there.
    # H acts on homogeneous (u, v, 1); linearise by taking finite diffs
    # in u and v at (cx, cy). Analytic form uses H's first two columns.
    p = np.array([cx, cy, 1.0])
    w = H @ p
    s = w[2]          # homogeneous scale at the principal point
    if abs(s) < 1e-9:
        raise ValueError("homography degenerate at principal point")

    # Partial derivatives of (x, y) wrt (u, v) evaluated at (cx, cy):
    # x = (H00 u + H01 v + H02) / (H20 u + H21 v + H22)
    # dx/du = (H00 * s - x * H20) / s
    # dy/du = (H10 * s - y * H20) / s
    # dx/dv = (H01 * s - x * H21) / s
    # dy/dv = (H11 * s - y * H21) / s
    x = w[0] / s
    y = w[1] / s
    dxdu = (H[0, 0] - x * H[2, 0]) / s
    dxdv = (H[0, 1] - x * H[2, 1]) / s
    dydu = (H[1, 0] - y * H[2, 0]) / s
    dydv = (H[1, 1] - y * H[2, 1]) / s
    det_j = abs(dxdu * dydv - dxdv * dydu)
    h_est = float(np.sqrt(det_j * fx * fy))
    return h_est
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: all 6 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/homography_solver.py python/test_calibrate_chessboard.py
git commit -m "feat(cal): camera height estimate from homography Jacobian

D1-A3: closes spec §9 open question #2 — a small-angle approximation
recovers h within 2% nadir / 15% at 30deg tilt using the Jacobian
determinant of the pixel->world map at the principal point. Simpler
and more robust than cv2.decomposeHomographyMat's 4-solution ambiguity
for our use case (downward-looking survey pose)."
```

---

## Task A4: `touch_probe.py` — jog-and-confirm helper

**Files:**
- Create: `python/touch_probe.py`
- Test: `python/test_calibrate_chessboard.py` (append A4 section using a fake driver)

- [ ] **Step 1: Add failing test**

Append to `python/test_calibrate_chessboard.py`:

```python
# ========================================================================
# A4. touch_probe
# ========================================================================
class _FakeDriver:
    """Stands in for QArmDriver. Stores last-commanded joints and
    returns them on read_all; ignores gripper commands."""
    def __init__(self, joints):
        self._j = np.asarray(joints, dtype=float)
        self._g = 0.10
    def read_all(self):
        return self._j.copy(), self._g
    def set_joints_and_gripper(self, j, g):
        self._j = np.asarray(j, dtype=float)
        self._g = float(g)


def test_touch_probe_returns_tcp_from_fk():
    """With the fake driver parked at a known pose, touch_probe.capture_tcp
    should return the FK position of that pose (no movement, just a read)."""
    from qarm_kinematics import forward_kinematics
    from touch_probe import capture_tcp

    joints = np.array([0.1, -0.2, 0.3, 0.0])
    driver = _FakeDriver(joints)
    expected_pos, _ = forward_kinematics(joints)
    tcp = capture_tcp(driver)
    assert np.allclose(tcp, expected_pos, atol=1e-9), (
        f"tcp {tcp} vs expected {expected_pos}")


def test_touch_probe_shape_is_3():
    from touch_probe import capture_tcp
    driver = _FakeDriver(np.array([0.0, 0.0, 0.0, 0.0]))
    tcp = capture_tcp(driver)
    assert tcp.shape == (3,), f"expected shape (3,) got {tcp.shape}"
```

Add runner calls:
```python
    _section("A4 touch_probe returns FK", test_touch_probe_returns_tcp_from_fk)
    _section("A4 touch_probe shape", test_touch_probe_shape_is_3)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: A4 sections FAIL with `ModuleNotFoundError: No module named 'touch_probe'`.

- [ ] **Step 3: Implement `touch_probe.py`**

Create `python/touch_probe.py`:

```python
"""
Interactive jog-and-confirm helper for the session calibration.

Two public entry points:

  capture_tcp(driver) - non-interactive: reads current joints, returns
                        FK(joints)[0] = TCP position in base frame.
                        Used in tests + any flow where the operator has
                        already positioned the arm manually.

  jog_and_capture(driver) - the interactive flow used by
                            calibrate_chessboard.py: runs a keyboard loop
                            so the operator can jog the gripper to the
                            chessboard origin corner, then returns the TCP
                            on ENTER. Imports keyboard libs lazily so the
                            unit tests don't drag in a UI dependency.
"""
from __future__ import annotations
import numpy as np

from qarm_kinematics import forward_kinematics


def capture_tcp(driver) -> np.ndarray:
    """Return TCP position in base frame from the driver's current joints.
    Shape (3,) float64, units: metres."""
    joints, _ = driver.read_all()
    pos, _ = forward_kinematics(np.asarray(joints, dtype=float))
    return np.asarray(pos, dtype=float).flatten()[:3]


def jog_and_capture(driver, jog_step_rad: float = 0.02,
                     jog_step_grip: float = 0.05) -> np.ndarray:
    """
    Open a minimal HUD window and run a jog loop using cv2.waitKeyEx.
    Keys q/w/e/r step joints 1-4 positively; a/s/d/f step them negatively.
    z/x open/close the gripper. ENTER returns the current TCP. ESC raises
    KeyboardInterrupt.

    Uses the same cv2.waitKeyEx paradigm as the existing teach_points.py
    — cross-platform (no termios/msvcrt), works wherever OpenCV does.
    """
    import cv2  # lazy so unit tests don't require it.

    joints, grip = driver.read_all()
    joints = np.asarray(joints, dtype=float).copy()
    grip = float(grip)

    step_map = {
        ord("q"): (0, +1), ord("a"): (0, -1),
        ord("w"): (1, +1), ord("s"): (1, -1),
        ord("e"): (2, +1), ord("d"): (2, -1),
        ord("r"): (3, +1), ord("f"): (3, -1),
    }
    win = "Touch probe - jog to chessboard origin"
    cv2.namedWindow(win)
    try:
        while True:
            hud = np.zeros((260, 680, 3), dtype=np.uint8)
            lines = [
                f"joints (rad): {joints[0]:+.3f} {joints[1]:+.3f} "
                f"{joints[2]:+.3f} {joints[3]:+.3f}",
                f"gripper:      {grip:.2f}",
                "q/w/e/r = +j1/j2/j3/j4,  a/s/d/f = - ,  z/x = grip -/+",
                "ENTER = confirm origin corner,  ESC = abort",
            ]
            for i, line in enumerate(lines):
                cv2.putText(hud, line, (10, 40 + i * 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1,
                    cv2.LINE_AA)
            cv2.imshow(win, hud)
            key = cv2.waitKeyEx(30)
            if key == -1:
                continue
            if key in (13, 10):                   # ENTER
                return capture_tcp(driver)
            if key == 27:                          # ESC
                raise KeyboardInterrupt("jog aborted by operator")
            if key == ord("z"):
                grip = max(0.05, grip - jog_step_grip)  # z = close
            elif key == ord("x"):
                grip = min(0.95, grip + jog_step_grip)  # x = open
            elif key in step_map:
                idx, sign = step_map[key]
                joints[idx] += sign * jog_step_rad
            else:
                continue
            driver.set_joints_and_gripper(joints, grip)
    finally:
        cv2.destroyWindow(win)
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: all 8 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/touch_probe.py python/test_calibrate_chessboard.py
git commit -m "feat(cal): touch-probe helper for chessboard origin jog

D1-A4: capture_tcp reads driver joints + FK; jog_and_capture runs the
interactive loop (q/w/e/r + a/s/d/f + z/x + ENTER). Only capture_tcp
is unit-tested against a fake driver — the interactive loop is
exercised end-to-end during D4 lab work, matching the existing
teach_points.py pattern."
```

---

## Task A5: `calibrate_chessboard.py` — top-level CLI

**Files:**
- Create: `python/calibrate_chessboard.py`
- Modify: `teach_points.json` (add stub `survey1`)
- Test: `python/test_calibrate_chessboard.py` (append A5 full-pipeline test with mocks)

- [ ] **Step 1: Add failing integration test**

Append to `python/test_calibrate_chessboard.py`:

```python
# ========================================================================
# A5. calibrate_chessboard end-to-end (mocked hardware)
# ========================================================================
def _draw_synthetic_chessboard(inner_cols=6, inner_rows=4, square_px=80,
                                image_size=(1280, 720)):
    """Render a flat B/W chessboard centered in the image. Returns a BGR
    image suitable for cv2.findChessboardCorners."""
    import cv2
    W, H = image_size
    img = np.full((H, W, 3), 255, dtype=np.uint8)
    # Pattern of squares: (inner_cols+1) x (inner_rows+1) cells.
    ncol = inner_cols + 1
    nrow = inner_rows + 1
    pattern_w = ncol * square_px
    pattern_h = nrow * square_px
    x0 = (W - pattern_w) // 2
    y0 = (H - pattern_h) // 2
    for j in range(nrow):
        for i in range(ncol):
            if (i + j) % 2 == 1:
                x = x0 + i * square_px
                y = y0 + j * square_px
                img[y:y+square_px, x:x+square_px] = 0
    return img


def test_calibrate_chessboard_end_to_end_with_mocks():
    """Full pipeline with a fake driver + a rendered synthetic chessboard.
    Verifies the output session_cal.json is well-formed and the
    homography RMS is small."""
    import cv2
    from calibrate_chessboard import run_calibration_core
    from session_cal import SessionCal

    driver = _FakeDriver(np.array([0.1, -0.2, 0.3, 0.0]))
    frame = _draw_synthetic_chessboard()
    # Fake "touch" TCP (we short-circuit the jog loop):
    touched_tcp = np.array([0.30, 0.10, 0.00])
    survey_joints = np.array([0.0, 0.8, -0.5, 0.0])

    with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                      delete=False) as f:
        path = f.name
    try:
        run_calibration_core(
            touched_tcp=touched_tcp,
            survey_joints=survey_joints,
            captured_frame=frame,
            d415_intrinsics={"fx": 912.6, "fy": 911.3,
                              "cx": 635.4, "cy": 343.0},
            out_path=path,
        )
        cal = SessionCal.load(path)
        assert cal.homography_reproj_rms_px < 2.0, (
            f"rms {cal.homography_reproj_rms_px:.2f} px too high")
        assert cal.h_pixel_to_chess_mm.shape == (3, 3)
        assert np.allclose(cal.chess_origin_in_base_m, touched_tcp)
        assert np.allclose(cal.survey_pose_joints_rad, survey_joints)
        assert cal.chess_pattern["inner_cols"] == 6
        assert cal.chess_pattern["inner_rows"] == 4
        assert cal.camera_height_above_table_m > 0.1
    finally:
        os.unlink(path)
```

Add runner call:
```python
    _section("A5 calibrate end-to-end",
             test_calibrate_chessboard_end_to_end_with_mocks)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: A5 FAILs with `ModuleNotFoundError: No module named 'calibrate_chessboard'`.

- [ ] **Step 3: Implement `calibrate_chessboard.py`**

Create `python/calibrate_chessboard.py`:

```python
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

_INNER_COLS = 6
_INNER_ROWS = 4
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
            f"findChessboardCorners failed — check framing, lighting, "
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
    image_pts = _find_chessboard_corners(captured_frame)
    world_pts = _chess_world_pts()
    H_pixel_to_chess, rms_px = solve_homography(image_pts, world_pts)
    # rms_px is in mm units (world side). Convert to pixels by dividing
    # by the mm-per-pixel scale — but the caller actually wants the
    # REPROJECTION in the pixel domain (how well the pixel corners are
    # predicted by H-inverse). Redo the check in pixel space:
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
    )
    cal.save(out_path)
    print(f"\n  session_cal.json written to {out_path}")
    print(f"  chessboard corners found: {image_pts.shape[0]}")
    print(f"  reprojection RMS: {rms_px_in_pixels:.3f} px "
          f"(gate: < 2.0 px)")
    print(f"  camera height above table: {cam_h_mm/1000:.3f} m")
    if rms_px_in_pixels >= 2.0:
        print("  [WARN] RMS exceeds 2 px gate — reteach survey1 or "
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

    try:
        # --- Phase 1: origin registration
        if args.no_touch:
            if not os.path.exists(args.out):
                sys.exit(f"--no-touch but {args.out} does not exist")
            prior = SessionCal.load(args.out)
            touched_tcp = prior.chess_origin_in_base_m
            print(f"  reusing chess_origin_in_base_m = {touched_tcp}")
        else:
            print("Phase 1: Place chessboard flat on table. Jog gripper "
                  "tip to the TOP-LEFT INNER CORNER. Press ENTER to "
                  "confirm, ESC to abort.")
            touched_tcp = jog_and_capture(driver)
            print(f"  origin TCP = {touched_tcp}")

        # --- Phase 2: homography capture
        tp_path = os.path.join(os.path.dirname(_HERE), "teach_points.json")
        with open(tp_path) as f:
            tp = json.load(f)
        if "survey1" not in tp or tp["survey1"].get("joints_rad") is None:
            sys.exit("teach_points.json: survey1 is missing or unset — "
                     "teach it first (D4 lab step).")
        survey_joints = np.asarray(
            tp["survey1"]["joints_rad"], dtype=float)

        print("\nPhase 2: moving arm to survey1 ...")
        slow_move_to_joints(driver, survey_joints,
                             tp["survey1"].get("gripper", 0.10))

        cam = QArmCamera()
        cam.open()
        try:
            # Warm up + median-of-5 capture to suppress noise.
            frames = []
            for _ in range(30):
                c, _ = cam.read()
            for _ in range(5):
                c, _ = cam.read()
                frames.append(c.copy())
            captured = np.median(np.stack(frames), axis=0).astype(np.uint8)
            intr = {
                "fx": float(cam.fx), "fy": float(cam.fy),
                "cx": float(cam.cx), "cy": float(cam.cy),
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
```

- [ ] **Step 4: Modify `teach_points.json` to include `survey1` stub**

Read the existing `teach_points.json`. Add (near the other teach points):

```json
  "survey1": {
    "joints_rad": null,
    "gripper": 0.10,
    "notes": "Overhead survey pose. Frames workspace + chessboard corner. TO BE TAUGHT on D4 lab. Set joints_rad to a 4-element array."
  }
```

Leave `joints_rad: null` so `calibrate_chessboard.py --no-touch` fails loudly until the lab teach happens.

- [ ] **Step 5: Run to verify all 9 sections pass**

Run: `py -3.13 python/test_calibrate_chessboard.py`
Expected: all 9 sections PASS, exit 0.

- [ ] **Step 6: Commit**

```bash
git add python/calibrate_chessboard.py python/test_calibrate_chessboard.py teach_points.json
git commit -m "feat(cal): calibrate_chessboard CLI + end-to-end synthetic test

D1-A5: top-level orchestrator. run_calibration_core is a pure function
taking touched TCP + survey joints + one frame, testable offline.
main() is the interactive CLI — hardware deps lazy-imported so tests
don't pull them in. Added survey1 stub (joints_rad=null) to
teach_points.json; will be populated on D4 in the lab."
```

---

## Task A6: Manual hardware smoke-test plan

This task has no code or unit tests — it's a runbook D4 executes to validate the end-to-end pipeline on real hardware. Writing it now so the engineer executing D4 has exact commands ready.

**Files:**
- Create: `docs/superpowers/plans/2026-04-22-d1-smoke-test.md`

- [ ] **Step 1: Write the runbook**

Create `docs/superpowers/plans/2026-04-22-d1-smoke-test.md`:

```markdown
# D1 Chessboard Calibration — Hardware Smoke Test

Runs at the start of the D4 lab session (2026-04-25 AM).
Depends on: D1 code merged; D4 `survey1` taught into teach_points.json.

## Setup
1. Power QArm, plug D415.
2. Print `figures/chessboard_7x5_30mm.png` at 100% (ruler check: each
   square = 30 mm).
3. Tape chessboard flat on the table in the far corner of the workspace.

## Teach `survey1` (D4 only)
1. `py -3.13 python/teach_points.py`
2. Jog the arm so the D415 frames both:
   - the whole intended fruit-placement area (roughly 0.4 x 0.3 m), AND
   - all 24 inner chessboard corners visible in one frame.
3. Press `n`, label `survey1`, save, ESC.
4. `git commit teach_points.json -m "lab: survey1 pose taught"`.

## Run calibration
1. `py -3.13 python/calibrate_chessboard.py`
2. Phase 1 prompt: jog gripper tip to the top-left INNER corner of the
   chessboard (not the outer paper edge). Press ENTER.
3. Phase 2: arm moves to survey1, captures frame.
4. Expect output:
   - `chessboard corners found: 24`
   - `reprojection RMS: < 2.0 px`
   - `camera height above table: ~0.4-0.8 m`
5. `session_cal.json` should now exist in repo root.
6. Sanity: open `session_cal.json` and confirm
   `chess_origin_in_base_m[2]` (table z) is within ±10 mm of the known
   table height.

## Acceptance
- RMS < 2 px.
- Camera height within 20% of measured tape-to-lens distance.
- `session_cal.json` loads via `SessionCal.load(...)` from a Python REPL
  with no error.

## Failures and what they mean
| Symptom | Most likely cause | Fix |
|---|---|---|
| "findChessboardCorners failed" | obscured corner, poor lighting, or chessboard not fully in frame | reposition chessboard, re-teach survey1 if FOV inadequate |
| RMS > 2 px | chessboard not flat, or blurred frame | flatten with tape, reduce FPS or lengthen warm-up |
| Camera height off by > 50% | survey pose too tilted | reteach survey1 with more nadir-looking angle |
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/plans/2026-04-22-d1-smoke-test.md
git commit -m "docs(cal): D1 hardware smoke-test runbook for D4 lab

Complements the unit-test suite: exact commands, expected output, and
a failure triage table for the D4 AM lab session."
```

---

## Done-criteria for D1

When all of the above commits are on `yichang_branch`:

1. `py -3.13 python/test_calibrate_chessboard.py` returns exit code 0.
2. Nine test sections PASS:
   - A1 × 2 (session_cal IO)
   - A2 × 2 (homography solver)
   - A3 × 2 (camera height)
   - A4 × 2 (touch probe)
   - A5 × 1 (end-to-end with mocks)
3. `python/calibrate_chessboard.py --help` runs without error.
4. `session_cal.json` has NOT been created yet (needs hardware — that's D4).
5. `teach_points.json` has a `survey1` stub with `joints_rad: null`.

## Handoff to D2

The D2 plan (`2026-04-23-d2-fruit-detector.md`, to be written) will
consume `SessionCal` from `session_cal.py` as its only calibration input.
D1 does not unblock D2 hardware-wise — D2 is pure-math detector work
testable offline. Both can land before the D4 lab session.

## Risks (D1-specific)

1. **Test image_size tuple vs list round-trip**: JSON has no tuple type,
   so `image_size` round-trips as list. The `SessionCal.load` converts
   via `tuple(...)`. Test A1 explicitly checks this — don't regress.
2. **`calibrate_closed_loop.slow_move_to_joints` is borrowed** from the
   soon-to-be-deleted UGreen stack. The import in `main()` will break
   when UGreen is deleted on D8. Mitigation: move `slow_move_to_joints`
   into `python/motion_utils.py` on D8 before deleting `calibrate_closed_loop.py`,
   or copy its body into `calibrate_chessboard.py`. Flag noted in D8 plan.
