# Accurate-Pick Projection Rewrite + Plastic-Box Swap + Orientation Grasp — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the sorter pick accurately at any position in the workspace (not just x≈0.35 m), photograph only after the arm has settled, swap the strawberry class for a 17.5 × 11.5 cm plastic clamshell box, and (time permitting) rotate the wrist so long objects are grabbed across their short edge.

**Architecture:** At survey1 the D415 pose in the robot base frame is recovered once per session with `cv2.solvePnP` against the chessboard, removing the nadir-pinhole approximation and the hand-patched origin. At detection time each pixel is back-projected into a camera-frame ray, transformed into base frame, and intersected with the table plane at the fruit's top-of-object height (from D415 depth, with per-class fallback). `survey_capture.capture_fruits` gets a 1.5 s dwell plus a sanity check that refuses to continue if the arm did not reach survey1. The detector grows a `plastic_box` class (rectangular contour + white-label confirmation, strawberry class removed) and a `grasp_angle_rad` field computed per-object from `cv2.minAreaRect` — the controller converts that base-frame angle into the wrist joint via arm yaw at pick time plus a lab-calibrated gripper-mount offset.

**Tech Stack:** Python 3.13 · NumPy · OpenCV 4.x (`solvePnP`, `Canny`, `minAreaRect`, `findChessboardCornersSB`) · Quanser QArm SDK (no QUARC) · Intel RealSense D415 · pytest.

**Spec:** `docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md`

---

## Phase tiers and stop points

- **P0 (MUST ship first):** Tasks 0.1 – 0.8. Delivers accurate picks for the existing three classes at any workspace position, with settled-arm capture. Stop here, run lab acceptance (§3.6 of spec), only continue if it passes.
- **P1 (~2 days after P0):** Tasks 1.1 – 1.9. Strawberry removed, `plastic_box` added, tomato de-confused, UI / teach-points rewired.
- **P2 (if time allows):** Tasks 2.1 – 2.5. Orientation-aware wrist rotation for banana and plastic_box.

Each task block ends with a **commit** step. Do not skip TDD — every production change is preceded by a failing test that becomes passing.

---

## File Structure (what each file owns)

### New files
- `python/calibrate_extrinsics.py` — one public function `solve_survey1_extrinsics(frame_bgr, session_cal, K, dist_coeffs) -> tuple[np.ndarray, np.ndarray, float]` returning `(R_cam_in_base, C_cam_in_base, reproj_rms_px)`. Does the `cv2.solvePnP` call and chess-corner → base-frame bookkeeping. No hardware imports, unit-testable offline.
- `python/test_calibrate_extrinsics.py` — synthetic pinhole tests for the above.

### Modified files (one responsibility per edit, listed in execution order)
1. `python/session_cal.py` — add `cam_extrinsics_survey1: dict | None` field with serialize/deserialize (P0). Add `gripper_mount_offset_rad: float` field (P2).
2. `python/fruit_detector.py` — replace `pixel_to_base_frame` body with ray-plane projection (P0). Add `_detect_plastic_box`, strip strawberry, pass `boxes` into `_detect_tomato_contours`, add `grasp_angle_rad` field (P1/P2). Update `Detection` docstring.
3. `python/survey_capture.py` — add 1.5 s dwell + sanity check at top of `capture_fruits` (P0).
4. `python/calibrate_chessboard.py` — call `solve_survey1_extrinsics` from `run_calibration_core` after homography is computed; persist to session_cal (P0).
5. `python/sorting_controller.py` — `BASKETS` dict (P1), strawberry string references (P1), `_start_move` / `_ik_safe` gamma wiring (P2).
6. `python/picker_viewer.py` — remap `s` → `p`, update `_TYPE_COLORS`, update HUD hint string (P1).
7. `teach_points.json` — rename `placeberries` → `placebox`, re-teach coordinates in lab (P1).
8. `python/test_fruit_detector.py` — box detection tests, tomato-exclusion test, strawberry tests removed (P1). Box/banana angle tests (P2).
9. `python/test_integration.py`, `python/test_pick_single.py` — `'strawberry'` → `'plastic_box'` (P1); wrist-gamma assertion (P2).
10. `python/test_survey_capture.py` — settle-not-reached raises RuntimeError (P0).
11. `LAB_RUNBOOK.md` — add extrinsics step (P0), gripper-mount-offset calibration (P2).

---

# P0 — Projection rewrite + capture settle

## Task 0.1: Extend `SessionCal` with `cam_extrinsics_survey1`

**Files:**
- Modify: `python/session_cal.py`
- Test:  `python/test_session_cal.py` (create if missing)

- [ ] **Step 1: Write the failing test**

Create `python/test_session_cal.py` (or append if it exists):

```python
"""Round-trip SessionCal through JSON with cam_extrinsics_survey1."""
from __future__ import annotations
import os
import sys
import json
import tempfile
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from session_cal import SessionCal


def _base_cal() -> SessionCal:
    return SessionCal(
        timestamp="2026-04-24T12:00:00",
        chess_origin_in_base_m=np.array([0.30, 0.00, 0.00]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.array([0.0, 0.5, 0.7, 0.0]),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics={"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.254,
        image_size=(640, 480),
    )


def test_roundtrip_without_extrinsics():
    c = _base_cal()
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        path = f.name
    try:
        c.save(path)
        loaded = SessionCal.load(path)
        assert loaded.cam_extrinsics_survey1 is None
    finally:
        os.unlink(path)


def test_roundtrip_with_extrinsics():
    c = _base_cal()
    R = np.eye(3)
    t = np.array([0.0, 0.0, 0.3])
    c.cam_extrinsics_survey1 = {
        "R_cam_in_base": R.tolist(),
        "C_cam_in_base_m": t.tolist(),
        "reproj_rms_px": 0.42,
    }
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        path = f.name
    try:
        c.save(path)
        loaded = SessionCal.load(path)
        assert loaded.cam_extrinsics_survey1 is not None
        np.testing.assert_allclose(
            np.asarray(loaded.cam_extrinsics_survey1["R_cam_in_base"]), R)
        np.testing.assert_allclose(
            np.asarray(loaded.cam_extrinsics_survey1["C_cam_in_base_m"]), t)
        assert loaded.cam_extrinsics_survey1["reproj_rms_px"] == 0.42
    finally:
        os.unlink(path)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_session_cal.py -v`
Expected: FAIL — attribute `cam_extrinsics_survey1` does not exist.

- [ ] **Step 3: Add the field, update save/load**

In `python/session_cal.py`, modify the dataclass and methods:

```python
@dataclass
class SessionCal:
    timestamp: str
    chess_origin_in_base_m: np.ndarray
    h_pixel_to_chess_mm: np.ndarray
    survey_pose_joints_rad: np.ndarray
    chess_pattern: dict
    d415_intrinsics: dict
    homography_reproj_rms_px: float
    camera_height_above_table_m: float
    image_size: tuple
    # Added 2026-04-24: {"R_cam_in_base": 3x3 list, "C_cam_in_base_m": 3 list,
    # "reproj_rms_px": float} — None before first solvePnP run.
    cam_extrinsics_survey1: dict | None = None

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
            "cam_extrinsics_survey1": self.cam_extrinsics_survey1,
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
            cam_extrinsics_survey1=d.get("cam_extrinsics_survey1"),
        )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `py -3.13 -m pytest python/test_session_cal.py -v`
Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add python/session_cal.py python/test_session_cal.py
git commit -m "feat(session_cal): add cam_extrinsics_survey1 field"
```

---

## Task 0.2: `solve_survey1_extrinsics` — synthetic-pinhole unit test

**Files:**
- Create: `python/test_calibrate_extrinsics.py`
- Create: `python/calibrate_extrinsics.py` (stub only this task)

- [ ] **Step 1: Write the failing test**

`python/test_calibrate_extrinsics.py`:

```python
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
```

Also create an empty stub at `python/calibrate_extrinsics.py`:

```python
"""SolvePnP wrapper to recover D415 pose in base frame at survey1.
See docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md §3.1.
"""
from __future__ import annotations
import numpy as np


def solve_survey1_extrinsics(corners_2d, corners_3d_base, K, dist_coeffs):
    raise NotImplementedError
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_calibrate_extrinsics.py -v`
Expected: FAIL — `NotImplementedError`.

- [ ] **Step 3: Implement `solve_survey1_extrinsics`**

Replace body of `python/calibrate_extrinsics.py`:

```python
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
    # tvec is base origin expressed in camera frame.
    # Camera centre in base frame = -R_cam_in_base^T @ tvec
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `py -3.13 -m pytest python/test_calibrate_extrinsics.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/calibrate_extrinsics.py python/test_calibrate_extrinsics.py
git commit -m "feat(calibrate): solve_survey1_extrinsics via cv2.solvePnP"
```

---

## Task 0.3: Rewrite `pixel_to_base_frame` — ray-plane projection

**Files:**
- Modify: `python/fruit_detector.py:299-333` (replace `pixel_to_base_frame`)
- Modify: `python/test_fruit_detector.py` — add ray-plane unit test

- [ ] **Step 1: Write the failing test**

Append to `python/test_fruit_detector.py`:

```python
def test_pixel_to_base_frame_uses_extrinsics_when_present():
    """With known extrinsics (camera directly above chess origin at 0.3 m,
    nadir), pixel = principal point + 0 should land at chess origin XY,
    and z should match fruit_top_z."""
    import numpy as np
    from session_cal import SessionCal
    from fruit_detector import pixel_to_base_frame

    K = {"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0}
    origin = np.array([0.30, 0.00, 0.00])
    R_cam_in_base = np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
    ])  # camera looking down (+Z_cam = -Z_base)
    C_cam_in_base = np.array([0.30, 0.00, 0.30])

    cal = SessionCal(
        timestamp="t",
        chess_origin_in_base_m=origin,
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics=K,
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.30,
        image_size=(640, 480),
        cam_extrinsics_survey1={
            "R_cam_in_base": R_cam_in_base.tolist(),
            "C_cam_in_base_m": C_cam_in_base.tolist(),
            "reproj_rms_px": 0.1,
        },
    )
    # Pixel on the optical axis, fruit top 50 mm above the table.
    xyz = pixel_to_base_frame((320, 240), fruit_top_z_mm=50.0, session_cal=cal)
    np.testing.assert_allclose(xyz[:2], origin[:2], atol=1e-4)
    np.testing.assert_allclose(xyz[2], 0.050, atol=1e-4)


def test_pixel_to_base_frame_raises_when_extrinsics_missing():
    """Without extrinsics, projection must fail loudly; we will not
    silently fall back to the old nadir-pinhole formula."""
    import numpy as np
    import pytest
    from session_cal import SessionCal
    from fruit_detector import pixel_to_base_frame
    cal = SessionCal(
        timestamp="t",
        chess_origin_in_base_m=np.zeros(3),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics={"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.30,
        image_size=(640, 480),
    )
    with pytest.raises(ValueError):
        pixel_to_base_frame((320, 240), 50.0, cal)


def test_pixel_to_base_frame_offset_camera():
    """Non-nadir camera (5 cm lateral offset, 15° forward tilt) still
    projects a central pixel to a deterministic base point — verifies
    ray-plane math, not homography."""
    import numpy as np
    from session_cal import SessionCal
    from fruit_detector import pixel_to_base_frame

    K = {"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0}
    tilt = np.deg2rad(15.0)
    R_cam_in_base = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(np.pi + tilt), -np.sin(np.pi + tilt)],
        [0.0, np.sin(np.pi + tilt),  np.cos(np.pi + tilt)],
    ])
    C_cam_in_base = np.array([0.40, 0.05, 0.33])

    cal = SessionCal(
        timestamp="t",
        chess_origin_in_base_m=np.array([0.30, 0.00, 0.00]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics=K,
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.33,
        image_size=(640, 480),
        cam_extrinsics_survey1={
            "R_cam_in_base": R_cam_in_base.tolist(),
            "C_cam_in_base_m": C_cam_in_base.tolist(),
            "reproj_rms_px": 0.1,
        },
    )
    # Sanity: same pixel at different fruit heights must produce same XY
    # for a nadir camera, but DIFFERENT XY for this tilted camera. We only
    # check that the result is finite and finite.
    xyz_low  = pixel_to_base_frame((320, 240), 10.0, cal)
    xyz_high = pixel_to_base_frame((320, 240), 80.0, cal)
    assert np.all(np.isfinite(xyz_low))
    assert np.all(np.isfinite(xyz_high))
    assert not np.allclose(xyz_low[:2], xyz_high[:2])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v -k pixel_to_base_frame`
Expected: FAIL — old implementation ignores extrinsics.

- [ ] **Step 3: Rewrite `pixel_to_base_frame`**

In `python/fruit_detector.py`, replace lines 299-333 (whole function):

```python
def pixel_to_base_frame(center_px: tuple, fruit_top_z_mm: float,
                         session_cal) -> np.ndarray:
    """Convert (pixel, fruit_top_z_mm) to base-frame XYZ (metres).

    Uses the per-session camera extrinsics recovered at survey1 by
    cv2.solvePnP (see calibrate_extrinsics.solve_survey1_extrinsics).
    Back-projects the pixel into a camera-frame ray, transforms the ray
    into base frame using session_cal.cam_extrinsics_survey1, and
    intersects it with the plane z_base = fruit_top_z_mm / 1000.
    """
    extr = session_cal.cam_extrinsics_survey1
    if not extr:
        raise ValueError(
            "session_cal.cam_extrinsics_survey1 is missing — "
            "rerun calibrate_chessboard.py to solvePnP the survey pose")
    intr = session_cal.d415_intrinsics
    fx, fy = float(intr["fx"]), float(intr["fy"])
    cx_p, cy_p = float(intr["cx"]), float(intr["cy"])
    u, v = float(center_px[0]), float(center_px[1])

    ray_cam = np.array([(u - cx_p) / fx, (v - cy_p) / fy, 1.0],
                        dtype=np.float64)
    ray_cam /= np.linalg.norm(ray_cam)

    R_cam_in_base = np.asarray(extr["R_cam_in_base"], dtype=np.float64)
    C = np.asarray(extr["C_cam_in_base_m"], dtype=np.float64)
    ray_base = R_cam_in_base @ ray_cam

    target_z = float(fruit_top_z_mm) / 1000.0
    if abs(ray_base[2]) < 1e-6:
        raise ValueError("ray is parallel to table plane; no intersection")
    lam = (target_z - C[2]) / ray_base[2]
    if lam <= 0:
        raise ValueError("ray-plane intersection behind camera; invalid")
    return C + lam * ray_base
```

- [ ] **Step 4: Run tests**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v`
Expected: all three `pixel_to_base_frame*` tests PASS. Other fruit-detector tests may now fail because their fixtures lack extrinsics — that is expected and fixed in the next task.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(projection): ray-plane pixel_to_base_frame via extrinsics"
```

---

## Task 0.4: Update existing `test_fruit_detector.py` fixtures to include extrinsics

**Files:**
- Modify: `python/test_fruit_detector.py` (existing fixtures that construct SessionCal)

- [ ] **Step 1: Find every `SessionCal(...)` construction**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v`

Record which tests fail with the new `ValueError("…extrinsics…missing")`.

- [ ] **Step 2: Add a helper and reuse across tests**

At the top of `python/test_fruit_detector.py`, add:

```python
def _nadir_extrinsics(height_m: float = 0.30, origin_m=(0.30, 0.0, 0.0)):
    """Build a cam_extrinsics_survey1 dict for a perfectly nadir camera
    above (origin_m.xy, origin_m.z + height_m). Used to keep legacy
    detector tests close to their old behaviour."""
    import numpy as np
    R = np.array([[1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0],
                  [0.0, 0.0, -1.0]], dtype=np.float64)
    C = np.array([origin_m[0], origin_m[1], origin_m[2] + height_m],
                  dtype=np.float64)
    return {
        "R_cam_in_base": R.tolist(),
        "C_cam_in_base_m": C.tolist(),
        "reproj_rms_px": 0.1,
    }
```

- [ ] **Step 3: Thread it through every existing fixture**

For each SessionCal construction that previously relied on the nadir-pinhole formula, add:

```python
cal.cam_extrinsics_survey1 = _nadir_extrinsics(
    height_m=cal.camera_height_above_table_m,
    origin_m=tuple(cal.chess_origin_in_base_m.tolist()),
)
```

immediately after the `SessionCal(...)` call (or pass as kwarg if the dataclass is constructed directly).

- [ ] **Step 4: Run the full detector test suite**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add python/test_fruit_detector.py
git commit -m "test(detector): update fixtures to provide cam_extrinsics"
```

---

## Task 0.5: Wire `solve_survey1_extrinsics` into `calibrate_chessboard`

**Files:**
- Modify: `python/calibrate_chessboard.py`

- [ ] **Step 1: Read the full `run_calibration_core` function**

Run: `py -3.13 python/calibrate_chessboard.py --help`

Open `python/calibrate_chessboard.py`, locate `run_calibration_core`. Identify where `SessionCal(...)` is constructed (after the homography is solved).

- [ ] **Step 2: Extract chess corners (already computed) and intrinsics**

Inside `run_calibration_core`, after the homography has been solved and before `session_cal.save(...)`, add:

```python
from calibrate_extrinsics import solve_survey1_extrinsics

K_matrix = np.array([
    [intrinsics["fx"], 0.0, intrinsics["cx"]],
    [0.0, intrinsics["fy"], intrinsics["cy"]],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
dist = np.zeros(5, dtype=np.float64)  # D415 rectified by SDK

# corners_3d_base: same grid as _chess_world_pts(), shifted by origin
corners_2d = refined_corners  # whatever the existing variable is
origin = np.asarray(chess_origin_in_base_m, dtype=np.float64)
corners_3d_base = np.zeros((len(corners_2d), 3), dtype=np.float32)
for k, (col, row) in enumerate(
        (i, j) for j in range(_INNER_ROWS) for i in range(_INNER_COLS)):
    corners_3d_base[k] = [
        origin[0] + col * _SQUARE_MM / 1000.0,
        origin[1] + row * _SQUARE_MM / 1000.0,
        origin[2],
    ]

try:
    R_cam, C_cam, pnp_rms = solve_survey1_extrinsics(
        corners_2d=corners_2d.astype(np.float32),
        corners_3d_base=corners_3d_base,
        K=K_matrix, dist_coeffs=dist)
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
```

Then pass `cam_extrinsics_survey1=cam_extrinsics_survey1` into the `SessionCal(...)` constructor.

- [ ] **Step 3: Smoke-run the module's unit tests**

Run: `py -3.13 -m pytest python/test_calibrate_chessboard.py -v`
Expected: PASS — the existing core path should still work; tests will not exercise the new branch unless they supply intrinsics + corner data. If any test fails because it asserts on a `SessionCal(...)` positional-argument order, update the test.

- [ ] **Step 4: Commit**

```bash
git add python/calibrate_chessboard.py
git commit -m "feat(calibrate): call solve_survey1_extrinsics in run_calibration_core"
```

---

## Task 0.6: `capture_fruits` — 1.5 s settle + joint sanity check

**Files:**
- Modify: `python/survey_capture.py`
- Modify: `python/test_survey_capture.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_survey_capture.py`:

```python
def test_capture_fruits_raises_when_arm_not_settled(monkeypatch):
    """If read_all returns joints far from the target survey pose after
    the 1.5 s dwell, capture_fruits must raise RuntimeError."""
    import numpy as np
    import pytest
    from survey_capture import capture_fruits
    from session_cal import SessionCal

    class StuckDriver:
        def read_all(self):
            # Returns joints that differ from survey by 0.2 rad on joint 2.
            return np.array([0.0, 0.5, 0.9, 0.0]), 0.1

    class DummyCam:
        intrinsics = {"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0}
        def read(self):
            import numpy as np
            return np.full((480, 640, 3), 100, dtype=np.uint8), \
                   np.full((480, 640), 300, dtype=np.uint16)

    cal = SessionCal(
        timestamp="t",
        chess_origin_in_base_m=np.zeros(3),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.array([0.0, 0.5, 0.7, 0.0]),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics={"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.30,
        image_size=(640, 480),
    )

    # Stub out slow_move_to_joints (otherwise test would try to import driver).
    import survey_capture
    monkeypatch.setattr(survey_capture, "_settle_sleep_s", 0.01)
    monkeypatch.setattr(
        "calibrate_closed_loop.slow_move_to_joints",
        lambda *_args, **_kw: None,
        raising=False,
    )

    with pytest.raises(RuntimeError, match="settle"):
        capture_fruits(StuckDriver(), DummyCam(), cal)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_survey_capture.py::test_capture_fruits_raises_when_arm_not_settled -v`
Expected: FAIL — no settle logic yet.

- [ ] **Step 3: Add settle + sanity check to `capture_fruits`**

Edit `python/survey_capture.py`. At module top, after other constants, add:

```python
_SETTLE_SLEEP_S = 1.5           # fixed dwell after slow_move_to_joints
_SETTLE_JOINT_TOL_RAD = 0.05    # ~2.9° joint-norm tolerance
# Mutable monkeypatch hook for tests that don't want to sleep:
_settle_sleep_s = _SETTLE_SLEEP_S
```

Replace the beginning of `capture_fruits` (lines 114-117):

```python
    from calibrate_closed_loop import slow_move_to_joints  # existing helper
    slow_move_to_joints(driver, session_cal.survey_pose_joints_rad,
                        float(0.10))
    # Dwell so the position-mode PID can actually reach the target
    # (it has ~0.5-1.0 s tracking lag on big descents).
    time.sleep(_settle_sleep_s)
    # Sanity: confirm the arm actually got there; abort loudly if not.
    measured_joints, _ = driver.read_all()
    err = np.linalg.norm(
        np.asarray(measured_joints, dtype=float)
        - np.asarray(session_cal.survey_pose_joints_rad, dtype=float))
    if err > _SETTLE_JOINT_TOL_RAD:
        raise RuntimeError(
            f"arm failed to settle at survey1 "
            f"(joint-norm err {err:.3f} rad > {_SETTLE_JOINT_TOL_RAD})")
    color, depth, _ = _warmup_and_capture(camera)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `py -3.13 -m pytest python/test_survey_capture.py -v`
Expected: PASS. Existing survey_capture tests must also pass; if they rely on `capture_fruits` without a driver that supports `read_all`, update them to use a simple mock.

- [ ] **Step 5: Commit**

```bash
git add python/survey_capture.py python/test_survey_capture.py
git commit -m "feat(capture): 1.5s settle + sanity check before shooting"
```

---

## Task 0.7: `preflight.check_session_cal` — reject stale sessions

**Files:**
- Modify: `python/preflight.py`

- [ ] **Step 1: Locate `check_session_cal`**

Open `python/preflight.py` around lines 95+. The function currently validates age, RMS, and survey pose.

- [ ] **Step 2: Add an extrinsics gate**

Inside `check_session_cal` (after homography RMS check passes), add:

```python
if cal.cam_extrinsics_survey1 is None:
    return False, ("cam_extrinsics_survey1 missing in session_cal.json "
                    "— rerun calibrate_chessboard.py")
pnp_rms = cal.cam_extrinsics_survey1.get("reproj_rms_px", float("inf"))
if pnp_rms > 5.0:
    return False, (f"solvePnP RMS {pnp_rms:.2f} px > 5 px "
                    "— recalibrate")
```

- [ ] **Step 3: Run preflight offline**

Run: `py -3.13 python/preflight.py --offline`
Expected: the session_cal check may FAIL (because your current `session_cal.json` lacks extrinsics). This is the desired behaviour — it proves the gate works. Rerun calibration in lab.

- [ ] **Step 4: Commit**

```bash
git add python/preflight.py
git commit -m "feat(preflight): require cam_extrinsics_survey1 in session_cal"
```

---

## Task 0.8: P0 lab acceptance checkpoint

This is a **verification step, not a coding task**. Do not start P1 until this passes.

- [ ] **Step 1: Run calibration on real hardware**

1. Power QArm, open D415, place chessboard so the top-left inner corner is at a known base-frame position.
2. `py -3.13 python/calibrate_chessboard.py`
3. Verify the console prints `solvePnP: RMS = <= 2.0 px` and `camera at base XYZ = [.., .., ..]` that matches tape-measured camera height ±2 cm.

- [ ] **Step 2: Verify pick accuracy at three positions**

1. Place a tomato at each of three X positions on the same Y row: `x = 0.25 m`, `x = 0.35 m`, `x = 0.45 m`.
2. Run `py -3.13 python/main_final.py`. In the picker, click each tomato in turn.
3. Measure the offset between gripper tip and fruit centre for each pick. **Acceptance: all three ≤ 5 mm.**

- [ ] **Step 3: Verify capture settle**

1. Click 'r' in the picker — the arm should move to survey1, sit for 1.5 s, then refresh.
2. Unplug or jam the arm mid-move; press 'r' — the picker should log `RuntimeError: arm failed to settle at survey1 ...` and continue running (user can re-press 'r').

- [ ] **Step 4: Commit lab notes**

Append findings to `docs/PROGRESS.md` (one short paragraph + numbers). Commit:

```bash
git add docs/PROGRESS.md
git commit -m "docs(progress): P0 lab acceptance results"
```

**Stop here. Do not proceed to P1 until Step 2 measurements are ≤ 5 mm.**

---

# P1 — plastic_box swap + tomato de-confusion

## Task 1.1: Remove `strawberry` class from detector + HSV + tests

**Files:**
- Modify: `python/fruit_detector.py`
- Modify: `python/test_fruit_detector.py`
- Modify: `python/sorting_controller.py`
- Modify: `python/test_integration.py`, `python/test_pick_single.py`

- [ ] **Step 1: Search for all `"strawberry"` references**

Run: `py -3.13 -c "import subprocess; subprocess.run(['grep', '-rn', 'strawberry', 'python/'])"`

Or using Windows PowerShell: `Select-String -Path python/*.py -Pattern strawberry`.

- [ ] **Step 2: Delete `_detect_strawberry_contours`, `_taper_score`, `_has_green_in_top_strip`**

In `python/fruit_detector.py`, remove:
- `_STRAWBERRY_MIN_AREA`, `_STRAWBERRY_MAX_AREA`, `_STRAWBERRY_MIN_CALYX` constants
- `_taper_score` function
- `_has_green_in_top_strip` function
- `_detect_strawberry_contours` function
- `"strawberry"` entries in `HSV_RANGES` and `_FRUIT_TOP_Z_MM`
- The `("strawberry", _detect_strawberry_contours)` tuple from `detect_fruits`' `per_class` list.

Update the `Detection.fruit_type` docstring to `"banana" | "tomato" | "plastic_box"`.

- [ ] **Step 3: Update controller + tests**

In `python/sorting_controller.py`:
- Remove `'strawberry'` from `BASKETS`.
- Update any docstring / error message that enumerates `'banana', 'tomato', 'strawberry'`.

In `python/test_integration.py` and `python/test_pick_single.py`:
- Rename `'strawberry'` to `'plastic_box'` in any fruit type strings.
- Delete any test that is strictly strawberry-specific (e.g., tests that verify calyx detection).

In `python/test_fruit_detector.py`:
- Delete `test_detect_strawberry_*` tests.
- Delete any synthetic-strawberry helpers.

- [ ] **Step 4: Run the whole Python test suite**

Run: `py -3.13 -m pytest python/ -v`
Expected: PASS for everything that does not depend on plastic_box (not implemented yet). No residual `'strawberry'` mentions.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/sorting_controller.py \
        python/test_fruit_detector.py python/test_integration.py \
        python/test_pick_single.py
git commit -m "refactor(detector): drop strawberry class"
```

---

## Task 1.2: Add `grasp_angle_rad` to `Detection` (default 0.0)

**Files:**
- Modify: `python/fruit_detector.py`
- Modify: `python/test_fruit_detector.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_fruit_detector.py`:

```python
def test_detection_has_grasp_angle_default_zero():
    import numpy as np
    from fruit_detector import Detection
    d = Detection(
        fruit_type="banana",
        center_px=(100, 100),
        center_base_m=np.array([0.35, 0.00, 0.03]),
        confidence=0.8,
        area_px=5000,
        bbox=(80, 80, 40, 40),
    )
    assert d.grasp_angle_rad == 0.0
    dct = d.to_dict()
    assert "grasp_angle_rad" in dct
    assert dct["grasp_angle_rad"] == 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_fruit_detector.py::test_detection_has_grasp_angle_default_zero -v`
Expected: FAIL (`grasp_angle_rad` not an attribute).

- [ ] **Step 3: Add field + serialise**

In `python/fruit_detector.py`:

```python
@dataclass
class Detection:
    fruit_type: str              # "banana" | "tomato" | "plastic_box"
    center_px: tuple             # (cx, cy) in image pixels
    center_base_m: np.ndarray    # shape (3,) XYZ in robot base frame, metres
    confidence: float            # 0..1
    area_px: int
    bbox: tuple                  # (x, y, w, h) — opencv convention
    grasp_angle_rad: float = 0.0 # base-frame closing-axis angle; set by P2

    def to_dict(self) -> dict[str, Any]:
        return {
            "fruit_type": self.fruit_type,
            "center_px": list(self.center_px),
            "center_base_m": np.asarray(self.center_base_m).tolist(),
            "confidence": float(self.confidence),
            "area_px": int(self.area_px),
            "bbox": list(self.bbox),
            "grasp_angle_rad": float(self.grasp_angle_rad),
        }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `py -3.13 -m pytest python/test_fruit_detector.py::test_detection_has_grasp_angle_default_zero -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(detector): add grasp_angle_rad to Detection"
```

---

## Task 1.3: `_detect_plastic_box` — synthetic rectangle unit test

**Files:**
- Modify: `python/fruit_detector.py`
- Modify: `python/test_fruit_detector.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_fruit_detector.py`:

```python
def _draw_box_bgr(size=(640, 480), center=(320, 240),
                   rect_px=(200, 130), angle_deg=0.0):
    """Draw a synthetic plastic-box-like BGR image: transparent body is a
    dim mid-grey rectangle; white label patches sit in three places."""
    import cv2
    import numpy as np
    img = np.full((size[1], size[0], 3), 180, dtype=np.uint8)  # off-white table
    rect = ((center[0], center[1]), (rect_px[0], rect_px[1]), angle_deg)
    box_pts = cv2.boxPoints(rect).astype(np.int32)
    cv2.polylines(img, [box_pts], isClosed=True, color=(80, 80, 80),
                   thickness=3)
    # Three label patches — 28x22 px each, inside the box
    for ox, oy in [(-60, -40), (60, -40), (0, 40)]:
        cv2.rectangle(img,
                       (center[0] + ox - 14, center[1] + oy - 11),
                       (center[0] + ox + 14, center[1] + oy + 11),
                       (255, 255, 255), -1)
    return img


def test_detect_plastic_box_axis_aligned():
    """Axis-aligned synthetic box should produce exactly one detection
    with fruit_type='plastic_box'."""
    from fruit_detector import _detect_plastic_box
    bgr = _draw_box_bgr(angle_deg=0.0)
    hits = _detect_plastic_box(bgr)
    # Expect exactly one candidate near image centre.
    assert len(hits) == 1
    (cx, cy), area, bbox, conf, (rect_cx, rect_cy), (rect_w, rect_h), \
        rect_angle = hits[0]
    assert abs(cx - 320) < 10
    assert abs(cy - 240) < 10
    assert conf > 0.5


def test_detect_plastic_box_rotated_45():
    from fruit_detector import _detect_plastic_box
    bgr = _draw_box_bgr(angle_deg=45.0)
    hits = _detect_plastic_box(bgr)
    assert len(hits) >= 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v -k plastic_box`
Expected: FAIL — `_detect_plastic_box` does not exist.

- [ ] **Step 3: Implement `_detect_plastic_box`**

In `python/fruit_detector.py`, insert before `detect_fruits`:

```python
_BOX_MIN_AREA_PX = 15000
_BOX_MAX_AREA_PX = 60000
_BOX_MIN_ASPECT = 1.3
_BOX_MAX_ASPECT = 1.8
_BOX_WHITE_V_MIN = 200
_BOX_WHITE_S_MAX = 40
_BOX_WHITE_MIN_RATIO = 0.08


def _detect_plastic_box(bgr: np.ndarray) -> list:
    """Detect 17.5x11.5 cm transparent plastic box via rectangular
    contour + white-label confirmation.

    Returns hits: list of
      ((cx, cy), area_px, (bx, by, bw, bh), confidence,
       (rect_cx, rect_cy), (rect_w, rect_h), rect_angle_deg)
    The rotated-rect tuple is used by P2 (orientation) and by the tomato
    de-confusion pass to mask these regions out of the red HSV mask.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 150)
    # Thicken edges so clamshell rim contours close cleanly.
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hits = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < _BOX_MIN_AREA_PX or area > _BOX_MAX_AREA_PX:
            continue
        (rect_cx, rect_cy), (rect_w, rect_h), rect_angle = cv2.minAreaRect(c)
        if min(rect_w, rect_h) < 1.0:
            continue
        aspect = max(rect_w, rect_h) / min(rect_w, rect_h)
        if aspect < _BOX_MIN_ASPECT or aspect > _BOX_MAX_ASPECT:
            continue

        # White-label confirmation: mask the rotated ROI, count pixels
        # where V > 200 and S < 40.
        mask_roi = np.zeros(gray.shape, dtype=np.uint8)
        box_pts = cv2.boxPoints(((rect_cx, rect_cy), (rect_w, rect_h),
                                   rect_angle)).astype(np.int32)
        cv2.drawContours(mask_roi, [box_pts], 0, 255, -1)
        roi_pixels = hsv[mask_roi > 0]
        if roi_pixels.size == 0:
            continue
        white_ratio = float(np.mean(
            (roi_pixels[:, 2] > _BOX_WHITE_V_MIN)
            & (roi_pixels[:, 1] < _BOX_WHITE_S_MAX)))
        if white_ratio < _BOX_WHITE_MIN_RATIO:
            continue

        bx, by, bw, bh = cv2.boundingRect(c)
        # Confidence: how well aspect matches 17.5/11.5 = 1.52, scaled by
        # white_ratio up to a cap.
        aspect_score = 1.0 - min(1.0, abs(aspect - 1.52) / 0.3)
        conf = float(0.5 * aspect_score
                      + 0.5 * min(1.0, white_ratio / 0.20))
        hits.append((
            (int(rect_cx), int(rect_cy)),
            int(area),
            (int(bx), int(by), int(bw), int(bh)),
            conf,
            (float(rect_cx), float(rect_cy)),
            (float(rect_w), float(rect_h)),
            float(rect_angle),
        ))
    return hits
```

- [ ] **Step 4: Run tests**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v -k plastic_box`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(detector): add _detect_plastic_box rectangular + label"
```

---

## Task 1.4: `_detect_tomato_contours` — exclude box regions

**Files:**
- Modify: `python/fruit_detector.py`
- Modify: `python/test_fruit_detector.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_fruit_detector.py`:

```python
def test_tomato_excluded_inside_box_rect():
    """Red blob inside a detected box's rotated rect must not produce
    a tomato Detection."""
    import numpy as np
    import cv2
    from fruit_detector import _detect_tomato_contours
    img = np.full((480, 640, 3), 180, dtype=np.uint8)
    # Red blob in centre
    cv2.circle(img, (320, 240), 40, (0, 0, 200), -1)
    # "Detected box" rotated rect covers the blob
    box_rects = [((320.0, 240.0), (200.0, 130.0), 0.0)]
    hits = _detect_tomato_contours(img, box_rects=box_rects)
    assert hits == []

    # Control: no box => tomato is detected
    hits_bare = _detect_tomato_contours(img, box_rects=())
    assert len(hits_bare) >= 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_fruit_detector.py::test_tomato_excluded_inside_box_rect -v`
Expected: FAIL — current signature does not accept `box_rects`.

- [ ] **Step 3: Update `_detect_tomato_contours`**

Replace the function signature + top of body in `python/fruit_detector.py`:

```python
def _detect_tomato_contours(bgr: np.ndarray, box_rects: tuple = ()) -> list:
    """Detect tomatoes (red, round). `box_rects` is a sequence of
    (rect_cx, rect_cy), (rect_w, rect_h), rect_angle tuples from
    _detect_plastic_box; the matching rotated regions are blanked out on
    the red HSV mask before contour detection, so red strawberries
    visible through a transparent box do not become false tomatoes."""
    mask = hsv_mask(bgr, HSV_RANGES["tomato"])
    for rect in box_rects:
        pts = cv2.boxPoints(rect).astype(np.int32)
        cv2.drawContours(mask, [pts], 0, 0, -1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    # ... existing loop body unchanged ...
```

- [ ] **Step 4: Run tests**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v -k tomato`
Expected: PASS (all tomato tests, including new exclusion test).

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(detector): tomato excludes rotated box rects"
```

---

## Task 1.5: `detect_fruits` — orchestrate `box → banana → tomato(exclude)`

**Files:**
- Modify: `python/fruit_detector.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_fruit_detector.py`:

```python
def test_detect_fruits_runs_box_first_and_excludes_tomato():
    """A synthetic scene with a box containing a red blob + a standalone
    tomato should yield one plastic_box and one tomato, no strawberries."""
    import numpy as np
    import cv2
    from fruit_detector import detect_fruits
    from session_cal import SessionCal

    img = np.full((480, 640, 3), 180, dtype=np.uint8)
    # Box outline + white label patches in left half
    cv2.rectangle(img, (120, 180), (320, 310), (80, 80, 80), 3)
    for cx_, cy_ in [(160, 210), (280, 210), (220, 285)]:
        cv2.rectangle(img, (cx_ - 14, cy_ - 11), (cx_ + 14, cy_ + 11),
                       (255, 255, 255), -1)
    # Red blob inside box (simulates strawberries visible through film)
    cv2.circle(img, (220, 250), 18, (0, 0, 200), -1)
    # Standalone tomato on the right
    cv2.circle(img, (480, 240), 34, (0, 0, 200), -1)

    depth = np.full((480, 640), 300, dtype=np.uint16)
    cal = SessionCal(
        timestamp="t",
        chess_origin_in_base_m=np.array([0.30, 0.00, 0.00]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics={"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.30,
        image_size=(640, 480),
        cam_extrinsics_survey1=_nadir_extrinsics(0.30, (0.30, 0.00, 0.00)),
    )
    dets = detect_fruits(img, depth, cal)
    types = [d.fruit_type for d in dets]
    assert types.count("plastic_box") == 1
    assert types.count("tomato") == 1
    assert "strawberry" not in types
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_fruit_detector.py::test_detect_fruits_runs_box_first_and_excludes_tomato -v`
Expected: FAIL — box detection not wired into `detect_fruits`.

- [ ] **Step 3: Rewrite `detect_fruits`**

Replace the body of `detect_fruits` in `python/fruit_detector.py`:

```python
def detect_fruits(color_bgr: np.ndarray, depth_mm: np.ndarray,
                   session_cal) -> list[Detection]:
    """Top-level detector. Returns every object found with confidence
    >= CONFIDENCE_MIN. Uses D415 depth when available+plausible; falls
    back to per-type default heights otherwise (see _resolve_top_z).

    Processing order: plastic_box first (produces rotated rects for
    tomato exclusion), then banana, then tomato(exclude=boxes).
    """
    results: list[Detection] = []
    box_rects: list = []

    # ---- plastic_box ----
    for (cx, cy), area, bbox, conf, rect_ctr, rect_sz, rect_angle \
            in _detect_plastic_box(color_bgr):
        if conf < CONFIDENCE_MIN:
            continue
        top_z_mm = _resolve_top_z(
            "plastic_box", depth_mm, (cx, cy), session_cal)
        try:
            xyz_base = pixel_to_base_frame(
                (cx, cy), top_z_mm, session_cal)
        except ValueError:
            continue
        results.append(Detection(
            fruit_type="plastic_box",
            center_px=(int(cx), int(cy)),
            center_base_m=xyz_base,
            confidence=float(conf),
            area_px=int(area),
            bbox=tuple(int(v) for v in bbox),
            grasp_angle_rad=0.0,   # P2 will compute
        ))
        box_rects.append((rect_ctr, rect_sz, rect_angle))

    # ---- banana ----
    for (cx, cy), area, bbox, conf in _detect_banana_contours(color_bgr):
        if conf < CONFIDENCE_MIN:
            continue
        top_z_mm = _resolve_top_z(
            "banana", depth_mm, (cx, cy), session_cal)
        try:
            xyz_base = pixel_to_base_frame(
                (cx, cy), top_z_mm, session_cal)
        except ValueError:
            continue
        results.append(Detection(
            fruit_type="banana",
            center_px=(int(cx), int(cy)),
            center_base_m=xyz_base,
            confidence=float(conf),
            area_px=int(area),
            bbox=tuple(int(v) for v in bbox),
            grasp_angle_rad=0.0,
        ))

    # ---- tomato (excluding box regions) ----
    for (cx, cy), area, bbox, conf in _detect_tomato_contours(
            color_bgr, box_rects=tuple(box_rects)):
        if conf < CONFIDENCE_MIN:
            continue
        top_z_mm = _resolve_top_z(
            "tomato", depth_mm, (cx, cy), session_cal)
        try:
            xyz_base = pixel_to_base_frame(
                (cx, cy), top_z_mm, session_cal)
        except ValueError:
            continue
        results.append(Detection(
            fruit_type="tomato",
            center_px=(int(cx), int(cy)),
            center_base_m=xyz_base,
            confidence=float(conf),
            area_px=int(area),
            bbox=tuple(int(v) for v in bbox),
            grasp_angle_rad=0.0,
        ))

    return results
```

Also add `"plastic_box": 20` to `_FRUIT_TOP_Z_MM`:

```python
_FRUIT_TOP_Z_MM = {
    "banana": 25,
    "tomato": 50,
    "plastic_box": 40,   # clamshell stands ~40 mm; lab-verify
}
```

- [ ] **Step 4: Run the full detector suite**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(detector): detect_fruits orchestrates box/banana/tomato"
```

---

## Task 1.6: Controller + basket updates for `plastic_box`

**Files:**
- Modify: `python/sorting_controller.py:42-46` (BASKETS)
- Modify: `python/sorting_controller.py:179, 243` (strings)

- [ ] **Step 1: Write the failing test**

Append to `python/test_integration.py`:

```python
def test_basket_for_plastic_box_present():
    from sorting_controller import FruitSortingController
    c = FruitSortingController(MockQArm(), pick_only=False)
    assert "plastic_box" in c.BASKETS
    assert list(c.BASKETS["plastic_box"]) == [0.0, 0.0, 0.0]
    assert "strawberry" not in c.BASKETS
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_integration.py::test_basket_for_plastic_box_present -v`
Expected: FAIL.

- [ ] **Step 3: Update `BASKETS`**

In `python/sorting_controller.py`, replace lines 42-46:

```python
    # Lab-taught basket positions (base frame, metres).
    # plastic_box is a PLACEHOLDER — IK reachability check will reject
    # it at runtime until teach_points.json.placebox is filled in.
    BASKETS = {
        'banana':      np.array([-0.30, -0.20, 0.05]),
        'tomato':      np.array([ 0.00, -0.35, 0.05]),
        'plastic_box': np.array([ 0.00,  0.00, 0.00]),
    }
```

And update the docstring / default fallback on line 243:

```python
self.target_basket = self.BASKETS.get(basket_name, self.BASKETS['tomato'])
```

(unchanged — the key lookup just needs `plastic_box` in the dict). Update the `pick_single` docstring on line 179 to list `'banana', 'tomato', 'plastic_box'`.

- [ ] **Step 4: Run tests**

Run: `py -3.13 -m pytest python/test_integration.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/sorting_controller.py python/test_integration.py
git commit -m "feat(controller): plastic_box basket + docs, drop strawberry"
```

---

## Task 1.7: Remap `s` key → `p`, update HUD

**Files:**
- Modify: `python/picker_viewer.py`

- [ ] **Step 1: Update key bindings, colour, and HUD text**

In `python/picker_viewer.py`:

```python
_TYPE_COLORS = {
    "banana":      (0, 255, 255),    # yellow
    "tomato":      (0, 0, 255),      # red
    "plastic_box": (200, 100, 255),  # pink-magenta (reusing old strawberry hue)
}
```

Replace the HUD help string in `_hud_text`:

```python
    return (f"{n_fruits} fruits  |  residual {r} mm  |  "
            f"mode: {mode}  |  click / b t p / r / ESC")
```

Replace the key-dispatch block (line 233-236):

```python
        elif key in (ord('b'), ord('t'), ord('p')):
            ftype = {ord('b'): 'banana',
                       ord('t'): 'tomato',
                       ord('p'): 'plastic_box'}[key]
```

- [ ] **Step 2: Update any `test_picker_viewer.py` strings**

Search: `Select-String -Path python/test_picker_viewer.py -Pattern "strawberry|'s'"`

Replace any `'strawberry'` → `'plastic_box'` and any `ord('s')` → `ord('p')` in test expectations.

- [ ] **Step 3: Run tests**

Run: `py -3.13 -m pytest python/test_picker_viewer.py -v`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "feat(ui): remap s->p, plastic_box HUD"
```

---

## Task 1.8: `teach_points.json` — rename `placeberries` → `placebox`

**Files:**
- Modify: `teach_points.json`

- [ ] **Step 1: Edit the JSON**

In `teach_points.json`:
- Rename the key `placeberries` to `placebox`.
- Leave coordinates as-is for now; they will be re-taught in lab during the P1 acceptance step.

- [ ] **Step 2: Check no code still references `placeberries`**

Run: `Select-String -Path python/*.py, *.md -Pattern placeberries`

If any hits: update to `placebox`.

- [ ] **Step 3: Commit**

```bash
git add teach_points.json
git commit -m "chore(teach): rename placeberries -> placebox (lab re-teach pending)"
```

---

## Task 1.9: P1 lab acceptance checkpoint

- [ ] **Step 1: Teach the new `placebox` point**

In the lab, jog the arm so the gripper is positioned above the new plastic-box basket. Record joints + XYZ. Edit `teach_points.json.placebox` to match. Commit that single change.

- [ ] **Step 2: Verify detection in lab lighting**

1. Place box + tomato + banana on the table within reach.
2. `py -3.13 python/main_final.py`, open picker.
3. Expected HUD: "3 fruits", labels `plastic_box` / `tomato` / `banana` all visible with non-zero confidence.
4. Press 'b', 't', 'p' in turn — each should pick only that category.

- [ ] **Step 3: Tune HSV / area thresholds if needed**

If box is missed or tomato false-fires on box:
- Log `white_ratio`, `aspect`, `area` at detection time (add a `print` in `_detect_plastic_box` and revert after tuning).
- Adjust `_BOX_MIN_AREA_PX`, `_BOX_MAX_AREA_PX`, `_BOX_WHITE_MIN_RATIO` based on observed values.
- Re-run.

- [ ] **Step 4: Commit lab findings + any threshold tweaks**

```bash
git add python/fruit_detector.py teach_points.json docs/PROGRESS.md
git commit -m "docs(progress): P1 lab acceptance + threshold tune"
```

**Stop here if Phase 2 deferred.**

---

# P2 — orientation-aware grasp

## Task 2.1: Add `gripper_mount_offset_rad` to `SessionCal`

**Files:**
- Modify: `python/session_cal.py`
- Modify: `python/test_session_cal.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_session_cal.py`:

```python
def test_gripper_mount_offset_roundtrip():
    c = _base_cal()
    c.gripper_mount_offset_rad = 0.12
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        path = f.name
    try:
        c.save(path)
        loaded = SessionCal.load(path)
        assert loaded.gripper_mount_offset_rad == 0.12
    finally:
        os.unlink(path)


def test_gripper_mount_offset_default_zero():
    c = _base_cal()
    assert c.gripper_mount_offset_rad == 0.0
```

- [ ] **Step 2: Run, fail, implement**

Add to `SessionCal` dataclass:

```python
    gripper_mount_offset_rad: float = 0.0   # P2: calibrated in lab
```

Update `save` and `load` accordingly (include field in payload, read with `.get(..., 0.0)`).

- [ ] **Step 3: Run test to verify it passes**

Run: `py -3.13 -m pytest python/test_session_cal.py -v`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add python/session_cal.py python/test_session_cal.py
git commit -m "feat(session_cal): gripper_mount_offset_rad field"
```

---

## Task 2.2: Compute `grasp_angle_rad` in banana + box detectors

**Files:**
- Modify: `python/fruit_detector.py`
- Modify: `python/test_fruit_detector.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_fruit_detector.py`:

```python
def test_banana_detection_has_nonzero_grasp_angle():
    """Synthetic banana lying at ~45° in the image should produce a
    base-frame grasp_angle_rad roughly close to ±pi/4 (exact value
    depends on nadir assumption)."""
    import numpy as np
    import cv2
    from fruit_detector import detect_fruits
    from session_cal import SessionCal

    img = np.full((480, 640, 3), 50, dtype=np.uint8)
    # Elongated yellow ellipse at 45°
    cv2.ellipse(img, (320, 240), (110, 30), 45.0, 0, 360, (80, 220, 230), -1)
    depth = np.full((480, 640), 300, dtype=np.uint16)
    cal = SessionCal(
        timestamp="t",
        chess_origin_in_base_m=np.array([0.30, 0.00, 0.00]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30.0},
        d415_intrinsics={"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.30,
        image_size=(640, 480),
        cam_extrinsics_survey1=_nadir_extrinsics(0.30, (0.30, 0.00, 0.00)),
    )
    dets = detect_fruits(img, depth, cal)
    bananas = [d for d in dets if d.fruit_type == "banana"]
    assert len(bananas) == 1
    # grasp_angle must be non-zero (object is elongated); exact sign
    # depends on base-frame orientation, so just check magnitude.
    assert abs(bananas[0].grasp_angle_rad) > 0.3
```

- [ ] **Step 2: Run, fail, implement**

Add a helper in `python/fruit_detector.py`:

```python
def _long_axis_base_angle(rect_ctr, rect_sz, rect_angle_deg,
                            session_cal, top_z_mm) -> float:
    """Project the two long-axis endpoints of a rotated rectangle to
    base frame (via pixel_to_base_frame at the same fruit_top_z) and
    return the long-axis angle in base XY, radians.

    grasp_angle_rad = long_axis_angle + pi/2 (short-edge direction;
    this is the direction the gripper should close along).
    """
    cx, cy = rect_ctr
    w, h = rect_sz
    # minAreaRect angle semantics vary; treat the larger dimension as the
    # long axis, regardless of what OpenCV reports.
    if h >= w:
        long_dx = np.cos(np.deg2rad(rect_angle_deg + 90.0))
        long_dy = np.sin(np.deg2rad(rect_angle_deg + 90.0))
        half = h / 2.0
    else:
        long_dx = np.cos(np.deg2rad(rect_angle_deg))
        long_dy = np.sin(np.deg2rad(rect_angle_deg))
        half = w / 2.0
    p_plus  = (cx + long_dx * half, cy + long_dy * half)
    p_minus = (cx - long_dx * half, cy - long_dy * half)
    base_plus  = pixel_to_base_frame(p_plus,  top_z_mm, session_cal)
    base_minus = pixel_to_base_frame(p_minus, top_z_mm, session_cal)
    theta_long = float(np.arctan2(base_plus[1] - base_minus[1],
                                    base_plus[0] - base_minus[0]))
    return theta_long + np.pi / 2.0
```

Then modify `_detect_banana_contours` to also return the rotated rect so orientation can be computed. Simplest path: keep the current tuple shape but compute `minAreaRect` eagerly and return it alongside:

```python
def _detect_banana_contours(bgr: np.ndarray) -> list:
    """...returns ((cx, cy), area, bbox, confidence,
                    rect_ctr, rect_sz, rect_angle_deg)"""
    mask = hsv_mask(bgr, HSV_RANGES["banana"])
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    hits = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < _BANANA_MIN_AREA or area > _BANANA_MAX_AREA:
            continue
        rect = cv2.minAreaRect(c)  # ((cx,cy),(w,h),angle)
        (rect_cx, rect_cy), (w_r, h_r), rect_angle = rect
        if w_r < 1 or h_r < 1:
            continue
        aspect = max(w_r, h_r) / min(w_r, h_r)
        if aspect < _BANANA_MIN_ASPECT:
            continue
        x, y, w_b, h_b = cv2.boundingRect(c)
        aspect_score = 0.5 + min(0.5, (aspect - _BANANA_MIN_ASPECT) / 2.0)
        hits.append((
            (int(rect_cx), int(rect_cy)), int(area),
            (int(x), int(y), int(w_b), int(h_b)),
            float(aspect_score),
            (float(rect_cx), float(rect_cy)),
            (float(w_r), float(h_r)),
            float(rect_angle),
        ))
    return hits
```

Update `detect_fruits` banana branch:

```python
    for (cx, cy), area, bbox, conf, rect_ctr, rect_sz, rect_angle \
            in _detect_banana_contours(color_bgr):
        if conf < CONFIDENCE_MIN:
            continue
        top_z_mm = _resolve_top_z(
            "banana", depth_mm, (cx, cy), session_cal)
        try:
            xyz_base = pixel_to_base_frame(
                (cx, cy), top_z_mm, session_cal)
            grasp_angle = _long_axis_base_angle(
                rect_ctr, rect_sz, rect_angle, session_cal, top_z_mm)
        except ValueError:
            continue
        results.append(Detection(
            fruit_type="banana",
            center_px=(int(cx), int(cy)),
            center_base_m=xyz_base,
            confidence=float(conf),
            area_px=int(area),
            bbox=tuple(int(v) for v in bbox),
            grasp_angle_rad=grasp_angle,
        ))
```

And do the same for the plastic_box branch — pass the rotated rect through `_long_axis_base_angle` into `grasp_angle_rad`.

- [ ] **Step 3: Run tests**

Run: `py -3.13 -m pytest python/test_fruit_detector.py -v`
Expected: banana-angle test PASS; other tests still PASS.

- [ ] **Step 4: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(detector): compute grasp_angle_rad for banana + box"
```

---

## Task 2.3: `_ik_safe` / `_start_move` — consume `grasp_angle_rad`

**Files:**
- Modify: `python/sorting_controller.py` (lines 494-509 and `_start_move`)
- Modify: `python/test_integration.py`

- [ ] **Step 1: Write the failing test**

Append to `python/test_integration.py`:

```python
def test_wrist_rotates_for_nonzero_grasp_angle():
    """When pick_single is called with a detection carrying non-zero
    grasp_angle_rad, the MockQArm's commanded joint phi[3] must differ
    from the zero-grasp case."""
    import numpy as np
    from fruit_detector import Detection
    from sorting_controller import FruitSortingController

    target = np.array([0.35, 0.10, 0.03])
    det_flat = Detection(
        fruit_type="banana",
        center_px=(100, 100),
        center_base_m=target,
        confidence=0.9,
        area_px=5000,
        bbox=(80, 80, 40, 40),
        grasp_angle_rad=0.0,
    )
    det_rot = Detection(
        fruit_type="banana",
        center_px=(100, 100),
        center_base_m=target,
        confidence=0.9,
        area_px=5000,
        bbox=(80, 80, 40, 40),
        grasp_angle_rad=np.deg2rad(45),
    )

    mocks = [MockQArm(), MockQArm()]
    wrists = []
    for mock, det in zip(mocks, [det_flat, det_rot]):
        c = FruitSortingController(mock, pick_only=True)
        c.T_TRANSIT = 0.02; c.T_APPROACH = 0.02; c.T_PICK = 0.02
        c.T_DWELL = 0.01; c.T_GRIP = 0.01; c.T_SETTLE = 0.01
        # pick_single now must accept a Detection, not just xyz:
        c.pick_single_detection(det)
        # Record last commanded phi[3]
        wrists.append(mock.cmd_log[-1][0][3])
    assert not np.isclose(wrists[0], wrists[1])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `py -3.13 -m pytest python/test_integration.py::test_wrist_rotates_for_nonzero_grasp_angle -v`
Expected: FAIL — `pick_single_detection` and gamma wiring don't exist.

- [ ] **Step 3: Thread `grasp_angle_rad` through the controller**

In `python/sorting_controller.py`:

1. Add a helper:

```python
def _base_angle_to_wrist_joint(self, target_pos, grasp_angle_rad):
    """Convert a base-frame closing-axis angle into the wrist joint
    angle (phi[3]) to command. Uses the yaw the arm will adopt at the
    target position plus the calibrated gripper-mount offset."""
    ty, tx = float(target_pos[1]), float(target_pos[0])
    yaw = float(np.arctan2(ty, tx))
    offset = float(getattr(self.session_cal, "gripper_mount_offset_rad", 0.0))
    raw = grasp_angle_rad - yaw - offset
    # Wrap to [-pi, pi]
    raw = float(np.arctan2(np.sin(raw), np.cos(raw)))
    # Clamp to JOINT_LIMITS[3] with a small safety margin; fall back to 0
    # with a warning if outside.
    from qarm_kinematics import JOINT_LIMITS
    lo, hi = JOINT_LIMITS[3]
    if raw < lo + 0.05 or raw > hi - 0.05:
        print(f"  [controller] wrist angle {raw:+.2f} rad out of range, "
              f"falling back to 0")
        return 0.0
    return raw
```

2. Update `_ik_safe` + `_start_move` callers to pass `gamma`:

Replace line 500 in `_ik_safe`:

```python
            phi = inverse_kinematics(target_pos, gamma=gamma)
```

and change signature to `def _ik_safe(self, target_pos, gamma=0.0):`.

3. Add `pick_single_detection(detection)`:

```python
def pick_single_detection(self, detection) -> bool:
    """Same as pick_single, but accepts a Detection and sets the wrist
    gamma according to detection.grasp_angle_rad."""
    target = np.asarray(detection.center_base_m, dtype=float)
    gamma = self._base_angle_to_wrist_joint(
        target, float(detection.grasp_angle_rad))
    self._pick_gamma = gamma
    return self.pick_single(target, detection.fruit_type)
```

4. Modify `_start_move` so it reads `self._pick_gamma` when non-None, else uses 0.0. Pass it down through `_ik_safe` and into `inverse_kinematics`.

5. Call `self._pick_gamma = None` at the start of `pick_single` (so the default path stays gamma=0).

- [ ] **Step 4: Thread through `picker_viewer._pick_one`**

Edit `python/picker_viewer.py:108-118`:

```python
def _pick_one(controller, detection) -> bool:
    """Dispatch one synchronous pick. Returns True on success."""
    print(f"  [picker] picking {detection.fruit_type} at "
          f"{detection.center_base_m.round(3)} "
          f"(conf={detection.confidence:.2f}, "
          f"angle={np.rad2deg(detection.grasp_angle_rad):.0f}deg)")
    try:
        return bool(controller.pick_single_detection(detection))
    except Exception as ex:
        print(f"  [picker] pick_single raised: {ex}")
        return False
```

(Add `import numpy as np` if not present.)

- [ ] **Step 5: Run tests**

Run: `py -3.13 -m pytest python/test_integration.py python/test_pick_single.py -v`
Expected: all PASS, including the new wrist-rotates test.

- [ ] **Step 6: Commit**

```bash
git add python/sorting_controller.py python/picker_viewer.py \
        python/test_integration.py
git commit -m "feat(controller): wrist gamma from Detection.grasp_angle_rad"
```

---

## Task 2.4: Gripper-mount-offset calibration helper (lab script)

**Files:**
- Create: `python/calibrate_gripper_offset.py`

- [ ] **Step 1: Write the helper**

```python
"""Tiny lab script to calibrate session_cal.gripper_mount_offset_rad.

Procedure:
  1. Place a banana lying along the +X axis from base (long axis along X).
  2. Run: py -3.13 python/calibrate_gripper_offset.py
  3. The arm moves the gripper above the banana at pick height with
     wrist phi[3] = 0. User sights along the gripper and reports whether
     jaws are aligned with the banana long axis.
  4. Script increments phi[3] in 5° steps until user confirms 'aligned'.
  5. Script writes the confirmed phi[3] value as
     session_cal.gripper_mount_offset_rad and saves session_cal.json.
"""
from __future__ import annotations
import os
import sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from qarm_driver import QArmDriver
from qarm_kinematics import inverse_kinematics
from session_cal import SessionCal

REPO = os.path.abspath(os.path.join(_HERE, ".."))
SESSION_CAL = os.path.join(REPO, "session_cal.json")


def main():
    cal = SessionCal.load(SESSION_CAL)
    target = np.array([0.35, 0.00, 0.05])  # 35 cm forward, pick height
    q = QArmDriver(); q.connect()
    try:
        gamma = 0.0
        while True:
            phi = inverse_kinematics(target, gamma=gamma)
            q.set_joints_and_gripper(phi, 0.5)
            ans = input(
                f"gamma={np.rad2deg(gamma):+.0f} deg. "
                "'a'=aligned, '+'=+5deg, '-'=-5deg, 'q'=quit: ")
            if ans == "a":
                cal.gripper_mount_offset_rad = float(gamma)
                cal.save(SESSION_CAL)
                print(f"saved gripper_mount_offset_rad = {gamma:+.3f} rad")
                return
            if ans == "+":
                gamma += np.deg2rad(5)
            elif ans == "-":
                gamma -= np.deg2rad(5)
            elif ans == "q":
                return
    finally:
        q.card.close()
        q._connected = False


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Smoke-run syntax**

Run: `py -3.13 -c "import calibrate_gripper_offset"`
Expected: no import error (runs the module top-level without calling `main`).

- [ ] **Step 3: Commit**

```bash
git add python/calibrate_gripper_offset.py
git commit -m "feat(calibrate): gripper_mount_offset_rad calibration script"
```

---

## Task 2.5: P2 lab acceptance

- [ ] **Step 1: Calibrate gripper offset**

Run: `py -3.13 python/calibrate_gripper_offset.py`
Follow prompts. Verify the saved offset is persisted in `session_cal.json`.

- [ ] **Step 2: Test rotated picks**

1. Box at long edge = 0°, 45°, 90° to base x — pick each.
2. Banana at 0°, 45° — pick each.

**Acceptance:** Wrist visibly rotates before descent, gripper jaws align with the short edge of the object, all five picks succeed.

- [ ] **Step 3: Commit progress**

```bash
git add docs/PROGRESS.md
git commit -m "docs(progress): P2 lab acceptance"
```

---

# Self-Review (author notes, not executable)

**Spec coverage check:**
- §3.1 solvePnP geometric model → Task 0.2
- §3.1 ray-plane projection runtime → Task 0.3
- §3.3 error handling (ok=False, RMS gate) → Task 0.2
- §3.3 runtime ray checks → Task 0.3
- §3.4 capture settle + sanity → Task 0.6
- §3.5 session_cal schema change → Task 0.1
- §3.5 calibrate_extrinsics new module → Task 0.2
- §3.5 preflight gate → Task 0.7
- §3.5 fruit_detector projection rewrite → Task 0.3
- §3.6 unit tests for solvePnP + ray-plane + settle → Tasks 0.2, 0.3, 0.6
- §3.6 lab acceptance → Task 0.8
- §4.1 strawberry removal → Task 1.1
- §4.1 _detect_plastic_box → Task 1.3
- §4.1 tomato exclude boxes → Task 1.4
- §4.1 detect_fruits orchestration → Task 1.5
- §4.2 BASKETS + placebox + 's'→'p' → Tasks 1.6, 1.7, 1.8
- §4.3 lab acceptance → Task 1.9
- §5.1 grasp_angle_rad per class → Task 2.2
- §5.2 gamma = grasp_angle - yaw - offset → Task 2.3
- §5.2 gripper_mount_offset_rad calibration → Tasks 2.1, 2.4
- §5.3 lab acceptance → Task 2.5
- §6.2 session_cal migration: missing extrinsics → load is permissive (None), preflight gates — covered in 0.1/0.7
- §6.3 LAB_RUNBOOK updates → lab-task followups (informal)

**Placeholder scan:** no TBD / TODO / "implement later" in steps. All thresholds (0.05 rad settle tol, 5 px RMS cap, 15000/60000 area gate, 0.08 white ratio) are concrete numerical values.

**Type consistency:** `Detection.grasp_angle_rad` is float throughout. `cam_extrinsics_survey1` is `dict | None` in every task. `_detect_plastic_box` returns a 7-tuple; `_detect_banana_contours` in Task 2.2 also returns a 7-tuple; `detect_fruits` unpacks both with identical shape.

**Naming discipline:** `solve_survey1_extrinsics` (Task 0.2) = name used in Tasks 0.5 and by the spec. `pick_single_detection` (Task 2.3) = name used by `picker_viewer` in Task 2.3 Step 4. `gripper_mount_offset_rad` (Task 2.1) = name used in Task 2.3 Step 3 and Task 2.4.

---

**End of plan.**
