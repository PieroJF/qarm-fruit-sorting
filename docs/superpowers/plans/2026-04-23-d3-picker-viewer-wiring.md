# D3 — Picker Viewer + main_final Wiring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire the D1 chessboard calibration + D2 fruit detector into an operator-facing picker UI (click-to-pick + category batch) and rewrite `main_final.py` to drive the new autonomous loop. Retire the UGreen + hardcoded-queue code paths.

**Architecture:** Two new Python modules (`survey_capture.py`, `picker_viewer.py`) plus one new public method (`FruitSortingController.pick_single`) layered over the existing, untouched FSM. `main_final.py` becomes a thin wrapper that loads `session_cal.json`, connects hardware, and hands off to `picker_viewer.run_picker_loop`. `preflight.py` swaps its UGreen/closed-loop checks for session-cal + chessboard-still-visible checks. Simulink facade shrinks (demo queue deleted) since autonomous mode shells out to `main_final.py` per spec §4.4.

**Tech Stack:** Python 3.13, OpenCV 4.13, numpy 2.4, Quanser SDK (`qarm_driver`, `QArmCamera`), existing repo modules (`sorting_controller`, `session_cal`, `fruit_detector`, `qarm_kinematics`).

**Spec anchor:** `docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md` §3.4, §3.5, §4.1, §4.2, §4.4, §4.5.

---

## Invariants

- Python interpreter is `py -3.13`. Tests run with `py -3.13 python/test_<name>.py`. Exit code = number of failures.
- Never rewrite Python core — only wrap it. The FSM (`State` enum, `_step()`, `_start_move()`, trajectory primitives) is unchanged structurally; only `pick_single` is added to the controller.
- Do NOT touch `qarm_driver.py`, `qarm_kinematics.py`, `trajectory.py`, `camera.py`, `fruit_detector.py`, `session_cal.py`, `calibrate_chessboard.py`, `touch_probe.py`, `homography_solver.py`.
- Tests use the existing pattern from `test_integration.py`: bare `assert` statements, functions that return `(name: str, ok: bool, msg: str)`, a `main()` that runs all and returns fail count.
- `MockQArm` already exists in `test_integration.py` — reuse it by importing from that module.

---

## File structure

### New files

| Path | Responsibility |
|---|---|
| `python/survey_capture.py` | Move arm to `survey1`, warm up + capture D415 RGB+depth, chessboard residual self-check, call `detect_fruits`. Returns `(list[Detection], dict_of_diagnostics)`. |
| `python/picker_viewer.py` | OpenCV window loop: click-to-pick, `b/t/s` category batch, `r` refresh, `ESC` exit. Calls `survey_capture.capture_fruits` + `controller.pick_single`. |
| `python/test_pick_single.py` | Unit tests for `FruitSortingController.pick_single` against `MockQArm`. |
| `python/test_survey_capture.py` | Unit tests for survey_capture helpers (warmup logic, residual computation). |
| `python/test_picker_viewer.py` | Unit tests for picker_viewer pure helpers (nearest-click, type-filter, annotate). |

### Modified files

| Path | Change |
|---|---|
| `python/sorting_controller.py` | Extract `_drive_until_done(dt)` helper; add public `pick_single(base_xyz, fruit_type, dt=0.01) -> bool`. |
| `python/sorting_controller_sim.py` | Add `load_single_pick(base_xyz, fruit_type)` parity helper (queue-setup only; Simulink's solver drives `step()`). |
| `python/main_final.py` | Delete hardcoded-fruits fallback, delete `detect_fruits_live`, `--no-camera`, `--pick-only`. Load `session_cal.json` (fail-fast if missing/stale), connect driver+camera, hand off to `picker_viewer.run_picker_loop`. Preserve `--dry-run`. |
| `python/preflight.py` | Delete `check_ugreen` (#3) and `check_visual_reference` (#7). Replace `check_calibration` (#4) with `check_session_cal`. Add `check_chessboard_still_visible` (#3'). Update `CHECKS` list numbering. |
| `matlab_facade/py_controller.m` | Delete hardcoded demo queue fallback (lines 55–61). Empty queue now means "nothing to sort". |

---

## Task summary

1. **Task 1** — `pick_single()` on hardware controller (TDD, MockQArm).
2. **Task 2** — `load_single_pick()` parity on sim controller.
3. **Task 3** — `survey_capture.py` helpers + integration wrapper.
4. **Task 4** — `picker_viewer.py` pure helpers (no OpenCV loop).
5. **Task 5** — `picker_viewer.run_picker_loop` event loop.
6. **Task 6** — Rewrite `main_final.py`.
7. **Task 7** — Refactor `preflight.py`.
8. **Task 8** — Strip demo queue from `matlab_facade/py_controller.m`.
9. **Task 9** — Full test suite green; update `PROGRESS.md`.

---

## Task 1: `FruitSortingController.pick_single()`

**Files:**
- Modify: `python/sorting_controller.py` (add `_drive_until_done`, add `pick_single`; lines ~116–150 refactor)
- Create: `python/test_pick_single.py`

### Design

- Extract the `while self.state != State.DONE:` loop body from `run_autonomous` into `_drive_until_done(self, dt)` so `pick_single` can reuse it.
- `pick_single(base_xyz, fruit_type, dt=0.01) -> bool`:
  - Snapshots `prior = self.sorted_count`.
  - Sets `self.fruit_queue = [{'pos': np.asarray(base_xyz, dtype=float), 'type': fruit_type}]`.
  - Resets `self.state = State.GO_HOME` and `self._traj_end_pos = None`.
  - Calls `self._drive_until_done(dt)`.
  - Returns `self.sorted_count > prior`.
- Unreachable fruits: FSM's existing `_ik_safe` in `SELECT_FRUIT` routes to `GO_HOME` with queue empty → `DONE` → `sorted_count` unchanged → `pick_single` returns `False`. Correct behavior, no extra code needed.

- [ ] **Step 1.1: Write failing test for successful single pick**

Create `python/test_pick_single.py`:

```python
"""Tests for FruitSortingController.pick_single synchronous entry point."""
import os, sys, time
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from sorting_controller import FruitSortingController, State
from test_integration import MockQArm


def _compressed(c):
    c.T_TRANSIT = 0.05; c.T_APPROACH = 0.05; c.T_PICK = 0.05
    c.T_DWELL = 0.02; c.T_GRIP = 0.02
    return c


def test_pick_single_reachable_returns_true():
    name = "pick_single_reachable"
    q = MockQArm()
    c = _compressed(FruitSortingController(q, pick_only=True))
    ok = c.pick_single(np.array([0.35, 0.10, 0.13]), "strawberry", dt=0.001)
    assert ok, "pick_single returned False for reachable fruit"
    assert c.sorted_count == 1, f"sorted_count = {c.sorted_count}"
    assert c.state == State.DONE, f"state = {c.state.name}"
    return name, True, "reachable fruit sorted once"


def test_pick_single_unreachable_returns_false():
    name = "pick_single_unreachable"
    q = MockQArm()
    c = _compressed(FruitSortingController(q, pick_only=True))
    ok = c.pick_single(np.array([2.0, 0.0, 0.5]), "tomato", dt=0.001)
    assert not ok, "pick_single returned True for out-of-reach fruit"
    assert c.sorted_count == 0, f"sorted_count = {c.sorted_count}"
    return name, True, "unreachable skipped, sorted_count unchanged"


def test_pick_single_twice_accumulates():
    name = "pick_single_twice"
    q = MockQArm()
    c = _compressed(FruitSortingController(q, pick_only=True))
    assert c.pick_single(np.array([0.35, 0.10, 0.13]), "strawberry", dt=0.001)
    assert c.pick_single(np.array([0.30, -0.05, 0.08]), "tomato", dt=0.001)
    assert c.sorted_count == 2, f"sorted_count = {c.sorted_count}"
    return name, True, "two back-to-back picks sorted_count==2"


TESTS = [test_pick_single_reachable_returns_true,
         test_pick_single_unreachable_returns_false,
         test_pick_single_twice_accumulates]


def main():
    fails = 0
    for fn in TESTS:
        try:
            name, ok, msg = fn()
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {name}  {msg}")
            if not ok: fails += 1
        except Exception as ex:
            import traceback
            fails += 1
            print(f"  [FAIL] {fn.__name__}: {ex}")
            traceback.print_exc()
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed")
    return fails


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 1.2: Run test — verify all three fail with AttributeError**

Run: `py -3.13 python/test_pick_single.py`
Expected: 3 FAIL with `AttributeError: 'FruitSortingController' object has no attribute 'pick_single'`.

- [ ] **Step 1.3: Refactor `run_autonomous` to use `_drive_until_done`**

In `python/sorting_controller.py`, replace the body of `run_autonomous` (lines 116–150). Find this block:

```python
    def run_autonomous(self, dt=0.01):
        """
        Run the full autonomous sorting loop.
        ...
        """
        print("\n=== AUTONOMOUS FRUIT SORTING ===")
        print(f"Fruits to sort: {len(self.fruit_queue)}")
        print(f"Baskets: {list(self.BASKETS.keys())}")
        print()

        self.state = State.GO_HOME
        start_time = time.time()

        while self.state != State.DONE:
            loop_start = time.time()
            try:
                self._step(time.time())
            except Exception as ex:
                print(f"[ERROR] state {self.state.name}: {ex}")
                print("[ERROR] aborting sort — arm held at last commanded pose")
                break
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        total_time = time.time() - start_time
        print(f"\n=== SORTING COMPLETE ===")
        print(f"Sorted {self.sorted_count} fruits in {total_time:.1f} seconds")
```

Replace with:

```python
    def run_autonomous(self, dt=0.01):
        """
        Run the full autonomous sorting loop.

        Parameters
        ----------
        dt : float
            Control loop period (seconds). Default 100 Hz — plenty for
            trajectory tracking at QArm motion bandwidths.
        """
        print("\n=== AUTONOMOUS FRUIT SORTING ===")
        print(f"Fruits to sort: {len(self.fruit_queue)}")
        print(f"Baskets: {list(self.BASKETS.keys())}")
        print()

        self.state = State.GO_HOME
        start_time = time.time()
        self._drive_until_done(dt)
        total_time = time.time() - start_time
        print(f"\n=== SORTING COMPLETE ===")
        print(f"Sorted {self.sorted_count} fruits in {total_time:.1f} seconds")

    def _drive_until_done(self, dt):
        """Tight control loop. Pumps _step at `dt` cadence until state==DONE
        or an exception is caught. Shared between run_autonomous and
        pick_single."""
        while self.state != State.DONE:
            loop_start = time.time()
            try:
                self._step(time.time())
            except Exception as ex:
                print(f"[ERROR] state {self.state.name}: {ex}")
                print("[ERROR] aborting — arm held at last commanded pose")
                break
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def pick_single(self, base_xyz, fruit_type, dt=0.01):
        """
        Run ONE synchronous pick-place cycle for a single target.

        Parameters
        ----------
        base_xyz : array_like, shape (3,)
            XYZ in robot base frame (metres).
        fruit_type : str
            One of 'banana', 'tomato', 'strawberry' — selects the basket.
        dt : float
            Control loop period; same semantics as run_autonomous.

        Returns
        -------
        bool
            True if sorted_count increased (pick+place succeeded);
            False if the target was unreachable or an exception aborted
            the cycle.
        """
        prior = self.sorted_count
        self.fruit_queue = [{
            'pos': np.asarray(base_xyz, dtype=float),
            'type': str(fruit_type),
        }]
        self.state = State.GO_HOME
        self._traj_end_pos = None
        self._drive_until_done(dt)
        return self.sorted_count > prior
```

- [ ] **Step 1.4: Run test to verify it passes**

Run: `py -3.13 python/test_pick_single.py`
Expected: 3/3 passed, exit 0.

- [ ] **Step 1.5: Regression — run existing integration tests**

Run: `py -3.13 python/test_integration.py`
Expected: all existing tests still pass (the refactor preserves `run_autonomous` behavior).

- [ ] **Step 1.6: Commit**

```bash
git add python/sorting_controller.py python/test_pick_single.py
git commit -m "$(cat <<'EOF'
feat(ctrl): pick_single synchronous entry + _drive_until_done refactor

run_autonomous now delegates its control loop to _drive_until_done so
pick_single can reuse it. pick_single sets queue=[target], resets to
GO_HOME, drives to DONE, returns True on success.
EOF
)"
```

---

## Task 2: `StepController.load_single_pick()` (sim parity)

**Files:**
- Modify: `python/sorting_controller_sim.py` (add `load_single_pick` after `__init__`, around line 88)

### Design

The sim is stateless — its `step()` is driven by the Simulink solver (or test harness), so the sim cannot have a blocking `pick_single`. Per spec §4.4, Simulink's autonomous mode shells out to `main_final.py`, so the sim itself never runs autonomous. The parity requirement (§1.3) is satisfied by a queue-setup helper that an outside driver can use before entering its `step()` loop.

- [ ] **Step 2.1: Add the `load_single_pick` method**

In `python/sorting_controller_sim.py`, just after the `__init__` method (around line 88, before `def step(`):

```python
    def load_single_pick(self, base_xyz, fruit_type):
        """
        Queue a single target for one pick-place cycle, and reset FSM.

        Parity with FruitSortingController.pick_single for sim callers
        that drive step() themselves (e.g. Simulink facade, test harness).
        The caller is responsible for pumping step() until the returned
        `done` flag is True; this helper only sets up state.

        Parameters
        ----------
        base_xyz : array_like, shape (3,)
        fruit_type : str  — one of 'banana' | 'tomato' | 'strawberry'.
        """
        self.fruit_queue = [{
            'pos': np.asarray(base_xyz, dtype=float),
            'type': str(fruit_type),
        }]
        self.state = S_GO_HOME
        self.t = 0.0
        self.sorted_count = 0
        self._started = False
        self._last_ee = self.HOME_POS.copy()
        self._last_gripper = 0.0
```

- [ ] **Step 2.2: Smoke-check the import**

Run: `py -3.13 -c "from sorting_controller_sim import StepController; c = StepController(); c.load_single_pick([0.3, 0.1, 0.02], 'tomato'); print(c.fruit_queue, c.state)"`
Expected: prints the queue (one dict) and state id `1` (S_GO_HOME).

- [ ] **Step 2.3: Commit**

```bash
git add python/sorting_controller_sim.py
git commit -m "feat(sim): load_single_pick parity helper for one-cycle jobs"
```

---

## Task 3: `python/survey_capture.py`

**Files:**
- Create: `python/survey_capture.py`
- Create: `python/test_survey_capture.py`

### Design

Two pure helpers plus one orchestrator.

- `_warmup_and_capture(camera, warmup_timeout_s=10.0) -> (color, depth, intr)` — polls `camera.read()` until `color.mean() > 5`, then medians 5 frames. Same pattern as `calibrate_chessboard.py` and `diag_detector.py`. Mockable by passing a fake camera whose `read()` returns fixed frames.
- `_chessboard_residual(color, session_cal) -> (residual_mm | None, corners_found: int)` — runs `cv2.findChessboardCorners`, then projects stored `H_pixel_to_chess_mm` back to image, computes per-corner error in mm (using chess-plane → pixel inverse). Returns `(None, 0)` if corners not found (don't error — just warn in diagnostics).
- `capture_fruits(driver, camera, session_cal) -> (list[Detection], dict)` — moves to `session_cal.survey_pose_joints_rad`, opens camera (caller supplies — see below), warms up, captures, computes residual, calls `detect_fruits`. Returns detections + diagnostics dict with keys `chessboard_residual_mm`, `chessboard_corners_found`, `warnings` (list of strings).

**Camera ownership decision:** `capture_fruits` takes an already-opened `camera` object (caller opens/closes). This matches `diag_detector.py` / `diag_pick_one.py` pattern and avoids opening the D415 N times per session.

**Warning thresholds** (per spec §3.4):
- residual > 3 mm → `warnings.append("chessboard residual {r:.1f} mm > 3 mm — drift detected")`
- residual > 10 mm → raise `RuntimeError("chessboard residual {r:.1f} mm > 10 mm — rerun calibrate_chessboard.py")`

- [ ] **Step 3.1: Write failing test for `_warmup_and_capture`**

Create `python/test_survey_capture.py`:

```python
"""Tests for survey_capture helpers."""
import os, sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from survey_capture import _warmup_and_capture, _chessboard_residual


class FakeCamera:
    """Returns `dark_count` dark frames, then bright frames forever."""
    def __init__(self, dark_count=3, img_shape=(720, 1280, 3)):
        self.dark_count = dark_count
        self.img_shape = img_shape
        self._reads = 0
        self.intrinsics = {"fx": 912.6, "fy": 911.3,
                            "cx": 635.4, "cy": 343.0}

    def read(self):
        self._reads += 1
        if self._reads <= self.dark_count:
            return (np.zeros(self.img_shape, dtype=np.uint8),
                    np.zeros(self.img_shape[:2], dtype=np.uint16))
        # Bright frame: uniform value 100 + some variation
        c = np.full(self.img_shape, 100, dtype=np.uint8)
        d = np.full(self.img_shape[:2], 300, dtype=np.uint16)
        return c, d


def test_warmup_skips_dark_frames():
    name = "warmup_skips_dark"
    cam = FakeCamera(dark_count=3)
    color, depth, intr = _warmup_and_capture(cam, warmup_timeout_s=2.0)
    assert color.shape == (720, 1280, 3), f"shape {color.shape}"
    assert color.mean() > 90, f"mean {color.mean()}"
    assert intr["fx"] == 912.6
    # dark_count warm-ups + 5 capture reads = 8 total
    assert cam._reads == 8, f"reads {cam._reads}"
    return name, True, f"warmed up past {cam.dark_count} dark frames"


def test_warmup_timeout_raises():
    name = "warmup_timeout"
    # dark_count very high, timeout short → must raise
    cam = FakeCamera(dark_count=10_000)
    try:
        _warmup_and_capture(cam, warmup_timeout_s=0.2)
    except RuntimeError as ex:
        return name, "warm-up timeout" in str(ex).lower(), str(ex)
    return name, False, "expected RuntimeError, got none"


def test_residual_no_chessboard_returns_none():
    """Plain grey image → findChessboardCorners fails → (None, 0)."""
    name = "residual_no_chessboard"
    from session_cal import SessionCal
    sc = SessionCal(
        timestamp="2026-04-23T00:00:00",
        chess_origin_in_base_m=np.array([0.4, 0.1, 0.0]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 8, "rows": 6, "square_mm": 30.0,
                       "inner_cols": 7, "inner_rows": 5},
        d415_intrinsics={"fx": 912.6, "fy": 911.3,
                          "cx": 635.4, "cy": 343.0},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=0.26,
        image_size=(1280, 720),
    )
    grey = np.full((720, 1280, 3), 128, dtype=np.uint8)
    residual, n = _chessboard_residual(grey, sc)
    assert residual is None and n == 0, f"got ({residual}, {n})"
    return name, True, "no corners → (None, 0)"


TESTS = [test_warmup_skips_dark_frames,
         test_warmup_timeout_raises,
         test_residual_no_chessboard_returns_none]


def main():
    fails = 0
    for fn in TESTS:
        try:
            name, ok, msg = fn()
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {name}  {msg}")
            if not ok: fails += 1
        except Exception as ex:
            import traceback
            fails += 1
            print(f"  [FAIL] {fn.__name__}: {ex}")
            traceback.print_exc()
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed")
    return fails


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3.2: Run test — verify it fails with ImportError**

Run: `py -3.13 python/test_survey_capture.py`
Expected: ImportError / ModuleNotFoundError on `from survey_capture import ...`.

- [ ] **Step 3.3: Create `python/survey_capture.py`**

```python
"""
One-shot survey capture: move to survey1, grab RGB+depth, self-check
chessboard residual, run detect_fruits.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.4.
"""
from __future__ import annotations
import os
import sys
import time
from typing import Any

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from session_cal import SessionCal
from fruit_detector import detect_fruits, Detection

_INNER_COLS = 7
_INNER_ROWS = 5
_SQUARE_MM = 30.0
_WARN_RESIDUAL_MM = 3.0
_ERROR_RESIDUAL_MM = 10.0


def _warmup_and_capture(camera, warmup_timeout_s: float = 10.0):
    """Poll camera.read() until color.mean()>5, then median 5 frames.

    Returns (color_bgr: HxWx3 uint8, depth_mm: HxW uint16, intrinsics: dict).
    Raises RuntimeError on timeout.
    """
    deadline = time.time() + warmup_timeout_s
    while time.time() < deadline:
        try:
            c, _ = camera.read()
        except Exception:
            continue
        if c.mean() > 5:
            break
    else:
        raise RuntimeError(
            "camera warm-up timeout - no valid frame after "
            f"{warmup_timeout_s:.0f} s.")
    frames = []
    depth_latest = None
    for _ in range(5):
        c, d = camera.read()
        frames.append(c.copy())
        depth_latest = d.copy() if d is not None else None
    color = np.median(np.stack(frames), axis=0).astype(np.uint8)
    intr = {k: float(camera.intrinsics[k]) for k in ("fx", "fy", "cx", "cy")}
    return color, depth_latest, intr


def _chessboard_residual(color_bgr: np.ndarray, session_cal: SessionCal):
    """Try to re-find chessboard corners in the live frame; project stored
    homography to image and compute per-corner delta in mm.

    Returns (residual_mm: float | None, corners_found: int).
    None if corners not detected — caller decides how to warn.
    """
    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, (_INNER_COLS, _INNER_ROWS),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None, 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    refined = cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1), criteria
    ).reshape(-1, 2).astype(np.float32)
    world_mm = np.array(
        [[i * _SQUARE_MM, j * _SQUARE_MM]
         for j in range(_INNER_ROWS) for i in range(_INNER_COLS)],
        dtype=np.float32)
    H = np.asarray(session_cal.h_pixel_to_chess_mm, dtype=np.float64)
    pts_h = np.hstack([refined, np.ones((refined.shape[0], 1), dtype=np.float32)])
    proj = (H @ pts_h.T).T
    proj_mm = proj[:, :2] / proj[:, 2:3]
    err_mm = np.linalg.norm(proj_mm - world_mm, axis=1)
    return float(np.sqrt(np.mean(err_mm ** 2))), int(refined.shape[0])


def capture_fruits(driver, camera, session_cal: SessionCal):
    """
    Survey-capture orchestrator.

    1. slow_move_to_joints(driver, survey_pose)
    2. warm up + capture RGB+depth
    3. compute chessboard residual (self-check)
    4. run detect_fruits

    Parameters
    ----------
    driver : QArmDriver  (already connected)
    camera : QArmCamera  (already opened)
    session_cal : SessionCal

    Returns
    -------
    detections : list[Detection]
    diagnostics : dict with keys
        chessboard_residual_mm (float | None)
        chessboard_corners_found (int)
        warnings (list[str])

    Raises
    ------
    RuntimeError if residual > 10 mm (calibration broken).
    """
    from calibrate_closed_loop import slow_move_to_joints  # existing helper
    slow_move_to_joints(driver, session_cal.survey_pose_joints_rad,
                        float(0.10))
    color, depth, _ = _warmup_and_capture(camera)
    residual_mm, n_corners = _chessboard_residual(color, session_cal)
    warnings: list[str] = []
    if residual_mm is None:
        warnings.append("chessboard not detected in survey frame — "
                         "residual self-check skipped")
    else:
        if residual_mm > _ERROR_RESIDUAL_MM:
            raise RuntimeError(
                f"chessboard residual {residual_mm:.1f} mm > "
                f"{_ERROR_RESIDUAL_MM:.0f} mm — rerun "
                "calibrate_chessboard.py")
        if residual_mm > _WARN_RESIDUAL_MM:
            warnings.append(
                f"chessboard residual {residual_mm:.1f} mm > "
                f"{_WARN_RESIDUAL_MM:.0f} mm — drift detected")
    detections = detect_fruits(color, depth, session_cal)
    diagnostics = {
        "chessboard_residual_mm": residual_mm,
        "chessboard_corners_found": n_corners,
        "warnings": warnings,
        "color_frame": color,   # for the viewer to render
    }
    return detections, diagnostics
```

- [ ] **Step 3.4: Run test to verify it passes**

Run: `py -3.13 python/test_survey_capture.py`
Expected: 3/3 passed, exit 0.

- [ ] **Step 3.5: Commit**

```bash
git add python/survey_capture.py python/test_survey_capture.py
git commit -m "$(cat <<'EOF'
feat(vision): survey_capture — move, capture, chessboard self-check, detect

Orchestrator for one survey-pose D415 frame: drives arm to survey1,
medians 5 RGB frames, re-runs findChessboardCorners to report residual
drift in mm, then calls detect_fruits. Raises on >10 mm residual.
EOF
)"
```

---

## Task 4: `picker_viewer.py` — pure helpers

**Files:**
- Create: `python/picker_viewer.py` (helpers only; `run_picker_loop` in Task 5)
- Create: `python/test_picker_viewer.py`

### Design

Four pure helpers, all testable without OpenCV windows:

1. `_nearest_detection(detections, click_xy, max_r_px=50) -> Detection | None` — Euclidean pixel distance; returns nearest within radius, else None.
2. `_filter_by_type(detections, fruit_type) -> list[Detection]` — trivial list comprehension.
3. `_annotate(color_bgr, detections, residual_mm, warnings) -> np.ndarray` — returns a new image with bbox + label per detection, residual HUD top-left, warnings in amber. Does NOT show window.
4. `_hud_text(n_fruits, residual_mm, mode="idle") -> str` — single string for the HUD line.

Type-to-color map matches spec §3.5 (banana=yellow, tomato=red, strawberry=pink-magenta). Reuse from `diag_detector.py` `_TYPE_COLORS`.

- [ ] **Step 4.1: Write failing tests**

Create `python/test_picker_viewer.py`:

```python
"""Tests for picker_viewer pure helpers."""
import os, sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fruit_detector import Detection
from picker_viewer import (
    _nearest_detection, _filter_by_type, _annotate, _hud_text,
)


def _mk_det(ftype, cx, cy, conf=0.8):
    return Detection(
        fruit_type=ftype,
        center_px=(cx, cy),
        center_base_m=np.array([0.3, 0.1, 0.02]),
        confidence=conf,
        area_px=500,
        bbox=(cx - 10, cy - 10, 20, 20),
    )


def test_nearest_within_radius():
    name = "nearest_within"
    dets = [_mk_det("tomato", 100, 100),
            _mk_det("banana", 300, 300)]
    got = _nearest_detection(dets, (110, 95), max_r_px=50)
    assert got is not None
    assert got.fruit_type == "tomato"
    return name, True, "picks the closer one"


def test_nearest_outside_radius_returns_none():
    name = "nearest_outside"
    dets = [_mk_det("tomato", 100, 100)]
    got = _nearest_detection(dets, (500, 500), max_r_px=50)
    assert got is None
    return name, True, "out-of-range click → None"


def test_nearest_empty_list_returns_none():
    name = "nearest_empty"
    assert _nearest_detection([], (100, 100), max_r_px=50) is None
    return name, True, "empty detection list → None"


def test_filter_by_type():
    name = "filter_by_type"
    dets = [_mk_det("tomato", 1, 1), _mk_det("banana", 2, 2),
            _mk_det("tomato", 3, 3)]
    got = _filter_by_type(dets, "tomato")
    assert len(got) == 2 and all(d.fruit_type == "tomato" for d in got)
    return name, True, "keeps only matching type"


def test_annotate_returns_image_same_shape():
    name = "annotate_shape"
    img = np.full((720, 1280, 3), 50, dtype=np.uint8)
    dets = [_mk_det("tomato", 100, 100), _mk_det("banana", 300, 200)]
    out = _annotate(img, dets, residual_mm=1.2, warnings=["test warn"])
    assert out.shape == img.shape
    assert out.dtype == np.uint8
    # Annotation must modify pixels (bbox draw)
    assert not np.array_equal(out, img), "annotate left image unchanged"
    return name, True, "shape preserved, pixels modified"


def test_hud_text_contains_counts():
    name = "hud_text"
    msg = _hud_text(n_fruits=5, residual_mm=1.8, mode="idle")
    assert "5" in msg and "1.8" in msg
    return name, True, f"'{msg}'"


TESTS = [
    test_nearest_within_radius, test_nearest_outside_radius_returns_none,
    test_nearest_empty_list_returns_none, test_filter_by_type,
    test_annotate_returns_image_same_shape, test_hud_text_contains_counts,
]


def main():
    fails = 0
    for fn in TESTS:
        try:
            name, ok, msg = fn()
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {name}  {msg}")
            if not ok: fails += 1
        except Exception as ex:
            import traceback
            fails += 1
            print(f"  [FAIL] {fn.__name__}: {ex}")
            traceback.print_exc()
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed")
    return fails


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4.2: Run test — expect ImportError**

Run: `py -3.13 python/test_picker_viewer.py`
Expected: ModuleNotFoundError on `from picker_viewer import ...`.

- [ ] **Step 4.3: Create `python/picker_viewer.py` (helpers only)**

```python
"""
Operator-facing picker UI: click-to-pick + category-batch autonomous loop.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.5.

Event loop lives in run_picker_loop(). Everything above it is pure and
unit-tested.
"""
from __future__ import annotations
import os
import sys
import time

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fruit_detector import Detection

_TYPE_COLORS = {
    "banana":     (0, 255, 255),    # yellow (BGR)
    "tomato":     (0, 0, 255),      # red
    "strawberry": (200, 100, 255),  # pink-magenta
}
_WARN_COLOR = (0, 180, 220)         # amber
_HUD_COLOR = (255, 255, 255)
_RESIDUAL_OK = (0, 200, 0)
_RESIDUAL_WARN = (0, 180, 220)
_RESIDUAL_BAD = (0, 0, 255)

CLICK_MAX_R_PX = 50


def _nearest_detection(detections, click_xy, max_r_px=CLICK_MAX_R_PX):
    """Return the detection with smallest pixel distance to click_xy,
    within max_r_px. None if no detection qualifies."""
    if not detections:
        return None
    cx, cy = float(click_xy[0]), float(click_xy[1])
    best = None
    best_d2 = (max_r_px + 1) ** 2
    for d in detections:
        dx = d.center_px[0] - cx
        dy = d.center_px[1] - cy
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best = d
    if best_d2 > max_r_px * max_r_px:
        return None
    return best


def _filter_by_type(detections, fruit_type):
    """Keep only detections whose fruit_type matches."""
    return [d for d in detections if d.fruit_type == fruit_type]


def _residual_color(residual_mm):
    if residual_mm is None or residual_mm < 3.0:
        return _RESIDUAL_OK
    if residual_mm < 10.0:
        return _RESIDUAL_WARN
    return _RESIDUAL_BAD


def _annotate(color_bgr, detections, residual_mm, warnings):
    """Draw bboxes + labels + HUD on a copy of color_bgr."""
    out = color_bgr.copy()
    for d in detections:
        color = _TYPE_COLORS.get(d.fruit_type, (255, 255, 255))
        x, y, w, h = d.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        label = f"{d.fruit_type} {d.confidence:.2f}"
        cv2.putText(out, label, (x, max(18, y - 6)),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        cv2.circle(out, tuple(int(v) for v in d.center_px), 4, color, -1)
    hud = _hud_text(len(detections), residual_mm, "idle")
    cv2.putText(out, hud, (10, 28),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, _HUD_COLOR, 2, cv2.LINE_AA)
    # residual chip (top-right)
    if residual_mm is not None:
        rc = _residual_color(residual_mm)
        cv2.circle(out, (out.shape[1] - 30, 28), 10, rc, -1)
    for i, w in enumerate(warnings or []):
        cv2.putText(out, w, (10, 56 + i * 24),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, _WARN_COLOR, 1,
                     cv2.LINE_AA)
    return out


def _hud_text(n_fruits, residual_mm, mode):
    r = f"{residual_mm:.1f}" if residual_mm is not None else "n/a"
    return (f"{n_fruits} fruits  |  residual {r} mm  |  "
            f"mode: {mode}  |  click / b t s / r / ESC")
```

- [ ] **Step 4.4: Run test to verify helpers pass**

Run: `py -3.13 python/test_picker_viewer.py`
Expected: 6/6 passed.

- [ ] **Step 4.5: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "feat(viewer): picker_viewer pure helpers — nearest, filter, annotate, HUD"
```

---

## Task 5: `picker_viewer.run_picker_loop()`

**Files:**
- Modify: `python/picker_viewer.py` (append `run_picker_loop` + the `_pick_one`/`_pick_category` helpers)

### Design

Append to the module from Task 4. The event loop, per spec §3.5:

- cv2 window `"Picker"` with `setMouseCallback`.
- Initial capture via `survey_capture.capture_fruits`.
- Main loop: `cv2.waitKey(30)`.
  - Mouse left click → `_nearest_detection` → `_pick_one(det)` → re-capture.
  - `b`/`t`/`s` → `_pick_category(type)` → category-batch loop with re-capture between picks.
  - `r` → re-capture, no pick.
  - `ESC` → break out of loop; sets a flag to abort in-progress batch.
- Errors inside `_pick_one` or `_pick_category` never crash the loop — caught, shown as red banner 3 s, re-capture, continue.
- Hard cap 20 picks per batch.
- Stuck-target detection: same (cx, cy) rounded to 5 px attempted twice → mark skipped.
- Category-batch picks fruit closest to `controller.HOME_POS[:2]` first each round (spec Q4).

Unit-testing this function is hard (OpenCV UI). We verify:
- The module imports cleanly.
- A `_pick_one` call with a mock controller invokes `controller.pick_single` with the right args.

- [ ] **Step 5.1: Write a test for `_pick_one` dispatch**

Append to `python/test_picker_viewer.py` BEFORE the `TESTS = [...]` list:

```python
class FakeController:
    def __init__(self, succeed=True):
        self.calls = []
        self._succeed = succeed
        self.sorted_count = 0
    def pick_single(self, base_xyz, fruit_type, dt=0.01):
        self.calls.append((np.asarray(base_xyz).copy(), fruit_type))
        if self._succeed:
            self.sorted_count += 1
            return True
        return False


def test_pick_one_calls_controller():
    from picker_viewer import _pick_one
    name = "pick_one_dispatch"
    det = _mk_det("tomato", 100, 100)
    det.center_base_m = np.array([0.42, 0.11, 0.04])
    ctrl = FakeController(succeed=True)
    ok = _pick_one(ctrl, det)
    assert ok is True
    assert len(ctrl.calls) == 1
    pos, ftype = ctrl.calls[0]
    assert np.allclose(pos, [0.42, 0.11, 0.04])
    assert ftype == "tomato"
    return name, True, "ctrl.pick_single called with det base_m + type"
```

And add `test_pick_one_calls_controller` to the `TESTS` list.

- [ ] **Step 5.2: Run test — expect ImportError**

Run: `py -3.13 python/test_picker_viewer.py`
Expected: ImportError on `_pick_one` (not yet defined).

- [ ] **Step 5.3: Append to `python/picker_viewer.py`**

Add below the helpers from Task 4:

```python
# ------------------------------------------------------------------------
# Pick dispatchers
# ------------------------------------------------------------------------

MAX_CATEGORY_PICKS = 20
RETRY_LIMIT_PER_TARGET = 2


def _pick_one(controller, detection) -> bool:
    """Dispatch one synchronous pick. Returns True on success."""
    print(f"  [picker] picking {detection.fruit_type} at "
          f"{detection.center_base_m.round(3)} "
          f"(conf={detection.confidence:.2f})")
    try:
        return bool(controller.pick_single(
            detection.center_base_m, detection.fruit_type))
    except Exception as ex:
        print(f"  [picker] pick_single raised: {ex}")
        return False


def _nearest_to_home(detections, home_xy_m):
    if not detections:
        return None
    hx, hy = float(home_xy_m[0]), float(home_xy_m[1])
    return min(detections, key=lambda d: (
        (d.center_base_m[0] - hx) ** 2 +
        (d.center_base_m[1] - hy) ** 2))


def _pixel_key(detection, bucket_px=5):
    cx, cy = detection.center_px
    return (int(cx) // bucket_px * bucket_px,
            int(cy) // bucket_px * bucket_px)


def _pick_category(driver, camera, session_cal, controller,
                     fruit_type, should_abort_fn):
    """Category batch: re-capture, pick nearest-to-home, repeat until
    none left, limit hit, or abort signalled. Returns count picked."""
    from survey_capture import capture_fruits
    picks_done = 0
    retries = {}
    while picks_done < MAX_CATEGORY_PICKS:
        if should_abort_fn():
            print("  [picker] batch aborted by ESC")
            break
        try:
            dets, diag = capture_fruits(driver, camera, session_cal)
        except Exception as ex:
            print(f"  [picker] capture failed mid-batch: {ex}")
            break
        matches = _filter_by_type(dets, fruit_type)
        if not matches:
            break
        target = _nearest_to_home(matches, controller.HOME_POS[:2])
        key = _pixel_key(target)
        if retries.get(key, 0) >= RETRY_LIMIT_PER_TARGET:
            print(f"  [picker] skipping stuck target at {key}")
            continue
        success = _pick_one(controller, target)
        if success:
            picks_done += 1
            retries.pop(key, None)
        else:
            retries[key] = retries.get(key, 0) + 1
    return picks_done


# ------------------------------------------------------------------------
# Event loop
# ------------------------------------------------------------------------

def run_picker_loop(driver, camera, session_cal, controller) -> None:
    """Interactive OpenCV picker. Blocks until ESC.

    Parameters
    ----------
    driver     : QArmDriver (connected)
    camera     : QArmCamera (opened)
    session_cal: SessionCal
    controller : FruitSortingController
    """
    from survey_capture import capture_fruits
    window = "Picker"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)

    state = {"click": None, "abort": False}

    def _on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    cv2.setMouseCallback(window, _on_mouse)

    def _refresh():
        dets, diag = capture_fruits(driver, camera, session_cal)
        frame = _annotate(diag["color_frame"], dets,
                            diag.get("chessboard_residual_mm"),
                            diag.get("warnings", []))
        cv2.imshow(window, frame)
        return dets, diag

    try:
        dets, diag = _refresh()
    except Exception as ex:
        print(f"  [picker] initial capture failed: {ex}")
        cv2.destroyWindow(window)
        return

    while True:
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            print("  [picker] ESC — exiting")
            break
        if state["click"] is not None:
            click = state["click"]; state["click"] = None
            target = _nearest_detection(dets, click)
            if target is None:
                print(f"  [picker] no fruit near {click}")
            else:
                _pick_one(controller, target)
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key in (ord('b'), ord('t'), ord('s')):
            ftype = {ord('b'): 'banana',
                       ord('t'): 'tomato',
                       ord('s'): 'strawberry'}[key]
            matches = _filter_by_type(dets, ftype)
            if not matches:
                print(f"  [picker] no {ftype} visible")
            else:
                print(f"  [picker] category batch: {ftype}")
                n = _pick_category(driver, camera, session_cal,
                                    controller, ftype,
                                    lambda: state["abort"])
                print(f"  [picker] {ftype} batch done: {n} picked")
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key == ord('r'):
            print("  [picker] refresh")
            try:
                dets, diag = _refresh()
            except Exception as ex:
                print(f"  [picker] re-capture failed: {ex}")

    cv2.destroyWindow(window)
```

- [ ] **Step 5.4: Run picker_viewer tests**

Run: `py -3.13 python/test_picker_viewer.py`
Expected: 7/7 passed (helpers + pick_one_dispatch).

- [ ] **Step 5.5: Smoke import**

Run: `py -3.13 -c "from picker_viewer import run_picker_loop; print('OK')"`
Expected: prints `OK`.

- [ ] **Step 5.6: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "$(cat <<'EOF'
feat(viewer): run_picker_loop + category-batch + retry logic

OpenCV window dispatches click → pick_single(nearest), b/t/s → category
batch (nearest-to-home each round, max 20 picks, retry cap 2 per stuck
target), r → refresh, ESC → exit. Between picks always re-captures.
EOF
)"
```

---

## Task 6: Rewrite `python/main_final.py`

**Files:**
- Modify: `python/main_final.py` (full rewrite of `main()`; helpers at top preserved only if referenced)

### Design

Per spec §4.1:

- Load `session_cal.json` (fail-fast on missing).
- Age check: if `datetime.now() - session_cal.timestamp > 12 h` → print warning but continue (user may be resuming a session — don't hard-block).
- Keep `--dry-run`: prints `session_cal` summary + `No fruits detected` if empty, exits before any arm motion.
- Delete `--no-camera`, `--pick-only`, the hardcoded-fruits fallback, and `detect_fruits_live`.
- Build driver + camera, call `picker_viewer.run_picker_loop(driver, camera, session_cal, controller)`.
- On exit: `driver.home()`, close camera, disconnect driver.

- [ ] **Step 6.1: Read current main_final.py once more for anchors**

Run: `py -3.13 -c "import main_final; print(dir(main_final))"` — just to confirm imports.

- [ ] **Step 6.2: Rewrite `python/main_final.py`**

Replace the entire file contents with:

```python
"""
Top-level launcher for the autonomous fruit-sorting pipeline.

Loads session_cal.json, connects the QArm + D415, and hands off to
picker_viewer.run_picker_loop for the click-to-pick / category-batch UI.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §4.1.
"""
from __future__ import annotations
import argparse
import datetime as _dt
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from session_cal import SessionCal

SESSION_CAL_PATH = os.path.join(os.path.dirname(_HERE), "session_cal.json")
STALE_HOURS = 12


def _load_session_cal(path):
    if not os.path.exists(path):
        sys.exit(f"[abort] {path} missing — run calibrate_chessboard.py")
    cal = SessionCal.load(path)
    try:
        age = _dt.datetime.now() - _dt.datetime.fromisoformat(cal.timestamp)
        age_h = age.total_seconds() / 3600.0
        if age_h > STALE_HOURS:
            print(f"[warn] session_cal is {age_h:.1f} h old "
                  f"(threshold {STALE_HOURS} h) — consider recalibrating")
    except Exception:
        pass
    return cal


def main():
    parser = argparse.ArgumentParser(
        description="QArm autonomous fruit-sorting picker")
    parser.add_argument("--dry-run", action="store_true",
        help="Print plan and exit without moving the arm.")
    args = parser.parse_args()

    print("=" * 60)
    print("  QArm Fruit Sorting — Autonomous Picker")
    print("=" * 60)
    print(f"  session_cal  : {SESSION_CAL_PATH}")
    cal = _load_session_cal(SESSION_CAL_PATH)
    print(f"  cam height   : {cal.camera_height_above_table_m:.3f} m")
    print(f"  chess origin : {cal.chess_origin_in_base_m.round(3)}")
    print(f"  homography   : RMS {cal.homography_reproj_rms_px:.2f} px")

    if args.dry_run:
        print("\n[dry-run] session_cal OK. Skipping hardware + picker.")
        return 0

    # Connect hardware
    from qarm_driver import QArmDriver
    from camera import QArmCamera
    from sorting_controller import FruitSortingController
    from picker_viewer import run_picker_loop

    driver = QArmDriver()
    driver.connect()
    camera = QArmCamera()
    camera.open()
    controller = FruitSortingController(driver)
    try:
        run_picker_loop(driver, camera, cal, controller)
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C — exiting")
    finally:
        try: camera.close()
        except Exception: pass
        try: driver.home(duration=3.0)
        except Exception: pass
        try: driver.disconnect()
        except Exception: pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6.3: Smoke run `--dry-run`**

Run: `py -3.13 python/main_final.py --dry-run`
Expected: prints session_cal summary, exits 0, no hardware touched.

- [ ] **Step 6.4: Commit**

```bash
git add python/main_final.py
git commit -m "$(cat <<'EOF'
refactor(main_final): rewrite for session_cal + picker_viewer hand-off

Replace hardcoded fruit queue + --no-camera fallback + --pick-only with
a thin driver that loads session_cal.json, connects driver+camera, and
hands off to run_picker_loop. --dry-run preserved for offline sanity.
EOF
)"
```

---

## Task 7: Refactor `python/preflight.py`

**Files:**
- Modify: `python/preflight.py`

### Design

Per spec §4.2:

- Delete `check_ugreen` and `check_visual_reference` (their imports too).
- Replace `check_calibration` → `check_session_cal`:
  - file exists? age < 12 h? `homography_reproj_rms_px < 2.0`? `survey_pose_joints_rad` present?
- Add `check_chessboard_still_visible`:
  - Move arm to `survey1`, capture, call `_chessboard_residual`.
  - Report residual and warn if > 3 mm, fail if > 10 mm or corners not found.
  - Skip in `--offline` mode.
- Update CHECKS list numbering (shrinks from 7 → 5 + optional 1).

- [ ] **Step 7.1: Open `python/preflight.py` and apply these edits**

At the top of the file, find `CALIB_FILE = ...`. Replace with:

```python
SESSION_CAL = os.path.join(REPO, "session_cal.json")
```

Remove the `REF_FRAME`, `BASELINE`, `REF_TCP` constants (no longer needed).

Delete the entire `check_ugreen` function (around lines 101–107).

Replace `check_calibration` (around lines 110–134) with:

```python
def check_session_cal():
    if not os.path.exists(SESSION_CAL):
        return False, f"{os.path.basename(SESSION_CAL)} missing — "
                       f"run calibrate_chessboard.py"
    try:
        from session_cal import SessionCal
        cal = SessionCal.load(SESSION_CAL)
    except Exception as ex:
        return False, f"load: {ex}"
    rms = float(cal.homography_reproj_rms_px)
    try:
        age_h = (datetime.now()
                 - datetime.fromisoformat(cal.timestamp)
                 ).total_seconds() / 3600.0
    except Exception:
        age_h = None
    warn = []
    ok = True
    if rms >= 2.0:
        warn.append(f"RMS={rms:.2f}px >= 2.0"); ok = False
    if age_h is not None and age_h > 12:
        warn.append(f"age={age_h:.1f}h > 12"); ok = False
    if cal.survey_pose_joints_rad is None or len(cal.survey_pose_joints_rad) != 4:
        warn.append("survey_pose missing"); ok = False
    detail = f"RMS={rms:.2f}px age={age_h:.1f}h" if age_h is not None \
        else f"RMS={rms:.2f}px age=?"
    if warn:
        detail += f"  warnings={warn}"
    return ok, detail
```

Delete the entire `check_visual_reference` function (around lines 185–234).

Add a new `check_chessboard_still_visible` somewhere after `check_session_cal`:

```python
def check_chessboard_still_visible():
    """Move arm to survey1, capture one frame, check chessboard residual."""
    try:
        from session_cal import SessionCal
        from camera import QArmCamera
        from qarm_driver import QArmDriver
        from survey_capture import _warmup_and_capture, _chessboard_residual
        from calibrate_closed_loop import slow_move_to_joints
    except Exception as ex:
        return False, f"import: {ex}"
    if not os.path.exists(SESSION_CAL):
        return False, "session_cal missing"
    try:
        cal = SessionCal.load(SESSION_CAL)
    except Exception as ex:
        return False, f"load: {ex}"
    q = QArmDriver()
    try:
        q.connect(); time.sleep(0.3)
        slow_move_to_joints(q, cal.survey_pose_joints_rad, 0.10)
    except Exception as ex:
        try: q.card.close()
        except Exception: pass
        return False, f"arm move: {ex}"
    cam = QArmCamera()
    try:
        cam.open()
        color, _, _ = _warmup_and_capture(cam)
    except Exception as ex:
        try: cam.close()
        except Exception: pass
        try: q.card.close()
        except Exception: pass
        return False, f"capture: {ex}"
    finally:
        try: cam.close()
        except Exception: pass
        try: q.card.close()
        except Exception: pass
        q._connected = False
    residual, n = _chessboard_residual(color, cal)
    if residual is None:
        return False, f"chessboard not found ({n} corners)"
    if residual > 10.0:
        return False, f"residual={residual:.1f}mm > 10 — recalibrate"
    warn = " (drift warn)" if residual > 3.0 else ""
    return True, f"residual={residual:.1f}mm corners={n}{warn}"
```

Replace the `CHECKS = [...]` list (around lines 237–245) with:

```python
CHECKS = [
    ("1 QArm       ", check_qarm,                      False),
    ("2 D415       ", check_d415,                      False),
    ("3 SessionCal ", check_session_cal,               True),
    ("4 Chessboard ", check_chessboard_still_visible,  False),
    ("5 HSV        ", check_hsv,                       False),
    ("6 Tests      ", check_offline_tests,             True),
]
```

Update `check_offline_tests` test_files list (around line 167) to reflect reality:

```python
    test_files = [
        os.path.join(HERE, "test_integration.py"),
        os.path.join(HERE, "test_fruit_detector.py"),
        os.path.join(HERE, "test_calibrate_chessboard.py"),
        os.path.join(HERE, "test_pick_single.py"),
        os.path.join(HERE, "test_survey_capture.py"),
        os.path.join(HERE, "test_picker_viewer.py"),
    ]
```

- [ ] **Step 7.2: Run `--offline` preflight**

Run: `py -3.13 python/preflight.py --offline`
Expected: SessionCal + Tests checks run (both should pass), others skipped, exit 0.

- [ ] **Step 7.3: Commit**

```bash
git add python/preflight.py
git commit -m "$(cat <<'EOF'
refactor(preflight): swap UGreen/VisualRef for session_cal + chessboard

Deleted check_ugreen + check_visual_reference. check_calibration is now
check_session_cal (reads session_cal.json, gates on RMS + age + survey
pose). New check_chessboard_still_visible moves to survey1 and verifies
the homography residual hasn't drifted.
EOF
)"
```

---

## Task 8: Strip demo queue from `matlab_facade/py_controller.m`

**Files:**
- Modify: `matlab_facade/py_controller.m`

### Design

Per spec §4.4: the facade shells out to `main_final.py` in autonomous mode, so the demo fruit fallback in `build_queue()` is dead code. Delete it. An empty `fruit_list` now results in `StepController` with an empty queue — which goes straight to DONE on first step. That's correct behavior.

- [ ] **Step 8.1: Apply the edit**

Open `matlab_facade/py_controller.m`. Find the `build_queue` function (around lines 48–77). Inside it, find the block that injects demo fruits when `fruit_list` is empty:

```matlab
    if isempty(fruit_list)
        positions = py.list({py.list({0.30, 0.20, 0.02}), ...
                              py.list({0.25, -0.15, 0.02})});
        types = py.list({'strawberry', 'banana'});
        return
    end
```

Replace with:

```matlab
    if isempty(fruit_list)
        % Empty queue = nothing to sort. Autonomous mode drives via
        % main_final.py shell-out (see spec §4.4), not via Simulink FSM.
        positions = py.list({});
        types = py.list({});
        return
    end
```

- [ ] **Step 8.2: Commit**

```bash
git add matlab_facade/py_controller.m
git commit -m "refactor(facade): drop demo fruit fallback from build_queue"
```

---

## Task 9: Full green + docs update

**Files:**
- Modify: `PROGRESS.md` (short status note)

- [ ] **Step 9.1: Run the full test suite**

Run (all four, in order):

```bash
py -3.13 python/test_integration.py
py -3.13 python/test_fruit_detector.py
py -3.13 python/test_calibrate_chessboard.py
py -3.13 python/test_pick_single.py
py -3.13 python/test_survey_capture.py
py -3.13 python/test_picker_viewer.py
```

Expected: every file exits 0.

- [ ] **Step 9.2: Dry-run main_final**

Run: `py -3.13 python/main_final.py --dry-run`
Expected: loads session_cal, prints summary, exits 0. No hardware.

- [ ] **Step 9.3: Update PROGRESS.md**

Open `PROGRESS.md`. Find the top "Sprint state" section (around lines 10-12). Insert a new status line before the existing block:

```markdown
## 0. Sprint state (2026-04-23)

**D3 complete** — picker_viewer + survey_capture + pick_single + main_final rewrite landed. Preflight swapped UGreen/VisualRef for session_cal + chessboard-still-visible. MATLAB facade demo queue gutted. Pending: D4 lab tuning of HSV, first end-to-end pick test (D4 AM).

Test suites:
- test_integration.py (FSM regressions) — still green
- test_pick_single.py (3 new tests) — green
- test_survey_capture.py (3 new tests) — green
- test_picker_viewer.py (7 new tests) — green

---

```

- [ ] **Step 9.4: Final commit**

```bash
git add PROGRESS.md
git commit -m "docs: PROGRESS.md — D3 complete, awaiting D4 lab"
```

---

## Self-review checklist (ran against spec)

**Spec coverage:**
- §3.4 `survey_capture.py` → Task 3 ✓
- §3.5 `picker_viewer.py` (all 7 flow items, retries, stuck-target, hard cap, ESC) → Tasks 4 + 5 ✓
- §3.5 `FruitSortingController.pick_single` → Task 1 ✓
- §1.3 sim parity for `pick_single` → Task 2 (as queue-setup helper) ✓
- §4.1 `main_final.py` rewrite (session_cal age gate, --dry-run preserved, --pick-only removed) → Task 6 ✓
- §4.2 preflight rewire (delete #3 UGreen, #7 VisualRef; replace #4; add #3' chessboard) → Task 7 ✓
- §4.4 `matlab_facade/py_controller.m` demo queue deletion → Task 8 ✓
- §4.5 `LAB_RUNBOOK.md` edits → **deferred to D8** (documentation cleanup phase per memory `project_fruitsorting.md`). Flag in self-review; not this plan.

**Placeholder scan:** no TBD / implement later / handle edge cases. All code blocks complete.

**Type consistency:**
- `Detection.center_px` is `tuple`, `Detection.center_base_m` is `np.ndarray`, `Detection.bbox` is `tuple` — consistent across tasks.
- `SessionCal.h_pixel_to_chess_mm` (lowercase) — consistent with D1 landed code.
- `FruitSortingController.HOME_POS` is `np.ndarray(3,)` — consistent.

**Deferred items (not blockers):**
1. `LAB_RUNBOOK.md` cleanup — D8 scope, documented in memory.
2. `hsv_tuner.py` — D4-AM scope (spec §3.3 timeline note).
3. Simulink `build_autonomous_stateflow.m` updates to call `main_final.py` — D6 scope.

---

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-04-23-d3-picker-viewer-wiring.md`. Two execution options:

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks.
**2. Inline Execution** — execute tasks in this session using executing-plans, batch with checkpoints.

Which approach?
