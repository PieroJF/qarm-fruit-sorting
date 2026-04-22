# Overhead Vision Pipeline Redesign — FruitSorting QArm Project

**Date**: 2026-04-22
**Deadline**: 2026-05-01 14:00 (9 days)
**Team**: Piero Flores, Zihen Huang, Ran Zhang, Yichang Chao
**Supersedes**: the UGreen floor-camera closed-loop calibration flow introduced 2026-04-20

This spec captures the design decisions for replacing the current semi-automated vision subsystem with a single-camera overhead-shot pipeline. It was produced via a brainstorm session on 2026-04-22 and approved by the user before plan generation.

---

## 1. Context and goals

### 1.1 Problem with the current pipeline

The 2026-04-20 sprint introduced a UGreen floor camera + closed-loop hand-eye calibration (`calibrate_closed_loop.py`). Validated in the lab on 2026-04-21, but the session-start cost is high:

- Arm must be parked behind the bench (`homeplace0`) so UGreen has an unobstructed view.
- Baseline capture (`ugreen_baseline.png`) must be taken before any fruit is placed.
- Eight calibration fruits (strawberries at `cal_01..04`, tomatoes at `cal_05..08`) must be placed at exact teach points.
- Hand-eye solve runs, produces RMS residuals; if RMS > 50 mm the file is written as `.rejected.json` and the session can't proceed.
- A visual-reference snapshot (`ugreen_pickhome1_reference.png` + `ugreen_pickhome1_tcp.json`) must be captured at `pickhome1` before preflight will go green.
- Fruit positions for the actual sort run are then **hardcoded** in a demo queue (`py_controller.m`) because the UGreen pipeline only closes the loop on TCP, not on fruit detection.

Net result: ~20 minutes of session setup, fruit positions still hardcoded, and the whole thing breaks if the UGreen camera shifts.

### 1.2 Goals of this redesign

- **Sub-1-minute session calibration**, using only equipment the team already owns (printed 7×5 30 mm chessboard, D415).
- **Fully automatic fruit detection** — no hardcoded positions. One overhead photo produces a list of `(fruit_type, base_frame_XYZ)` for the picker to consume.
- **Two-mode operator UI** — the detection result is shown in an annotated OpenCV window with two input paths:
  - **Mouse click** → pick that single fruit (supervised, one cycle per click).
  - **Keyboard `b` / `t` / `s`** → pick every banana / tomato / strawberry currently visible, one after another, re-capturing between each pick (supervised batch by category).

  Both paths share the same underlying `controller.pick_single(...)` entry point; the batch keys are a thin loop over single picks.
- **Self-checking at runtime** — every detection frame also re-locates the chessboard; a reprojection residual flags calibration drift immediately.
- **Robust 3-class discrimination** of banana / tomato / strawberry using pure OpenCV (HSV + shape + green-calyx signature).
- **Retire the UGreen stack entirely** to cut ~1500 lines of code, 2 preflight checks, 9 tests, and most of `LAB_RUNBOOK.md`.

### 1.4 Rubric framing note

The coursework brief requires an "autonomous" mode defined as "full vision → plan → pick → place loop." Both input paths satisfy this:

- **Category-batch** (`b`/`t`/`s`): one keystroke kicks off an unbroken vision→plan→pick→place→re-detect loop that runs until every fruit of that category is sorted. This is the primary "autonomous mode" demonstrated in the video — "operator types `b`, arm autonomously picks and places all bananas."
- **Click-to-pick**: each click triggers the same full autonomous cycle on one fruit. Useful as a debugging / recovery path when the detector misclassifies a fruit.

In both paths the operator's role is *supervision* (selecting what to action), not *teleop*. Remote (teleop) mode is a separate, already-implemented modality. This framing will be stated explicitly in the report.

### 1.3 Invariants preserved

- **Simulink is the facade; Python does the real work.** `matlab_facade/` unchanged at the contract level; only the Python detector changes.
- **No QUARC** — all hardware I/O still routes via `py.qarm_driver.QArmDriver`.
- **Single arm-mounted camera**: D415. UGreen is deleted.
- **FSM / trajectory / gripper / driver untouched structurally.** `sorting_controller.py` gains one new public entry point `pick_single(base_xyz, fruit_type)` that wraps the existing state machine to run one cycle synchronously; the state transitions, the trajectory planner, the gripper ramp, and the driver are all unchanged. `sorting_controller_sim.py` gets the parallel addition for Simulink parity. `trajectory.py`, `qarm_kinematics.py`, `qarm_driver.py` get zero edits.

---

## 2. Architecture

```
Session Start
  └─ calibrate_chessboard.py
       ├─ jog gripper to chessboard origin corner (reuse teach_points.py UX)
       │    → records T_chess_origin_in_base
       ├─ move to SURVEY_POSE
       │    → capture D415 frame
       │    → findChessboardCorners + findHomography
       │    → H_pixel_to_chess + reprojection RMS
       └─ writes session_cal.json

Run (click-driven loop)
  └─ survey_capture.py
  │    ├─ arm → SURVEY_POSE
  │    ├─ D415 capture (RGB + depth)
  │    ├─ chessboard re-detect → residual self-check (warn if > 3 mm)
  │    ├─ fruit_detector.detect(color, depth)
  │    │    ├─ HSV masks per class (yellow / red-smooth / red-with-green-top)
  │    │    ├─ shape gating (aspect ratio, circularity, area)
  │    │    ├─ green-calyx signature for strawberry discrimination
  │    │    ├─ depth sampling (5×5 median at blob center, outlier reject)
  │    │    └─ parallax correction using measured fruit-top Z
  │    └─ returns [(fruit_type, XYZ_base, confidence), ...]
  │
  └─ picker_viewer.py (new)
       ├─ annotated image: each detection drawn as bbox + type label + conf
       ├─ mouse click              → pick nearest detection within 50 px
       ├─ key b / t / s            → pick all bananas / tomatoes / strawberries
       │                              (loop: re-capture → pick nearest of type → repeat
       │                               until none left or retry-limit hit)
       ├─ key r                    → re-capture without picking
       ├─ on any pick path: hand off to sorting_controller.pick_single
       ├─ between picks: re-run survey_capture → redraw
       └─ ESC to exit (also aborts an in-progress batch between picks)

FSM (unchanged)
  └─ sorting_controller runs ONE pick cycle per click; routes to
     basket A/B/C based on the clicked fruit's type.
```

### 2.1 Key design decisions (recap of brainstorm)

| # | Decision | Choice | Rationale |
|---|---|---|---|
| Q1 | Fiducial | **Chessboard 7×5 30 mm** (already printed) | Max corner density, OpenCV native, team already owns it |
| Q2 | Base-frame registration | **Jog-touch origin corner** | Flexible, ~30 s, reuses existing teach_points.py jog-and-save |
| Q3 | Runtime chessboard | **Stays in table corner** | Per-frame residual = live drift detector at near-zero cost |
| Q4 | Detector | **Pure OpenCV layered features** | No new deps, ships in 1.5 days, 3 classes are linearly separable in colour+shape |
| Q5 | Z-height | **D415 depth channel** | RGB-D already available; accurate parallax correction |
| Q6 | Scope | **Delete UGreen entirely** | Repo -1500 lines, session setup -15 min, preflight -2 checks |

---

## 3. New modules

### 3.1 `python/calibrate_chessboard.py`

**Purpose**: Produce `session_cal.json`. Run once at the start of each lab session.

**Interface**:
```
python calibrate_chessboard.py             # interactive
python calibrate_chessboard.py --no-touch  # reuse last T_chess_origin_in_base
```

**Flow**:
1. Load `teach_points.json`. If `survey1` is missing, abort with instructions to teach it first.
2. **Phase 1 — Origin registration**:
   - Prompt: "Place chessboard flat on table. Jog gripper tip until it touches the top-left inner corner of the chessboard pattern. Press ENTER to confirm, ESC to abort."
   - Reuse the keyboard jog loop from `teach_points.py` (q/w/e/r/a/s/d/f and z/x for gripper).
   - On ENTER: read current joints → FK → extract TCP position → save as `T_chess_origin_in_base`.
3. **Phase 2 — Homography capture**:
   - `slow_move_to_joints(survey1.joints_rad)` to move arm clear of the chessboard.
   - D415 capture (use `camera.QArmCamera`, 30 frames warm-up + median of 5 frames to suppress noise).
   - `cv2.findChessboardCorners(gray, (6,4))` — note: a 7×5 chessboard has 6×4 inner corners.
   - `cv2.cornerSubPix` for sub-pixel refinement.
   - Build world points: `[[i*30, j*30, 0] for j in range(4) for i in range(6)]` (mm, chess-origin frame).
   - `H, _ = cv2.findHomography(image_pts, world_pts_xy)` — planar homography from pixel to chess-XY in mm.
   - Compute reprojection RMS; fail hard if > 2 px (calibration unusable).
4. Write `session_cal.json`.
5. Run sanity check: reproject each corner via H and print the delta per corner (for visual inspection).

**Output** (`session_cal.json`):
```json
{
  "timestamp": "2026-04-22T14:30:00",
  "chess_origin_in_base_m": [x, y, z_table],
  "H_pixel_to_chess_mm": [[...], [...], [...]],
  "survey_pose_joints_rad": [j1, j2, j3, j4],
  "chess_pattern": {"cols": 7, "rows": 5, "square_mm": 30, "inner_cols": 6, "inner_rows": 4},
  "d415_intrinsics": {"fx": 912.6, "fy": 911.3, "cx": 635.4, "cy": 343.0},
  "homography_reproj_rms_px": 0.45,
  "image_size": [1280, 720]
}
```

**Work estimate**: 1 day.

### 3.2 `python/fruit_detector.py` (rewrite)

**Purpose**: Given a D415 RGB-D frame pair and `session_cal.json`, return fruit detections with positions in robot base frame.

**Interface**:
```python
@dataclass
class Detection:
    fruit_type: str          # "banana" | "tomato" | "strawberry"
    center_px: tuple         # (cx, cy) in image pixels
    center_base_m: np.ndarray  # shape (3,), XYZ in base frame, meters
    confidence: float        # 0..1
    area_px: int
    bbox: tuple              # (x, y, w, h)

def detect_fruits(color: np.ndarray, depth: np.ndarray,
                  session_cal: dict) -> list[Detection]: ...
```

**Per-class detection logic**:

**Banana** (easy — yellow is isolated on the colour wheel):
1. HSV mask on yellow (default: H∈[18,35], S∈[80,255], V∈[80,255] — tunable).
2. Morphology: `MORPH_OPEN` 5×5 then `MORPH_CLOSE` 9×9.
3. `findContours`, for each:
   - Area ∈ [800, 30000] px²
   - Fit minAreaRect → aspect ratio = max_side / min_side
   - Accept if aspect > 1.8
4. Confidence = aspect_ratio_score × hsv_purity_score (both in [0,1]).

**Tomato** (red + round + no green on top):
1. HSV mask on red (handle wrap-around: mask1 H∈[0,10] + mask2 H∈[170,180], S∈[80,255], V∈[60,255]).
2. Same morphology.
3. For each contour:
   - Area ∈ [400, 15000] px²
   - Circularity = 4π·area / perimeter² > 0.7
   - **Green-above test**: look at a band of pixels 10–20 px above the blob's top edge. Count pixels in green HSV range (H∈[35,85], S∈[60,255]). If green pixels > 5% of band area → this is a strawberry, not a tomato.
4. Confidence = circularity × hsv_purity × (1 - green_above_ratio).

**Strawberry** (red + has green calyx + tapered shape):
1. Same red HSV mask as tomato.
2. For each contour that was *rejected* by the tomato path due to the green-above test, OR that fits the strawberry size range:
   - Area ∈ [200, 4000] px² (smaller than tomato)
   - Taper score: compute `width_at_top_20% / width_at_bottom_20%` of the bounding box. Strawberry < 1.0 (wider at top, narrower at bottom); tomato ≈ 1.0.
   - **Green-calyx confirmation**: the green-above test from the tomato path, inverted — must trigger.
3. Confidence = green_calyx_score × taper_score × hsv_purity.

**Depth sampling + parallax correction** (applied to every accepted detection):
1. Sample a 5×5 patch of depth around `center_px`.
2. Filter: drop zeros, drop values outside table±150 mm range.
3. If ≥ 8 valid samples → `fruit_top_z = median(valid)`; else **reject detection** (depth unreliable, per Q5 decision).
4. Pinhole reprojection using D415 intrinsics: back-project `(center_px, fruit_top_z)` to camera frame.
5. Transform camera → base: camera-to-base isn't directly known, but we have H_pixel_to_chess (at z=0) and T_chess_origin_in_base. So: take `(center_px, 0)` → H → chess XY; then correct for parallax using fruit_top_z / camera_height_above_table ratio.

   Specifically, if C = (cx, cy) is the principal point, (u, v) is the blob center pixel, and camera-height-above-table is h, then parallax-corrected pixel is:
   ```
   u' = C[0] + (u - C[0]) * (h - fruit_top_z) / h
   v' = C[1] + (v - C[1]) * (h - fruit_top_z) / h
   ```
   Then apply H to (u', v') → table XY in chess frame → add T_chess_origin_in_base → base XYZ.

   Camera-height-above-table h is derived once per session from the homography: the distance from the camera's optical centre to the chessboard plane, recovered via `cv2.decomposeHomographyMat` during calibration and stored in `session_cal.json` as `camera_height_above_table_m`.

**Confidence thresholds**: drop any detection with `confidence < 0.35`.

**Work estimate**: 1.5 days.

### 3.3 `python/hsv_tuner.py`

**Purpose**: Lab-lighting HSV tuning utility. Run once per session in a new lab, or when lighting changes noticeably.

**Interface**: `python hsv_tuner.py`

**Flow**:
1. Open D415 live view in `cv2.namedWindow`.
2. Mouse click on a fruit → sample a 10×10 HSV patch at click location → update rolling mean/std per class.
3. Keys:
   - `1` / `2` / `3` → switch sampling class (banana / tomato / strawberry).
   - `g` → switch to green-calyx sampling.
   - `ENTER` → print current ranges to stdout and write to `hsv_ranges.json`.
   - `q` → quit.
4. Overlay: current mask for the active class drawn semi-transparently on the live view.

**Output** (`hsv_ranges.json`):
```json
{
  "banana":      {"h": [18,35],  "s": [80,255], "v": [80,255]},
  "tomato":      {"h_wrap1": [0,10], "h_wrap2": [170,180], "s": [80,255], "v": [60,255]},
  "strawberry":  {"h_wrap1": [0,10], "h_wrap2": [170,180], "s": [80,255], "v": [60,255]},
  "green_calyx": {"h": [35,85],  "s": [60,255], "v": [40,255]}
}
```

`fruit_detector.py` loads this file at import time; if absent, falls back to the defaults baked into its constants.

**Work estimate**: 0.5 days.

### 3.4 `python/survey_capture.py`

**Purpose**: One-shot entry point called by `picker_viewer.py` each time the annotated image needs to refresh (session start, after each pick).

**Interface**:
```python
def capture_fruits(driver: QArmDriver, camera: QArmCamera,
                   session_cal: dict) -> tuple[list[Detection], dict]:
    """Returns (detections, diagnostics).
    diagnostics includes chessboard_residual_px and warnings list."""
```

**Flow**:
1. `slow_move_to_joints(driver, survey_pose_joints)`.
2. Camera warm-up (20 frames).
3. Capture RGB + depth (median of 5 frames for RGB; depth as-is).
4. **Self-check**: re-run `findChessboardCorners`; if corners found, compute reprojection residual with the stored H. Warn if > 3 mm (chessboard or camera shifted), error-out if > 10 mm.
5. Call `fruit_detector.detect_fruits(color, depth, session_cal)`.
6. Return detections + diagnostics.

**Work estimate**: 0.5 days.

### 3.5 `python/picker_viewer.py`

**Purpose**: The operator-facing "click-to-pick" UI and the top-level loop of `main_final.py` in autonomous mode.

**Interface**:
```python
def run_picker_loop(driver: QArmDriver, camera: QArmCamera,
                    session_cal: dict, controller: FruitSortingController) -> None: ...
```

**Flow**:
1. Initial capture: call `survey_capture.capture_fruits()` → `(detections, diag)`.
2. Render annotated frame:
   - For each detection, draw bbox + label `"{type} ({conf:.2f})"` with colour per type (banana=yellow, tomato=red, strawberry=pink).
   - Draw chessboard outline if visible (green if residual < 3 mm, amber if > 3 mm, red if > 10 mm).
   - HUD overlay: `N fruits | residual X.X mm | click a fruit or ESC to quit`.
   - If `diag.warnings` non-empty, draw them in amber at top.
3. `cv2.namedWindow("Picker")` + `cv2.setMouseCallback`.
4. Event loop (`cv2.waitKey(30)`):
   - **Left click**: find nearest detection within 50 px radius.
     - If none: show toast "no fruit here — click closer" for 1.5 s, continue loop.
     - If found: call `_pick_one(detection)` (see below). Loop.
   - **`b` / `t` / `s` key** (category batch): filter current detections to that fruit type.
     - If zero matches: show toast "no {type} visible" for 1.5 s, continue loop.
     - Otherwise: call `_pick_category(type)` (see below). Loop.
   - **`r` key** (refresh): re-run `survey_capture` without picking (useful if operator repositions fruits mid-session).
   - **ESC key**: graceful exit — arm returns to `pickhome1`, window closes, function returns. If pressed during a category batch, the in-progress pick finishes; the remaining batch is cancelled; then the normal exit runs.
5. `_pick_one(detection)`:
   - Highlight that bbox in bright green for 200 ms.
   - Call `controller.pick_single(detection.center_base_m, fruit_type=detection.fruit_type)` — blocks until done. Viewer shows "picking {type}..." overlay.
   - On return: re-run `survey_capture` → refresh detections + frame.
6. `_pick_category(type)`:
   - Internal loop:
     ```
     retries_per_target = {}
     while not ESC_pressed:
         dets = survey_capture.capture_fruits(...)
         matches = [d for d in dets if d.fruit_type == type]
         if not matches: break                                # done
         next = min(matches, key=lambda d: dist_from_home(d))
         key = pixel_key(next)                                 # (cx,cy) rounded to 5px
         if retries_per_target.get(key, 0) >= 2:               # stuck on same blob
             show_toast(f"skipping stuck {type} at {key}", 2s)
             mark_skipped(key)
             continue
         _pick_one(next)
         if pick_succeeded: retries_per_target.pop(key, None)
         else: retries_per_target[key] = retries_per_target.get(key, 0) + 1
     ```
   - Hard cap: total picks in one batch ≤ 20 (defensive upper bound for a table that never has more than 14 fruits).
   - HUD overlay during batch: `"auto-picking {type}: {n_done}/{n_total_seen}"`.
7. Errors during any pick (IK fail, gripper fail, controller exception): print trace, show red banner "pick failed: {reason}" for 3 s, re-capture, loop. Never crash the viewer.

**Click-to-detection matching**: Euclidean distance in image pixels between click and each detection's `center_px`. Nearest within 50 px wins (50 px ≈ 25 mm at SURVEY_POSE's ~0.6 m range — roughly the radius of a small tomato).

**Controller API extension**: `FruitSortingController` needs a new public method `pick_single(base_xyz: np.ndarray, fruit_type: str) -> bool` that runs ONE pick-place cycle synchronously. This is a thin wrapper around the existing FSM — set the fruit queue to `[target]`, drive the FSM to DONE, return. The existing FSM state machine is unchanged; only the entry point is added. `sorting_controller_sim.py` gets the same method for parity.

**Work estimate**: 1 day.

---

## 4. Modified modules

### 4.1 `python/main_final.py`

- On startup: load `session_cal.json`. If missing or older than 8 hours → print "run calibrate_chessboard.py first" and exit (no auto-cal — calibration always requires a human for the jog-touch step anyway).
- Entry point is now: connect driver + camera → call `picker_viewer.run_picker_loop(...)` → graceful shutdown.
- Hardcoded demo fruit queue: **deleted**.
- Preserve `--dry-run` flag (no arm movement — picker just prints clicks + would-be commands to stdout, useful for offline UI demos). `--pick-only` is removed as it no longer has meaning under click-to-pick.

**4.1.1 Interaction contract**: One input (click or category key) = one or more pick cycles, always synchronous. Re-capture happens between every pick — both single and batch — so the detector always acts on a fresh view. Why: (a) every pick is supervised at the category level or finer, so a bad detection never turns into an unrecoverable multi-pick cascade (operator can press ESC to cut a batch short); (b) re-capturing between picks eliminates the "fruit got bumped" failure mode that would otherwise require explicit re-capturing on a timer; (c) the category-batch and single-click paths share one underlying `pick_single` implementation — no duplicated motion logic.

### 4.2 `python/preflight.py`

- **Remove** check #3 (UGreen) and check #7 (VisualRef).
- **Replace** check #4 (Calib) with a new `check_session_cal`:
  - File exists? Age < 12 h? `homography_reproj_rms_px < 2`? Survey pose exists in teach_points?
- **Add** check #3' `check_chessboard_still_visible`: move arm to survey pose (only in non-offline mode), capture one D415 frame, look for chessboard corners, report residual vs stored H.
- Check #5 (HSV) kept but enhanced: print per-class blob count + highest confidence blob per class.
- Check #6 (Tests) kept, runs new test files.

### 4.3 `teach_points.json`

Add one new entry:
```json
"survey1": {
    "joints_rad": [...],
    "gripper": 0.10,
    "notes": "Overhead survey pose. Frames the workspace + chessboard corner. Teach in lab."
}
```

Taught in the lab during D4. Camera must see:
- The whole intended fruit-placement area (target: ~0.4 × 0.3 m).
- The chessboard in one corner of the frame, all 24 inner corners visible, no extreme oblique angle (chessboard plane angle < 30° from the image plane — enforced by a print during `calibrate_chessboard.py`).

### 4.4 `matlab_facade/py_controller.m`

The hardcoded demo fruit queue is deleted. The Simulink facade's autonomous mode does **not** replicate the click-to-pick UI (OpenCV mouse callbacks can't be driven through Simulink sensibly). Instead:

- Simulink autonomous mode becomes a thin wrapper: on mode enter, it shells out to `main_final.py` (which runs the picker loop in its own OpenCV window) and waits for it to exit. This keeps Simulink as the top-level control surface for rubric purposes while the real UI runs in Python — consistent with the existing Simulink-is-a-facade architecture.
- Remove the hardcoded demo fruit queue from `py_controller.m`.
- Remove `py_detect_test.m` (no longer needed — real detection runs in the Python picker).

This is a reduction in Simulink complexity, not an increase.

### 4.5 `LAB_RUNBOOK.md`

Delete sections:
- "Closed-loop calibration (once per session)"
- "UGreen intrinsic calibration"
- "Visual reference snapshot"

Add section:
```
## Session calibration (chessboard, ~30 s)

1. Place chessboard (7x5, 30mm) flat on the table. Clip the top-left inner corner
   to a fixed reference mark on the table (masking tape cross is fine).
2. Place fruits anywhere in the chessboard-free region.
3. Run:
       py -3.13 python/calibrate_chessboard.py
   - Jog gripper tip to the top-left inner corner, press ENTER.
   - Arm moves to survey1 and snaps the calibration photo.
   - Check reported reprojection RMS < 2 px. If not, reteach survey1.
```

---

## 5. Deletions

### 5.1 Python files
- `python/ugreen_tracker.py`
- `python/ugreen_intrinsics.py`
- `python/calibrate_closed_loop.py`
- `python/probe_ugreen.py`
- `python/setup_baseline_ref.py`
- `python/test_ugreen_tracker.py`
- `python/test_calibrate_closed_loop.py`

### 5.2 Artifacts
- `logs/ugreen_baseline.png`
- `logs/ugreen_pickhome1_reference.png`
- `logs/ugreen_pickhome1_tcp.json`
- `calibration.json` and all `calibration_bak_*.json`
- `ugreen_intrinsics.json` (if present)

### 5.3 Teach points
- `cal_01` … `cal_08` in `teach_points.json` (no longer needed — no closed-loop calibration fruits).

### 5.4 Documentation
- SPRINT_REPORT_2026-04-20.md: UGreen sections still valid as history; retain.
- Update PROGRESS.md and PROJECT_CONTEXT.md to reflect the new pipeline (part of D8 work).

---

## 6. Testing plan

### 6.1 Unit tests (dev machine, no hardware)

`python/test_calibrate_chessboard.py` (new):
- Synthetic chessboard image + known homography → recovered H within 1 px RMS.
- Given a known `T_chess_origin_in_base` and a point in chess XY → `base XYZ` round-trips correctly.

`python/test_fruit_detector.py` (new — replaces parts of the old `test_integration.py` vision checks):
- Synthetic images: solid yellow elongated rectangle → 1 banana, zero tomatoes, zero strawberries.
- Synthetic red circle → 1 tomato; synthetic red triangle with green top-stripe → 1 strawberry.
- Parallax: given known `fruit_top_z` and known principal point, the corrected pixel shift matches the formula.
- Depth outlier rejection: 5×5 patch with 10 zeros + 15 valid → should return median of valid; with 20 zeros → reject.

`python/test_survey_capture.py` (new):
- Chessboard residual threshold triggers the right warning/error bands.

### 6.2 Integration test (lab, D4)

Acceptance criteria for "pipeline works":
1. `calibrate_chessboard.py` completes with RMS < 2 px.
2. **Click path**: `main_final.py` opens the picker. Operator click-picks 2 fruits of each type (6 total). Each pick: IK reaches target within 10 mm, gripper closes on fruit, fruit delivered to the correct basket.
3. **Category path**: Fresh placement of 14 fruits (6 strawberry + 3 banana + 5 tomato). Operator presses `b` → arm autonomously picks all 3 bananas → stops. Presses `t` → all 5 tomatoes. Presses `s` → all 6 strawberries. ≤ 2 total pick retries across the whole 14-fruit run.
4. Chessboard residual stays < 3 mm throughout (printed in HUD).
5. Re-capture between picks successfully removes the just-picked fruit from the detection list.
6. ESC during a category batch cleanly halts after the in-progress pick finishes.

If (2) or (3) fails on a specific fruit class, return to `hsv_tuner.py` and re-tune that class, then retry.

---

## 7. Risks and mitigations

### 7.1 D415 depth at fruit edges

Depth stream is invalid (0) at object edges and in shadows. Mitigation: sample 5×5 median at blob *center* (not edge), reject detection if < 8 valid samples.

**Residual risk**: small strawberries whose bounding box is < 20 px across may have a center patch that's all edge. Open: may need to sample the whole blob's valid depth pixels and take median of the full distribution, not just a 5×5 patch. Defer decision to D4 lab session.

### 7.2 Green calyx occluded

A strawberry seen from directly above may have its calyx hidden under a leaf on top. Mitigation: if `green_above_ratio < 0.05` but the blob is small (< 2000 px²) and tapered (taper score < 0.9), call it a strawberry anyway with reduced confidence. Ambiguous case: a very small tomato could trip this. Mitigate by size gating: strawberries never exceed 4000 px² at survey pose; tomatoes typically exceed 3000 px². Overlap region [3000, 4000] relies on the taper score.

### 7.3 Survey pose doesn't frame both workspace and chessboard

Possible at lab, depending on D415 field of view. Mitigation: pre-compute expected FOV using D415 intrinsics at a range of poses and pick the one that frames a 0.5 × 0.4 m region. If even that's not enough, reduce workspace to 0.35 × 0.25 m — still enough for 14 fruits.

### 7.4 Chessboard gets bumped mid-session

Covered by the per-cycle residual self-check. If residual spikes > 10 mm, `survey_capture` errors out and prints "rerun calibrate_chessboard.py". No autonomous recovery — operator intervention required.

### 7.5 Operator clicks during arm motion

The viewer window is frozen (events not serviced) while `pick_single` runs synchronously. If the operator clicks or presses keys during this window, those events are silently dropped by OpenCV — no queue buildup. Confirmed safe. Documented in picker HUD: overlay text "picking {type}... (input ignored)" during the synchronous phase. The one exception is ESC, which is *checked between picks* inside a batch loop — so a long category batch can be aborted cleanly by holding ESC until the viewer services input.

### 7.6 Click ambiguity when fruits cluster

If two small strawberries are within 50 px of a click point, the nearest-centre wins. If centres coincide (overlapping fruits), the detector would have merged them into one blob anyway, so this case reduces to "detector under-segmented" — a detection problem, not a picker problem. Mitigation belongs in `fruit_detector.py` (morphology tuning), not in the picker.

### 7.7 Python 3.13 OpenCV call drift

OpenCV 4.13 on Python 3.13 — preflight has already confirmed `findChessboardCorners` and HSV ops work. No new OpenCV APIs used beyond what's already running. Low risk.

---

## 8. Timeline

| Day | Date | Task |
|-----|------|------|
| D1 | 2026-04-22 (today) | `calibrate_chessboard.py` + `session_cal` schema + `test_calibrate_chessboard.py` |
| D2 | 2026-04-23 | `fruit_detector.py` rewrite + `test_fruit_detector.py` (full day, 1.5-day scope compressed by reusing existing HSV-range tuning code) |
| D3 | 2026-04-24 | `survey_capture.py` + `picker_viewer.py` + `sorting_controller.pick_single` extension + `main_final.py` wiring |
| D4 | 2026-04-25 (Sat) | **Lab AM**: teach `survey1`, run `hsv_tuner.py` to tune in lab lighting. **PM**: end-to-end dry-run, 3-of-each acceptance test |
| D5 | 2026-04-26 (Sun) | **Lab buffer**: fix whatever D4 surfaces; record demo video if pipeline stable |
| D6 | 2026-04-27 | Simulink facade hookup (autonomous mode shells out to `main_final.py`); remove UGreen from facade wiring |
| D7 | 2026-04-28 | **Lab backup day** (if D5 buffer got consumed). Otherwise: demo video polish, Simulink end-to-end test |
| D8 | 2026-04-29 | Delete UGreen modules + tests + artifacts; update PROGRESS.md / PROJECT_CONTEXT.md / LAB_RUNBOOK.md |
| D9 | 2026-04-30 | Report final pass, risk assessment form, buffer |
| — | 2026-05-01 14:00 | **Submit** |

**Note on `hsv_tuner.py`**: moved from D3 to D4-AM. It's an interactive tool that only makes sense with real D415 + lab lighting; writing it offline is wasted work. D4 morning writes it (30 min in an interactive REPL-style) and uses it immediately.

---

## 9. Open questions (to resolve during implementation)

1. **Fruit height table for depth sanity check**: actual typical sizes of lab strawberries/tomatoes/bananas. Measure in D4 and bake into `fruit_detector.py` gating.
2. **Homography decomposition for camera height**: `cv2.decomposeHomographyMat` returns up to 4 solutions; need a disambiguation heuristic (expect z-normal pointing up, chess plane below camera). Implement + validate on D1.
3. **Click radius tuning**: 50 px is a starting guess. If operators find it too hard to "miss" (adjacent fruits trigger wrong pick) lower it; if clicks often return "no fruit here" raise it. Defer to D4.
4. **Within-batch ordering**: current plan is "nearest-to-home first". If the arm ends up doing a zig-zag travel pattern that stresses the shoulder joint, switch to "nearest-to-last-pick" (travelling-salesman-ish greedy). Cheap to swap in `_pick_category`. Defer to D4.
