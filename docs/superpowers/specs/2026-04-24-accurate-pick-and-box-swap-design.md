# 2026-04-24 — Accurate-Pick Projection Rewrite + Plastic-Box Target Swap + Orientation-Aware Grasp

**Status:** design, awaiting implementation plan
**Branch at spec time:** `applied_robot_vision`
**Deadline:** 2026-05-01

## 1. Motivation

Three linked problems are blocking final delivery:

1. **Pick accuracy is position-dependent.** With the current `fruit_detector.pixel_to_base_frame` (nadir-pinhole parallax approximation) and a hand-patched `chess_origin_in_base_m` in `session_cal.json`, only fruit placed on the `x ≈ 0.35 m` line from the base can be picked reliably. Fruit placed anywhere else drifts by up to 10 cm. Root cause: the D415 is mounted on the wrist with (a) a lateral offset from the TCP and (b) a forward-down tilt — it is **not nadir** — so the nadir-pinhole formula is geometrically wrong, and the hand patch happened to cancel the error only at a single `x`.
2. **Photo-localization fires before the arm settles.** `survey_capture.capture_fruits()` calls `slow_move_to_joints(…, 0.10)` and immediately runs `_warmup_and_capture()`. QArm position-mode PID has ~0.5–1.0 s tracking lag; the resulting image is taken mid-motion and the detected fruit positions drift with whichever direction the arm is still moving in.
3. **Target set needs to change.** The "strawberry" class is being replaced by a 17.5 × 11.5 cm transparent plastic clamshell box (Aldi strawberry pack — with a clear plastic tray + printed plastic film wrap covering red strawberries inside). The existing red-HSV + green-calyx detector cannot distinguish this box from a tomato, and the box is large enough that a robust grasp needs wrist orientation control.

## 2. Priority tiers and scope

Scope is split into three priority tiers. **P0 is the minimum shippable deliverable**; the team will stop after P0, run a lab verification, and only then start P1 and (if time remains) P2.

| Tier | Outcome | Est. effort |
|------|---------|-------------|
| **P0** | System can pick **existing three classes** (banana/tomato/strawberry) accurately at any position in the workspace, and photos are taken only after the arm has settled. | ~2 days |
| **P1** | Strawberry replaced by `plastic_box` detection (rectangular contour + white-label confirm), tomato de-confused, place target rewired. | ~2 days |
| **P2** | Banana and plastic_box grasps use wrist rotation to align with the object's short-edge direction (the gripper-closing axis). | ~1–2 days |

`Detection.grasp_angle_rad` is introduced in P1 as a data-model stub defaulted to 0.0, then filled in P2 — this keeps P1 and P2 independent of each other.

Deliverables that are explicitly **out of scope**:

- Full Tsai-Lenz hand-eye calibration (detection only happens at `survey1`, so a single per-pose extrinsic is sufficient).
- HSV retuning beyond what's needed to make box/tomato separable in lab lighting.
- Simulink façade / `main_final` workflow changes beyond wiring the new keyboard key.

## 3. P0 — solvePnP projection rewrite + capture settle

### 3.1 Geometric model

Replace the nadir-pinhole parallax formula in `fruit_detector.pixel_to_base_frame` with:

```
# Calibration step (run once per session during preflight, at survey1 pose):
corners_2d  = findChessboardCornersSB(image)              # (N, 2) pixel
corners_3d  = chess_grid_points_base_frame(session_cal)    # (N, 3) base metres,
                                                           # derived from clean
                                                           # chess_origin_in_base_m
                                                           # (hand patch removed)
ok, rvec, tvec = cv2.solvePnP(corners_3d, corners_2d, K, dist_coeffs)
R_chess_to_cam = cv2.Rodrigues(rvec)[0]
# Invert so we have camera pose in base frame:
R_cam_in_base  = R_chess_to_cam.T
C_cam_in_base  = -R_chess_to_cam.T @ tvec    # camera optical centre, base metres
# Persist both in session_cal.json.cam_extrinsics_survey1

# Detection-time projection per pixel (u, v) with known fruit_top_z:
ray_cam  = normalize([(u - K.cx)/K.fx, (v - K.cy)/K.fy, 1.0])
ray_base = R_cam_in_base @ ray_cam
λ        = (fruit_top_z - C_cam_in_base.z) / ray_base.z
target   = C_cam_in_base + λ * ray_base
```

`fruit_top_z` is still sourced from D415 depth at the blob centre with the existing per-class fallback (25/50/25 mm); that logic stays.

### 3.2 Alternatives considered and rejected

- **Homography + constant TCP offset.** Only corrects the mean lateral offset; still wrong at any `z` other than the chessboard plane. Treats the symptom, not the cause.
- **Full Tsai-Lenz hand-eye.** Requires multi-pose chessboard capture and solves a larger system. Correct but over-engineered: detection only happens at `survey1`, so a single pose extrinsic is equivalent and far cheaper.

### 3.3 Error handling

- `solvePnP` returns `ok=False` or reprojection RMS > 5 px → mark `session_cal.calibration_valid = False`, preflight refuses to start the picker.
- Runtime ray hits `ray_base.z ≥ 0` (ray horizontal or upward) or `λ < 0` (target behind camera) → return `None` for that detection; detector drops the candidate with a warning.

### 3.4 Capture settle

In `survey_capture.capture_fruits()`, immediately after `slow_move_to_joints(...)`:

```python
time.sleep(1.5)
joints, _ = driver.read_all()
err = np.linalg.norm(np.asarray(joints) - np.asarray(session_cal.survey_pose_joints_rad))
if err > 0.05:   # ~2.9° total joint-norm tolerance
    raise RuntimeError(
        f"arm failed to settle at survey1 (joint-norm err {err:.3f} rad)"
    )
```

`picker_viewer.run_picker_loop()` already routes `'r'` to `capture_fruits()`, so the refresh key inherits settle + sanity check with no UI change.

### 3.5 P0 files touched

- `python/session_cal.py` — add `cam_extrinsics_survey1: {R: 3x3, t: 3}` field; keep `camera_height_above_table_m` readable for backward compat but no code path consumes it.
- `python/calibrate_extrinsics.py` *(new)* — `solve_survey1_extrinsics(bgr, session_cal, K, dist) -> None` (mutates + persists).
- `python/preflight.py` (or whichever module runs the calibrate session) — call the new solver after chessboard is detected.
- `python/fruit_detector.py` — replace body of `pixel_to_base_frame`; remove the nadir-pinhole correction block.
- `python/survey_capture.py` — add dwell + sanity check at top of `capture_fruits`.
- `teach_points.json` / `session_cal.json` — drop any manually patched `chess_origin_in_base_m` offset (values will be re-measured in the lab step).

### 3.6 P0 tests

- **Unit, synthetic.** Build a known `(R, t, K)`, forward-project a chessboard grid, call `cv2.solvePnP`, assert reconstructed origin within 1 mm and axes within 0.5°.
- **Unit, ray-plane.** For known camera pose and table plane, feed chosen `(u, v, z)` and assert `target` matches closed-form answer to µm.
- **Unit, settle.** Mock driver returns joints that differ from target by 0.1 rad → assert `capture_fruits` raises `RuntimeError`.
- **Lab acceptance (blocks shipping P0).** Chessboard placed at three lab positions (centre, left-front, right-back); measure real vs solvePnP for each — pass when all three ≤ 5 mm.

## 4. P1 — plastic_box detection + strawberry removal + tomato de-confusion

### 4.1 Detection pipeline changes

`python/fruit_detector.py`:

- Remove `_detect_strawberry_contours` and its `HSV_RANGES["strawberry"]` entry. `green_calyx` range stays (cheap, unused but harmless).
- Add `_detect_plastic_box(bgr, depth, session_cal) -> list[Detection]`:
  1. `gray = cvtColor(bgr, GRAY)`
  2. `edges = cv2.Canny(gray, 60, 150)`
  3. `contours, _ = cv2.findContours(edges, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)`
  4. For each contour: `(cx, cy), (w, h), angle = cv2.minAreaRect(cnt)`; filter by `area_px ∈ [15000, 60000]` and `max(w,h)/min(w,h) ∈ [1.3, 1.8]` (initial gate; lab-tuned).
  5. In the rotated ROI, confirm white-label presence: share of pixels with `HSV.V > 200 ∧ HSV.S < 40` must exceed 0.08.
  6. Emit `Detection(fruit_type="plastic_box", center_px=(cx,cy), ...)` with `grasp_angle_rad = 0.0` (filled in P2).
- Modify `_detect_tomato_contours(bgr, hsv, boxes=()) -> list[Detection]`: before running the existing circularity / calyx logic, blank out every box's rotated bounding box on the red mask (`cv2.drawContours(mask, [cv2.boxPoints(rect).astype(int)], 0, 0, -1)`).
- `detect_fruits` orchestration order: `boxes → bananas → tomatoes(exclude=boxes)`; final list is the concatenation.

`cv2.minAreaRect` angle convention varies between OpenCV 4.5 and 4.6+; normalize to "angle of the long edge in image coords, in `[-π/2, π/2)`" with a dedicated helper and unit-test against both conventions.

### 4.2 Controller and UI changes

`python/sorting_controller.py`:

```python
BASKETS = {
    'banana':      np.array([-0.30, -0.20, 0.05]),
    'tomato':      np.array([ 0.00, -0.35, 0.05]),
    'plastic_box': np.array([ 0.00,  0.00, 0.00]),   # LAB-TEACH REQUIRED
}
```

The `(0, 0, 0)` placeholder will fail the IK reachability check at runtime — by design, the system refuses to place a box until the teach point is filled. `teach_points.json`: remove `placeberries`, add `placebox` with a lab-measured pose.

`python/picker_viewer.py`: `'s'` key (strawberry batch) becomes `'p'` (plastic_box batch). All `"strawberry"` string references across `sorting_controller.py`, `picker_viewer.py`, `fruit_detector.py`, and every test file are removed. `Detection.fruit_type` legal set is `{"banana", "tomato", "plastic_box"}`.

### 4.3 P1 tests

- **Unit.** `test_fruit_detector.py` gains `test_plastic_box_rectangular_contour` (synthetic 175×115-ish rectangle with white label patches, assert one `Detection` with `fruit_type=="plastic_box"`) and `test_tomato_excluded_inside_box` (red blob inside a detected box bounding box does not emit a `Detection`).
- **Integration.** `test_integration.py` and `test_pick_single.py` — replace every `'strawberry'` with `'plastic_box'`, assert go-to-basket uses `BASKETS['plastic_box']`.
- **Lab acceptance.** Place box + tomato + banana in the same frame, run the picker, verify all three categorize correctly and pick succeeds (basket position for box is lab-taught and filled into `teach_points.json` before this step).

## 5. P2 — orientation-aware grasp

### 5.1 Data flow

`Detection.grasp_angle_rad` is computed at detection time and consumed by the controller when it builds the pick pose.

- `_detect_plastic_box` / `_detect_banana_contours`: take the `minAreaRect` long-edge endpoints (two pixels on the long axis), run each through the P0 ray-plane projector to get two base-frame points, compute `θ_long = atan2(Δy_base, Δx_base)`, then set `grasp_angle_rad = wrap_to_wrist_range(θ_long + π/2)`. If the wrapped angle exceeds `±160°` (QArm wrist limit), fall back to `0.0` and log a warning — the pick will still be attempted at the default pose.
- `_detect_tomato_contours`: `grasp_angle_rad = 0.0` (round → no preferred orientation).

This "project two pixels, atan2" formulation avoids the approximation of using pixel-space angle directly; at non-nadir camera poses the direct formula can be off by up to ~5°, enough to bite with a 11.5 cm gap on the box.

### 5.2 Controller wiring

`sorting_controller.py`:

- Change `_start_move` (and any other site that calls `inverse_kinematics(target, gamma=0.0)`) to receive the full `Detection` object or at least a target `gamma`.
- `_ik_safe` pass-through: gamma must survive the reachability check and be reported in any trace event.

Important subtlety — **`gamma` in `inverse_kinematics` is the wrist joint angle `phi[3]`, not a base-frame angle**, but `Detection.grasp_angle_rad` is computed in base frame. The two are related through the arm's yaw at pick time and a fixed gripper-mount offset:

```
phi[3] = grasp_angle_rad_base  -  phi[0]_at_pick  -  gripper_mount_offset
```

`phi[0]_at_pick = atan2(target.y, target.x)` (arm yaws to face the target). `gripper_mount_offset` is a per-robot constant (how the gripper closes relative to the forearm when `phi[3] = 0`); it must be calibrated once with a short lab test (place an object with known orientation, sweep `phi[3]`, find the value where jaws align with the object). Store the offset in `session_cal.json` as `gripper_mount_offset_rad`; document the calibration procedure in `LAB_RUNBOOK.md`.

`_start_move` therefore computes `gamma = wrap_to_wrist_range(detection.grasp_angle_rad - atan2(ty, tx) - session_cal.gripper_mount_offset_rad)` before calling IK. The same wrist-limit fallback (§5.1) applies after this conversion.

### 5.3 P2 tests

- **Unit.** Synthesize a rectangle rotated by known θ ∈ {0°, 30°, 60°, 89°}; assert `grasp_angle_rad` matches within ±2°.
- **Integration.** `MockQArm.cmd_log` inspected for `joints[3]` — assert it equals the commanded `gamma` for a non-zero detection, and `0.0` for a tomato detection.
- **Lab acceptance.** Place the box at three orientations (long edge at 0°, 45°, 90° to base x) and a banana at two (0°, 45°); verify the wrist rotates before descent and grip succeeds.

## 6. Cross-cutting concerns

### 6.1 Regression gates

All unit and integration tests in `python/test_*.py` are expected to pass on every PR. Any test that depends on a real D415 (`test_survey_capture.py` beyond Mock level) is gated behind a `@pytest.mark.hardware` flag so CI stays green.

### 6.2 session_cal.json migration

Sessions saved before this change carry `camera_height_above_table_m` but no `cam_extrinsics_survey1`. Load-time behaviour: if `cam_extrinsics_survey1` is missing, `SessionCal.load` sets `calibration_valid = False` and the picker refuses to start — user must re-run preflight. No silent fallback to the old formula.

### 6.3 Lab runbook update

`LAB_RUNBOOK.md` gains one step after "chessboard capture": "Run `solve_survey1_extrinsics`, verify reprojection RMS ≤ 5 px and camera height consistent with tape-measure to ±2 cm." `COMPLETE_GUIDE.md` refresh is deferred to a final cleanup pass.

### 6.4 Docs

`docs/PROGRESS.md` gets a new row per tier as it completes. This spec lives at `docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md`.

## 7. Risks

- **D415 intrinsics drift.** solvePnP consumes `K`; if the stored intrinsics are stale, the extrinsic solve compensates with a biased `(R, t)` and the picks are globally shifted. Mitigation: preflight prints `fx, fy, cx, cy`; lab runbook has a tape-measure sanity check on camera height.
- **Box detection false positives on other clutter.** White-label confirmation plus aspect-ratio gate should be sufficient, but if the lab table has other rectangular white objects (notebook, box lid), confusion is possible. Mitigation: the fruit-only workspace is strict; clutter lives off-table.
- **P2 wrist-angle wrap.** If `θ_long + π/2` exceeds wrist limits, fallback to 0 means the long-axis grip is attempted at a potentially bad angle. In practice this happens only at arbitrary planar rotations near wrist extremes; lab will show whether it matters.

## 8. Acceptance criteria (shipping)

- **P0 shipped** when lab acceptance test in §3.6 passes (≤ 5 mm error at three chessboard positions) and all §3.6 unit tests are green.
- **P1 shipped** when §4.3 lab acceptance passes and `'plastic_box'` is the only non-banana, non-tomato class anywhere in the code.
- **P2 shipped** when §5.3 lab acceptance passes across five orientations.
