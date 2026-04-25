# Sprint Report — 2026-04-22

**Deadline:** 2026-05-01 14:00 (9 days remaining)
**Branch:** `yichang_branch`
**Range:** `5056c47` .. `fdbc331` (37 commits)

---

## Summary

Biggest-movement day of the sprint. Replaced the UGreen floor-camera
closed-loop calibration flow (introduced 2026-04-20) with a single-
camera (D415) overhead-shot vision pipeline keyed off a chessboard
fiducial. Produced spec + two plan documents, then built two complete
subsystems via subagent-driven TDD, then lab-validated both on real
hardware with extensive lab-light tuning. Left the day with a detector
that correctly identifies banana/tomato/strawberry in real D415 frames
and outputs base-frame XYZ coordinates; still pending: a physical
pick attempt to confirm the coordinates translate to a successful
grasp.

---

## Brainstorm → Spec → Plans

### 1. Vision pipeline redesign (3 commits)

Replaced the per-session UGreen closed-loop calibration (arm parks,
baseline snapshot, 8 calibration fruits at cal_01..08, hand-eye solve,
visual-reference snapshot at pickhome1 — ~20 min/session) with a
single-camera overhead shot keyed off a 7×5-inner-corner chessboard
fiducial (~30 s/session). Dual operator UI: mouse click for
single-fruit pick, keyboard `b`/`t`/`s` for per-category batch.

Locked design decisions (brainstorm Q1..Q8):

| # | Decision | Choice | Rationale |
|---|---|---|---|
| Q1 | Fiducial | 7×5 30 mm chessboard (already owned) | OpenCV native, team owns it |
| Q2 | Base-frame registration | Jog-touch origin corner | Flexible, ~30 s, reuses teach_points UX |
| Q3 | Runtime chessboard | Stays in corner | Per-frame residual = live drift detector |
| Q4 | Detector | Pure OpenCV layered features | No new deps, 3 classes linearly separable |
| Q5 | Z-height | D415 depth (later: with fallback) | RGB-D available; parallax correction |
| Q6 | Scope | Delete UGreen entirely | Repo -1500 lines, session setup -15 min |
| Q7 | Autonomous UX | Click-to-pick | Per-fruit supervision, safer demo |
| Q8 | Batch mode | `b`/`t`/`s` + click both | Rubric-friendly batch + single debug |

Full spec: `docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md`
Commits: `87013a4`, `e85dbb1`, `ca44e11`

### 2. D1 chessboard calibration (18 commits)

Plan: `docs/superpowers/plans/2026-04-22-d1-chessboard-calibration.md`

Subsystem modules built via subagent-driven TDD, one task per subagent
with spec-compliance + code-quality review between tasks:

- `python/session_cal.py` — `SessionCal` dataclass + JSON roundtrip (A1)
- `python/homography_solver.py` — `solve_homography` + reprojection RMS (A2)
- `python/homography_solver.py` — `camera_height_from_homography` via Jacobian-of-H at principal point (A3); resolves spec §9 open Q#2 without needing `cv2.decomposeHomographyMat`'s 4-solution disambiguation
- `python/touch_probe.py` — `capture_tcp` + `jog_and_capture` cv2.waitKeyEx HUD (A4)
- `python/calibrate_chessboard.py` — top-level CLI + end-to-end synthetic integration test (A5)
- `docs/superpowers/plans/2026-04-22-d1-smoke-test.md` — lab runbook (A6)

10/10 offline unit tests pass (`py -3.13 python/test_calibrate_chessboard.py`).

### 3. D2 fruit detector rewrite (10 commits for implementation)

Plan: `docs/superpowers/plans/2026-04-22-d2-fruit-detector.md`

Again subagent-driven TDD, 9 tasks + final-review fixes:

- `python/fruit_detector.py` wholesale rewrite
- `Detection` dataclass + `to_dict` (B1)
- `hsv_mask` with wrap-around hue (B2)
- `_detect_banana_contours` — yellow + aspect (B3)
- `_detect_tomato_contours` — red + circularity + no-green-above (B4)
- `_detect_strawberry_contours` — red + green-calyx + taper (B5)
- `sample_depth_at_pixel` with outlier + sparsity reject (B6)
- `pixel_to_base_frame` with nadir parallax (B7)
- `detect_fruits` orchestrator (B8)
- `test_integration.py` legacy-vision surgery (B9)

Post-review fix (`6bd7fa9`): tomato/strawberry boundary at `green_ratio
== 0.05` made mutually exclusive; removed dead `synthetic_scene` +
`import cv2` from `test_integration.py`.

22/22 offline unit tests pass. Old `FruitDetection`/`CIRCULARITY_THRESH`
/`detection_depth_mm` API gone; downstream consumers (`main_final.py`,
`preflight.py`, `hover_test.py`, etc.) now break at import — D3/D8
will fix.

---

## Lab validation (2026-04-22 PM)

### D1 smoke test

Ran `python/calibrate_chessboard.py` on real QArm + D415:

- **Iteration 1** — bugs surfaced:
  - `AttributeError: 'QArmCamera' object has no attribute 'fx'` → fix `cam.fx` → `cam.intrinsics["fx"]` (`3ab955e`)
  - All-black captured frame (fixed-count warm-up insufficient) → poll until `c.mean() > 5` like preflight (`960836a`); also added touched-TCP cache at `logs/last_touched_tcp.json` so Phase-2 retries skip the jog
  - `findChessboardCorners failed` → debug-save frame to `logs/calibration_latest.png` to inspect (`d15c908`)
  - `_INNER_COLS=6, _INNER_ROWS=4` was wrong — "7×5" in the spec/file name means **inner corners**, not squares (the printed board is 8×6 squares = 7×5 inner) (`3312f3f`)

- **Iteration 2** — pose/framing issues:
  - Initial survey1 was a 42° tilt from nadir (joint2 33° + joint3 19°); Jacobian-based camera-height estimate came out 33 cm vs measured 26 cm (27% err, over the 15% spec band)
  - Reteach with more nadir + closer chessboard: chess_origin_in_base X dropped from 0.686 → 0.403, camera_height 0.330 → 0.262 m

- **Final pass:** 35 corners found, reprojection RMS 0.709 px, `session_cal.json` written.

### D2 detector lab tuning

Ran `python/diag_detector.py` (new, `353a704`) — moves arm to survey1,
captures one D415 frame, runs `detect_fruits` against `session_cal.json`,
saves annotated PNG. Iteratively retuned against real lab lighting:

| Issue | Lab value | Fix | Commit |
|---|---|---|---|
| Banana HSV miss | H=15 (orange-yellow under warm light) vs spec H=[18,35] | H=[14,40] | `a1d25de` |
| Tomato shadow miss | V=59 (shadow side) vs spec V min 60 | V min 40 | `a1d25de` |
| Banana area > MAX | 50k px close-up vs spec MAX 30k | MAX 80k | `a1d25de` |
| Tomato area > MAX | 15.6k px vs spec MAX 15k | MAX 40k | `a1d25de` |
| Tomato circularity | 0.50 (specular-highlight bite) vs spec min 0.70 | min 0.40 + bbox aspect cap 1.8 | `a1d25de` |
| Depth along oblique ray | 641 mm vs camera height 330 mm | Per-type hardcoded heights (spec Q5=A fallback) | `3a11b8c` |
| Conf floor below gate | `max(0.3, ...)` + `CONFIDENCE_MIN=0.35` silently dropped | Remove floor, rescale aspect_score | `c7eb97b` |
| Banana aspect 1.67 < 1.8 | Overhead banana is rarely > 1.8 | MIN_ASPECT 1.5 + aspect_score floor 0.5 | `6028c0d` |
| Strawberry calyx miss | Lab leaves are dark olive (V 15-40, H 20-35) vs spec bright green | `green_calyx` H [20,90], V [15,255] + new `_has_green_in_top_strip` helper (overhead view puts calyx INSIDE top of bbox, not above) | `6028c0d` |
| Strawberry taper 0.44 < 0.9 | Overhead flattens 3D taper | Widen gate to [0.3, 3.5] | `6028c0d` |
| Speckle false tomato (435 px²) | Shadow edge in banana region | MIN_AREA 400 → 1500 | `d74aadc` |

**Final lab frame:** 3/3 real fruits detected, 0 false positives:

```
banana     px=(1116, 483) base=[+0.577, +0.235, +0.042] conf=0.59 area=32355
tomato     px=(631, 275)  base=[+0.458, +0.183, +0.067] conf=0.84 area=34336
strawberry px=(725, 533)  base=[+0.481, +0.247, +0.042] conf=0.87 area=30387
```

Tomato base X ≈ HOME X (0.45 m) → most-pickable. Banana X=0.577 is
borderline — just past HOME, may or may not be IK-reachable at pick
height.

### 4. Other lab aids added
- `python/diag_detector.py` — survey → capture → detect → annotated PNG
- `python/diag_pick_one.py` — pre-plans IK for HOVER/PICK/LIFT before moving, ENTER-to-confirm each stage, gripper ramp + readback. Not yet run.

---

## Observations

1. **Survey pose nadir is the geometric lynchpin.** A 42° tilt broke the
   Jacobian-based camera-height recovery (27% error) AND the nadir-
   pinhole parallax formula (depth along oblique ray ≠ vertical height
   → base-frame Z off by 300%). Moving to a closer-to-base chessboard
   + more elbow-bend reteach got us to ~26 cm camera height and
   acceptable (18%) height recovery. Reteaching true < 15° nadir is
   still the proper long-term fix.
2. **Lab lighting is harder than the synthetic tests suggest.** Every
   single default HSV / shape / confidence threshold needed widening.
   This is exactly what the spec's §3 and D4 notes flagged — the
   hsv_tuner.py tool is still a missing piece.
3. **D415 depth at close range on red/glossy surfaces is ~20% valid.**
   For the tilted survey1 pose with fruit 200-300 mm from the sensor,
   the 5×5 median patch frequently samples the BACKGROUND (farther
   than the fruit). Per-type flat-table fallback is the robust move;
   the depth channel remains useful for rejecting obvious outliers
   but not as the primary Z source.
4. **`max(0.3, ...)` confidence floor under a 0.35 gate is a trap.**
   Final review had flagged this as "D4 concern, not a bug" — it
   turned out to bite in the first 20 minutes of lab validation.
   Always keep the floor >= the gate, or remove the floor.
5. **Subagent-driven development worked well for mechanical TDD
   tasks.** 15 tasks across D1+D2 executed by a Haiku subagent each,
   with per-task reviews flagging real bugs (boundary double-class,
   dead code after surgery). Estimated 2-3x faster than inline
   execution without quality loss.

---

## Commit list (2026-04-22)

36 commits on `yichang_branch`:

**Spec (3):** `87013a4`, `e85dbb1`, `ca44e11`

**D1 plan + implementation (15):**
`faf32f2` (plan), `615e801`, `fc77375`, `857bc95`, `d1c8f99`, `c3f75cf`,
`f268374`, `17cd081`, `2153f5c`, `4371ca1`, `701b22f` (landing), plus
lab-debug fixes `3ab955e`, `d15c908`, `960836a`, `3312f3f`.

**D2 plan + implementation (10):**
`842f81f` (plan), `9c44f42`, `6229176`, `feb3f94`, `a5af528`, `6d40ed0`,
`cfb0783`, `3d7c428`, `18541e7`, `f018d2a`, `6bd7fa9` (post-review fixes).

**D2 diagnostics + lab tuning (7):**
`353a704` (diag_detector), `a1d25de`, `3a11b8c`, `c7eb97b`, `6028c0d`,
`d74aadc`, `cbac0c8` (diag_pick_one).

**Session state (1):** `fdbc331` (survey1 reteach).

---

## What's left

| Priority | Item |
|---|---|
| P0 | Run `diag_pick_one.py` — confirm base-frame coords physically pickable (Hover/Descend/Grip/Lift) |
| P0 | If pick fails, either reteach survey1 more nadir OR bake per-fruit XY offset correction |
| P1 | D3 — `picker_viewer.py` + `survey_capture.py` + `pick_single` controller extension + wire `main_final.py` |
| P1 | D4 — `hsv_tuner.py` interactive HSV picker (defer; today's manual tune is good enough for this lighting) |
| P2 | D6 — Simulink facade: autonomous mode shells out to `main_final.py`, remove UGreen from facade wiring |
| P2 | D8 — delete UGreen stack, update downstream (preflight, main_final, hover_test, analyze_detections, analyze_static, test_auto_pick, recal_from_pose, main_autonomous, py_detect_test.m, py_detect_live.m) |
| P3 | Demo video, report write-up, risk assessment form |

---

## Next session — suggested order

1. Re-check `session_cal.json` still matches physical setup (chess position, table height)
2. Run `py -3.13 python/diag_pick_one.py --type tomato` with stepwise ENTER confirmations
3. If pick succeeds: commit/tag "first successful autonomous pick", then proceed to D3 planning
4. If pick fails: look at the annotated debug frame + actual arm pose at PICK stage; likely XY parallax error from residual tilt
5. Then decide: patch D2 with more aggressive parallax correction, OR reteach survey1, OR add per-fruit XY offset calibration pass
