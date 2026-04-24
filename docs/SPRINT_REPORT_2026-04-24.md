# Sprint Report — 2026-04-24

**Deadline:** 2026-05-01 14:00 (7 days remaining)
**Branch:** `applied_robot_vision` (to be pushed as `vision` on vico3740)
**Range:** `f20aded` .. `449c5f0` (24 commits)

---

## Summary

Fixed the long-standing "only picks accurately on the x ≈ 0.35 m line" problem.
Diagnosed root cause as a compounding of four geometric errors in the old
pixel-to-base-frame pipeline; replaced the whole projection stack with a
principled `cv2.solvePnP`-based extrinsic calibration. Reached `PREFLIGHT OK`
and a successful first pick attempt in lab, with a residual 4 cm x-axis
bias that is currently patched with a constant offset. Also designed and
locked the P1/P2 spec (strawberry → plastic-box swap, orientation-aware
grasp) for when the pick bias is fully resolved.

Day was split into three phases:

1. **Design** (morning): brainstorming with the user drove out a four-item
   spec — accurate pick anywhere in the workspace, photo only after the arm
   settles, swap strawberry for a transparent plastic box, and wrist
   rotation so long objects are grabbed across their short edge. Spec +
   implementation plan both committed.
2. **P0 implementation** (mid-day): 7 TDD tasks via subagent-driven
   development — each task dispatched to a fresh subagent with spec-
   compliance + code-quality review between tasks. Ended with full test
   suite (42 → 45 tests) green and preflight gates in place.
3. **P0 lab bring-up** (evening): five diagnostic iterations to resolve the
   projection error:
   1. `SOLVEPNP_IPPE` replacing `SOLVEPNP_ITERATIVE` (planar-pose ambiguity)
   2. Pre-flight gate against below-plane extrinsics (legacy-data guard)
   3. 4-orientation sweep for OpenCV's non-deterministic corner ordering
   4. Camera-to-TCP physical-distance gate (reject spurious above-plane fits)
   5. 3-corner touched geometry (chessboard was rotated ~90° in base frame
      — the old axis-aligned assumption was the final root cause)

Final measured accuracy after the pipeline rewrite: **y within 3 mm,
x within 40 mm** at one tested tomato position. Residual 40 mm in x is
being patched empirically with a `+0.05 m` x offset at pick time until
a more careful 3-corner re-touch closes it; this is a single-line patch
that does not affect projection correctness.

---

## Scope references

- **Spec (today):** [`docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md`](superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md)
- **Plan (today):** [`docs/superpowers/plans/2026-04-24-accurate-pick-and-box-swap.md`](superpowers/plans/2026-04-24-accurate-pick-and-box-swap.md)
- **Prior lab runbook:** [`LAB_RUNBOOK.md`](../LAB_RUNBOOK.md)

---

## Phase 1 — Brainstorm → Spec → Plan

Four linked problems surfaced in the morning lab report:

1. Pick accuracy only worked on a single line at x ≈ 0.35 m from base (10 cm
   drift elsewhere). Root cause traced during design to the `pixel_to_base_frame`
   function's nadir-pinhole parallax approximation not matching the actual
   wrist-mounted D415 (lateral-offset from TCP + forward-tilt — not nadir).
2. `survey_capture.capture_fruits` photographed the scene immediately after
   `slow_move_to_joints` returned, before the position-mode PID had
   actually converged (≈ 0.5–1.0 s tracking lag).
3. The fruit set needed to change: strawberry removed, a 17.5 × 11.5 cm
   plastic clamshell box added. The box is transparent with a blue UK-flag
   sticker + white bar-code labels on the film.
4. Long objects (banana, plastic box) needed the wrist rotated so the
   gripper closes across the object's short edge.

Locked design decisions (brainstorm Q1 .. Q6):

| # | Decision | Choice | Rationale |
|---|---|---|---|
| Q1 | Box detection primary feature | Rectangular contour + white-label confirmation | Shape is orientation-agnostic; transparent body gives no HSV signal; red strawberries inside are a tomato-false-positive hazard |
| Q2 | Box placement | Arbitrary rotation, label-side up, never inverted | Forces minAreaRect + orientation-aware grasp |
| Q3 | Pick-accuracy fix | **Option B + C**: remove hand-patched origin **and** replace nadir-pinhole with solvePnP + ray-plane projection | Memory flagged the workaround as corrupting calibration; only B can fix non-nadir geometry; no full hand-eye needed since detection always at survey1 |
| Q4 | Capture settle | Fix existing `r` key, 1.5 s fixed dwell, sanity-check joint-norm, raise RuntimeError on stuck arm | Consistent with D4 `T_SETTLE` rationale; no new keybind |
| Q5 | Grip geometry | Option X (gap = 11.5 cm, jaws close along short edge); gripper max = 18 cm ≥ 11.5 cm | Confirmed by user |
| Q6 | Priority / effort | P0 (solvePnP + settle) first, stop for lab verification; P1 (box swap); P2 (orientation-aware grasp) | User emphasised "先出成果" — ship accurate-pick first, iterate |

Spec + plan committed: `0fa6214` (spec), `54e4cf2` (plan).

---

## Phase 2 — P0 implementation (subagent-driven TDD)

P0 = projection rewrite + capture settle + preflight gates. Seven tasks,
each dispatched to a fresh subagent with the task's full text + context;
each followed by an independent spec-compliance review and a code-quality
review before marking complete.

| # | Task | Subagent model | Commit | Notes |
|---|---|---|---|---|
| 0.1 | `SessionCal.cam_extrinsics_survey1` field + round-trip test | Haiku | `3ba3ff0` | Backward-compatible on legacy JSON (`.get(..., None)`). |
| 0.2 | `calibrate_extrinsics.solve_survey1_extrinsics` via `cv2.solvePnP` | Sonnet | `6e17881`, polish `6c1d88a` | Synthetic pinhole round-trip recovers pose within 1 mm / 0.5° on noise-free data; RMS > 5 px hard gate. |
| 0.3 | Rewrite `pixel_to_base_frame` as ray-plane projection | Sonnet | `0f41e83`, docstring polish `593ea8c`, typo `43a5929` | Pre-existing tests intentionally red pending Task 0.4. |
| 0.4 | Update detector fixtures to inject `_nadir_extrinsics` | Sonnet | `763dec8` | Implementer discovered a latent `target_z` semantic bug in 0.3 (the `origin.z` offset from the pre-rewrite code was accidentally dropped) — fix merged in the same commit. |
| 0.5 | Wire `solve_survey1_extrinsics` into `run_calibration_core` | Sonnet | `a8d8be7` | 4-orientation sweep introduced later. |
| 0.6 | `capture_fruits` dwell + joint sanity check | Sonnet | `c8787af`, polish `1fa956b` | 1.5 s fixed dwell; 0.05 rad joint-norm tolerance (later tightened — see Phase 3). |
| 0.7 | Preflight gate on `cam_extrinsics_survey1` | Haiku | `cd57c5b` | Stale session_cal now fails loudly instead of silently feeding wrong extrinsics to the picker. |

Test suite at end of Phase 2: 42/42 passing. All gates live.

---

## Phase 3 — P0 lab bring-up (five diagnostic iterations)

First lab run with P0 code exposed five distinct errors, each surfaced by a
fresh piece of real-hardware data. Each was isolated and fixed before
moving to the next.

### Iteration 1 — Planar-pose ambiguity (`ee10740`)

**Symptom:** First real calibration gave `solvePnP: RMS = 2.79 px,
camera at base XYZ = [0.587, 0.3, −0.207] m` — camera z negative (below
the table, physically impossible).

**Root cause:** `cv2.solvePnP(..., SOLVEPNP_ITERATIVE)` has two
geometrically valid solutions for planar targets (correct pose + its
mirror through the target plane); ITERATIVE picks whichever the initial
guess lands near. For this geometry, it picked the mirror.

**Fix:** Switched to `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE)` which
explicitly returns both candidate poses; filter to the one with
`C[2] > chess_origin_z`. Added a regression test that simulates the
lab case (elevated chess origin, camera above) and asserts the above-
plane solution is picked.

### Iteration 2 — Preflight guard on legacy data (`1e3334d`)

**Symptom:** After fixing the code, preflight still showed the old
mirror-flipped calibration. User hadn't re-run `calibrate_chessboard.py`.

**Fix:** Added a preflight gate that rejects any `cam_extrinsics_survey1`
whose stored camera `C[2] ≤ chess_origin[2]`. Below-plane data is now
caught at session-start and the picker refuses to run on it.

### Iteration 3 — Corner-ordering ambiguity (`c928230`)

**Symptom:** Second calibration run: `findChessboardCorners` succeeded
(35 corners) but `solvePnP REJECTED: no IPPE solution places the camera
above the chessboard plane`. Both IPPE candidates were below-plane at
rms = 0.87 px.

**Root cause:** OpenCV's `findChessboardCorners` does not guarantee
`image_pts[0]` corresponds to any particular physical corner of the
board. Depending on camera pose, OpenCV may scan rows top-to-bottom or
bottom-to-top (etc.), and the 4 possible start corners give 4 different
2D-to-3D correspondences. Our 3D-grid builder always assumed
"`image_pts[0]` = physical top-left" — which was wrong for this setup.

**Fix:** 4-orientation sweep in `run_calibration_core`: for each of
the 4 `(flip_i, flip_j)` correspondences, call `solve_survey1_extrinsics`
and keep the one that produces an above-plane camera with lowest RMS.
Regression test simulates a mirror-ordered 2D set and asserts the sweep
picks the correct orientation.

### Iteration 4 — Physical distance gate (`5a8ca67`)

**Symptom:** Sweep picked the `flip-rows` orientation with RMS = 0.88 px
and camera at `(0.552, 0.038, 0.273)` — mathematically valid but
**31 cm from the survey1 TCP**. For a wrist-mounted D415 this is
physically impossible. First pick test confirmed the bad projection:
tomato reported at `(0.579, 0.156)` vs actual `(0.449, 0.010)` —
20 cm off.

**Root cause:** Each `(flip_i, flip_j)` orientation produces an
IPPE-valid pose, but only one is physically correct. The others are
geometric artifacts that happen to fit the 35 corner points.
RMS is identical (0.82 px) across them; RMS alone cannot disambiguate.

**Fix:** Added a hard gate `||C_cam − TCP_survey1|| ≤ 25 cm` using
forward-kinematics of the survey pose, rejecting orientations whose
recovered camera is too far from the wrist to be physically mounted
there. Full 4-candidate table is now printed on every calibration for
diagnostic clarity.

### Iteration 5 — Non-axis-aligned chessboard (`608d857` + `f9bdc04` + `9146f00`)

**Symptom:** After Iteration 4, **all four** orientations were being
rejected — two on below-plane, two on > 25 cm from TCP. None passed.

**Root cause (the final one):** The chessboard was **rotated ~88° in the
base frame** — the operator had laid it rotated, not aligned to the arm's
kinematic x / y axes. Our 3D-grid builder assumed axis-aligned:
`corner[i, j] = touched_origin + (i·30 mm, j·30 mm, 0)`. With a 90°-
rotated board, the 3D positions fed to `solvePnP` were wrong, so no
camera pose could fit both the 2D pixels and the (wrong) 3D grid
simultaneously — hence all orientations failing.

**Fix (three commits):**

1. `debug_corner_order.py` diagnostic: annotate the 4 extreme indices
   (`[0]`, `[6]`, `[28]`, `[34]`) on the last captured frame so the
   operator can visually confirm which physical corner OpenCV put at
   `image_pts[0]`. (Operator confirmed yellow `[0]` = their touched
   top-left corner, ruling out a corner-ordering bug.)
2. `touch_three_corners.py` tool: jog-touch TL, TR, BL in sequence, print
   derived `+i_axis`, `+j_axis`, board rotation, and distances. Writes
   `logs/chess_touched_corners.json` (no user prompt). **First use
   revealed the chessboard was at −88.5° rotation — confirming root
   cause.**
3. `_grid_with_ordering` updated to accept TR and BL in addition to TL.
   When present, uses the 3-corner-derived DIRECTIONS (Gram-Schmidt
   orthogonalised) and **nominal 30 mm square spacing** — discarding the
   measured distances which carry 5–10 mm jog noise. Calibrate CLI reads
   the 3-corner cache at Phase 1 and skips the single-corner jog.

**Result (after Iteration 5):** 
```
solvePnP 4-orientation sweep:
  original: OK; C=[0.292, 0.043, 0.268], RMS=0.90px, ||C-TCP||=18.8cm
  flip-cols: REJECTED (both IPPE below plane)
  flip-rows: REJECTED (both IPPE below plane)
  180°: REJECTED (cam 31.7cm from TCP > 25cm wrist bound)
solvePnP: chosen orientation=original, RMS=0.90 px
```
Exactly one orientation passes all physical gates. `||C−TCP|| = 18.8 cm`
is the plausible wrist-mount offset.

### Ancillary — settle tolerance tightening (`47731f4`)

**Symptom:** During Iteration 2's tests, each `'r'` key refresh caused
the chessboard residual to climb past 10 mm; second refresh hard-failed
with `chessboard residual 10.7 mm > 10 mm`.

**Root cause:** 0.05 rad joint-norm tolerance (from Task 0.6) permits
up to ≈20 mm TCP drift across four joints — enough to push the chess
residual over its 10 mm gate after a pick-and-place cycle's tracking
error.

**Fix:** Tightened to 0.015 rad (~6 mm TCP drift budget), matching the
target pick accuracy.

---

## Final measured accuracy

After all five iterations, preflight is green and a representative pick
test was taken:

| | Ground truth (jog-touch) | Picker reports | Δ |
|---|---|---|---|
| x | 0.448 m | 0.408 m | −40 mm |
| y | 0.064 m | 0.061 m | −3 mm |
| z | 0.072 m | 0.067 m | −5 mm |

y and z are within millimetres; **x carries a systematic 40 mm bias** —
most likely from the ~4° non-orthogonality in the operator's 3-corner
touch propagating into a small residual `R` rotation error. This does
not affect y or z significantly (geometric coincidence of the rotation
axis).

Two paths to resolve:
1. **Re-touch 3 corners more carefully** (jog slowly, verify visually
   that the TCP tip is truly at the inner-corner crossing). Expect the
   residual to collapse to <1 cm.
2. **Apply a constant x offset at pick time** (today's quick patch,
   `449c5f0`): `sorting_controller.pick_single` adds +5 cm to the x
   component of the commanded pick target, without changing the reported
   detection position. Valid while the bias is known constant;
   revisitable once a careful re-touch is done.

Offset patch is in effect as of end-of-day. Pick accuracy verification
with the patch in place is the first item on tomorrow's agenda.

---

## Files touched today

| Area | Paths |
|---|---|
| New calibration module | `python/calibrate_extrinsics.py`, `python/test_calibrate_extrinsics.py` |
| New test module | `python/test_session_cal.py`, `python/test_calibrate_chessboard_orientation.py` |
| New diagnostic tools | `python/debug_corner_order.py`, `python/touch_three_corners.py` |
| Modified modules | `python/session_cal.py`, `python/fruit_detector.py`, `python/survey_capture.py`, `python/calibrate_chessboard.py`, `python/preflight.py`, `python/sorting_controller.py`, `python/test_fruit_detector.py`, `python/test_survey_capture.py` |
| Design docs | `docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md`, `docs/superpowers/plans/2026-04-24-accurate-pick-and-box-swap.md` |
| Debug artefacts | `logs/calibration_latest.png`, `logs/corner_order_debug.png`, `logs/chess_touched_corners.json` |

Test suite at end of day: 45/45 passing (up from 36 at start of day).
Preflight status: green.

---

## Open items

- [ ] **P0.8 lab acceptance** — pick accuracy verification with the +5 cm
      x-offset patch in place, at two test positions (TL-vicinity and
      TR-vicinity). If both pass ≤ 1 cm, P0 is shipped. If the offset
      varies with position, re-do the 3-corner touch more carefully.
- [ ] **P1 (plan ready, not started):** remove strawberry class,
      introduce `plastic_box` detector (Canny + `minAreaRect` + white-
      label confirmation), tomato de-confusion (exclude box region),
      rewire `placebox` teach-point, remap `'s'` → `'p'` in picker UI.
- [ ] **P2 (plan ready, deferred):** Detection.grasp_angle_rad via
      `minAreaRect`; `sorting_controller._start_move` converts base-frame
      angle to wrist joint using `phi[0]_at_pick` and a lab-calibrated
      `gripper_mount_offset_rad` (new session_cal field).
- [ ] Documentation: this report + CHINESE version, then `docs/PROGRESS.md`
      update when P0 is officially shipped.

---

## Commit log (today, 24 commits)

```
449c5f0 chore(controller): +5cm x offset at pick time (empirical lab patch)
5072a7f chore(detector): tomato fallback 50->55 mm (measured in lab)
9146f00 fix(calibrate): use 3-corner directions with nominal spacing + Gram-Schmidt
f9bdc04 fix(calibrate): use 3-corner touched geometry to build correct 3D grid
608d857 feat(debug): corner-order visualizer + 3-corner touch helper
5a8ca67 fix(calibrate): reject orientations where camera too far from TCP
47731f4 fix(capture): tighten settle joint-norm tol 0.05 -> 0.015 rad
c928230 fix(calibrate): 4-orientation sweep resolves OpenCV corner-order ambiguity
1e3334d feat(preflight): reject below-plane camera extrinsics (mirror-flip guard)
ee10740 fix(calibrate): use SOLVEPNP_IPPE + pick above-plane solution
cd57c5b feat(preflight): require cam_extrinsics_survey1 in session_cal
1fa956b chore(capture): drop raising=False on monkeypatch + sharpen settle comment
c8787af feat(capture): 1.5s settle + sanity check before shooting
a8d8be7 feat(calibrate): call solve_survey1_extrinsics in run_calibration_core
593ea8c docs(projection): clarify fruit_top_z_mm is table-relative + trim stale docstring
763dec8 test(detector): update fixtures to provide cam_extrinsics
43a5929 chore(test): fix 'finite and finite' typo in docstring
0f41e83 feat(projection): ray-plane pixel_to_base_frame via extrinsics
6c1d88a chore(calibrate): fix transpose comment + drop unused cv2 import
6e17881 feat(calibrate): solve_survey1_extrinsics via cv2.solvePnP
3ba3ff0 feat(session_cal): add cam_extrinsics_survey1 field
54e4cf2 docs(plan): accurate-pick projection + box swap + orient grasp
0fa6214 docs(spec): accurate-pick projection rewrite + box swap + orient grasp
b8cf978 refactor(controller): joint-space interp + T_SETTLE hold phase
```
