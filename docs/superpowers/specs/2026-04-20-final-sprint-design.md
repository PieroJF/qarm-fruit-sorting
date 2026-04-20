# Final Sprint Design — FruitSorting QArm Project

**Date**: 2026-04-20
**Deadline**: 2026-05-01 14:00 (11 days)
**Team**: Piero Flores, Zihen Huang, Ran Zhang, Yichang Chao

This spec captures the design decisions for the remaining work on the Applied Robotics final project. It was produced via a brainstorm session on 2026-04-20 and approved by the user before plan generation.

---

## 1. Context and goals

Sort 14 fruits (6 strawberry → A, 3 banana → B, 5 tomato → C) with a Quanser QArm 4-DOF arm and an Intel RealSense D415 arm-mounted camera. Two required operating modes:

- **Autonomous** — full vision → plan → pick → place loop.
- **Remote** — Simulink-fronted teleop; operator performs a manual pick.

Architectural invariants carried over from the 2026-04-14 PROGRESS.md:

- **Simulink is the facade; Python does the real work.** No rewrite of the Python core, only wrap it.
- **No QUARC** — all hardware I/O routed via `py.qarm_driver.QArmDriver` through `py_qarm_io.m`.
- **Camera is arm-mounted.** `T_cam_to_base` is pose-dependent; calibration is done at the detection pose (`pickhome1`) and only valid there.

New capability added during the 2026-04-20 session: **UGreen floor-level camera** now provides an external, fixed ground-truth view of the workspace. The gripper and fruits are both visible from `pickhome1`, enabling closed-loop visual feedback for calibration, preflight, and pick validation.

## 2. Features in scope

Ten features. Each is a self-contained unit with one clear purpose, well-defined interface, and its own tests where applicable.

### 2.1 Remote mode (Simulink-fronted manual teleop)

A unified `FruitSorting_Hardware.slx` model with a 3-way mode switch: **Autonomous / Remote / Release**. Any mode flip triggers a **halt-first** transition (freeze 0.5 s at current joints, then activate the new mode), preventing jerks when the operator switches mid-run.

Remote mode sub-architecture:

- **Two sub-modes inside remote**: joint-space and Cartesian, selected by a toggle. Joint-space exposes 4 jog buttons per joint (+/-) plus gripper open/close. Cartesian exposes +/- jog buttons for X, Y, Z, and wrist γ plus gripper.
- **Jog-pendant paradigm** (not continuous sliders): each button press nudges the arm by a step (default 5 mm linear, 5 deg angular), executed via `slow_move_to_joints` with a smoothstep profile. This reuses the paradigm validated in `teach_points.py` and is safe by construction.
- **Pick manual, minimum scope**: the operator must be able to pick a strawberry with the remote HMI. Fine jog step selector included; gripper controlled via open/close buttons that reuse the ramped+readback pattern from `sorting_controller`'s CLOSE/OPEN_GRIPPER states — extracted into a shared `sorting_controller.set_gripper_ramp(target)` helper during implementation.
- **Camera feed via companion OpenCV window** (not embedded in `.slx`). A `py_remote_view.py` script opens a `cv2.namedWindow` showing the D415 live feed with status overlay (current joints, xyz, gripper, step, sub-mode, last command). The Simulink `.slx` launches this script when remote mode activates and closes it on exit.
- **Safety interlocks**:
  - **E-stop** button in the `.slx`: immediately commands the current joint positions (PID holds) and opens the gripper to the safe value (0.15, not 0.0). Re-arm via separate button.
  - **Halt-first** on any mode flip.
  - **Workspace box check** in Cartesian jog: target must be inside `(x ∈ [-0.10, 0.55], y ∈ [-0.55, 0.55], z ∈ [0.02, 0.50])`. Out-of-box nudges are refused and logged in the camera overlay.
- **Out of scope** for this sprint: watchdog for Simulink lag, real-time velocity limiting beyond the implicit jog-step limit.

### 2.2 Stateflow chart for autonomous FSM (a-min depth)

Replace the persistent MATLAB Function handle in `py_controller.m` with a Stateflow chart that owns state transitions and timing. Complex per-state behavior (IK safety, gripper ramp + readback, smart pick Z, pre-flight reachability) remains in Python helpers called from Stateflow entry actions. The Python `StepController` class is retained as the reference implementation for tests.

This keeps Stateflow readable (12 states, clean transitions, no embedded code oceans) and preserves a single source of truth for the tested behaviors. The cost is ~1 day instead of the ~2-3 days a full native port (a-full) would take.

### 2.3 Logging and report plots (level b)

Two complementary loggers capturing the same run from different perspectives:

- **Simulink To Workspace** blocks capture: state_id trace, joint_cmd[4], joint_actual[4], gripper_cmd, gripper_actual, detection_count, sorted_count, simulation time. Saved to a `.mat` file at simulation end.
- **Python TraceLogger** (existing in `teach_points.py`, extended to `sorting_controller.py`) captures high-level events: DETECTION_FRAME, PICK_ATTEMPT, GRIPPER_READBACK, IK_FAIL, PICKHOME_SKIP, SORT_COMPLETE. Plain-text log with ISO timestamps.

A postprocessing script `scripts/generate_report_plots.py` consumes both logs and emits four figures to `figures/`:

- Joint trajectories over a pick cycle (4 subplots).
- Gripper commanded vs actual (readback behavior).
- FSM state timeline (Gantt-style).
- Detection confidence + fruit depth per frame.

Cross-referencing between logs is by wall-clock timestamp.

### 2.4 Preflight script with 6 checks

`python/preflight.py` runs in ~30 seconds before a lab session and reports pass/fail per check with remedial suggestions. Checks:

1. **QArm** — `connect → read_all → set_joints(current)`. Catches HILError -843 (board locked) early.
2. **D415** — warmup to non-black color and >50% valid depth coverage.
3. **UGreen** — open idx=3 via MSMF, capture one frame, sanity-check mean brightness.
4. **Calibration** — load `calibration.json`, report RMS, pose_label, timestamp. Flag if RMS > 50 mm or age > 7 days.
5. **HSV detector** — capture one D415 frame, run `fruit_detector.detect_fruits`, print circularity/saturation/area per blob so thresholds can be tuned in place.
6. **Integration tests** — run `test_integration.py` against mocks (zero hardware dependency).
7. **Visual teach-point check** — arm moves to `pickhome1`, UGreen captures a frame, image-diff against a stored reference proves the arm landed where it should.

Output: a green `PREFLIGHT OK` or red `PREFLIGHT FAIL: <reason>` with a single suggested next command.

### 2.5 UGreen floor-camera integration

The UGreen camera is a fixed USB webcam at DirectShow index 3, MSMF backend, 1280×720. Its pose is assumed stable within a lab session; re-captured baseline at the start of each session.

Core module `python/ugreen_tracker.py` provides:

- `capture()` — grab one frame after a short warmup.
- `tcp_from_diff(frame, baseline)` — returns the pixel (col, row) that best represents the gripper TCP. Algorithm: absolute BGR difference against the baseline; threshold; find the connected component whose bounding-box bottom-center sits in the upper half of the frame (where the arm reaches from above). Return the bottom-center pixel of that component as the TCP approximation.
- `detect_fruits_on_table(frame)` — project fruits visible in the UGreen view to base-frame XY by intersecting pixel rays with the known table plane (z ≈ 0.13 m). Requires `T_ugreen_to_base` from the closed-loop calibration.

Reasoning for the image-diff approach over marker-based tracking: the purple post-its applied to the gripper are visually indistinguishable from other bright components in the tight crop, and the 2026-04-20 test confirmed the ROI is swamped by background noise when thresholding on brightness. Image diff against an empty baseline gives a clean silhouette of the arm without needing fiducials, and the "bottom pixel" heuristic works because the arm always reaches the table from above.

### 2.6 Closed-loop hand-eye calibration (UGreen-driven)

`python/calibrate_closed_loop.py` runs without operator clicks:

1. Arm moves to each of N teach points (≥ 6, including variation in Z) with the gripper open, post-its optional.
2. At each point, the arm holds still, UGreen captures a frame, `tcp_from_diff` extracts the pixel. This gives a pairing `(pixel_ugreen, xyz_base)`.
3. UGreen intrinsics are estimated via OpenCV's pinhole model using a printed chessboard (one-time setup — user shows a 7×5 inner-corner chessboard to the UGreen camera for ~20 poses, script does `cv2.calibrateCamera`). **Prerequisite**: user prints the chessboard pattern (generated by the script) once before first calibration.
4. With intrinsics + N pairings, solve PnP (`cv2.solvePnP`) for `T_ugreen_to_base`.
5. Residuals printed per point; overall RMS reported. Target: < 15 mm.

Once `T_ugreen_to_base` is known, we can cross-validate the D415 calibration: for each fruit position in `cal_01..cal_08`, UGreen sees the fruit at a pixel, projects to base frame via the known table plane, and compares to the teach-point XYZ. Discrepancy = D415 calibration error.

### 2.7 Recalibrate D415 in lab (single-pose, 8 points)

Uses `recal_from_pose.py` already committed. Fresh correspondences collected in-session: fresas at `cal_01..cal_04`, tomates at `cal_05..cal_08`. Target RMS: ≤ 20 mm at `pickhome1`. Closed-loop verification by UGreen afterwards.

### 2.8 HSV tuning for lab lighting

One-time tuning pass: preflight check #5 captures a D415 frame with known fruits in known positions, prints circ/saturation/area per blob. Operator tunes `HSV_RANGES`, `CIRCULARITY_THRESH`, `SATURATION_THRESH` in `fruit_detector.py` until all three fruit types classify correctly. Re-run preflight until green.

### 2.9 Pick + place end-to-end (full sort)

Three new teach points required in-session: `basket_a`, `basket_b`, `basket_c`. Fallback — rename existing `placeberries` → `basket_a`, `placetomato` → `basket_c`, and create a new `basket_b` for banana.

Acceptance criteria:

- Arm reaches each basket from `pickhome1` without joint-limit violation (verified by `_ik_safe` pre-flight).
- Descent to each basket from `SAFE_Z` does not collide with the basket rim (verified visually in a dry run with the gripper open).
- First cycle: a single strawberry is picked from `cal_01` and placed in `basket_a` cleanly. Only after this works do we commit to the full 14-fruit run.

### 2.10 Documentation refresh

Update `PROGRESS.md` and `PROJECT_CONTEXT.md` with:

- Camera arm-mounted finding and per-pose calibration architecture.
- Completed work from 2026-04-17 and 2026-04-20 sessions (commits `aede68b`, `971e442`, `633b174`, `6086b38`, `7d2d0d2`, plus this sprint).
- UGreen integration and closed-loop calibration workflow.
- Current feature status table.

`COMPLETE_GUIDE.md` refresh only if time permits; the final report likely supersedes it.

### 2.11 Demo recording (video only)

No live demo. Video record a clean run of the full 14-fruit sort in the final lab session. Edit to 3–5 min highlight reel. Separate cut for remote mode (~1 min showing jog pendant performing a manual pick). Both cuts stitched into the final submission video.

## 3. Execution categorization

Features split by whether they depend on lab hardware access:

**Dev-machine (doable now, no hardware):**
- Remote mode `.slx` + jog pendant subsystem (2.1).
- Stateflow chart for autonomous (2.2).
- Logging extension + plot generator (2.3).
- Preflight script skeleton (2.4 — the hardware-dependent checks are stubbed to run under a `--mock` flag).
- UGreen tracker + image-diff TCP (2.5 — validated with existing frames).
- Documentation refresh (2.10).
- Unit tests for every new component.

**Hardware-required (must be in lab):**
- Execute preflight live (2.4).
- Closed-loop calibration run (2.6).
- D415 recalibration in lab (2.7).
- HSV tuning pass (2.8).
- Pick + place verification (2.9).
- Video recording (2.11).

## 4. Testing strategy

- **Offline** — `test_integration.py` continues to run the 10 existing tests. New tests added:
  - UGreen tracker on synthetic diff (mock baseline + mock arm overlay).
  - Remote mode jog-step computation (target clamped to workspace box).
  - Stateflow chart state transitions via Simulink Test (one test per state pair).
- **Lab** — preflight must pass (6 checks green) before any other motion. Closed-loop calibration must report RMS ≤ 20 mm before any pick. First real pick is a single-fruit dry run; only then commit to the 14-fruit sequence.

## 5. Error handling summary

| Failure mode | Recovery |
|---|---|
| IK fail on computed target | Hold last pose, log warning, skip that fruit in autonomous; refuse nudge in remote. |
| Gripper stall | Ramp + readback already handles it. `_held_grip` tracks the settled value. |
| QArm HIL -843 | Preflight catches it; user power-cycles the QArm. |
| Black D415 frame at warmup | Preflight waits up to 10 s for color+depth; if still black, fail preflight with remedial message. |
| UGreen baseline invalidated (lighting change) | Re-capture baseline. Preflight check #7 will detect mismatch against stored reference. |
| Calibration RMS > 50 mm | Preflight fails; user reruns `calibrate_closed_loop.py`. |
| E-stop pressed | Immediate halt + grip-safe open. |
| Workspace box violation in remote | Refuse nudge, overlay warning. |

## 6. Out-of-scope explicit

Items deliberately **not** in this sprint, to protect the deadline:

- **Formal AX=XB hand-eye calibration** solving `T_cam_to_ee` once and for all. The closed-loop per-pose approach (2.6) is good enough at 36 mm → target 20 mm, which is well inside the gripper margin (45 mm per side for tomatoes).
- **Stateflow full native port** (a-full). The a-min variant keeps Python as the source of truth for complex behavior.
- **Continuous slider HMI for remote**. Jog pendant chosen over continuous sliders for safety.
- **Live demo**. Video-only per section 2.11.
- **Risk assessment form**. Already complete (confirmed by user on 2026-04-20).
- **Report writing and video editing**. Owned by teammates; tracked externally.

## 7. Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| UGreen pose shifts between sessions | Medium | Preflight check 7 compares to a stored `ugreen_pickhome1_reference.png`; any mismatch >20 px per axis fails preflight. |
| Post-its fall off during pick | High | Image-diff approach doesn't depend on post-its. |
| D415 calibration RMS never reaches 20 mm | Low | 36 mm is already inside the gripper margin. 20 mm is a stretch goal, not a blocker. |
| Stateflow introduces new bugs in autonomous | Medium | The existing Python `StepController` stays in place as a fallback; the Stateflow port is additive. Integration tests run against the Python core. |
| Remote mode HMI not ready by final demo | Medium | Build joint-space first (half the scope is usable on its own); Cartesian sub-mode added only after joint-space is green. |
| Lab session unexpectedly short | High | Preflight runs in 30 s. Closed-loop calibration in ~5 min. Pick dry-run in ~3 min. Budget for one full 14-fruit video take: 15 min. Fits in a 45-min session. |

## 8. Success criteria

The sprint is done when:

1. All 10 existing integration tests remain green; new tests added for remote logic + UGreen tracker also green.
2. Preflight runs end-to-end with all 7 checks passing in the lab.
3. A single strawberry is picked from `cal_01` and dropped into `basket_a` cleanly — verified by UGreen image-diff.
4. A 14-fruit autonomous run records on video, with ≥ 12 of 14 fruits sorted to the correct basket.
5. A remote-mode video (~ 1 min) shows the jog pendant performing a manual pick.
6. `PROGRESS.md` and `PROJECT_CONTEXT.md` reflect the final state.
7. All commits pushed to master.
