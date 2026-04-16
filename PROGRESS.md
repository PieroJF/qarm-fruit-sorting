# FruitSorting — Progress Snapshot

**Last updated:** 2026-04-14
**Deadline:** 2026-05-01 14:00 (Applied Robotics final, University of Birmingham)
**Team:** Piero Flores · Zihen Huang · Ran Zhang · Yichang Chao
**Working directory:** `C:\Users\Mugin\Downloads\Compressed\Nueva carpeta_2\FinalProject_FruitSorting`

---

## 1. Task recap (coursework brief)

Sort **14 fruits** with a Quanser QArm (4-DOF) + Intel RealSense D415:

| Fruit       | Count | Target basket |
|-------------|-------|---------------|
| Strawberry  | 6     | Basket A      |
| Banana      | 3     | Basket B      |
| Tomato      | 5     | Basket C      |

Two operating modes required by the rubric:
- **Autonomous** — full vision → plan → pick → place loop.
- **Remote-controlled** — human teleop fallback.

Hardware baseline: QArm 4-DOF serial arm, D415 RGB-D camera, Quanser SDK for Windows.

---

## 2. Architectural decision (locked)

> **Simulink is the facade. Python does the real work.**

This is the user's explicit requirement and overrides the shape of the original repo (which is 100% Python with orphan `.m` files). We do **not** rewrite Python; we wrap it.

- **Python core** (untouched unless asked): kinematics, detector, driver, controller, trajectory.
- **Simulink facade** (`matlab_facade/`): `.slx` models whose MATLAB Function blocks call Python via `py.*` / `coder.extrinsic`. No TCP bridge, no socket shim — direct in-process calls.
- **Solver:** fixed-step `ode4`, `dt = 0.002 s` (500 Hz) — matches Python loop rate so behavior stays consistent across both domains.
- **Feedback loops** broken with `Unit Delay` blocks to avoid algebraic-loop errors.

Vertical-slice discipline: build one subsystem → simulate → validate → add the next. No big-bang integration.

---

## 3. Environment (verified 2026-04-14)

| Component | Status | Path / Version |
|-----------|--------|----------------|
| MATLAB    | ✅ R2025a (Simulink, Simscape, Stateflow) | `C:\Program Files\MATLAB\R2025a` |
| Python    | ⚠️ 3.13.5 — pyenv warns "not officially supported" but calls work | `C:\Python313\python.exe` |
| Quanser SDK (Windows) | ✅ installed | `C:\Program Files\Quanser\Quanser SDK\` |
| **QUARC** | ❌ **NOT installed** — Simulink cannot talk to QArm natively | lab machine only |
| Python deps | ✅ `quanser.hardware`, `quanser.devices`, `numpy 2.4`, `opencv 4.13` import cleanly | — |
| MATLAB MCP server | ✅ v0.7.0, registered as `matlab` (stdio, nodesktop) | `C:\Users\Mugin\.claude\tools\matlab-mcp-core-server.exe` |

**MCP tools available:** `detect_matlab_toolboxes`, `check_matlab_code`, `evaluate_matlab_code`, `run_matlab_file`, `run_matlab_test_file`. No Simulink-specific MCP tools — Simulink work is driven by writing `.m` build scripts and evaluating them.

**Consequence of no QUARC on dev machine:** all hardware I/O must route through `py.qarm_driver.QArmDriver`. Simulink never touches the arm directly. `py_qarm_io.m` has three modes:
- `mode=0` **SIMULATE** (default) — echoes commanded joints, no driver instantiation.
- `mode=1` **HARDWARE** — real QArm via Python driver.
- `mode=2` **RELEASE** — shutdown / safe.

---

## 4. Repository layout

```
FinalProject_FruitSorting/
├── PROJECT_CONTEXT.md          ← read this first next session
├── COMPLETE_GUIDE.md / .pdf    ← long-form write-up
├── PROGRESS.md                 ← (this file)
├── figures/
├── matlab_code/                ← legacy orphan .m, not used
├── simulink_models/            ← legacy, not used
├── scripts/
├── python/                     ← core, DO NOT rewrite
│   ├── qarm_kinematics.py      ← FK/IK, sub-µm accuracy, validated
│   ├── qarm_driver.py          ← QArmDriver (Quanser SDK wrapper)
│   ├── fruit_detector.py       ← HSV + shape classifier
│   ├── camera.py               ← D415 frame grabber
│   ├── trajectory.py           ← quintic / joint-space planner
│   ├── sorting_controller.py   ← time.time()-driven FSM (remote/CLI)
│   ├── sorting_controller_sim.py ← NEW: dt-driven twin for Simulink
│   ├── main_autonomous.py
│   ├── main_remote.py
│   └── validate_python.py      ← passes end-to-end
└── matlab_facade/              ← the Simulink facade we are building
    ├── setup_pyenv.m           ← pyenv + sys.path wiring
    ├── py_ik.m / py_fk.m       ← kinematics wrappers
    ├── py_detect_test.m        ← vision bridge, returns 8×5 matrix + count
    ├── py_controller.m         ← persistent StepController handle
    ├── py_qarm_io.m            ← QArm I/O facade (mode 0/1/2)
    ├── build_slice_IK.m        + run_slice_IK.m       → SLICE 1
    ├── build_slice_vision.m    + run_slice_vision.m   → SLICE 2
    ├── build_slice_fsm.m       + run_slice_fsm.m      → SLICE 3
    ├── build_hardware_model.m  + run_FruitSorting.m   → SLICE 5 (top-level)
    ├── run_FruitSorting_hw.m                          → hardware entry
    ├── FruitSorting_slice.slx              (IK)
    ├── FruitSorting_slice_vision.slx       (vision)
    ├── FruitSorting_slice_fsm.slx          (FSM)
    └── FruitSorting_Hardware.slx           (top-level integration)
```

---

## 5. Vertical slices — status matrix

| # | Slice | Model | Build script | Run script | Status | Evidence |
|---|-------|-------|--------------|------------|--------|----------|
| 1 | **IK bridge** | `FruitSorting_slice.slx` | `build_slice_IK.m` | `run_slice_IK.m` | ✅ **PASS** | joint vector from `py.qarm_kinematics.inverse_kinematics` propagates through MATLAB Function block end-to-end |
| 2 | **Vision** | `FruitSorting_slice_vision.slx` | `build_slice_vision.m` | `run_slice_vision.m` | ✅ **PASS** | Synthetic BGR image → 1 banana + 1 tomato + 1 strawberry detected, fixed 8×5 output shape |
| 3 | **FSM / controller** | `FruitSorting_slice_fsm.slx` | `build_slice_fsm.m` | `run_slice_fsm.m` | ✅ **PASS** | 601 ticks, all 11/11 states visited, `DONE` reached |
| 4 | **Kinematics + trajectory integration** | — | — | — | ⚠️ **merged into slice 5** (not a standalone slice) | — |
| 5 | **Top-level hardware model** | `FruitSorting_Hardware.slx` | `build_hardware_model.m` | `run_FruitSorting.m` | ✅ **PASS (sim)** | FSM → QArmIO → feedback loop, `qarm_mode` Constant gates sim vs hardware |

**All five slices validated in simulation as of 2026-04-14.** No slice currently exercises real hardware.

---

## 6. Key implementation notes

### 6.1 `sorting_controller_sim.py` (new file)

A dt-driven twin of `sorting_controller.py` with:
- No `time.time()` calls — receives `dt` from Simulink.
- Integer state IDs `0..11` exported as `state_id` output port (readable by Stateflow / logs).
- Same transition logic as the original; original file left untouched so `main_remote.py` and CLI flows still work.

Driven from MATLAB via `py_controller.m`, which holds a **persistent handle** to `py.sorting_controller_sim.StepController` so state survives between Simulink ticks:

```
(joints_cur, dt, reset)  →  (phi[4], gripper, state_id, done)
```

### 6.2 `py_detect_test.m`

Builds a synthetic BGR test image, calls `py.fruit_detector.detect_fruits`, returns a **fixed-size** `8×5` matrix `[type_id, cy, cx, area, conf]` padded with zeros plus a `count` scalar. Fixed shape is required by MATLAB Function block output signatures.

### 6.3 `py_qarm_io.m`

Persistent QArmDriver facade. Default mode `0` (SIMULATE) simply echoes the commanded joints — works on any machine without QUARC/Quanser hardware. Mode `1` instantiates the real driver; mode `2` releases it cleanly.

---

## 7. Gotchas learned (hard-won, do not re-pay)

1. **Python 3.13 pyenv warning is harmless for our calls.** If it ever bites, install Python 3.12 and repoint `setup_pyenv.m`.
2. **`forward_kinematics` returns a Python tuple** `(pos, R)`. In MATLAB unpack via `cell(tup)` — not indexing.
3. **MATLAB Function blocks calling Python need:**
   - `coder.extrinsic('py.module.func')`
   - Pre-declared output sizes (`phi = zeros(4,1)`)
   - Explicit `double()` cast on extrinsic returns — **without the cast the values don't propagate to output ports**. Lost a full slice-1 iteration to this.
4. **`To Workspace` in Array format** logs vector signals as `[signal_dim × 1 × N_samples]` 3-D arrays. Always `squeeze(x)'` before indexing.
5. **Constant block `OutMin`/`OutMax` must be scalar doubles.** Vector limits fail at build time.
6. **Feedback loops require `Unit Delay`** to break algebraic loops (applied on both the FSM slice and the top-level model).
7. **Controller must use accumulated sim time**, not `time.time()`. Mixing the two makes dt-driven transitions misfire.
8. **HSV thresholds, `T_cam_to_base`, and basket world coordinates are all placeholders** — cannot be tuned without real camera and robot base frame.

---

## 8. Pending work

### 8.1 Lab-only (blocking final submission)

1. **Install QUARC** on the lab machine. Set `qarm_mode` Constant in `FruitSorting_Hardware.slx` to `1` and verify `py_qarm_io` hardware path end-to-end. Dry-run first with arm powered but base unclamped.
2. **Replace hardcoded demo fruit queue** in `py_controller.m` with positions from the vision pipeline. Requires:
   - `T_cam_to_base` extrinsic calibration (hand-eye).
   - Live D415 frames routed through `py.camera.get_frame`.
3. **Tune `fruit_detector.HSV_RANGES` + `CIRCULARITY_THRESH`** on the real camera feed under lab lighting. Current values were hand-picked for the synthetic test image.
4. **Integrate the vision subsystem into the top-level model.** Slice 2 is currently standalone; `FruitSorting_Hardware.slx` uses a mocked fruit list.

### 8.2 Dev-machine (can be done before lab day)

5. **Wire vision → planner in Simulink** even with mocked frames, so the lab visit only has to swap in real frames, not restructure wiring.
6. **Add a Stateflow chart** mirroring `sorting_controller_sim` states so the FSM is visible/editable in Simulink (currently the FSM is a MATLAB Function block + persistent handle — rubric may expect a visible Stateflow diagram).
7. **Remote-control mode** — not yet present in the facade. `main_remote.py` exists but is not called from any `.slx`. Decide whether to expose teleop through a MATLAB Function block or leave it as a pure Python entry point and document it in the report.
8. **Trajectory smoothing** — `trajectory.py` has a quintic planner but `py_controller.m` currently issues raw joint targets. Integrate so motion is smoother and jerk-limited.
9. **Logging / plots** — add `To Workspace` logging on every slice for the report figures. Currently only slices 1/3 log.
10. **Risk assessment form** — `Risk-Assessment-Form-AR - 2024-2025.docx` is unfilled. Required deliverable.

### 8.3 Report & deliverables

11. Final report PDF (team deliverable, not yet drafted).
12. Demo video (autonomous + remote), rubric asks for both modes.
13. **`COMPLETE_GUIDE.md` is drafted** but needs a refresh after slice 5 passed.
14. `PROJECT_CONTEXT.md` is the canonical session-loader — keep it and `PROGRESS.md` in sync.

---

## 9. Known environmental issues (unrelated to project code)

- **npm is corrupted** on the dev machine: `C:\Users\Mugin\AppData\Roaming\npm\node_modules\npm\...\@npmcli\fs\lib\common\` is missing `node.js`, so `npx -y ...` fails with `MODULE_NOT_FOUND`. This breaks npx-based MCP servers (`sequential-thinking`, `context7`). Fix plan: delete the broken user-prefix npm and reinstall via the bundled `C:\Program Files\nodejs\npm.cmd`. Pending user approval — **does not affect MATLAB/Python pipeline**, only Claude Code tooling.
- `context7` MCP server is **not declared** in `~/.claude.json`; must be added after npm is repaired.
- `sequential-thinking` is declared at user scope but cannot start until npm is repaired.

---

## 10. How to resume next session (checklist)

1. Open Claude Code in this working directory.
2. Auto-memory will load `project_fruitsorting.md` — confirms high-level state.
3. Read `PROJECT_CONTEXT.md` and **this file (`PROGRESS.md`)** for the full snapshot.
4. If running MATLAB: start MATLAB R2025a, `cd` to `FinalProject_FruitSorting/matlab_facade`, run `setup_pyenv`, then any `run_slice_*.m` / `run_FruitSorting.m`.
5. If doing lab work: bring a USB with the repo, install QUARC on the lab PC, then follow §8.1 in order.

---

## 11. Timeline to deadline

| Date | Milestone |
|------|-----------|
| **2026-04-14** (today) | All 5 sim slices PASS. Facade architecture locked. |
| 2026-04-15 → 2026-04-21 | Dev-machine pending items §8.2 (#5–#10). Draft report skeleton. |
| 2026-04-22 → 2026-04-28 | **Lab week** — execute §8.1 (#1–#4). Record demo video. |
| 2026-04-29 → 2026-04-30 | Final report pass, risk assessment, polish. Buffer day. |
| **2026-05-01 14:00** | **Submission deadline.** |

**17 calendar days remain.** Lab access is the critical-path bottleneck — every lab-only item must be done on real hardware, so the lab week cannot slip.
