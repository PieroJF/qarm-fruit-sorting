# FinalProject_FruitSorting — Project Context Snapshot

_Last updated: 2026-04-20. Deadline: **Fri 1 May 2026, 14:00**._

> **State-of-sprint** (2026-04-20): 14/14 integration tests + 7/7 UGreen tracker tests + 2/2 closed-loop calibration tests. Phase A of the final-sprint plan (`docs/superpowers/plans/2026-04-20-final-sprint.md`) is complete through Task A7.5. Phase B (lab hardware) pending session access.
>
> **Key architectural finding (2026-04-17 lab session):** the D415 camera is **arm-mounted on the QArm wrist**. `T_cam_to_base` is pose-dependent — the calibration is derived at `pickhome1` and is only valid at that pose. See `recal_from_pose.py` for the per-pose workaround. Proper hand-eye (AX=XB) solve was descoped — we added a UGreen floor-level camera instead, which gives an external fixed reference for closed-loop visual calibration and pick validation via image diff.
>
> **Spec + plan:** `docs/superpowers/specs/2026-04-20-final-sprint-design.md` and `docs/superpowers/plans/2026-04-20-final-sprint.md` are the source of truth for the remaining work.
>
> **Lab runbook:** `LAB_RUNBOOK.md` documents session flow from power-on to shutdown.

## 1. Project Identity

- **Course**: Applied Robotics (04 39984), University of Birmingham
- **Team**: Piero Flores, Zihen Huang, Ran Zhang, Yichang Chao
- **Task**: Sort 14 fruits (6 strawberries, 3 bananas, 5 tomatoes) into 3 baskets using a **Quanser QArm** 4-DOF + gripper and an **Intel RealSense D415** RGBD camera. Must support **autonomous** mode and **remote (keyboard)** teleoperation.
- **Stack in use**: **Quanser SDK for Windows** (HIL + Video3D), Python 3.11, NumPy, OpenCV.
- **Deliverables**: 22-page report, ≤5-slide presentation (12 min + 3 Q&A), live demo on real hardware, peer assessment.

## 2. ⚠️ Architecture Mismatch vs. User's Intent

**User intent** (stated now): MATLAB/Simulink must be the **facade**, Python does the real work underneath. Ideal = Simulink blocks calling Python functions (e.g. MATLAB Function block with `py.module.fn(...)`, or a Level-2 S-Function / `pyrunfile`).

**Current repo reality**: The project is **100% Python** against the Quanser SDK. MATLAB exists only as reference `.m` files that mirror the Python logic. `scripts/create_FruitSorting_Model.m` is a stub Simulink generator explicitly labelled legacy / not used for deployment. There is **no `.slx` model wired to Python, no QUARC blocks, no MATLAB↔Python bridge**.

**Implication**: To meet the stated intent, we need to add a MATLAB/Simulink facade layer on top of the existing Python modules — not rewrite the core. See §7 for gap list.

## 3. Directory Tree

```
FinalProject_FruitSorting/
├── COMPLETE_GUIDE.md / .pdf        Operational guide (470 lines)
├── python/                          PRODUCTION CODE (validated)
│   ├── main_autonomous.py           Autonomous entry point
│   ├── main_remote.py               Keyboard teleop entry point
│   ├── qarm_driver.py               Quanser HIL wrapper (qarm_usb board)
│   ├── qarm_kinematics.py           FK + analytical IK + Newton-Raphson
│   ├── trajectory.py                Cubic splines (zero boundary vel)
│   ├── fruit_detector.py            HSV + circularity classification
│   ├── camera.py                    RealSense D415 via Quanser Video3D
│   ├── sorting_controller.py        13-state FSM @ 500 Hz
│   └── validate_python.py           Offline test suite — ALL PASS
├── matlab_code/                     REFERENCE .m ONLY (not wired up)
│   ├── qarm_FK.m / qarm_IK.m
│   ├── cubicTrajectory.m / multiSegmentTrajectory.m
│   ├── fruitDetector.m / pixelToWorld.m
│   └── fruitSortingController.m     (persistent-state FSM mirror)
├── scripts/
│   ├── calibrate_camera.m           SVD rigid-body cam→base fit
│   ├── create_FruitSorting_Model.m  Simulink stub (LEGACY, unused)
│   ├── test_fruit_detection.m       Synthetic image test
│   ├── validate_fruit_sorting.m     Generates report figures
│   └── generate_pdf_guide.py        Builds COMPLETE_GUIDE.pdf
└── figures/                         Report figures (already rendered)
    ├── trajectory_3D.png  ee_position_time.png
    ├── joint_angles_time.png  gripper_state.png  top_view.png
```

## 4. Python Modules — Quick Reference

| File | Role | Key API |
|---|---|---|
| `qarm_driver.py` | Hardware I/O via `quanser.hardware.HIL("qarm_usb")`. Channels 1000–1003 joints, 1004 gripper, 11005–11008 LEDs. Joint limits clamped. | `connect/disconnect`, `read_all`, `set_joints_and_gripper`, `home(duration)` |
| `qarm_kinematics.py` | DH params λ1=0.14, λ2=0.3536, λ3=0.40, β=0.1419. Analytical IK (4 solutions) → best valid → Newton-Raphson refinement (damped LS, λ=0.001, tol=1e-6). | `forward_kinematics(phi)`, `inverse_kinematics(p, gamma=0)` |
| `trajectory.py` | Cubic rest-to-rest: `a0=p_s, a2=3Δ/T², a3=-2Δ/T³`. | `cubic_trajectory`, `multi_segment_trajectory`, `smooth_move` |
| `fruit_detector.py` | HSV masks → morphology (7×7 ellipse) → contours → classify. Red split by circularity (>0.65 & sat>140 ⇒ tomato, else strawberry). | `detect_fruits(bgr, depth=None)` → `FruitDetection[]` |
| `camera.py` | Quanser `Video3D` RealSense wrapper. Default intrinsics fx=fy=615, cx=320, cy=240 @ 640×480 30 fps. | `open/read/close`, `pixel_to_world(row,col,depth_mm,T_cam_to_base)` |
| `sorting_controller.py` | 13-state FSM: INIT→GO_HOME→SCAN→SELECT_FRUIT→APPROACH→DESCEND→CLOSE_GRIPPER→ASCEND_PICK→MOVE_TO_BASKET→DESCEND_PLACE→OPEN_GRIPPER→ASCEND_PLACE→DONE. | `run_autonomous(dt=0.002)` |
| `main_autonomous.py` | Wires camera + controller. `USE_CAMERA=False`, `T_CAM_TO_BASE=np.eye(4)` placeholders. | CLI entry |
| `main_remote.py` | Quanser `Keyboard` device (text fallback). Arrows XY, Q/A Z, Space gripper, H home, ESC exit. EE_SPEED=0.05 m/step. | CLI entry |

### Fixed parameters (hardcoded in controller)

```
HOME_POS   = [0.45, 0.0, 0.49]    SAFE_Z = 0.20
APPROACH_Z = 0.15   PICK_Z = 0.02   PLACE_Z = 0.10
T_TRANSIT=2.0  T_APPROACH=1.0  T_PICK=0.8  T_DWELL=0.5
Baskets: strawberry (0.30,-0.20,0.05)  banana (-0.30,-0.20,0.05)  tomato (0.00,-0.35,0.05)
```

### Validation status (`validate_python.py`)

| Test | Result |
|---|---|
| FK/IK round-trip on 6 targets | PASS (max 0.0002 mm) |
| Cubic trajectory smoothness | PASS (boundary v ≈ 1e-6 m/s) |
| HSV detection on synthetic image | PASS (red + yellow detected) |
| Full 3-fruit controller sim | PASS (reaches DONE) |

## 5. MATLAB Reference Code

`matlab_code/*.m` mirrors the Python logic one-to-one (FK/IK with the same DH params, cubic splines, HSV detector with identical thresholds, 13-state controller via `persistent` variables, `pixelToWorld` with same D415 intrinsics). These files are **not called from anywhere** — they exist as documentation and as candidate wrappers for the facade.

`scripts/create_FruitSorting_Model.m` programmatically builds a `FruitSorting_Hardware.slx` with stub subsystems (QArm Plant, Autonomous Controller, Remote Controller, Vision, FK, scopes) at ode4 / dt=0.002. Header comment says to replace stubs with QUARC blocks; in practice nobody ran it and **no .slx file exists** in the repo.

## 6. What's Done vs. What's Missing

**Done**
- All core Python algorithms validated offline (sub-µm IK accuracy).
- Complete operational guide + PDF.
- Report figures pre-rendered from simulation.
- Calibration + synthetic-detection helper scripts.

**Missing — hardware & tuning (in-lab work)**
- Install Quanser SDK + Python wheels on lab PC.
- Collect ≥6 calibration points; compute `T_cam_to_base` via SVD; replace `np.eye(4)`.
- Tune HSV thresholds to lab lighting.
- Verify workspace reach; confirm basket positions match the rig.
- Full 14-fruit autonomous run + remote demo + video capture.
- Soft gripper physical design (mentioned in report spec, not in code).

**Missing — deliverables**
- 22-page report (all sections).
- ≤5-slide presentation.

## 7. Gap vs. User Intent: "MATLAB Facade → Python Core"

To satisfy the stated architecture, the following needs to be added (nothing in the repo does this today):

1. **MATLAB↔Python bridge decision**. Options, from lightest to heaviest:
   - (a) MATLAB `py.` module calls — `py.fruit_detector.detect_fruits(...)`. Trivial from an .m file or a MATLAB Function block that's wrapped via `coder.extrinsic('py.*')`. Will not codegen to QUARC but runs in normal sim.
   - (b) `pyrunfile` / `pyrun` inside a MATLAB Function block — passes NumPy arrays in/out.
   - (c) Level-2 MATLAB S-Function that owns a persistent Python interpreter and calls the modules per tick.
   - (d) A thin TCP/UDP or shared-memory shim — Python daemon holds the Quanser SDK session, Simulink sends setpoints / receives state each step. Most robust if we want the Simulink loop to remain real-time.
2. **A top-level `.slx`** with at minimum: a Vision subsystem (calls `py.fruit_detector`), a Planner/FSM subsystem (calls `py.sorting_controller` or re-implements the FSM in Stateflow and only calls `py.qarm_kinematics.inverse_kinematics`), a QArm Plant I/O subsystem (the delicate part — Quanser SDK lives in Python today, so Simulink either reaches the hardware through the Python bridge or we re-open it via QUARC blocks).
3. **An entry launcher** (`run_FruitSorting.m`) that opens the model and starts simulation, so graders see MATLAB as the face of the project.
4. **Report/presentation framing** adjusted so the "Python everywhere" story becomes "Simulink orchestrates, Python accelerates perception/kinematics."

**Key constraint**: Quanser SDK for Windows exposes both Python and MATLAB/Simulink (QUARC) bindings. If we use QUARC blocks for the hardware I/O and `py.*` only for vision + kinematics, the facade is believable and the real-time loop stays native. If we use the Python driver for hardware, we must route Simulink through a bridge — workable but adds latency.

## 8. Immediate Next Decisions (for the user)

- **Bridge choice** (a/b/c/d above) — drives all downstream work.
- **Who owns hardware I/O** in the facade model — QUARC blocks vs. the existing `qarm_driver.py`.
- **Which Python modules stay as-is** (recommend: `qarm_kinematics`, `trajectory`, `fruit_detector`) **vs. which get rebuilt in Simulink** (candidate: the 13-state FSM → Stateflow chart, which is visually compelling for the demo).

_End of snapshot._
