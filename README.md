# QArm Fruit Sorting System

**Autonomous and teleoperated fruit sorting** with a **Quanser QArm 4-DOF robot** and **Intel RealSense D415 RGB-D camera**.

> University of Birmingham | Applied Robotics (04 39984) | 2025-2026  
> Team: Piero Flores, Zihen Huang, Ran Zhang, Yichang Chao

---

## Overview

The system sorts **14 fruits** (6 strawberries, 3 bananas, 5 tomatoes) into 3 baskets using computer vision for detection and inverse kinematics for pick-and-place motion planning. Two operational modes are supported:

- **Autonomous mode** - full vision-to-place pipeline: capture frame, detect fruits via HSV + shape classification, compute world coordinates, plan trajectories, execute pick-and-place cycle.
- **Remote-control mode** - keyboard-driven teleoperation with real-time Cartesian jog and gripper control.

## Architecture

```
+-------------------+       py.* bridge        +------------------+
|   Simulink        |  <-------------------->  |   Python Core    |
|   Facade          |   coder.extrinsic()      |                  |
|                   |                           |  Kinematics      |
|  MATLAB Function  |                           |  Vision          |
|  blocks calling   |                           |  Trajectory      |
|  Python modules   |                           |  Controller FSM  |
|                   |                           |  Hardware Driver  |
+-------------------+                           +--------+---------+
                                                         |
                                                   Quanser SDK
                                                   (USB HIL)
                                                         |
                                               +---------+---------+
                                               |   QArm Hardware   |
                                               |   + D415 Camera   |
                                               +-------------------+
```

**Design principle:** *Simulink is the facade, Python does the real work.* All algorithms are implemented in Python/NumPy with the Quanser SDK for hardware I/O. The Simulink layer wraps Python via `py.*` calls inside MATLAB Function blocks, providing a visible and auditable control architecture for grading. No QUARC is required.

## Python Core Modules

| Module | Purpose | Key Feature |
|--------|---------|-------------|
| `qarm_driver.py` | Hardware I/O wrapper | Atomic joint+gripper read/write via Quanser HIL, joint-limit clamping, safe `home()` with PID settle |
| `qarm_kinematics.py` | FK + IK (4-DOF) | Analytical IK (4 solutions) + Newton-Raphson refinement. Max round-trip error: **0.0002 mm** |
| `trajectory.py` | Motion planning | Cubic rest-to-rest splines with zero boundary velocity, multi-segment chaining |
| `fruit_detector.py` | HSV + shape classifier | Detects strawberry/tomato (red, circularity threshold) and banana (yellow), morphological cleanup |
| `camera.py` | D415 RGB-D capture | Quanser Video3D wrapper, auto-exposure, pixel-to-world back-projection |
| `sorting_controller.py` | 13-state FSM | INIT -> SCAN -> APPROACH -> PICK -> BASKET -> PLACE -> repeat until DONE |
| `sorting_controller_sim.py` | dt-driven FSM twin | Same logic without `time.time()`, exports integer `state_id` for Simulink logging |

## Simulink Facade (Vertical Slices)

The facade was built incrementally using a vertical-slice methodology:

| Slice | Model | What it validates | Status |
|-------|-------|-------------------|--------|
| 1 | `FruitSorting_slice.slx` | IK bridge: Python IK results propagate through MATLAB Function block | PASS |
| 2 | `FruitSorting_slice_vision.slx` | Vision: synthetic image -> detect 3 fruits -> fixed 8x5 output matrix | PASS |
| 3 | `FruitSorting_slice_fsm.slx` | FSM: 601 ticks, all 11 states visited, DONE reached | PASS |
| 5 | `FruitSorting_Hardware.slx` | Top-level: FSM -> QArm I/O feedback loop, `qarm_mode` gates sim vs hardware | PASS (sim) |

## Tools

### Teach Pendant (`teach_points.py`)

Interactive jog tool with live D415 camera view for recording workspace waypoints:

- **Real-time keyboard jog** via OpenCV window (arrows for XY, r/f for Z, q/e for wrist, SPACE for gripper)
- **Save/load** named poses (home, basket_A/B/C, calibration points) to `teach_points.json`
- **Goto** saved points by pressing digit keys 1-9 (panel shows mapping in top-right)
- **Test routine** (z key): automated pick-and-place sequence across saved points
- **Gripper safety**: capped close/open commands (`GRIP_CLOSE_CMD=0.65`, `GRIP_OPEN_CMD=0.10`) prevent servo overload (-1289). Actual position readback after every gripper operation.
- **Trace logger**: every event (goto, gripper, jog, HIL errors) logged to `logs/robot_trace_*.log` with timestamps for post-mortem debugging

### Hand-Eye Calibration (`calibrate_hand_eye.py`)

Computes the camera-to-base transformation matrix `T_cam_to_base`:

1. Record 6+ calibration points with `teach_points.py` (labels `cal_01`..`cal_06`)
2. Run calibration: click each marker in the camera view, script reads depth
3. Solves rigid transform via **Umeyama SVD** (closed-form, no RANSAC)
4. Outputs `calibration.json` with 4x4 matrix, per-point residuals, and intrinsics

### Camera Preview (`camera_preview.py`)

Live D415 viewer showing color + depth (Jet colormap) side-by-side with FPS counter. Press `s` to save snapshots.

## Hardware

| Component | Specification |
|-----------|--------------|
| Robot | Quanser QArm 4-DOF serial arm (yaw, shoulder, elbow, wrist + gripper) |
| Camera | Intel RealSense D415, 1280x720 @ 30fps (color + depth) |
| Interface | Quanser SDK for Windows (Python HIL API, no QUARC required) |
| Compute | Windows 10, Python 3.13, MATLAB R2025a |

### QArm DH Parameters

| Link | a (m) | alpha | d (m) | theta |
|------|-------|-------|-------|-------|
| 1 | 0 | -pi/2 | 0.14 | theta_1 |
| 2 | 0.3536 | 0 | 0 | theta_2 |
| 3 | 0 | -pi/2 | 0 | theta_3 |
| 4 | 0 | 0 | 0.40 | theta_4 |

## Quick Start

### Requirements

```bash
# Python packages (use Python 3.13 — Quanser SDK installed there)
C:\Python313\python.exe -m pip install numpy opencv-python

# Quanser SDK wheels (run from SDK installer directory)
cd "C:\Program Files\Quanser\Quanser SDK\python"
install_quanser_python_api.bat
```

### Verify Environment

```bash
cd python
C:\Python313\python.exe validate_python.py
# Expected: 4/4 PASS (FK/IK, Trajectory, Detection, Controller)
```

### Run Teach Pendant (with robot connected)

```bash
C:\Python313\python.exe python/teach_points.py
```

### Run Autonomous Mode

```bash
C:\Python313\python.exe python/main_autonomous.py
```

### Run Remote Control

```bash
C:\Python313\python.exe python/main_remote.py
```

### Run Simulink Facade (MATLAB)

```matlab
cd matlab_facade
setup_pyenv
run_FruitSorting   % simulation mode
% run_FruitSorting_hw  % hardware mode (lab only)
```

## Safety Features

- **Gripper overload prevention**: close commands capped at 0.65, open at 0.10 (never 0.0 or 1.0). After every gripper operation, the actual position is read back so subsequent commands match reality (zero residual torque).
- **No idle command spamming**: the QArm position-mode PID holds setpoints autonomously. The control loop does not resend commands when idle, preventing persistent motor loading.
- **Joint limit clamping**: all IK solutions are clamped to per-axis limits before sending to hardware.
- **Z-height safeguards**: approach and transit heights prevent table collisions.
- **Crash-safe trace logging**: every command, state change, and error is logged line-buffered to disk. If the robot faults, the log file contains the full event trail up to the crash.

## Project Structure

```
FinalProject_FruitSorting/
|-- python/                     Core algorithms and tools
|   |-- qarm_driver.py          Quanser HIL hardware wrapper
|   |-- qarm_kinematics.py      FK/IK (sub-um accuracy)
|   |-- trajectory.py           Cubic spline planner
|   |-- fruit_detector.py       HSV + shape fruit classifier
|   |-- camera.py               RealSense D415 wrapper
|   |-- sorting_controller.py   13-state autonomous FSM
|   |-- sorting_controller_sim.py  dt-driven FSM for Simulink
|   |-- main_autonomous.py      Autonomous entry point
|   |-- main_remote.py          Teleoperation entry point
|   |-- teach_points.py         Jog + save poses + test routines
|   |-- calibrate_hand_eye.py   Camera-to-base SVD solver
|   |-- camera_preview.py       Live camera viewer
|   +-- validate_python.py      Offline test suite (4/4 PASS)
|
|-- matlab_facade/              Simulink wrapper (5 slices, all PASS sim)
|   |-- setup_pyenv.m           Python bridge init
|   |-- py_ik.m, py_fk.m        Kinematics wrappers
|   |-- py_detect_test.m        Vision bridge (fixed 8x5 output)
|   |-- py_controller.m         Persistent FSM handle
|   |-- py_qarm_io.m            Hardware I/O facade (modes 0/1/2)
|   |-- build_slice_*.m         Model builders
|   |-- run_slice_*.m           Slice runners
|   +-- *.slx                   Simulink models
|
|-- figures/                    Simulation plots and camera captures
|-- logs/                       Robot trace logs (one per session)
|-- scripts/                    Calibration helpers, PDF generator
|-- teach_points.json           Recorded workspace poses
+-- calibration.json            Hand-eye calibration output
```

## Status

| Component | Status |
|-----------|--------|
| Python core (all modules) | Validated offline, all tests pass |
| Simulink facade (5 slices) | All pass in simulation mode |
| QArm hardware communication | Verified (read/write joints, gripper, LEDs) |
| D415 camera capture | Working (1280x720, auto-exposure enabled) |
| Teach pendant + goto + routine | Working with gripper safety |
| Hand-eye calibration tool | Built, untested on real hardware |
| Full autonomous demo | Pending lab calibration + HSV tuning |
| Remote-control demo | Pending integration into facade |
| Final report + presentation | In progress |
