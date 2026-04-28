# Complete Step-by-Step Guide: QArm Fruit Sorting Final Project
## Applied Robotics (04 39984) - University of Birmingham

**Deadline:** Friday 1 May 2026, 2:00 PM
**Team:** Piero Flores, Zihen Huang, Ran Zhang, Yichang Chao
**Architecture:** Python + Quanser SDK for Windows

---

## TABLE OF CONTENTS

1. Project Overview
2. Software Status (What Is Done)
3. Manual Tasks (In Lab)
4. Environment Setup (Quanser SDK + Python)
5. Step-by-Step Lab Procedure
6. Camera Calibration Procedure
7. Running Autonomous Mode
8. Running Remote Control Mode
9. Testing and Validation
10. Report Writing Guide
11. Presentation Guide
12. Troubleshooting

---

## 1. PROJECT OVERVIEW

### Requirements
- Sort 14 fruits (6 strawberries, 3 bananas, 5 tomatoes) into 3 baskets
- Autonomous mode: robot detects, picks, and sorts fruits automatically
- Remote control mode: operator controls the robot via keyboard
- Hardware: Quanser QArm 4-DOF + Intel RealSense D415 RGBD camera
- Software: Python 3 + Quanser SDK for Windows (NOT QUARC/Simulink)

### Deliverables
1. Slides (5 max) with final presentation
2. Final report (22 pages) due 1 May 2026
3. Working demo on real QArm hardware
4. Peer assessment due 1 May 2026

---

## 2. SOFTWARE STATUS (What Is Done)

All core code is written in Python and has passed offline validation.

### Python modules (`FinalProject_FruitSorting/python/`)

| File | Purpose | Status |
|------|---------|--------|
| `qarm_driver.py` | QArm hardware driver (HIL over USB) | Ready |
| `qarm_kinematics.py` | Forward + inverse kinematics (pure NumPy) | Validated, max error 0.0002 mm |
| `trajectory.py` | Cubic-spline trajectory generation | Validated |
| `fruit_detector.py` | HSV + contour fruit classification (OpenCV) | Validated |
| `camera.py` | RealSense D415 wrapper (Quanser Video3D) | Ready (needs real camera) |
| `sorting_controller.py` | 13-state pick-and-place state machine | Validated in sim |
| `main_autonomous.py` | Entry point for autonomous mode | Ready |
| `main_remote.py` | Entry point for keyboard teleoperation | Ready |
| `validate_python.py` | Offline validation suite | All tests PASS |

### Validation results (from `validate_python.py`)

```
FK/IK                [PASS]   max error 0.0002 mm
Trajectory           [PASS]   zero boundary velocities
Detection            [PASS]   red + yellow fruit found
Controller           [PASS]   3 fruits sorted in state machine
```

---

## 3. MANUAL TASKS (In Lab)

These tasks REQUIRE physical access to the QArm:

- [ ] 3.1 Install Quanser SDK for Windows on the lab PC
- [ ] 3.2 Install Python dependencies (numpy, opencv-python, quanser)
- [ ] 3.3 Set up the physical workspace (fruits + 3 baskets)
- [ ] 3.4 Run `main_autonomous.py` with `USE_CAMERA=False` for a dry run
- [ ] 3.5 Calibrate the camera (compute `T_cam_to_base`)
- [ ] 3.6 Tune HSV thresholds for actual lab lighting
- [ ] 3.7 Test autonomous mode on real hardware
- [ ] 3.8 Test remote control mode on real hardware
- [ ] 3.9 Record videos and screenshots
- [ ] 3.10 Design soft gripper (report section)
- [ ] 3.11 Write the 22-page report
- [ ] 3.12 Prepare the final presentation

---

## 4. ENVIRONMENT SETUP (Quanser SDK + Python)

### 4.1 Install the Quanser SDK for Windows
1. Download the SDK installer from the Quanser student portal.
2. Run the installer with admin rights. Accept defaults. Location is typically `C:\Program Files\Quanser\QuanserSDK`.
3. Reboot after install so environment variables take effect.

### 4.2 Install Python 3.11 (x64)
The Quanser Python wheels target CPython 3.11 x64. Install from python.org and tick "Add python.exe to PATH".

### 4.3 Install the Quanser Python API
Open PowerShell as the lab user and run:

```powershell
cd "C:\Program Files\Quanser\QuanserSDK\python"
pip install quanser_api-*.whl
```

If there are multiple wheels (hardware, multimedia, devices, common), install all of them:

```powershell
pip install quanser_common-*.whl quanser_hardware-*.whl quanser_multimedia-*.whl quanser_devices-*.whl
```

### 4.4 Install third-party dependencies
```powershell
pip install numpy opencv-python
```

### 4.5 Verify the install
```powershell
python -c "from quanser.hardware import HIL; from quanser.multimedia import Video3D; print('Quanser OK')"
```

### 4.6 Run the offline validation
```powershell
cd "FinalProject_FruitSorting\python"
python validate_python.py
```
Expect: "All tests PASSED!" This confirms the kinematics, trajectory, detection, and controller all work WITHOUT the robot connected.

---

## 5. STEP-BY-STEP LAB PROCEDURE

### 5.1 Workspace setup (15 min)
1. Power on the QArm and plug the USB cable into the lab PC.
2. Wait for Windows to enumerate the device (a LED ring turns green).
3. Place the 3 baskets on the table measured from the QArm base:

```
           QArm Base (0,0)
               |
  Banana     Tomato     Strawberry
  Basket     Basket     Basket
(-0.30,     (0.00,     (0.30,
 -0.20)     -0.35)     -0.20)

   [Fruits on the table in front of the robot]
   (positive Y direction, Y in [0.15, 0.40], X in [-0.25, 0.25])
```

4. Baskets should be roughly 10 cm tall. Fruits sit at table level (z around 0.02 m above the base plane).

### 5.2 First hardware connectivity test (no motion)
In PowerShell:
```powershell
cd "FinalProject_FruitSorting\python"
python -c "from qarm_driver import QArmDriver; q=QArmDriver(); q.connect(); print(q.read_joint_positions()); q.disconnect()"
```
You should see 4 joint angles printed in radians. If this fails, go to Section 12 Troubleshooting.

### 5.3 Home position test (small motion)
```powershell
python -c "from qarm_driver import QArmDriver; q=QArmDriver(); q.connect(); q.home(duration=3.0); q.disconnect()"
```
The arm should smoothly move to zero joint angles and the gripper should open.

---

## 6. CAMERA CALIBRATION

Camera calibration finds the 4x4 transform `T_cam_to_base` from the RealSense optical frame to the QArm base frame.

### 6.1 Collect calibration points
1. Power on the camera (it enumerates automatically as a Video3D device).
2. Place a brightly coloured ball at a known position measured from the QArm base with a ruler.
3. Record: `(pixel_col, pixel_row, depth_mm, world_x, world_y, world_z)`.
4. Repeat for at least 6 positions spread across the workspace (front-left, front-right, back-left, back-right, centre-near, centre-far).

### 6.2 Run the calibration script
A calibration helper script lives at `FinalProject_FruitSorting/scripts/calibrate_camera.m` (MATLAB) or can be done in Python with the SVD rigid-body fit below. Paste your 6+ points into a file `calibration_points.csv` with columns `col,row,depth_mm,X,Y,Z` and run:

```python
import numpy as np, cv2
data = np.loadtxt('calibration_points.csv', delimiter=',', skiprows=1)
fx=fy=615.0; cx=320.0; cy=240.0   # RealSense defaults
P_cam = np.zeros((len(data),3))
for i,(col,row,d,X,Y,Z) in enumerate(data):
    z = d/1000.0
    P_cam[i] = [(col-cx)*z/fx, (row-cy)*z/fy, z]
P_base = data[:,3:6]
# Rigid-body SVD fit
c1 = P_cam.mean(0); c2 = P_base.mean(0)
H = (P_cam-c1).T @ (P_base-c2)
U,S,Vt = np.linalg.svd(H)
R = Vt.T @ U.T
if np.linalg.det(R) < 0: Vt[-1]*=-1; R=Vt.T@U.T
t = c2 - R@c1
T = np.eye(4); T[:3,:3]=R; T[:3,3]=t
np.save('T_cam_to_base.npy', T)
print('Reprojection error (mm):', np.linalg.norm((P_cam@R.T+t)-P_base,axis=1).mean()*1000)
```

Aim for less than 10 mm mean error. Save `T_cam_to_base.npy` in the `python/` folder.

### 6.3 Load calibration in the main scripts
In `main_autonomous.py` set `USE_CAMERA = True` and the script loads `T_cam_to_base.npy` at startup.

---

## 7. AUTONOMOUS MODE

### 7.1 State machine (13 states)
```
INIT -> GO_HOME -> APPROACH -> DESCEND -> CLOSE_GRIPPER
  ^                                            |
  |                                            v
  |                                      ASCEND_PICK
  |                                            |
  |                                            v
  |                                      MOVE_TO_BASKET
  |                                            |
  |                                            v
  |                                      DESCEND_PLACE
  |                                            |
  |                                            v
  |                                      OPEN_GRIPPER
  |                                            |
  |                                            v
  +----------- next fruit ---- ASCEND_PLACE --+
                 (or)
                 DONE
```

### 7.2 Run it
```powershell
cd "FinalProject_FruitSorting\python"
python main_autonomous.py
```

The script will:
1. Connect to the QArm
2. Home the arm
3. Capture an RGBD frame (if `USE_CAMERA=True`)
4. Detect fruits (`fruit_detector.py`)
5. Convert pixels to base-frame positions
6. Run the state machine for each fruit
7. Return home and disconnect cleanly

### 7.3 Key parameters (in `sorting_controller.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SAFE_Z` | 0.20 m | Safe transit height |
| `APPROACH_Z` | 0.15 m | Hover height above fruit |
| `PICK_Z` | 0.02 m | Pick height |
| *(dynamic)* | basket Z..SAFE_Z | Release height — IK-searched from basket's taught Z upward |
| `T_TRANSIT` | 2.0 s | Duration for long moves |
| `T_APPROACH` | 1.0 s | Duration for approach/retreat |
| `T_PICK` | 0.8 s | Duration for final descent |
| `T_DWELL` | 0.5 s | Gripper settle time |
| Basket positions | see file | EDIT to match your table |

Edit `BASKET_POSITIONS` in `sorting_controller.py` if your basket layout differs.

---

## 8. REMOTE CONTROL MODE

### 8.1 Run it
```powershell
python main_remote.py
```

The script opens a `quanser.devices.Keyboard` capture. If that device is not available it falls back to standard text input.

### 8.2 Controls

| Key | Action |
|-----|--------|
| Up Arrow | +X (away from operator) |
| Down Arrow | -X (toward operator) |
| Left Arrow | -Y (left) |
| Right Arrow | +Y (right) |
| Q | +Z (up) |
| A | -Z (down) |
| Space | Toggle gripper |
| H | Go home |
| Esc | Quit cleanly |

Speed gain: 0.05 m per key press. Commands are converted to joint targets via inverse kinematics at 500 Hz.

---

## 9. TESTING AND VALIDATION

### 9.1 Pre-deployment checklist
- [ ] Quanser SDK imports OK (`from quanser.hardware import HIL`)
- [ ] `validate_python.py` prints "All tests PASSED!"
- [ ] `qarm_driver.py` connects to real QArm and reads joints
- [ ] Camera calibration file `T_cam_to_base.npy` exists
- [ ] HSV thresholds tuned for actual lighting
- [ ] Basket positions measured and updated
- [ ] Emergency stop location known by every team member

### 9.2 Test sequence
1. Single fruit test: place ONE tomato, run auto, record video.
2. Multi fruit test: one of each type, verify basket assignment.
3. Full 14 fruit test: time the whole run, record.
4. Remote control test: sort 3 fruits manually, record.

### 9.3 Expected performance
- Single cycle: 10-12 s
- Full 14 fruit sort: 2-3 min
- Position accuracy: under 5 mm with calibrated camera

---

## 10. REPORT WRITING GUIDE

Structure (22 pages total):

**Part 1: Design and Simulation (12 pages)**

1. Title Page (1 p)
2. Introduction (1.5 p): objectives, QArm specs, problem statement
3. Kinematics (2 p): DH table, FK derivation, analytical IK + Newton-Raphson refinement, validation (max 0.0002 mm error)
4. Trajectory Generation (1.5 p): cubic splines, zero boundary velocities, multi-segment planning
5. Vision System (2 p): RealSense D415, HSV segmentation, circularity + saturation classification, calibration procedure
6. Control Architecture (2 p): state machine diagram, autonomous flow, remote flow, Python module layout (replace old Simulink block diagram with the Python module diagram below)
7. Soft Gripper Design (2 p): concept, grip force analysis, material selection

**Part 2: Validation (7 pages)**

8. Simulation Results (2 p): figures from `figures/`, performance metrics
9. Hardware Results (2.5 p): photos, sim vs hardware comparison, success rate
10. Discussion (1.5 p): discrepancies, limitations, challenges
11. Conclusion and Future Work (1 p)

**Feedback Page (1 p)**

### Software architecture figure (for Section 6)

```
  [RealSense D415]
         |  (Video3D)
         v
  [camera.py]  ->  [fruit_detector.py]  ->  fruit list
                                                |
                                                v
  [Clock] --> [sorting_controller.py] <-- [qarm_kinematics.py]
                     |                            ^
                     v                            |
           [trajectory.py] ----> joint targets ---+
                     |
                     v
               [qarm_driver.py]
                     |
                     v
               [Quanser HIL / USB]
                     |
                     v
                [QArm Hardware]
```

---

## 11. PRESENTATION GUIDE

Final presentation: 12 minutes + 3 min Q&A (covers all three assignments).

Slides for the final project portion:
- System Architecture (Python module diagram + state machine)
- Vision + Detection Results
- Autonomous Demo Results
- Remote Control Demo
- Challenges, Solutions, Conclusions

Video clips to record:
1. Autonomous sorting of 3+ fruits (~30 s)
2. Remote control sorting (~15 s)
3. Detection visualisation from `draw_detections()` overlay

---

## 12. TROUBLESHOOTING

### "ImportError: No module named quanser"
The Quanser Python wheels are not installed in the active Python interpreter. Re-run Section 4.3 with the same `python.exe` you use to launch scripts.

### "HIL open failed / board not found"
- Check USB cable and power LED on the QArm.
- Confirm board id. Default is `"0"`. If two arms are plugged in try `"1"`.
- Close any other program holding the board (another Python session, Device Manager dialogs).

### "Joints read OK but arm does not move"
- Card specific options may not be set. Ensure `position_mode=True` when constructing `QArmDriver`.
- Check joint limits. `set_joint_positions` clamps values to the physical limits, so a wildly out-of-range target becomes a small motion.

### "Fruit not detected"
- HSV thresholds must be tuned per lighting. Edit `HSV_RANGES` in `fruit_detector.py`.
- Use `draw_detections()` on a saved frame to iterate quickly.
- Check the depth image is not all zeros (RealSense needs a few frames to warm up).

### "Camera calibration error > 10 mm"
- Need more points (minimum 6, ideally 10+) spread across the workspace.
- Double check world coordinates with a ruler.
- Make sure the depth value used is in millimetres.

### "Robot moves too fast or jerky"
- Increase `T_TRANSIT` and `T_APPROACH` in `sorting_controller.py`.
- The cubic spline already enforces zero boundary velocities.
- Confirm the control loop runs at `dt=0.002` (500 Hz).

### "Gripper does not grip"
- The gripper channel is 1004 in the write buffer.
- Values: 0.0 = fully open, 1.0 = fully closed. Start at 0.7 and increase.
- Fragile fruits may need a soft gripper add-on (see report section 7).

---

## QUICK REFERENCE: FILE LOCATIONS

```
FinalProject_FruitSorting/
    python/
        qarm_driver.py         QArm HIL hardware driver
        qarm_kinematics.py     FK + IK (analytical + Newton-Raphson)
        trajectory.py          Cubic spline trajectory generation
        fruit_detector.py      HSV fruit detection (OpenCV)
        camera.py              RealSense D415 via Quanser Video3D
        sorting_controller.py  13-state pick-and-place controller
        main_autonomous.py     Autonomous entry point
        main_remote.py         Remote control entry point
        validate_python.py     Offline validation suite
    matlab_code/               Legacy MATLAB versions (reference)
    scripts/
        calibrate_camera.m     Camera calibration helper
        validate_fruit_sorting.m
    figures/                   Simulation figures for report
```

## TASK ASSIGNMENT SUGGESTION

| Member | Tasks |
|--------|-------|
| Piero  | Quanser SDK install, run hardware tests, integrate camera |
| Zihen  | Camera calibration, tune HSV thresholds |
| Ran    | Hardware testing + video recording |
| Yichang| Report writing + presentation slides |
| All    | Lab sessions, review report before submission |

---

## TIMELINE TO DEADLINE (1 May 2026)

| Week | Tasks |
|------|-------|
| Now  | Install Quanser SDK, run `validate_python.py`, dry run with `USE_CAMERA=False` |
| +1   | Camera calibration, tune detection, first hardware test |
| +2   | Full auto + remote testing, record videos |
| +3   | Write report, prepare slides, final testing |
| Final| Review and submit report + peer assessment |

---

*All Python code has passed offline validation. Remaining work is hardware integration, calibration, testing, and documentation.*
