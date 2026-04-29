# Final Presentation Design — FruitSorting QArm Project

**Date**: 2026-04-29
**Deadline**: 2026-05-01 14:00
**Team**: Piero Flores | Zihen Huang | Ran Zhang | Yichang Chao
**Format**: 12 minutes + 3 min Q&A, all members present
**Tool**: `python-pptx` script generating 16:9 widescreen `.pptx`

---

## 1. Scope

8-slide "main" deck covering the full project end-to-end. After generation, the team appends the 3 existing assignment slide decks (A1, A2, A3 — 5 slides each). Total: up to 23 slides for the 12-minute session.

The script reuses the Assignment 3 aesthetic (dark navy `#1A1A2E`, Quanser blue `#0096D6`, orange `#FF8C00`, green `#4CAF50`) and the same helper functions (`add_bg`, `add_textbox`, `add_rect`, `add_placeholder_box`, `header_bar`, `footer_bar`).

## 2. Marking Scheme Coverage

| # | Criterion | Weight | Covered in slides |
|---|-----------|--------|-------------------|
| 1 | Quality of slides | 15% | All (consistent theme, layout, typography) |
| 2 | Step-by-step detail + challenges | 30% | 2, 3, 4, 5, 7 |
| 3 | Experimental results | 20% | 6 |
| 4 | Theory-practice link | 20% | 3, 4, 8 |
| 5 | Video/images/demos | 10% | 1, 3, 5, 6, 8 (placeholder boxes) |
| 6 | Improvements from feedback | 5% | 7 |

## 3. Slide-by-Slide Design

### Slide 1 — Title + Project Overview

- **Header:** "Robotic Pick & Place for Fruit Sorting"
- **Subtitle:** "Applied Robotics (04 39984) — University of Birmingham"
- **Left:**
  - Sort 14 fruits (6 strawberry, 3 banana, 5 tomato) into 3 baskets
  - Two modes: Fully Autonomous + Remote-Controlled (GUI)
  - Hardware: Quanser QArm 4-DOF + Intel RealSense D415 RGBD (arm-mounted)
  - Software: Python 3.13, OpenCV, Tkinter, Vosk
- **Right:** Placeholder for lab photo of physical QArm setup
- **Footer:** Team names + Module Coordinator: Dr Amir Hajiyavand

### Slide 2 — System Architecture

- **Header:** "System Architecture: End-to-End Pipeline"
- **Center:** Horizontal pipeline: `D415 Camera → Calibration → Fruit Detection → Target Selection → IK Solver → Quintic Trajectory → Pick & Place FSM → Basket Sort`
- **Below pipeline, 3 columns:**
  - Sensing: D415 RGB+Depth, chessboard cal, session-cal homography, arm-mounted pose-dependent `T_cam_to_base`
  - Planning: HSV segmentation, shape filtering, ray-plane projection, per-fruit bias (calyx, inscribed disk)
  - Actuation: 4-DOF analytic IK, quintic spline trajectories, 13-state FSM, gripper ramp+readback
- **Bottom-right:** Two modes box (Autonomous: full loop / Remote: GUI + live feed)

### Slide 3 — Calibration & Vision Pipeline

- **Header:** "Calibration & Vision: From Pixels to Base-Frame Coordinates"
- **Left — Calibration:**
  - 7×5 inner corners, 30mm squares
  - Jog-touch origin → `chess_origin_in_base_m`
  - D415 from `survey1` → `findChessboardCorners` → `solvePnP`
  - Homography pixel-to-table projection
  - RMS gate: warn >3mm, error >10mm
  - Key equations: rigid transform, pinhole model, ray-plane intersection with `λ`
- **Right — Detection:**
  - Per-fruit HSV ranges (banana H:14-40, tomato/strawberry hue-wrap)
  - Morphological open/close → contour → area/circularity gates
  - Strawberry special: green calyx detection, widest inscribed disk, calyx direction bias
  - Placeholder: detection overlay screenshot

### Slide 4 — Control & Trajectory

- **Header:** "Control System: IK, Trajectories & Pick-Place FSM"
- **Left — IK & Trajectory:**
  - 4-DOF analytic, 4 solutions, joint-limit filter, optimal = min displacement
  - DH-to-physical: `φ₂ = θ₂ + π/2 - β`, `φ₃ = θ₃ + β`
  - Quintic: `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`, zero v and a at endpoints
  - Improvement over cubic: jerk-continuous
- **Right — FSM:**
  - 13-state diagram: INIT → GO_HOME → SCAN → SELECT → APPROACH → DESCEND → CLOSE → ASCEND → BASKET → DESCEND_PLACE → OPEN → ASCEND → DONE
  - Per-fruit tuning table (pick depth, x bias, grip close)
  - Gripper ramp+readback

### Slide 5 — GUI & Voice Control

- **Header:** "Human Interface: GUI + Voice Control"
- **Left — GUI:**
  - Layout diagram: live D415 feed, E-STOP, teleop buttons, mode toggle, status
  - Live camera during picks, E-STOP halts arm, concurrent-read safe
  - Placeholder: GUI screenshot
- **Right — Voice:**
  - Vosk wake-word protocol: "robot" → 2s window → command → dispatch
  - "stop" always dispatched
  - Grammar: robot, strawberry, banana, tomato, all, stop, home, refresh, survey
  - Threaded, offline, graceful degradation
  - Theory link: FSM pattern, real-time audio at 16kHz

### Slide 6 — Results & Validation

- **Header:** "Experimental Results & Validation"
- **Top row, 3 boxes:** Calibration metrics, Detection stats, Pick & Place outcomes
- **Middle:** Test suite summary — 60/60 tests, 6 suites + voice control tests
- **Bottom:** Placeholder boxes for trajectory plot, gripper state plot, top-view plot (from `figures/`)

### Slide 7 — Challenges, Solutions & Feedback

- **Header:** "Challenges, Solutions & Improvements from Feedback"
- **Left — 6 challenge/solution pairs:**
  1. Cal drift → session-cal quick re-cal <30s
  2. Strawberry grip loose → 0.90 + ramp+readback
  3. Tomato detection fail → fixed extrinsics passthrough + hue-wrap
  4. Concurrent D415 crash → auto-pause camera reader
  5. Settle tolerance too tight → relaxed + downstream residual guard
  6. Strawberry centroid off → inscribed disk + calyx bias
- **Right — Improvements from A1/A2 feedback:**
  - Quantitative analysis (RMS, settling, success rates)
  - Quintic replaces cubic
  - E-STOP with actual arm halt
  - Voice control
  - 60+ regression tests
  - Live camera during picks

### Slide 8 — Conclusion & Future Work

- **Header:** "Conclusion & Future Work"
- **Left — Conclusion:** 14 fruits sorted, 2 modes, voice control, theory→practice
- **Center — Theory-Practice table:**
  - IK: Spong & Vidyasagar → 4-solution solver
  - Trajectory: Craig Ch.7 → quintic splines
  - Visual servoing: Corke eye-in-hand → D415 solvePnP + homography
  - FSM: Stateflow theory → 13-state controller
- **Right — Future work:** full parallax projection, ML detection, soft gripper, multi-arm
- **Bottom:** Placeholder for video/GIF of autonomous cycle

## 4. Technical Implementation

- Script: `slides/generate_final_pptx.py`
- Output: `slides/FinalPresentation_FruitSorting.pptx`
- Dependencies: `python-pptx` (already used for A3)
- Reuse A3 helpers: `add_bg`, `add_textbox`, `add_rect`, `add_placeholder_box`, `header_bar`, `footer_bar`
- All placeholder boxes use `[INSERT: ...]` labels for manual screenshot/video insertion
- Slide dimensions: 13.333 × 7.5 inches (16:9)
- Fonts: Calibri (body), Consolas (code/equations)
- Footer: team names + slide N/8

## 5. Image Assets

Available in repo for auto-insertion (if file exists, insert; otherwise placeholder):
- `figures/trajectory_3D.png`
- `figures/ee_position_time.png`
- `figures/joint_angles_time.png`
- `figures/gripper_state.png`
- `figures/top_view.png`
- `figures/live_color.png`
- `figures/live_depth.png`
- `1Capture.PNG` / `Capture.PNG` (root — likely lab photos)

All other images: placeholder boxes for manual insertion.
