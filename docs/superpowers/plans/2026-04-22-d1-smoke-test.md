# D1 Chessboard Calibration — Hardware Smoke Test

Runs at the start of the D4 lab session (2026-04-25 AM).
Depends on: D1 code merged; D4 `survey1` taught into teach_points.json.

## Setup
1. Power QArm, plug D415.
2. Print `figures/chessboard_7x5_30mm.png` at 100% (ruler check: each
   square = 30 mm).
3. Tape chessboard flat on the table in the far corner of the workspace.

## Teach `survey1` (D4 only)
1. `py -3.13 python/teach_points.py`
2. Jog the arm so the D415 frames both:
   - the whole intended fruit-placement area (roughly 0.4 x 0.3 m), AND
   - all 24 inner chessboard corners visible in one frame.
3. Press `n`, label `survey1`, save, ESC.
4. `git commit teach_points.json -m "lab: survey1 pose taught"`.

## Run calibration
1. `py -3.13 python/calibrate_chessboard.py`
2. Phase 1 prompt: jog gripper tip to the top-left INNER corner of the
   chessboard (not the outer paper edge). Press ENTER.
3. Phase 2: arm moves to survey1, captures frame.
4. Expect output:
   - `chessboard corners found: 24`
   - `reprojection RMS: < 2.0 px`
   - `camera height above table: ~0.4-0.8 m`
5. `session_cal.json` should now exist in repo root.
6. Sanity: open `session_cal.json` and confirm
   `chess_origin_in_base_m[2]` (table z) is within ±10 mm of the known
   table height.

## Acceptance
- RMS < 2 px.
- Camera height within 20% of measured tape-to-lens distance.
- `session_cal.json` loads via `SessionCal.load(...)` from a Python REPL
  with no error.

## Failures and what they mean
| Symptom | Most likely cause | Fix |
|---|---|---|
| "findChessboardCorners failed" | obscured corner, poor lighting, or chessboard not fully in frame | reposition chessboard, re-teach survey1 if FOV inadequate |
| RMS > 2 px | chessboard not flat, or blurred frame | flatten with tape, reduce FPS or lengthen warm-up |
| Camera height off by > 50% | survey pose too tilted | reteach survey1 with more nadir-looking angle |
