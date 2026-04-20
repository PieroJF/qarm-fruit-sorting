# FruitSorting Lab Runbook

Session flow for using the QArm lab bench productively. Written 2026-04-20.

## Prerequisites on dev machine

- `C:/Python313/python.exe python/preflight.py --offline` is green.
- `git status` is clean (no uncommitted work before heading to the lab).

## On arrival at the lab

1. Power the QArm (red switch, wait ~10 s for LEDs to stabilise).
2. Plug D415 USB3 into the laptop.
3. Position the UGreen camera at ground level, lens toward the workspace.
   Do not move it again this session.
4. Wait 10 s, then run preflight (full):

       C:/Python313/python.exe python/preflight.py

5. If QArm check fails with -843, power-cycle the QArm base and retry.
6. If UGreen check fails, run `python/probe_ugreen.py` to re-discover
   the index.

## Closed-loop calibration (once per session)

1. Move the arm clear of UGreen view: open `teach_points.py`, press `g`,
   select `homeplace0`, ESC to save. (The arm now parks behind the
   bench.)
2. Capture baseline:

       C:/Python313/python.exe -c "from ugreen_tracker import capture, save_baseline; import os; save_baseline(capture(), os.path.join('logs', 'ugreen_baseline.png'))"

3. Put strawberries at cal_01..cal_04 and tomatoes at cal_05..cal_08.
4. Run the calibration:

       C:/Python313/python.exe python/calibrate_closed_loop.py

5. Check the reported RMS. Target < 15 mm; anything < 20 mm is usable.
   The script now emits three residuals: pixel, base-plane, and max-base.
   Gate: if max-base > 50 mm the file is written to
   `calibration_ugreen.rejected.json` instead of the real path.

## UGreen intrinsic calibration (one-time per physical setup)

Do this only on the first session, or if the UGreen camera's lens /
position changes.

1. Confirm `figures/chessboard_7x5_30mm.png` is printed at 100% scale
   (one square measures 30 mm with a ruler).
2. Collect 20 chessboard poses:

       C:/Python313/python.exe python/ugreen_intrinsics.py collect

   Tilt, translate, rotate the pattern so the inner corners cover the
   whole UGreen frame. Type `q` at any prompt to abort early.
3. Solve:

       C:/Python313/python.exe python/ugreen_intrinsics.py solve

   Target: reprojection RMS < 1.0 px. Written to `ugreen_intrinsics.json`.
4. `git add ugreen_intrinsics.json logs/ugreen_chessboards && git commit`

## Visual reference snapshot (once per session)

1. With baseline + fruits in place, arm at pickhome1:

       C:/Python313/python.exe python/preflight.py

   Step 7 will fail the first time because no reference exists.
2. Create the reference:

       C:/Python313/python.exe -c "from ugreen_tracker import capture, tcp_from_diff, load_baseline; import json, cv2; f = capture(); t = tcp_from_diff(f, load_baseline('logs/ugreen_baseline.png')); cv2.imwrite('logs/ugreen_pickhome1_reference.png', f); json.dump({'tcp': list(t)}, open('logs/ugreen_pickhome1_tcp.json', 'w'))"

3. Re-run preflight — all 7 green.

## Pick + place verification

1. Confirm basket teach points exist (`basket_a`, `basket_b`, `basket_c`).
   If not, enter `teach_points.py`, jog to each basket pose, press `n`
   to save with the matching label.
2. Dry run with one strawberry at cal_01:

       C:/Python313/python.exe python/main_final.py --pick-only

3. Confirm UGreen image-diff shows the fruit in the gripper after pick.
4. Full sort of 14 fruits:

       C:/Python313/python.exe python/main_final.py

## Demo video recording

Have OBS (or Snipping Tool) recording the UGreen companion window + the
Simulink `.slx` side by side. Target: one 14-fruit autonomous run, then
one ~60-s remote-mode manual pick. Cut to 3–5 min in post.

## Shutdown

1. `ctrl+c` in the terminal running `main_final`. Arm holds at last pose.
2. Close Simulink models (Save changes if prompted, otherwise discard).
3. Power down QArm (red switch).
4. Unplug D415 and UGreen.
5. `git status` — commit any working changes.
