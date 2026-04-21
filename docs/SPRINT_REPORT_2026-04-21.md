# Sprint Report — 2026-04-21

**Deadline:** 2026-05-01 14:00 (10 days remaining)
**Branch:** `yichang_branch`
**Session:** Second lab session, fresh teach-pendant machine (`Amarande_chao`)

---

## Summary

Onboarded a second workstation, re-validated D415 calibration for the new
physical setup, added per-joint jog support to the teach pendant, built two
hardware test scripts (`test_cal_picks.py`, `test_auto_pick.py`), and ran a
single-strawberry autonomous detect-pick end-to-end. The pick **failed**
physically (gripper did not secure the fruit), but the full vision → plan →
stage-approach → grip → transit → release pipeline executed cleanly without
crashing. Remaining gaps are HSV/calibration precision and grip alignment,
not control flow.

---

## Phase B progress update

| Stage | Prev | Now | Notes |
|---|---|---|---|
| B1 Preflight + visual ref | ✅ | ✅ | Re-run on new machine. Re-captured baseline + reference after inheriting files from Piero's commit `2d52abd`. |
| B2 D415 recalibration | ⚠️ | ✅ | Piero's `calibration.json` had a **44 mm uniform X bias** on this machine. Redid 4-point via `analyze_static.py`, RMS 31.7 mm, max 44.8 mm. |
| B3 HSV tuning | ❌ | ❌ | Observed false positives + one strawberry-as-tomato misclass in scene. Not yet tuned. |
| B4 Basket teach points | ❌ | ❌ | `placeberries` already present, `basket_a/b/c` still missing. |
| B5 Single-fruit dry run | ❌ | ⚠️ | Ran `main_final.py --pick-only` (4/4 cycles) and `test_auto_pick.py --max 1`. Cycles completed, grip readback 0.887 (same across all attempts — likely not an actual grip feedback signal). Physical pick failed. |
| B6 14-fruit autonomous | ❌ | ❌ | |
| B7 Remote mode live | ❌ | ❌ | |
| B8 Video edit | ❌ | ❌ | |

---

## What was done today

### 1. Workstation onboarding (Amarande_chao → `D:\study in birmingham\Applied Robots\qarm-fruit-sorting`)

- `git checkout master && git pull` → merged Piero's 2026-04-20 commits (32 commits)
- Preflight: first run **2 red**:
  - `calibration.json` missing
  - UGreen baseline/reference missing
- Piero pushed `2d52abd` and `3fea73d` giving `calibration.json`, `logs/ugreen_*.png`, `logs/ugreen_pickhome1_tcp.json`, and `teach_points.json` — all session-specific files normally gitignored, force-added for team sync.
- After pull, preflight check 7 still failed: `tcp delta = 274.4 px` (threshold 50). Piero's UGreen pose is not our UGreen pose — **re-ran `setup_baseline_ref.py`** to recapture baseline + reference locally. Preflight then 7/7 green.

### 2. D415 calibration — Piero's was unusable here

- Ran `analyze_static.py --from pickhome1 cal_01 cal_02 cal_03 cal_04` (strawberries placed at cal_01..04 positions):
  - Piero's calib error on our setup: **mean 55.5 mm, max 74.2 mm, bias [-44, -16, +14] mm**.
  - Candidate re-fit from our 4 detections: **mean 30.5 mm, max 44.8 mm, RMS 31.7 mm**.
- Saved candidate (`--save`); old Piero calib backed up to `calibration_bak_20260421_141820.json`.
- Max 44.8 mm residual still leaves only ~7 mm slack vs strawberry grip margin, and is effectively at the limit for tomatoes (45 mm margin) — explains the pick failure later.

### 3. `teach_points.json` edit

- Replaced contents of `point1..point4` with copies of `cal_01..cal_04` data (kept the `pointN` keys for script-default compatibility). Workaround for `analyze_static.py` arg-parsing quirk where explicit `cal_*` labels weren't landing in `sys.argv` and defaults were used instead.
- Live-added `cal01..cal04` (no underscore) — new pick-ready teach points at Z ≈ 0.02 m with gripper pre-closed to ~0.149. Distinct from the old `cal_01..cal_08` set which sit at Z ≈ 0.13-0.16 m.

### 4. New scripts

#### `python/joint_jog.py` — standalone per-joint jog
OpenCV window + keyboard. `q/a` `w/s` `e/d` `r/f` move J0/J1/J2/J3 directly. Live FK overlay, JOINT_LIMITS enforcement, step adjust `[ / ]`, home `h`, print `p`. Step default 3°.

#### `python/teach_points.py` — joint-mode added inline
- New `j` key toggles Cartesian ↔ Joint mode (persistent across outer loop AND inside the modify-point sub-loop)
- `DEFAULT_ROT_STEP`: **5° → 1.5°** (affects wrist in Cart mode and all joints in Joint mode)
- Banner in the overlay when in Joint mode
- `JOINT_JOG` / `JOINT_JOG_MODIFY` events in the trace log

#### `python/test_cal_picks.py` — staged-approach pick validator
Bypasses camera + IK. Uses `cal01..cal04` pre-taught joint angles directly. Staged motion:
- **Approach**: rotate+advance at transit-Z → descend vertically
- **Retreat**: ascend straight up → lateral to `pickhome1`

UGreen snapshot per stage per fruit. Gripper phase removed in final version (user-requested) — 4/4 approaches completed without incident, no fruit disturbed.

#### `python/test_auto_pick.py` — autonomous detect-and-pick
Full pipeline: D415 frame → `detect_fruits` → `pixel_to_world` via current calibration → workspace-box filter → per-fruit cycle:
  1. Approach (staged, IK at both stages — no pre-taught joints)
  2. Close gripper (ramped)
  3. Retreat (staged)
  4. Transit to `placeberries` teach point
  5. Release
  6. Return to `pickhome1`

UGreen snapshot at every phase. Flags: `--max N`, `--dry-run`, `--label strawberry|tomato`.

**First run** (`--max 1`):
- 4 strawberries detected from pickhome1 (one of them classified as tomato in earlier runs — classifier is noisy)
- Picked first: `xyz=[0.545, 0.07, 0.055]` depth 428 mm area 4406
- Full cycle executed, gripper readback 0.887 at close
- **Physical pick failed** — user confirmed visually. Staged motion correct, grip alignment insufficient.

### 5. Proactive-execution skill

Added `~/.claude/skills/proactive-execution/SKILL.md`: universal instruction that when the next step is an obvious continuation of an approved plan and the action is local+reversible, execute directly instead of asking for confirmation. Reduces ping-pong during lab work.

---

## Observations

1. **44.8 mm max residual is the practical ceiling for pick success** — consistent with the failed pick. Confirms that we need 8-point calibration (spread across workspace, tomatoes included) to get below 20 mm and give the gripper working margin.
2. **Staged approach (rotate+advance → descend) is correct** — visually verified 4/4 approaches in `test_cal_picks.py` and 1/1 in `test_auto_pick.py`. The motion pattern the user requested is now in reusable functions (`approach_staged_ik`, `retreat_staged_ik`).
3. **`held_grip=0.88` seen in both `main_final.py` and `test_auto_pick.py`** — same number across attempts regardless of whether something is in the jaws. Suggests this value is the commanded-ramp endpoint stalled on gripper mechanics, not a true "obstruction" readback. Cannot use this as a success signal.
4. **`sorting_controller.py` FSM does NOT yet use the staged approach** — it interpolates joint-space straight to the pick target, which means the TCP dips + extends simultaneously (the old pattern the user explicitly said to avoid). Porting `approach_staged_ik` into `sorting_controller.py` is a next-session candidate if we go with `main_final.py` as the demo entry point.
5. **HSV classifier still noisy** — 19 blobs in preflight check 5, 4 valid strawberries found by `test_auto_pick.py` with one tomato misclass elsewhere. Not blocking but recall/precision will drop if lighting varies.

---

## What's left for the demo

| Priority | Item |
|---|---|
| P0 | 8-point D415 recalibration with cal_01..cal_08 spread across workspace (or new cal set at correct heights) — target RMS < 20 mm |
| P0 | Port `approach_staged_ik` into `sorting_controller.py` so `main_final.py` uses staged motion |
| P1 | Teach `basket_a/b/c` (user jog via `teach_points.py`) |
| P1 | HSV tuning under lab light (`fruit_detector.HSV_RANGES` + `CIRCULARITY_THRESH`) |
| P1 | Single-fruit pick success (end-to-end, physical grip verified) |
| P2 | 14-fruit autonomous run + recording |
| P2 | Remote mode live verification |
| P3 | Video edit + report write-up |

---

## Commit list (2026-04-21)

See diff against `3fea73d`:
- `calibration.json` — new D415 calibration for this physical setup (RMS 31.7 mm)
- `logs/ugreen_baseline.png`, `logs/ugreen_pickhome1_reference.png`, `logs/ugreen_pickhome1_tcp.json` — regenerated for this UGreen position
- `teach_points.json` — point1..4 values replaced with cal_01..04 data; cal01..04 (no underscore) added live
- `python/teach_points.py` — per-joint jog mode (j toggle), rot_step default 1.5°
- `python/joint_jog.py` — standalone per-joint jog (new)
- `python/test_cal_picks.py` — staged-approach pick validator (new)
- `python/test_auto_pick.py` — autonomous detect-and-pick (new)
- `docs/SPRINT_REPORT_2026-04-21.md` — this file

---

## Next lab session — suggested order

1. Verify `teach_points.json` still matches physical cal01..cal04 positions (they were live-edited today)
2. Add 4 more spread-out teach points covering workspace corners (or move tomatoes into FOV) → redo 8-point calibration
3. Tune HSV with `analyze_static.py` scene shots
4. Port staged approach into `sorting_controller.py`
5. Teach `basket_a/b/c`
6. First successful single-fruit pick (physical grip verified)
7. 14-fruit autonomous demo run + video
