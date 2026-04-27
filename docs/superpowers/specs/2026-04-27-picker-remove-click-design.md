# Picker viewer: remove click-to-pick, keep category batch

**Date**: 2026-04-27
**Branch**: vision
**Touches**: `python/picker_viewer.py`, `python/test_picker_viewer.py`, `python/main_final.py`

## Motivation

`picker_viewer.run_picker_loop` currently exposes two ways to trigger a pick:

1. **Click-to-pick** — left-click a fruit in the OpenCV window; the arm picks the single nearest detection.
2. **Category batch** — press `b` / `t` / `s`; the arm auto-picks every banana / tomato / strawberry on the table, re-capturing from survey1 between picks until none remain.

Operator wants to remove (1) entirely. Rationale: the batch path already returns to survey1 between picks (via `survey_capture.capture_fruits`) and stops when no fruits of the chosen type are visible — i.e. it already does what (1) was added to enable, with no manual aim required. Click is now redundant cognitive load.

## Scope

In:
- Delete the click handler, mouse callback, click state, and the click branch in the event loop in `picker_viewer.py`.
- Delete the now-dead `_nearest_detection` and `CLICK_MAX_R_PX` and the three tests that cover them in `test_picker_viewer.py`.
- Update HUD hint text and module docstring to drop the "click" affordance.
- Update `main_final.py` docstring to drop the "click-to-pick" phrasing.

Out:
- `b` / `t` / `s` / `r` / `ESC` keybindings and their behavior — unchanged.
- `_pick_category` retry / skip / 20-pick cap — unchanged.
- `capture_fruits` survey1 round-trip — unchanged (already does the survey1 + photo loop the operator described).
- Initial `_refresh()` on entry to the loop — unchanged (option B from brainstorming).
- `_pick_one`, `_filter_by_type`, `_annotate`, `_hud_text`, `_residual_color`, `_pick_category`, `_nearest_to_home`, `_pixel_key` — unchanged.

## Design

### `python/picker_viewer.py`

Module docstring (line 1) drops "click-to-pick":

```python
"""Operator-facing picker UI: category-batch autonomous loop."""
```

Delete:
- `CLICK_MAX_R_PX` constant (line 34).
- `_nearest_detection` function (lines 37-54).

`_hud_text` (line 95) hint string changes from:

```
mode: {mode}  |  click / b t s / r / ESC
```

to:

```
mode: {mode}  |  b=banana  t=tomato  s=strawberry  r=refresh  ESC=quit
```

In `run_picker_loop`:
- Drop `"click"` key from the `state` dict (still need `"abort"` for `_pick_category`'s callback).
- Delete `_on_mouse` and the `cv2.setMouseCallback(window, _on_mouse)` line.
- Delete the `if state["click"] is not None:` branch and convert the surrounding `if/elif/elif` chain so the `b/t/s` branch becomes the leading `if` (no `elif` chain orphan).

### `python/test_picker_viewer.py`

Remove from the test list and drop `_nearest_detection` from the `from picker_viewer import (...)` line at `test_picker_viewer.py:10-12`:
- `test_nearest_within_radius` (line 26)
- `test_nearest_outside_radius_returns_none` (line 36)
- `test_nearest_empty_list_returns_none` (line 44)

After removal, the import line becomes `from picker_viewer import (_filter_by_type, _annotate, _hud_text,)`.

### `python/main_final.py`

Docstring line 5: replace "click-to-pick / category-batch UI" with "category-batch picker UI".

## Behavior after change

Operator runs `py -3.13 python/main_final.py`. After preflight:

1. `run_picker_loop` opens the window and runs one `_refresh()` — arm moves to survey1, captures one frame, draws detections.
2. Operator presses `b`, `t`, or `s`.
3. `_pick_category` loops: re-capture from survey1 → filter by type → if any → pick nearest-to-home → repeat.
4. Loop exits when no fruits of that type are visible, after 20 picks, after 2 retries on the same target, or on ESC.
5. Window stays open; operator can press another category key, `r` to refresh, or `ESC` to quit.

Mouse clicks anywhere in the window do nothing.

## Risks

- `test_picker_viewer.py` import line must drop the deleted symbols, or the whole test file fails to load. Verify by running `py -3.13 python/test_picker_viewer.py` after edits.
- No production code outside `picker_viewer.py` imports `_nearest_detection` or `CLICK_MAX_R_PX` (verified via grep across `python/` and `docs/`).
- HUD width may grow with the more verbose hint string; the window is 1280 px wide and the line renders at ~0.6 scale, so it fits.

## Verification

```bash
py -3.13 python/test_picker_viewer.py        # remaining tests stay green
py -3.13 -c "from picker_viewer import run_picker_loop; print('OK')"
```

Manual smoke test (lab):
1. Place 2 tomatoes and 1 banana under the camera.
2. Run `main_final.py`.
3. Press `t` — arm should pick both tomatoes, returning to survey1 between each, then stop. Banana untouched.
4. Press `b` — arm picks the banana, returns to survey1, stops.
5. Press `ESC` — clean exit.
