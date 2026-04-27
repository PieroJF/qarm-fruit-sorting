# Picker Remove Click — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove click-to-pick from `picker_viewer.run_picker_loop` so the only way to trigger a pick is the existing `b`/`t`/`s` category-batch keys, which already return to survey1 between picks and stop when no fruits of the chosen type remain.

**Architecture:** Pure deletion + one HUD string rewrite. No new abstractions, no behavioral additions. The category-batch path (`_pick_category` → `capture_fruits` → `pick_single`) is untouched; we strip the click branch that sat in parallel with it.

**Tech Stack:** Python 3.13, OpenCV (`cv2.waitKey`), the project's hand-rolled test runner in `python/test_picker_viewer.py` (no pytest).

**Spec:** `docs/superpowers/specs/2026-04-27-picker-remove-click-design.md`

---

## File Structure

| File | Change |
| --- | --- |
| `python/picker_viewer.py` | Modify — drop click constant, helper, mouse callback, click state, and click branch in event loop. Rewrite HUD hint. |
| `python/test_picker_viewer.py` | Modify — drop `_nearest_detection` from import, delete its 3 tests, drop them from the `TESTS` list. |
| `python/main_final.py` | Modify — one-line docstring rewording. |

No files created. No files split or moved. No production code outside `picker_viewer.py` imports the symbols being deleted (verified via grep across `python/`).

---

## Task 1: Slim picker_viewer.py + drop dead tests

**Files:**
- Modify: `python/picker_viewer.py`
- Modify: `python/test_picker_viewer.py`

- [ ] **Step 1: Update `python/test_picker_viewer.py` — drop import + tests + TESTS entries**

Edit the import block (lines 10-12). Replace:

```python
from picker_viewer import (
    _nearest_detection, _filter_by_type, _annotate, _hud_text,
)
```

with:

```python
from picker_viewer import (
    _filter_by_type, _annotate, _hud_text,
)
```

Delete the three test functions in their entirety (lines 26-47):

```python
def test_nearest_within_radius():
    name = "nearest_within"
    dets = [_mk_det("tomato", 100, 100),
            _mk_det("banana", 300, 300)]
    got = _nearest_detection(dets, (110, 95), max_r_px=50)
    assert got is not None
    assert got.fruit_type == "tomato"
    return name, True, "picks the closer one"


def test_nearest_outside_radius_returns_none():
    name = "nearest_outside"
    dets = [_mk_det("tomato", 100, 100)]
    got = _nearest_detection(dets, (500, 500), max_r_px=50)
    assert got is None
    return name, True, "out-of-range click -> None"


def test_nearest_empty_list_returns_none():
    name = "nearest_empty"
    assert _nearest_detection([], (100, 100), max_r_px=50) is None
    return name, True, "empty detection list -> None"
```

After deletion, `def test_filter_by_type():` should follow directly after `def _mk_det(...)` (with the standard two blank lines).

Update the `TESTS` list (lines 145-151). Replace:

```python
TESTS = [
    test_nearest_within_radius, test_nearest_outside_radius_returns_none,
    test_nearest_empty_list_returns_none, test_filter_by_type,
    test_annotate_returns_image_same_shape, test_hud_text_contains_counts,
    test_pick_one_calls_controller,
    test_pick_category_skips_stuck_target,
]
```

with:

```python
TESTS = [
    test_filter_by_type,
    test_annotate_returns_image_same_shape, test_hud_text_contains_counts,
    test_pick_one_calls_controller,
    test_pick_category_skips_stuck_target,
]
```

- [ ] **Step 2: Run tests now — they must still pass against the unchanged picker_viewer.py**

Run:

```bash
py -3.13 python/test_picker_viewer.py
```

Expected: `5/5 passed`. (`_nearest_detection` still exists in `picker_viewer.py` but nothing references it; the remaining 5 tests cover code we are not touching.) If anything fails, the deletion in Step 1 mis-snipped the file — re-read it and fix before moving on.

- [ ] **Step 3a: Update `picker_viewer.py` module docstring**

Edit `python/picker_viewer.py:1-8`. Replace:

```python
"""
Operator-facing picker UI: click-to-pick + category-batch autonomous loop.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.5.

Event loop lives in run_picker_loop(). Everything above it is pure and
unit-tested.
"""
```

with:

```python
"""
Operator-facing picker UI: category-batch autonomous loop.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.5.

Event loop lives in run_picker_loop(). Everything above it is pure and
unit-tested.
"""
```

- [ ] **Step 3b: Delete `CLICK_MAX_R_PX` and `_nearest_detection`**

Delete lines 34-54 of `picker_viewer.py` in their entirety:

```python
CLICK_MAX_R_PX = 50


def _nearest_detection(detections, click_xy, max_r_px=CLICK_MAX_R_PX):
    """Return the detection with smallest pixel distance to click_xy,
    within max_r_px. None if no detection qualifies."""
    if not detections:
        return None
    cx, cy = float(click_xy[0]), float(click_xy[1])
    best = None
    best_d2 = (max_r_px + 1) ** 2
    for d in detections:
        dx = d.center_px[0] - cx
        dy = d.center_px[1] - cy
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best = d
    if best_d2 > max_r_px * max_r_px:
        return None
    return best
```

After deletion, the previous `_HUD_COLOR = (255, 255, 255)` block should be followed (after a single blank line) by `def _filter_by_type(detections, fruit_type):`.

- [ ] **Step 3c: Update HUD hint string in `_hud_text`**

Edit `picker_viewer.py:95-98`. Replace:

```python
def _hud_text(n_fruits, residual_mm, mode):
    r = f"{residual_mm:.1f}" if residual_mm is not None else "n/a"
    return (f"{n_fruits} fruits  |  residual {r} mm  |  "
            f"mode: {mode}  |  click / b t s / r / ESC")
```

with:

```python
def _hud_text(n_fruits, residual_mm, mode):
    r = f"{residual_mm:.1f}" if residual_mm is not None else "n/a"
    return (f"{n_fruits} fruits  |  residual {r} mm  |  "
            f"mode: {mode}  |  b=banana  t=tomato  s=strawberry  "
            f"r=refresh  ESC=quit")
```

- [ ] **Step 3d: Simplify state dict + remove mouse callback**

Edit `picker_viewer.py:196-202` (inside `run_picker_loop`). Replace:

```python
    state = {"click": None, "abort": False}

    def _on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    cv2.setMouseCallback(window, _on_mouse)
```

with:

```python
    state = {"abort": False}
```

Note: `state["abort"]` must stay — `_pick_category` receives it as `lambda: state["abort"]` (line 246).

- [ ] **Step 3e: Remove click branch and promote `b/t/s` to leading `if` in event loop**

Edit `picker_viewer.py:219-257` (the `while True:` block). Replace:

```python
    while True:
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            print("  [picker] ESC — exiting")
            break
        if state["click"] is not None:
            click = state["click"]; state["click"] = None
            target = _nearest_detection(dets, click)
            if target is None:
                print(f"  [picker] no fruit near {click}")
            else:
                _pick_one(controller, target)
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key in (ord('b'), ord('t'), ord('s')):
            ftype = {ord('b'): 'banana',
                       ord('t'): 'tomato',
                       ord('s'): 'strawberry'}[key]
            matches = _filter_by_type(dets, ftype)
            if not matches:
                print(f"  [picker] no {ftype} visible")
            else:
                print(f"  [picker] category batch: {ftype}")
                n = _pick_category(driver, camera, session_cal,
                                    controller, ftype,
                                    lambda: state["abort"])
                print(f"  [picker] {ftype} batch done: {n} picked")
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key == ord('r'):
            print("  [picker] refresh")
            try:
                dets, diag = _refresh()
            except Exception as ex:
                print(f"  [picker] re-capture failed: {ex}")
```

with:

```python
    while True:
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            print("  [picker] ESC — exiting")
            break
        if key in (ord('b'), ord('t'), ord('s')):
            ftype = {ord('b'): 'banana',
                       ord('t'): 'tomato',
                       ord('s'): 'strawberry'}[key]
            matches = _filter_by_type(dets, ftype)
            if not matches:
                print(f"  [picker] no {ftype} visible")
            else:
                print(f"  [picker] category batch: {ftype}")
                n = _pick_category(driver, camera, session_cal,
                                    controller, ftype,
                                    lambda: state["abort"])
                print(f"  [picker] {ftype} batch done: {n} picked")
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key == ord('r'):
            print("  [picker] refresh")
            try:
                dets, diag = _refresh()
            except Exception as ex:
                print(f"  [picker] re-capture failed: {ex}")
```

The two changes from the original: (a) the `if state["click"] is not None:` block is gone, (b) the `b/t/s` branch is now the leading `if` (was `elif`).

- [ ] **Step 4: Run tests again — must still pass**

Run:

```bash
py -3.13 python/test_picker_viewer.py
```

Expected: `5/5 passed`. If `ImportError: cannot import name '_filter_by_type'` etc., Step 3b deleted too much — re-read `picker_viewer.py` and confirm `_filter_by_type`, `_annotate`, `_hud_text`, `_pick_one`, `_pick_category` all still exist.

- [ ] **Step 5: Smoke import + grep for stale references**

Run:

```bash
py -3.13 -c "from picker_viewer import run_picker_loop; print('OK')"
```

Expected: `OK`.

Then verify nothing else references the deleted symbols:

```bash
git grep -n "_nearest_detection\|CLICK_MAX_R_PX" -- python/ docs/
```

Expected: zero hits in `python/`. Hits in `docs/superpowers/plans/2026-04-23-d3-picker-viewer-wiring.md` are historical — leave them alone.

- [ ] **Step 6: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "$(cat <<'EOF'
feat(picker): remove click-to-pick, category batch is the only path

Click was redundant: the b/t/s batch path already returns to survey1
between picks and stops when none of the chosen type remain, which
is what the operator described wanting. Drop the mouse callback,
the click state, the _nearest_detection helper, and the click
branch in the event loop. HUD hint rewritten to spell out keys.
EOF
)"
```

---

## Task 2: Update main_final.py docstring

**Files:**
- Modify: `python/main_final.py:5`

- [ ] **Step 1: Edit the docstring line**

Read `python/main_final.py:1-10` to confirm the wording, then replace the substring in line 5:

`"picker_viewer.run_picker_loop for the click-to-pick / category-batch UI."`

with:

`"picker_viewer.run_picker_loop for the category-batch picker UI."`

- [ ] **Step 2: Smoke import**

Run:

```bash
py -3.13 -c "import main_final" 2>&1 | head -5
```

Expected: no output, or whatever the existing import side effects produce. Definitely no `SyntaxError`.

(If `main_final.py` has top-level argparse / driver-connect side effects that fail without hardware, switch to `py -3.13 -m py_compile python/main_final.py` instead — expected: no output.)

- [ ] **Step 3: Commit**

```bash
git add python/main_final.py
git commit -m "docs(main_final): drop 'click-to-pick' from module docstring"
```

---

## Self-review notes (for the implementer)

Spec coverage check against `docs/superpowers/specs/2026-04-27-picker-remove-click-design.md`:

| Spec item | Task / Step |
| --- | --- |
| Drop `_on_mouse`, `setMouseCallback`, `state["click"]`, click branch | Task 1, Steps 3d + 3e |
| HUD text rewrite | Task 1, Step 3c |
| Module docstring drops "click-to-pick" | Task 1, Step 3a |
| Delete `_nearest_detection`, `CLICK_MAX_R_PX` | Task 1, Step 3b |
| Delete the 3 test cases + import symbol + TESTS entries | Task 1, Step 1 |
| `main_final.py` docstring | Task 2, Step 1 |
| Behaviour on entry: keep initial `_refresh()` (option B) | Untouched — `_refresh()` call at the top of `run_picker_loop` is outside any edited line range |
| `_pick_category` retry / 20-cap / ESC abort unchanged | Untouched |

No placeholders. No "TBD", no "implement later", no "similar to Task N". All code blocks are complete.
