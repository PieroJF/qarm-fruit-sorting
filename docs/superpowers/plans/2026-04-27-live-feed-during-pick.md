# Live D415 Feed During Pick — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the OpenCV picker window updating with the live D415 feed while the arm is moving during a pick, instead of freezing the last detection frame.

**Architecture:** A daemon thread reads the camera at ~30 fps and stores the latest color frame under a lock (`_LiveFeed`). `FruitSortingController` exposes an optional `tick_observer` callable invoked once per FSM iteration. `picker_viewer._pick_one` registers a render observer that copies the latest frame, draws a status overlay, and `cv2.imshow`/`waitKey(1)` from the main thread. Mouse clicks during pick are gated off because the live feed shows arm-mounted POV and the cached `dets` no longer match the live pixels.

**Tech Stack:** Python 3.13, OpenCV 4.x (`cv2`), NumPy, Quanser SDK `Video3D` (existing `QArmCamera`), `threading` from stdlib. Tests follow the existing module pattern (`def test_*` returning `(name, ok, detail)`, `TESTS` list at bottom, `main()` runner).

**File Structure:**
- Modify `python/sorting_controller.py` — `FruitSortingController.__init__` adds `self.tick_observer = None`; `_drive_until_done` invokes the observer with try/except per FSM tick (single responsibility: arm control + optional side-effect hook).
- Modify `python/picker_viewer.py` — adds `_LiveFeed` class, `_make_render_observer` factory, `_make_mouse_callback` factory; updates `_pick_one`, `_pick_category`, `run_picker_loop` to wire camera+window through and to gate clicks via `state["picking"]`.
- Modify `python/test_pick_single.py` — adds 1 test for the controller hook.
- Modify `python/test_picker_viewer.py` — adds 4 tests for the viewer-side helpers and wiring.

No new modules. No external dependencies. No changes to `camera.py`, `survey_capture.py`, `session_cal.py`, `fruit_detector.py`, or anything in `matlab_facade/`.

**Branch:** `Patches` (continue work; no new worktree).

---

## Task 1: Add `tick_observer` hook to `FruitSortingController`

**Files:**
- Modify: `python/sorting_controller.py:91-113` (`__init__`) and `python/sorting_controller.py:161-184` (`_drive_until_done`).
- Test: `python/test_pick_single.py` — add new test before `TESTS = [...]`.

- [ ] **Step 1: Write the failing test**

Append before `TESTS = [...]` in `python/test_pick_single.py`:

```python
def test_pick_single_invokes_tick_observer():
    name = "pick_single_invokes_tick_observer"
    q = MockQArm()
    c = _compressed(FruitSortingController(q, pick_only=True))
    counter = {"n": 0}
    c.tick_observer = lambda: counter.__setitem__("n", counter["n"] + 1)
    ok = c.pick_single(np.array([0.35, 0.10, 0.13]), "strawberry", dt=0.001)
    assert ok, "pick_single returned False"
    assert counter["n"] >= 5, f"observer not called enough: n={counter['n']}"
    return name, True, f"observer fired {counter['n']} times"


def test_tick_observer_exception_does_not_crash_fsm():
    name = "tick_observer_exception_isolated"
    q = MockQArm()
    c = _compressed(FruitSortingController(q, pick_only=True))
    def _bad():
        raise RuntimeError("display blew up")
    c.tick_observer = _bad
    ok = c.pick_single(np.array([0.35, 0.10, 0.13]), "strawberry", dt=0.001)
    assert ok, "pick_single should still complete despite observer raising"
    assert c.sorted_count == 1
    return name, True, "observer error swallowed; FSM completed"
```

Update the `TESTS = [...]` list to include the two new functions:

```python
TESTS = [test_pick_single_reachable_returns_true,
         test_pick_single_unreachable_returns_false,
         test_pick_single_twice_accumulates,
         test_pick_single_invokes_tick_observer,
         test_tick_observer_exception_does_not_crash_fsm]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `C:/Python313/python.exe python/test_pick_single.py`
Expected: 3/5 pass, 2 new tests FAIL (`AttributeError: 'FruitSortingController' object has no attribute 'tick_observer'` for the first; the second may also fail or may falsely pass — both will pass once Step 3 is done).

- [ ] **Step 3: Add the hook to the controller**

In `python/sorting_controller.py`, in `__init__` (around line 105 where `_traj_target_joints` is set), add:

```python
        self._driving = False            # reentrancy guard for _drive_until_done
        # Optional zero-arg callable invoked once per FSM tick from
        # _drive_until_done. Used by picker_viewer to render the live D415
        # feed during arm motion. Errors are caught + printed so display
        # issues never crash arm control.
        self.tick_observer = None
```

(Insert the comment + line right after `self._driving = False`. Do not duplicate `self._driving`.)

Then in `_drive_until_done`, replace the inner `while self.state != State.DONE: ...` block to call the observer:

```python
    def _drive_until_done(self, dt):
        """Tight control loop. Pumps _step at `dt` cadence until state==DONE
        or an exception is caught. Shared between run_autonomous and
        pick_single. Not reentrant — guarded by `self._driving`."""
        if self._driving:
            raise RuntimeError(
                "FSM already being driven; run_autonomous / pick_single / "
                "set_gripper_ramp must not overlap")
        self._driving = True
        try:
            while self.state != State.DONE:
                loop_start = time.time()
                try:
                    self._step(time.time())
                except Exception as ex:
                    print(f"[ERROR] state {self.state.name}: {ex}")
                    print("[ERROR] aborting — arm held at last commanded pose")
                    break
                if self.tick_observer is not None:
                    try:
                        self.tick_observer()
                    except Exception as obs_ex:
                        print(f"[warn] tick_observer raised: {obs_ex}")
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            self._driving = False
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `C:/Python313/python.exe python/test_pick_single.py`
Expected: `5/5 passed`.

Run also `C:/Python313/python.exe python/test_integration.py` to confirm no regression.
Expected: `11/11 passed`.

- [ ] **Step 5: Commit**

```bash
git add python/sorting_controller.py python/test_pick_single.py
git commit -m "controller: optional tick_observer hook in _drive_until_done"
```

---

## Task 2: Add `_LiveFeed` class to `picker_viewer`

**Files:**
- Modify: `python/picker_viewer.py:9-15` (imports) and below the imports (new class).
- Test: `python/test_picker_viewer.py` — add new test before `TESTS = [...]`.

- [ ] **Step 1: Write the failing test**

Append before `TESTS = [...]` in `python/test_picker_viewer.py`:

```python
def test_live_feed_populates_latest_then_stops():
    name = "live_feed_populates_latest_then_stops"

    class _StubCam:
        def __init__(self):
            self.calls = 0
        def read(self):
            self.calls += 1
            return (np.full((720, 1280, 3), self.calls % 256, dtype=np.uint8),
                    np.zeros((720, 1280), dtype=np.uint16))

    from picker_viewer import _LiveFeed
    cam = _StubCam()
    feed = _LiveFeed(cam)
    feed.start()
    deadline = time.time() + 0.5
    frame = None
    while time.time() < deadline:
        frame = feed.latest()
        if frame is not None:
            break
        time.sleep(0.01)
    feed.stop()
    assert frame is not None, "latest() returned None within 500 ms"
    assert frame.shape == (720, 1280, 3), f"shape = {frame.shape}"
    assert feed._thread is None, "stop() did not clear thread"
    return name, True, f"latest shape={frame.shape}, cam.calls={cam.calls}"
```

Add `import time` and `import numpy as np` at the top of `test_picker_viewer.py` if not already present (check first; the existing file already uses numpy via the other tests).

Update `TESTS = [...]` to include the new function (preserve order; append at the end).

- [ ] **Step 2: Run test to verify it fails**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `ImportError: cannot import name '_LiveFeed' from 'picker_viewer'`.

- [ ] **Step 3: Implement `_LiveFeed`**

In `python/picker_viewer.py`, change the imports block at the top to add `threading` and `time`:

```python
from __future__ import annotations
import os
import sys
import threading
import time
import traceback

import numpy as np
import cv2
```

Then immediately after the existing constants block (right before `# ----- Pick dispatchers -----`, around line 100 in the current file), insert the new class:

```python
# ------------------------------------------------------------------------
# Live camera feed (used during arm motion to keep the picker window from
# freezing). The feed is the SOLE reader of QArmCamera while it runs
# because camera.read() shares internal buffers (camera.py:50-55, 168-174)
# and is not thread-safe. Callers MUST stop()+join the feed before any
# other code touches the camera.
# ------------------------------------------------------------------------

class _LiveFeed:
    def __init__(self, camera):
        self._cam = camera
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    def latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def _loop(self):
        while not self._stop.is_set():
            try:
                color, _ = self._cam.read()
                with self._lock:
                    self._latest = color
            except Exception:
                # Camera blip: keep last frame, retry next iteration.
                pass
```

- [ ] **Step 4: Run test to verify it passes**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `9/9 passed`.

- [ ] **Step 5: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "picker: _LiveFeed background camera reader (daemon + lock + Event)"
```

---

## Task 3: Add `_make_mouse_callback` factory (gates clicks during pick)

**Files:**
- Modify: `python/picker_viewer.py` — add new function near other module-level helpers.
- Test: `python/test_picker_viewer.py` — add new test.

- [ ] **Step 1: Write the failing test**

Append before `TESTS = [...]` in `python/test_picker_viewer.py`:

```python
def test_mouse_callback_drops_clicks_while_picking():
    name = "mouse_callback_drops_clicks_while_picking"
    from picker_viewer import _make_mouse_callback
    state = {"click": None, "picking": False}
    cb = _make_mouse_callback(state)
    cb(cv2.EVENT_LBUTTONDOWN, 100, 100, 0, None)
    assert state["click"] == (100, 100), "click should register when not picking"
    state["click"] = None
    state["picking"] = True
    cb(cv2.EVENT_LBUTTONDOWN, 200, 200, 0, None)
    assert state["click"] is None, "click should be dropped while picking"
    state["picking"] = False
    cb(cv2.EVENT_LBUTTONDOWN, 300, 300, 0, None)
    assert state["click"] == (300, 300), "clicks should resume after picking ends"
    return name, True, "click gated by picking flag"
```

Add the function name to `TESTS = [...]`.

- [ ] **Step 2: Run test to verify it fails**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `ImportError: cannot import name '_make_mouse_callback' from 'picker_viewer'`.

- [ ] **Step 3: Implement `_make_mouse_callback`**

In `python/picker_viewer.py`, add this function below the existing `_hud_text` function (around line 95, before the Pick dispatchers section):

```python
def _make_mouse_callback(state):
    """Build the OpenCV mouse callback. Drops left-clicks while
    state.get('picking') is True so clicks during arm motion don't
    match against stale `dets` (camera moves with the arm; the cached
    detection pixel coords no longer correspond to live image pixels)."""
    def _on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and not state.get("picking", False):
            state["click"] = (x, y)
    return _on_mouse
```

- [ ] **Step 4: Run test to verify it passes**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `10/10 passed`.

- [ ] **Step 5: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "picker: _make_mouse_callback factory gates clicks during pick"
```

---

## Task 4: Add `_make_render_observer` factory (throttled live render)

**Files:**
- Modify: `python/picker_viewer.py` — add new function below `_LiveFeed`.
- Test: `python/test_picker_viewer.py` — add new test.

- [ ] **Step 1: Write the failing test**

Append before `TESTS = [...]` in `python/test_picker_viewer.py`:

```python
def test_render_observer_throttles_and_overlays_state():
    name = "render_observer_throttles_and_overlays_state"

    captured = {"imshow": [], "waitKey": 0}
    orig_imshow = cv2.imshow
    orig_waitKey = cv2.waitKey
    cv2.imshow = lambda w, f: captured["imshow"].append((w, f.shape))
    cv2.waitKey = lambda d: (captured.__setitem__("waitKey",
                                                  captured["waitKey"] + 1)
                             or 0)
    try:
        from picker_viewer import _make_render_observer

        class _Feed:
            def latest(self):
                return np.zeros((720, 1280, 3), dtype=np.uint8)

        class _Ctrl:
            class state:
                name = "DESCEND"
            current_target = {"type": "tomato",
                              "pos": np.array([0.4, 0.1, 0.05])}

        observer = _make_render_observer(
            _Feed(), "win", _Ctrl(), fps_limit=30.0)
        observer()                  # first call: renders
        observer()                  # immediate: throttled
        time.sleep(0.05)            # > 1/30 s
        observer()                  # third call: renders again
        assert len(captured["imshow"]) == 2, \
            f"expected 2 renders, got {len(captured['imshow'])}"
        assert captured["imshow"][0][0] == "win"
        assert captured["imshow"][0][1] == (720, 1280, 3)
        assert captured["waitKey"] == 2
    finally:
        cv2.imshow = orig_imshow
        cv2.waitKey = orig_waitKey
    return name, True, f"{len(captured['imshow'])} renders, "\
                        f"{captured['waitKey']} waitKeys"


def test_render_observer_handles_none_target():
    name = "render_observer_handles_none_target"
    orig_imshow = cv2.imshow
    orig_waitKey = cv2.waitKey
    cv2.imshow = lambda w, f: None
    cv2.waitKey = lambda d: 0
    try:
        from picker_viewer import _make_render_observer

        class _Feed:
            def latest(self):
                return np.zeros((720, 1280, 3), dtype=np.uint8)

        class _Ctrl:
            class state:
                name = "GO_HOME"
            current_target = None

        observer = _make_render_observer(_Feed(), "win", _Ctrl())
        observer()  # must not raise
    finally:
        cv2.imshow = orig_imshow
        cv2.waitKey = orig_waitKey
    return name, True, "no exception with current_target=None"
```

Add both function names to `TESTS = [...]`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `ImportError: cannot import name '_make_render_observer' from 'picker_viewer'`.

- [ ] **Step 3: Implement `_make_render_observer`**

In `python/picker_viewer.py`, immediately below the `_LiveFeed` class, insert:

```python
def _make_render_observer(feed, window, controller, fps_limit=30.0):
    """Closure suitable for FruitSortingController.tick_observer.

    Reads the latest frame from `feed`, draws a status overlay derived
    from controller.state.name and controller.current_target, and shows
    it in `window`. Throttled to fps_limit so the FSM tick budget is
    not consumed by GUI work (cv2.imshow + waitKey costs ~1-3 ms; at
    100 Hz dt that would eat 10-30% of the loop)."""
    last = {"t": 0.0}
    period = 1.0 / float(fps_limit)

    def _render():
        now = time.time()
        if now - last["t"] < period:
            return
        last["t"] = now
        frame = feed.latest()
        if frame is None:
            return
        out = frame.copy()
        try:
            sname = controller.state.name
        except Exception:
            sname = "?"
        target = getattr(controller, "current_target", None)
        if target is not None:
            ftype = target.get("type", "?") if isinstance(target, dict) \
                else "?"
            pos = target.get("pos") if isinstance(target, dict) else None
            if pos is not None and len(pos) >= 3:
                label = (f"PICKING {ftype} @ "
                         f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
                         f"state: {sname}")
            else:
                label = f"PICKING {ftype} | state: {sname}"
        else:
            label = f"state: {sname}"
        cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                     0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(window, out)
        cv2.waitKey(1)

    return _render
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `12/12 passed`.

- [ ] **Step 5: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "picker: _make_render_observer with status overlay + 30 fps throttle"
```

---

## Task 5: Wire camera + window into `_pick_one`

**Files:**
- Modify: `python/picker_viewer.py:108-118` (`_pick_one`).
- Test: `python/test_picker_viewer.py` — add new test.

- [ ] **Step 1: Write the failing test**

Append before `TESTS = [...]` in `python/test_picker_viewer.py`:

```python
def test_pick_one_starts_and_clears_observer_when_camera_provided():
    name = "pick_one_lifecycle_with_camera"

    class _StubCam:
        def read(self):
            return (np.zeros((720, 1280, 3), dtype=np.uint8),
                    np.zeros((720, 1280), dtype=np.uint16))

    class _StubCtrl:
        def __init__(self):
            self.tick_observer = None
            self.observer_during_pick = "UNSET"
            self.calls = []
            class _State: name = "GO_HOME"
            self.state = _State()
            self.current_target = None

        def pick_single(self, xyz, ftype):
            self.observer_during_pick = self.tick_observer
            self.calls.append((tuple(xyz), ftype))
            return True

    from picker_viewer import _pick_one
    from fruit_detector import Detection

    det = Detection(fruit_type="tomato", center_px=(640, 360),
                     center_base_m=np.array([0.4, 0.0, 0.05]),
                     confidence=0.8, area_px=1000, bbox=(620, 340, 40, 40))
    ctrl = _StubCtrl()
    cam = _StubCam()
    ok = _pick_one(ctrl, det, camera=cam, window="win")
    assert ok, "stub pick_single returned True so _pick_one should too"
    assert ctrl.observer_during_pick is not None, \
        "tick_observer was not set during pick_single"
    assert ctrl.tick_observer is None, \
        "tick_observer not restored after pick_single returned"
    assert ctrl.calls == [((0.4, 0.0, 0.05), "tomato")]
    return name, True, "observer set during pick, cleared after"
```

Add the function name to `TESTS`. Note: the stub controller's `pick_single` does NOT invoke the observer, so no `cv2` calls happen in this test — no monkeypatch needed.

- [ ] **Step 2: Run test to verify it fails**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `TypeError: _pick_one() got an unexpected keyword argument 'camera'`.

- [ ] **Step 3: Update `_pick_one`**

Replace the existing `_pick_one` (after the recent Patches branch updates, it lives around line 108-119) with:

```python
def _pick_one(controller, detection, camera=None, window=None) -> bool:
    """Dispatch one synchronous pick. Returns True on success.

    If both `camera` and `window` are given, runs a live D415 feed
    overlay during the arm motion via controller.tick_observer.
    The feed is the sole reader of `camera` for its lifetime; on
    return (success or exception) the feed is stopped + joined and
    the previous tick_observer is restored.
    """
    print(f"  [picker] picking {detection.fruit_type} at "
          f"{detection.center_base_m.round(3)} "
          f"(conf={detection.confidence:.2f})")
    feed = None
    prev_observer = controller.tick_observer
    try:
        if camera is not None and window is not None:
            feed = _LiveFeed(camera)
            feed.start()
            controller.tick_observer = _make_render_observer(
                feed, window, controller)
        return bool(controller.pick_single(
            detection.center_base_m, detection.fruit_type))
    except Exception as ex:
        print(f"  [picker] pick_single raised: {ex}")
        traceback.print_exc()
        return False
    finally:
        controller.tick_observer = prev_observer
        if feed is not None:
            feed.stop()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `13/13 passed` (the existing `pick_one_dispatch` test still passes — it calls `_pick_one(ctrl, det)` with no camera/window, which keeps the old behavior).

- [ ] **Step 5: Commit**

```bash
git add python/picker_viewer.py python/test_picker_viewer.py
git commit -m "picker: _pick_one optional live feed via camera+window args"
```

---

## Task 6: Wire `_pick_category` and `run_picker_loop`

**Files:**
- Modify: `python/picker_viewer.py:136-174` (`_pick_category`) and `python/picker_viewer.py:181-259` (`run_picker_loop`).

No new tests in this task. Existing 13 tests stay green because:
- The new params on `_pick_category` are optional (`window=None` default).
- The new `state["picking"]` key has a `state.get("picking", False)` default in the callback.

- [ ] **Step 1: Update `_pick_category` signature and pass through**

In `python/picker_viewer.py`, change the signature of `_pick_category`:

```python
def _pick_category(driver, camera, session_cal, controller,
                     fruit_type, should_abort_fn, window=None,
                     picking_state=None):
```

Inside the loop body, find the line that calls `_pick_one`:

```python
        success = _pick_one(controller, target)
```

and replace it with:

```python
        if picking_state is not None:
            picking_state["picking"] = True
        try:
            success = _pick_one(controller, target,
                                  camera=camera, window=window)
        finally:
            if picking_state is not None:
                picking_state["picking"] = False
```

(`picking_state` is the same `state` dict from `run_picker_loop`. Threading it explicitly avoids hidden globals; default `None` keeps unit tests of `_pick_category` free to call without the state machinery.)

- [ ] **Step 2: Update `run_picker_loop`**

Locate `state = {"click": None, "abort": False}` (around line 196) and change it to:

```python
    state = {"click": None, "abort": False, "picking": False}
```

Replace `cv2.setMouseCallback(window, _on_mouse)` (line 202) and the inline `_on_mouse` definition above it with:

```python
    cv2.setMouseCallback(window, _make_mouse_callback(state))
```

(The inline `_on_mouse` function defined inside `run_picker_loop` becomes redundant — delete those four lines.)

In the click-handling branch of the main loop (around line 224-234), wrap the `_pick_one` call:

```python
        if state["click"] is not None:
            click = state["click"]; state["click"] = None
            target = _nearest_detection(dets, click)
            if target is None:
                print(f"  [picker] no fruit near {click}")
            else:
                state["picking"] = True
                try:
                    _pick_one(controller, target,
                                camera=camera, window=window)
                finally:
                    state["picking"] = False
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
```

In the category-batch branch (around line 235-251), update the `_pick_category` call to pass the new args:

```python
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
                                    lambda: state["abort"],
                                    window=window,
                                    picking_state=state)
                print(f"  [picker] {ftype} batch done: {n} picked")
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
```

- [ ] **Step 3: Run all picker tests to verify no regression**

Run: `C:/Python313/python.exe python/test_picker_viewer.py`
Expected: `13/13 passed`.

Run: `C:/Python313/python.exe python/test_pick_single.py`
Expected: `5/5 passed`.

Run: `C:/Python313/python.exe python/test_integration.py`
Expected: `11/11 passed`.

- [ ] **Step 4: Commit**

```bash
git add python/picker_viewer.py
git commit -m "picker: wire camera+window through pick_category + picking-flag gate"
```

---

## Task 7: Final verification + manual lab checklist in PROGRESS.md

**Files:**
- Modify: `PROGRESS.md` — append a manual verification section to the existing "Pending after `Patches` branch (2026-04-27)" block.

- [ ] **Step 1: Run the full test sweep**

Run each in turn:

```bash
C:/Python313/python.exe python/test_integration.py
C:/Python313/python.exe python/test_pick_single.py
C:/Python313/python.exe python/test_picker_viewer.py
C:/Python313/python.exe python/test_fruit_detector.py
C:/Python313/python.exe python/test_survey_capture.py
C:/Python313/python.exe python/test_session_cal.py
C:/Python313/python.exe python/test_calibrate_chessboard.py
C:/Python313/python.exe python/test_calibrate_chessboard_orientation.py
C:/Python313/python.exe python/test_calibrate_extrinsics.py
```

Expected total: `11/11 + 5/5 + 13/13` minimum from the suites we touched, plus the rest stay green. If any non-touched suite goes red, stop and investigate.

- [ ] **Step 2: Document manual lab verification in PROGRESS.md**

In `PROGRESS.md`, inside the existing "## 0. Pending after `Patches` branch (2026-04-27)" section, under **Hardware-blocked**, add the following bullets at the end of that bulleted list:

```markdown
- Live D415 feed during pick (Patches: 2026-04-27): with arm + D415 + `main_final.py` running, click a fruit and confirm (a) the picker window keeps refreshing the live POV during the ~10 s arm motion, (b) the status overlay shows the FSM state transitioning GO_HOME → APPROACH → DESCEND → CLOSE_GRIPPER → ASCEND_PICK, (c) clicks during pick are ignored (no second pick queued), (d) after pick completes the window snaps back to the detection-overlay frame from `_refresh()`, (e) repeated picks don't leak threads — process should not accumulate camera handles. Failure modes to watch for: black frames (camera read race), Windows "Not Responding" on the OpenCV window (waitKey not pumping), arm pause >100 ms beyond expected (observer cost too high — would need to lower fps_limit).
```

- [ ] **Step 3: Commit**

```bash
git add PROGRESS.md
git commit -m "progress: lab-verification checklist for live feed during pick"
```

- [ ] **Step 4: Push the branch**

```bash
git push origin Patches
```

Expected: 7 new commits pushed to `origin/Patches`. No master push, no PR yet.

---

## Self-Review

**Spec coverage** (against the auditoría hallazgos A-G):

- A (camera not thread-safe): Task 2 documents `_LiveFeed` as sole reader; Tasks 3 + 5 + 6 ensure feed is stopped+joined before `capture_fruits` runs again. ✓
- B (race after `_pick_one` returns): Task 2 `stop()` calls `thread.join(timeout=2.0)` before clearing; Task 5 finally-block runs that. ✓
- C (clicks during pick): Task 3 `_make_mouse_callback` gate; Task 6 sets `state["picking"]` around `_pick_one` and `_pick_category`. ✓
- D (observer raise kills FSM): Task 1 wraps observer call in try/except. ✓
- E (current_target None): Task 4 `_make_render_observer` null-checks `current_target` and `pos`. ✓
- F (throttle): Task 4 `fps_limit=30.0` parameter with monotonic-time gate. ✓
- G (signature changes): Task 5 (`_pick_one` optional kwargs) + Task 6 (`_pick_category` optional kwargs). Existing tests preserve old behavior because both new params default to `None`. ✓

**Placeholder scan:** No "TBD", "implement later", "similar to", or unsubstituted ellipses. Every code block contains the actual code that ships.

**Type/name consistency:**
- `tick_observer` (Tasks 1, 5, 6): always the attribute on `FruitSortingController`, always set/cleared by saving `prev_observer` then restoring.
- `_LiveFeed.latest()` returns `np.ndarray | None`; `_make_render_observer` checks `frame is None` before using.
- `state` dict keys: `"click"`, `"abort"` (existing), `"picking"` (new). Mouse callback uses `.get("picking", False)` so new key need not pre-exist for unit tests.
- `picking_state` parameter on `_pick_category` is the same `state` dict; passed only from `run_picker_loop`. Default `None` preserves any other caller (none today, but defensive).
- `_pick_one(controller, detection, camera=None, window=None)` and `_pick_category(driver, camera, session_cal, controller, fruit_type, should_abort_fn, window=None, picking_state=None)` are the only signature changes; both backwards-compatible.

No issues found in self-review.
