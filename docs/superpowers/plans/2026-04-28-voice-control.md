# Voice Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add optional Vosk-based voice control to the fruit-sorting GUI — wake word "robot" + 8 commands, graceful degradation when deps are missing.

**Architecture:** A standalone `VoiceController` class in `python/voice_control.py` runs a daemon thread that reads mic audio via PyAudio, feeds chunks to a Vosk `KaldiRecognizer` with a restricted grammar, and dispatches recognized commands to GUI callbacks via `root.after(0, ...)`. Integration into `main_gui.py` is guarded by a try/except import so voice is entirely opt-in.

**Tech Stack:** Python 3.13, `vosk` 0.3.45, `pyaudio` 0.2.14, `winsound` (stdlib), Tkinter.

**Spec:** `docs/superpowers/specs/2026-04-28-voice-control-design.md`

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `python/voice_control.py` | Create | `VoiceController` class — mic capture, Vosk recognition, wake word state machine, command dispatch, status label management |
| `python/test_voice_control.py` | Create | Unit tests — mock PyAudio stream, verify wake word parsing, command dispatch, timeout, stop-without-wake-word, graceful failures |
| `python/main_gui.py` | Modify | Import guard, instantiate `VoiceController` with command map, add voice status label below video, cleanup in `_on_close` |
| `.gitignore` | Modify | Add `models/` to prevent committing the 40MB Vosk model |

---

### Task 1: Install dependencies and download Vosk model

**Files:**
- Modify: `.gitignore`

- [ ] **Step 1: Install vosk and pyaudio**

Run:
```bash
C:/Python313/python.exe -m pip install vosk pyaudio
```

Expected: both install successfully with pre-built wheels for cp313-win_amd64.

- [ ] **Step 2: Download and extract the Vosk model**

Run:
```bash
mkdir -p models
curl -L -o models/vosk-model-small-en-us-0.15.zip https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
cd models && python -c "import zipfile; zipfile.ZipFile('vosk-model-small-en-us-0.15.zip').extractall('.')" && rm vosk-model-small-en-us-0.15.zip
```

Expected: `models/vosk-model-small-en-us-0.15/` directory exists with model files inside.

- [ ] **Step 3: Add models/ to .gitignore**

Add this line to `.gitignore` under the `# Python` section:

```
# Voice control model (40MB binary, downloaded per-machine)
models/
```

- [ ] **Step 4: Verify imports work**

Run:
```bash
C:/Python313/python.exe -c "import vosk; import pyaudio; print('OK')"
```

Expected: prints `OK`.

- [ ] **Step 5: Commit**

```bash
git add .gitignore
git commit -m "chore: add models/ to gitignore for Vosk model"
```

---

### Task 2: Write the wake word parser tests

**Files:**
- Create: `python/test_voice_control.py`

These tests exercise the command-parsing logic without touching audio hardware or Vosk. The parser is a pure function we will extract from `VoiceController`.

- [ ] **Step 1: Create the test file with all parser tests**

Create `python/test_voice_control.py`:

```python
"""Tests for voice_control module — parser + dispatch logic."""
import os, sys, time, threading

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from voice_control import parse_command, GRAMMAR_WORDS


# ── Parser unit tests ─────────────────────────────────────────────

def test_wake_then_command():
    name = "wake_then_command"
    state = {"phase": "idle", "wake_time": 0.0}
    result = parse_command("robot", state, time.time())
    assert result is None, "wake word alone must not dispatch"
    assert state["phase"] == "awaiting"
    result = parse_command("strawberry", state, time.time())
    assert result == "strawberry"
    assert state["phase"] == "idle"
    return name, True, "robot + strawberry dispatched"


def test_command_without_wake_ignored():
    name = "command_without_wake_ignored"
    state = {"phase": "idle", "wake_time": 0.0}
    result = parse_command("banana", state, time.time())
    assert result is None
    assert state["phase"] == "idle"
    return name, True, "banana without wake word ignored"


def test_stop_bypasses_wake_word():
    name = "stop_bypasses_wake"
    state = {"phase": "idle", "wake_time": 0.0}
    result = parse_command("stop", state, time.time())
    assert result == "stop"
    assert state["phase"] == "idle"
    return name, True, "stop dispatched without wake word"


def test_stop_during_awaiting():
    name = "stop_during_awaiting"
    state = {"phase": "awaiting", "wake_time": time.time()}
    result = parse_command("stop", state, time.time())
    assert result == "stop"
    assert state["phase"] == "idle"
    return name, True, "stop in awaiting state dispatched immediately"


def test_timeout_returns_to_idle():
    name = "timeout_returns_to_idle"
    state = {"phase": "awaiting", "wake_time": time.time() - 3.0}
    result = parse_command("strawberry", state, time.time())
    assert result is None
    assert state["phase"] == "idle"
    return name, True, "expired awaiting rejects command"


def test_unk_during_awaiting_ignored():
    name = "unk_during_awaiting"
    state = {"phase": "awaiting", "wake_time": time.time()}
    result = parse_command("[unk]", state, time.time())
    assert result is None
    assert state["phase"] == "awaiting", "unk must not reset state"
    return name, True, "[unk] ignored, stays in awaiting"


def test_repeated_robot_resets_timer():
    name = "repeated_robot_resets"
    state = {"phase": "idle", "wake_time": 0.0}
    parse_command("robot", state, 100.0)
    assert state["wake_time"] == 100.0
    parse_command("robot", state, 105.0)
    assert state["wake_time"] == 105.0, "second robot must reset timer"
    return name, True, "repeated robot resets wake_time"


def test_all_commands_recognized():
    name = "all_commands_recognized"
    commands = ["strawberry", "banana", "tomato", "all",
                "home", "refresh", "survey"]
    for cmd in commands:
        state = {"phase": "awaiting", "wake_time": time.time()}
        result = parse_command(cmd, state, time.time())
        assert result == cmd, f"{cmd} not recognized"
    return name, True, f"all {len(commands)} commands pass"


def test_grammar_words_matches_spec():
    name = "grammar_words_spec"
    expected = {"robot", "strawberry", "banana", "tomato", "all",
                "stop", "home", "refresh", "survey", "[unk]"}
    assert set(GRAMMAR_WORDS) == expected, \
        f"mismatch: {set(GRAMMAR_WORDS) ^ expected}"
    return name, True, f"{len(GRAMMAR_WORDS)} words match spec"


# ── Test runner ───────────────────────────────────────────────────

TESTS = [
    test_wake_then_command,
    test_command_without_wake_ignored,
    test_stop_bypasses_wake_word,
    test_stop_during_awaiting,
    test_timeout_returns_to_idle,
    test_unk_during_awaiting_ignored,
    test_repeated_robot_resets_timer,
    test_all_commands_recognized,
    test_grammar_words_matches_spec,
]


def main():
    fails = 0
    for fn in TESTS:
        try:
            name, ok, msg = fn()
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {name}  {msg}")
            if not ok:
                fails += 1
        except Exception as ex:
            import traceback
            fails += 1
            print(f"  [FAIL] {fn.__name__}: {ex}")
            traceback.print_exc()
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed")
    return fails


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run tests to verify they fail (module not yet created)**

Run:
```bash
C:/Python313/python.exe python/test_voice_control.py
```

Expected: `ModuleNotFoundError: No module named 'voice_control'` — all tests fail because the module doesn't exist yet.

- [ ] **Step 3: Commit test file**

```bash
git add python/test_voice_control.py
git commit -m "test: add voice control parser tests (red — module not yet created)"
```

---

### Task 3: Implement the voice_control module

**Files:**
- Create: `python/voice_control.py`

- [ ] **Step 1: Create the module with parser, grammar, and VoiceController class**

Create `python/voice_control.py`:

```python
"""
voice_control.py — Optional Vosk-based voice command module.

Requires: pip install vosk pyaudio
Model:    models/vosk-model-small-en-us-0.15/  (downloaded once)

Usage from main_gui.py:
    try:
        from voice_control import VoiceController
        _HAS_VOICE = True
    except ImportError:
        _HAS_VOICE = False
"""
from __future__ import annotations

import json
import os
import threading
import time

import vosk
import pyaudio

SAMPLE_RATE = 16000
CHUNK_FRAMES = 4000          # ~250 ms at 16 kHz
WAKE_TIMEOUT_S = 2.0

GRAMMAR_WORDS = [
    "robot", "strawberry", "banana", "tomato", "all",
    "stop", "home", "refresh", "survey", "[unk]",
]

VALID_COMMANDS = {
    "strawberry", "banana", "tomato", "all",
    "stop", "home", "refresh", "survey",
}

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_MODEL = os.path.join(_REPO_ROOT, "models", "vosk-model-small-en-us-0.15")


def parse_command(word: str, state: dict, now: float) -> str | None:
    """Pure wake-word state machine.

    Parameters
    ----------
    word  : single recognised token (lowercased).
    state : mutable dict with keys "phase" ("idle"|"awaiting") and
            "wake_time" (float epoch).
    now   : current time (time.time()).

    Returns the command string to dispatch, or None.
    """
    if word == "stop":
        state["phase"] = "idle"
        return "stop"

    if word == "[unk]":
        return None

    if word == "robot":
        state["phase"] = "awaiting"
        state["wake_time"] = now
        return None

    if state["phase"] == "awaiting":
        if now - state["wake_time"] > WAKE_TIMEOUT_S:
            state["phase"] = "idle"
            return None
        if word in VALID_COMMANDS:
            state["phase"] = "idle"
            return word

    return None


class VoiceController:
    """Threaded voice command listener for the fruit-sorting GUI.

    Parameters
    ----------
    root        : tk.Tk instance (for root.after scheduling).
    command_map : dict mapping command strings to zero-arg callables.
    model_path  : path to the extracted Vosk model directory.
    status_label: optional tk.Label whose text/fg are updated to reflect
                  voice state. Pass None to skip visual feedback.
    """

    def __init__(self, root, command_map: dict,
                 model_path: str = _DEFAULT_MODEL,
                 status_label=None):
        if not os.path.isdir(model_path):
            raise FileNotFoundError(
                f"Vosk model not found at {model_path}")

        self._root = root
        self._commands = dict(command_map)
        self._label = status_label
        self._stop_event = threading.Event()

        vosk.SetLogLevel(-1)
        model = vosk.Model(model_path)
        grammar = json.dumps(GRAMMAR_WORDS)
        self._recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE, grammar)

        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK_FRAMES,
        )

        self._state = {"phase": "idle", "wake_time": 0.0}
        self._update_status("\U0001f3a4 Voice", "grey")

        self._thread = threading.Thread(
            target=self._listen_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        try:
            self._stream.close()
        except Exception:
            pass
        try:
            self._pa.terminate()
        except Exception:
            pass

    def _listen_loop(self):
        was_idle = True
        while not self._stop_event.is_set():
            try:
                data = self._stream.read(
                    CHUNK_FRAMES, exception_on_overflow=False)
            except Exception:
                self._update_status("", "grey")
                break

            if self._recognizer.AcceptWaveform(data):
                text = json.loads(self._recognizer.Result()).get("text", "")
            else:
                text = json.loads(
                    self._recognizer.PartialResult()).get("partial", "")

            if not text:
                continue

            now = time.time()
            for word in text.strip().split():
                cmd = parse_command(word.lower(), self._state, now)
                if cmd is not None:
                    self._dispatch(cmd)

            if self._state["phase"] == "awaiting":
                if now - self._state["wake_time"] > WAKE_TIMEOUT_S:
                    self._state["phase"] = "idle"
                    self._update_status("\U0001f3a4 ???", "#cc0000")
                    self._root.after(1000, lambda: self._update_status(
                        "\U0001f3a4 Voice", "grey"))
                    was_idle = True
                elif was_idle:
                    self._beep()
                    self._update_status("\U0001f3a4 Listening...", "#cc8800")
                    was_idle = False
            else:
                was_idle = True

    def _beep(self):
        try:
            import winsound
            winsound.PlaySound("SystemAsterisk",
                               winsound.SND_ALIAS | winsound.SND_ASYNC)
        except Exception:
            pass

    def _dispatch(self, cmd: str):
        callback = self._commands.get(cmd)
        if callback is None:
            return

        self._update_status(f'\U0001f3a4 "{cmd}" ✓', "#008800")
        self._root.after(0, callback)
        self._root.after(1500, lambda: self._update_status(
            "\U0001f3a4 Voice", "grey"))

    def _update_status(self, text: str, fg: str):
        if self._label is None:
            return
        self._root.after(0, lambda: (
            self._label.configure(text=text, foreground=fg)))
```

- [ ] **Step 2: Run the parser tests to verify they pass**

Run:
```bash
C:/Python313/python.exe python/test_voice_control.py
```

Expected: `9/9 passed`.

- [ ] **Step 3: Commit**

```bash
git add python/voice_control.py
git commit -m "feat: add voice_control module (Vosk + PyAudio, wake word parser)"
```

---

### Task 4: Integrate VoiceController into main_gui.py

**Files:**
- Modify: `python/main_gui.py:38` (imports)
- Modify: `python/main_gui.py:161-171` (init — after `_build_ui`, before `root.after`)
- Modify: `python/main_gui.py:206-222` (`_build_ui` — add label after video_label)
- Modify: `python/main_gui.py:660-670` (`_on_close` — stop voice before destroy)

- [ ] **Step 1: Add the import guard after line 71**

After the existing import block (line 71: `from picker_viewer import _pick_category`), add:

```python
try:
    from voice_control import VoiceController
    _HAS_VOICE = True
except ImportError:
    _HAS_VOICE = False
```

- [ ] **Step 2: Add the voice status label in `_build_ui`**

After `self.video_label.pack()` (line 222), add:

```python
        self.voice_label = tk.Label(left, text="", font=("Arial", 10),
                                    bg="black", anchor="w")
        self.voice_label.pack(fill="x")
```

- [ ] **Step 3: Instantiate VoiceController in `__init__`**

After the camera thread start block (after line 168: `self._camera_thread.start()`), before the `root.after` line (line 171), add:

```python
        # --- voice control (optional) ---
        if _HAS_VOICE:
            command_map = {
                "strawberry": lambda: self._auto_one("strawberry"),
                "banana":     lambda: self._auto_one("banana"),
                "tomato":     lambda: self._auto_one("tomato"),
                "all":        self._auto_all,
                "stop":       self._emergency_stop,
                "home":       self._goto_survey1,
                "refresh":    self._refresh_detect,
                "survey":     self._goto_survey1,
            }
            try:
                self.voice = VoiceController(
                    self.root, command_map,
                    status_label=self.voice_label)
            except Exception as ex:
                print(f"voice control disabled: {ex}")
                self.voice = None
        else:
            self.voice = None
```

- [ ] **Step 4: Add voice cleanup in `_on_close`**

In `_on_close` (line 660), add voice cleanup after `self._uninstall_hooks()` (line 663) and before `try: self.trace.close()` (line 664):

```python
        if self.voice:
            try: self.voice.stop()
            except Exception: pass
```

- [ ] **Step 5: Verify the GUI still starts without voice deps**

Temporarily rename `voice_control.py` to test graceful degradation:

```bash
mv python/voice_control.py python/voice_control.py.bak
C:/Python313/python.exe python/main_gui.py
```

Expected: GUI starts normally, no crash, `_HAS_VOICE` is `False`. (The GUI will fail at QArm connect if not on the lab PC — that's expected. What matters is no ImportError crash.)

Then restore:
```bash
mv python/voice_control.py.bak python/voice_control.py
```

- [ ] **Step 6: Commit**

```bash
git add python/main_gui.py
git commit -m "feat: integrate voice control into GUI (optional, graceful degradation)"
```

---

### Task 5: Add dispatch integration tests

**Files:**
- Modify: `python/test_voice_control.py` (append new tests)

- [ ] **Step 1: Add dispatch and VoiceController integration tests**

Append to `python/test_voice_control.py`, before the `TESTS` list:

```python
# ── Dispatch + VoiceController integration tests ─────────────────

def test_dispatch_calls_callback():
    name = "dispatch_calls_callback"
    called = {"cmd": None}

    class FakeRoot:
        def after(self, ms, fn):
            fn()

    from voice_control import VoiceController

    # Build a minimal controller with a mock root and no real audio.
    # We test _dispatch directly, bypassing __init__ (which needs mic).
    vc = object.__new__(VoiceController)
    vc._root = FakeRoot()
    vc._label = None
    vc._commands = {
        "strawberry": lambda: called.__setitem__("cmd", "strawberry"),
    }
    vc._state = {"phase": "idle", "wake_time": 0.0}

    vc._dispatch("strawberry")
    assert called["cmd"] == "strawberry"
    return name, True, "callback invoked via _dispatch"


def test_dispatch_ignores_unknown_command():
    name = "dispatch_ignores_unknown"

    class FakeRoot:
        def after(self, ms, fn):
            fn()

    from voice_control import VoiceController
    vc = object.__new__(VoiceController)
    vc._root = FakeRoot()
    vc._label = None
    vc._commands = {}
    vc._state = {"phase": "idle", "wake_time": 0.0}

    vc._dispatch("nonexistent")  # must not raise
    return name, True, "unknown command silently ignored"


def test_update_status_with_label():
    name = "update_status_with_label"
    updates = []

    class FakeRoot:
        def after(self, ms, fn):
            fn()

    class FakeLabel:
        def configure(self, **kw):
            updates.append(kw)

    from voice_control import VoiceController
    vc = object.__new__(VoiceController)
    vc._root = FakeRoot()
    vc._label = FakeLabel()
    vc._commands = {}

    vc._update_status("\U0001f3a4 Voice", "grey")
    assert len(updates) == 1
    assert updates[0]["text"] == "\U0001f3a4 Voice"
    assert updates[0]["foreground"] == "grey"
    return name, True, "label updated with text + color"


def test_update_status_without_label():
    name = "update_status_no_label"

    class FakeRoot:
        def after(self, ms, fn):
            fn()

    from voice_control import VoiceController
    vc = object.__new__(VoiceController)
    vc._root = FakeRoot()
    vc._label = None
    vc._commands = {}

    vc._update_status("test", "red")  # must not raise
    return name, True, "no-op when label is None"
```

Then update the `TESTS` list to include the new tests:

```python
TESTS = [
    test_wake_then_command,
    test_command_without_wake_ignored,
    test_stop_bypasses_wake_word,
    test_stop_during_awaiting,
    test_timeout_returns_to_idle,
    test_unk_during_awaiting_ignored,
    test_repeated_robot_resets_timer,
    test_all_commands_recognized,
    test_grammar_words_matches_spec,
    test_dispatch_calls_callback,
    test_dispatch_ignores_unknown_command,
    test_update_status_with_label,
    test_update_status_without_label,
]
```

- [ ] **Step 2: Run all tests**

Run:
```bash
C:/Python313/python.exe python/test_voice_control.py
```

Expected: `13/13 passed`.

- [ ] **Step 3: Commit**

```bash
git add python/test_voice_control.py
git commit -m "test: add dispatch + status label integration tests for voice control"
```

---

### Task 6: Manual verification with microphone

This task is for lab/desktop testing with an actual microphone. Skip if no mic is available.

- [ ] **Step 1: Verify Vosk model loads and mic captures**

Run:
```bash
C:/Python313/python.exe -c "
import vosk, pyaudio, json, os
model = vosk.Model('models/vosk-model-small-en-us-0.15')
rec = vosk.KaldiRecognizer(model, 16000, json.dumps(['robot','stop','[unk]']))
pa = pyaudio.PyAudio()
stream = pa.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
print('Say something (3 seconds)...')
for _ in range(20):
    data = stream.read(4000, exception_on_overflow=False)
    if rec.AcceptWaveform(data):
        print('Final:', json.loads(rec.Result())['text'])
    else:
        partial = json.loads(rec.PartialResult())['partial']
        if partial: print('Partial:', partial)
stream.close(); pa.terminate()
print('Done')
"
```

Expected: partial/final results appear as you speak. "robot" and "stop" are recognised; other words appear as `[unk]` or empty.

- [ ] **Step 2: Test full GUI with voice**

Run `main_gui.py` on a machine with the QArm connected (or observe that it fails at QArm connect but the voice thread started before the crash):

```bash
C:/Python313/python.exe python/main_gui.py
```

Test sequence:
1. Say "robot" — label changes to "Listening...", beep plays
2. Say "strawberry" within 2s — label shows `"strawberry" ✓`, auto pick triggers
3. Say "stop" (no wake word) — emergency stop fires immediately
4. Say random words — `[unk]`, no action
5. Say "robot" then wait 3s — label shows `???`, returns to idle

- [ ] **Step 3: Final commit if any adjustments were needed**

```bash
git add -u
git commit -m "fix: voice control adjustments from manual testing"
```
