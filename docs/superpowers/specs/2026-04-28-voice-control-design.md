# Voice Control for Fruit Sorting GUI

_Date: 2026-04-28. Author: Claude + PieroJF._

## 1. Goal

Add optional voice control to `main_gui.py` so the operator can command the QArm fruit-sorting system hands-free. Voice acts as an alternative input — keyboard and GUI buttons remain fully functional. If voice dependencies are missing or the microphone is unavailable, the system degrades gracefully with zero impact.

## 2. Scope

- **In scope**: wake-word activation, 8 voice commands, visual feedback label, audible beep on wake word, graceful degradation.
- **Out of scope**: speech synthesis / text-to-speech responses, custom wake word training, multi-language support, teleop directional commands (up/down/left/right).

## 3. Technology Choice — Vosk with Restricted Grammar

**Vosk** (offline, lightweight) with a JSON grammar that constrains recognition to ~10 tokens. This eliminates hallucinated words and gives near-perfect accuracy on the small vocabulary.

Alternatives considered and rejected:
- **SpeechRecognition + Google API**: requires internet, higher latency (1-2s), no grammar restriction.
- **Whisper local**: heavier model (75-150MB), no native grammar restriction, higher CPU load risks interfering with the 500 Hz QArm control loop.

## 4. Voice Command Set

Wake word: **"robot"**. After the wake word is detected, the system listens for a command within a 2-second window. If no valid command follows, it returns to idle.

| Voice command | GUI action | Callback |
|---|---|---|
| "strawberry" | Pick one strawberry | `_auto_one("strawberry")` |
| "banana" | Pick one banana | `_auto_one("banana")` |
| "tomato" | Pick one tomato | `_auto_one("tomato")` |
| "all" | Pick all fruits | `_auto_all()` |
| "stop" | Emergency stop | `_emergency_stop()` |
| "home" | Go to home/survey pose | `_goto_survey1()` |
| "refresh" | Re-capture fruit detection | `_refresh_detect()` |
| "survey" | Go to survey pose | `_goto_survey1()` |

**Safety exception**: "stop" is accepted **without** the wake word at any time. In an emergency the operator should not need to say "robot" first.

Vosk grammar definition:

```json
["robot", "strawberry", "banana", "tomato", "all", "stop", "home", "refresh", "survey", "[unk]"]
```

`[unk]` absorbs any out-of-vocabulary word and is ignored by the parser.

## 5. Architecture

### 5.1 New Module: `python/voice_control.py`

Single class `VoiceController`:

```
VoiceController(root, command_map, model_path="models/vosk-model-small-en-us")
├── __init__()        — load Vosk model, open PyAudio stream, start daemon thread
├── start()           — begin listening (called from __init__)
├── stop()            — set stop_event, close stream, terminate PyAudio
├── _listen_loop()    — daemon thread: read audio chunks, feed recognizer, parse
├── _parse_command()  — wake word + command window state machine
├── _dispatch()       — root.after(0, callback) to invoke GUI action on main thread
└── _update_status()  — root.after(0, ...) to update visual indicator
```

### 5.2 Integration with `main_gui.py`

```python
try:
    from voice_control import VoiceController
    _HAS_VOICE = True
except ImportError:
    _HAS_VOICE = False
```

In `__init__`, after GUI setup:

```python
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
        self.voice = VoiceController(self.root, command_map)
    except Exception:
        self.voice = None
else:
    self.voice = None
```

In `_on_close`:

```python
if self.voice:
    self.voice.stop()
```

### 5.3 Threading Model

```
GUI main thread                 Voice daemon thread
─────────────                   ───────────────────
FruitSortingGUI.__init__()
  └─ VoiceController(root, cmd_map)
       └─ _listen_loop (daemon=True)
                                  open pyaudio stream (16kHz, mono, int16)
                                  while not stop_event:
                                    read 4000 frames (~250ms)
                                    feed to KaldiRecognizer
                                    on partial/final result:
                                      _parse_command(text)
                                      if valid → root.after(0, callback)

_on_close()
  └─ voice.stop()
       └─ stop_event.set(), stream.close(), pa.terminate()
```

- Daemon thread dies automatically on process exit.
- GUI callbacks run on the main thread via `root.after(0, ...)` — thread-safe.
- `_busy_guard` serialization applies: if the robot is mid-action, the voice command is rejected the same way a duplicate button click would be.

### 5.4 Wake Word State Machine

```
         ┌─────────┐   "robot"    ┌───────────┐  valid cmd   ┌──────────┐
    ───► │  IDLE   │ ──────────► │ AWAITING  │ ──────────►  │ DISPATCH │
         │         │              │ (2s timer) │              │          │
         └────┬────┘              └─────┬─────┘              └──────────┘
              │                         │ timeout / [unk]           │
              │                         ▼                          │
              │                    back to IDLE                    │
              ◄────────────────────────────────────────────────────┘
              
Exception: "stop" in any state → DISPATCH immediately
```

## 6. Visual Feedback

A `tk.Label` in the GUI toolbar area:

| State | Text | Color |
|---|---|---|
| Voice not available | _(label hidden)_ | — |
| Idle | `🎤 Voice` | Grey |
| Wake word heard | `🎤 Listening...` | Yellow |
| Command recognized | `🎤 "strawberry" ✓` | Green (1.5s → idle) |
| Unrecognized input | `🎤 ???` | Red (1s → idle) |

All label updates go through `root.after(0, ...)` from the voice thread.

Audible beep on wake word detection: `winsound.Beep(1000, 100)` — confirms "robot" was heard when the operator is not looking at the screen.

## 7. Dependencies

| Package | Purpose | Install |
|---|---|---|
| `vosk` | Offline speech recognition | `pip install vosk` |
| `pyaudio` | Microphone capture | `pip install pyaudio` |

Model: `vosk-model-small-en-us` (~40MB), placed at `models/vosk-model-small-en-us/`. Downloaded once manually or via a setup script.

Both packages are **optional** — not added to a mandatory requirements file. Voice is opt-in.

## 8. Graceful Degradation

The following failure modes are handled silently:

| Failure | Behavior |
|---|---|
| `vosk` or `pyaudio` not installed | `_HAS_VOICE = False`, no voice label shown |
| No microphone available | `VoiceController.__init__` raises, caught in GUI → `self.voice = None` |
| Model directory missing | Same as above — caught, voice disabled |
| Mic disconnected mid-session | `_listen_loop` catches `IOError`, sets stop_event, updates label to hidden |
| Recognition returns garbage | `[unk]` tokens ignored, no action taken |

In all cases the GUI continues operating normally via keyboard and buttons.

## 9. Testing Strategy

- **Unit tests** (`test_voice_control.py`): mock `pyaudio.Stream` to feed pre-recorded WAV chunks into `VoiceController._listen_loop`. Verify that "robot strawberry" dispatches `_auto_one("strawberry")`, that "stop" dispatches without wake word, that timeout returns to idle, and that `[unk]` tokens are ignored.
- **Manual test**: run GUI with a real microphone and say the commands. Verify label transitions and correct action execution.
- **Degradation test**: uninstall `vosk`, verify GUI starts normally without voice features.
