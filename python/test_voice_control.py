"""Tests for voice_control parser (parse_command, GRAMMAR_WORDS)."""
import os, sys, time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from voice_control import parse_command, GRAMMAR_WORDS


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _idle():
    """Return a fresh idle state dict."""
    return {"phase": "idle", "wake_time": 0.0}


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_wake_then_command():
    name = "wake_then_command"
    state = _idle()
    now = 1000.0
    # "robot" should set phase=awaiting, return None
    result = parse_command("robot", state, now)
    assert result is None, f"expected None after wake word, got {result!r}"
    assert state["phase"] == "awaiting", f"phase should be 'awaiting', got {state['phase']!r}"
    # "strawberry" shortly after should return "strawberry" and reset to idle
    result2 = parse_command("strawberry", state, now + 0.5)
    assert result2 == "strawberry", f"expected 'strawberry', got {result2!r}"
    assert state["phase"] == "idle", f"phase should reset to 'idle', got {state['phase']!r}"
    return name, True, "robot -> awaiting; strawberry -> 'strawberry', idle"


def test_command_without_wake_ignored():
    name = "command_without_wake_ignored"
    state = _idle()
    result = parse_command("banana", state, 1000.0)
    assert result is None, f"expected None in idle state, got {result!r}"
    assert state["phase"] == "idle", f"phase should remain 'idle', got {state['phase']!r}"
    return name, True, "banana in idle -> None, stays idle"


def test_stop_bypasses_wake_word():
    name = "stop_bypasses_wake_word"
    state = _idle()
    result = parse_command("stop", state, 1000.0)
    assert result == "stop", f"expected 'stop', got {result!r}"
    assert state["phase"] == "idle", f"phase should be 'idle' after stop, got {state['phase']!r}"
    return name, True, "stop in idle -> 'stop', stays/resets idle"


def test_stop_during_awaiting():
    name = "stop_during_awaiting"
    state = {"phase": "awaiting", "wake_time": 1000.0}
    result = parse_command("stop", state, 1000.5)
    assert result == "stop", f"expected 'stop', got {result!r}"
    assert state["phase"] == "idle", f"phase should reset to 'idle', got {state['phase']!r}"
    return name, True, "stop while awaiting -> 'stop', resets to idle"


def test_timeout_returns_to_idle():
    name = "timeout_returns_to_idle"
    now = 1000.0
    state = {"phase": "awaiting", "wake_time": now}
    # Send a command after >2s (WAKE_TIMEOUT_S = 2.0)
    result = parse_command("banana", state, now + 3.0)
    assert result is None, f"expected None on timeout, got {result!r}"
    assert state["phase"] == "idle", f"phase should reset to 'idle' on timeout, got {state['phase']!r}"
    return name, True, "command after 3s timeout -> None, resets to idle"


def test_unk_during_awaiting_ignored():
    name = "unk_during_awaiting_ignored"
    now = 1000.0
    state = {"phase": "awaiting", "wake_time": now}
    result = parse_command("[unk]", state, now + 0.1)
    assert result is None, f"expected None for [unk], got {result!r}"
    assert state["phase"] == "awaiting", f"phase should stay 'awaiting', got {state['phase']!r}"
    return name, True, "[unk] during awaiting -> None, stays awaiting"


def test_repeated_robot_resets_timer():
    name = "repeated_robot_resets_timer"
    state = _idle()
    now = 1000.0
    parse_command("robot", state, now)
    first_wake_time = state["wake_time"]
    # Second "robot" at a later time should update wake_time
    parse_command("robot", state, now + 1.5)
    assert state["wake_time"] > first_wake_time, (
        f"wake_time should have been updated: {state['wake_time']} vs {first_wake_time}"
    )
    assert state["wake_time"] == now + 1.5, (
        f"wake_time should be now+1.5={now + 1.5}, got {state['wake_time']}"
    )
    return name, True, f"second robot updated wake_time from {first_wake_time} to {state['wake_time']}"


def test_all_commands_recognized():
    name = "all_commands_recognized"
    # The 7 non-stop valid commands (all VALID_COMMANDS minus "stop")
    commands = ["strawberry", "banana", "tomato", "all", "home", "refresh", "survey"]
    failures = []
    now = 1000.0
    for cmd in commands:
        state = {"phase": "awaiting", "wake_time": now}
        result = parse_command(cmd, state, now + 0.5)
        if result != cmd:
            failures.append(f"{cmd!r} -> {result!r}")
        if state["phase"] != "idle":
            failures.append(f"after {cmd!r}: phase={state['phase']!r}")
    assert not failures, "failures: " + "; ".join(failures)
    return name, True, f"all {len(commands)} commands recognized"


def test_grammar_words_matches_spec():
    name = "grammar_words_matches_spec"
    expected = {"robot", "strawberry", "banana", "tomato", "all", "stop",
                "home", "refresh", "survey", "[unk]"}
    actual = set(GRAMMAR_WORDS)
    assert actual == expected, (
        f"GRAMMAR_WORDS mismatch\n  extra: {actual - expected}\n  missing: {expected - actual}"
    )
    assert len(GRAMMAR_WORDS) == 10, f"expected 10 words, got {len(GRAMMAR_WORDS)}"
    return name, True, f"GRAMMAR_WORDS = {sorted(GRAMMAR_WORDS)}"


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

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
