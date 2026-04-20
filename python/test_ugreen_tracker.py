"""Unit tests for ugreen_tracker. No hardware needed — tests use synthetic
frames stored as fixtures."""
import os
import sys
import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from ugreen_tracker import (tcp_from_diff, save_baseline, load_baseline,
                             MIN_ARM_PIXELS)


def _synth_baseline(h=480, w=640):
    """Empty workspace: grey desk with a white paper in the middle."""
    img = np.full((h, w, 3), 180, dtype=np.uint8)
    cv2.rectangle(img, (200, 200), (440, 340), 240, -1)
    return img


def _synth_with_arm(baseline):
    """Same scene with a dark arm silhouette entering from top-center."""
    img = baseline.copy()
    # Thin vertical arm (dark grey) and a gripper blob at its tip
    cv2.rectangle(img, (310, 0), (330, 260), 40, -1)
    cv2.circle(img, (320, 265), 15, 30, -1)
    return img


def test_diff_finds_arm_tip():
    baseline = _synth_baseline()
    frame = _synth_with_arm(baseline)
    tcp = tcp_from_diff(frame, baseline)
    assert tcp is not None, "expected a TCP, got None"
    col, row = tcp
    # Arm is at column ~320, tip row ~280. Allow loose tolerance.
    assert 300 < col < 340, f"col {col} not near 320"
    assert 260 < row < 295, f"row {row} not near 280"


def test_no_arm_returns_none():
    baseline = _synth_baseline()
    frame = baseline.copy()  # identical; no arm
    tcp = tcp_from_diff(frame, baseline)
    assert tcp is None, f"expected None for identical frames, got {tcp}"


def test_baseline_roundtrip(tmp_path=None):
    import tempfile
    tmp = tempfile.mkdtemp()
    base = _synth_baseline()
    path = os.path.join(tmp, "base.png")
    save_baseline(base, path)
    reloaded = load_baseline(path)
    assert reloaded.shape == base.shape
    assert np.allclose(reloaded, base)


def test_small_difference_rejected():
    """Single-pixel noise shouldn't register as 'arm present'."""
    baseline = _synth_baseline()
    frame = baseline.copy()
    frame[100, 100] = (0, 0, 0)  # one pixel changed
    tcp = tcp_from_diff(frame, baseline)
    assert tcp is None, f"expected None for 1-pixel change, got {tcp}"


if __name__ == "__main__":
    fails = 0
    for t in [test_diff_finds_arm_tip, test_no_arm_returns_none,
              test_baseline_roundtrip, test_small_difference_rejected]:
        try:
            t()
            print(f"  [OK]   {t.__name__}")
        except Exception as ex:
            print(f"  [FAIL] {t.__name__}: {ex}")
            fails += 1
    sys.exit(fails)
