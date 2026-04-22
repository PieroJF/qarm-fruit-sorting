"""
Unit tests for the rewritten fruit_detector.py (spec §3.2).

Run with:
    py -3.13 python/test_fruit_detector.py
Exit code is the number of failed sections (0 = all pass).
"""
import os
import sys
import traceback
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

_RESULTS = []


def _section(name, fn):
    try:
        fn()
        _RESULTS.append((name, True, None))
        print(f"  [PASS] {name}")
    except Exception as ex:
        _RESULTS.append((name, False, ex))
        print(f"  [FAIL] {name}: {ex}")
        traceback.print_exc()


# ========================================================================
# B1. Detection dataclass + module import
# ========================================================================
def test_detection_fields():
    from fruit_detector import Detection
    d = Detection(
        fruit_type="banana",
        center_px=(100, 200),
        center_base_m=np.array([0.3, 0.1, 0.03]),
        confidence=0.87,
        area_px=1500,
        bbox=(50, 150, 100, 80),
    )
    assert d.fruit_type == "banana"
    assert d.center_px == (100, 200)
    assert np.allclose(d.center_base_m, [0.3, 0.1, 0.03])
    assert d.confidence == 0.87
    assert d.area_px == 1500
    assert d.bbox == (50, 150, 100, 80)


def test_detection_to_dict():
    from fruit_detector import Detection
    d = Detection(
        fruit_type="tomato",
        center_px=(10, 20),
        center_base_m=np.array([0.1, 0.2, 0.04]),
        confidence=0.5,
        area_px=800,
        bbox=(5, 10, 20, 20),
    )
    out = d.to_dict()
    assert out["fruit_type"] == "tomato"
    assert out["center_px"] == [10, 20]
    assert out["center_base_m"] == [0.1, 0.2, 0.04]
    assert out["confidence"] == 0.5
    assert out["area_px"] == 800
    assert out["bbox"] == [5, 10, 20, 20]


if __name__ == "__main__":
    _section("B1 Detection fields", test_detection_fields)
    _section("B1 Detection to_dict", test_detection_to_dict)
    fails = sum(1 for _, ok, _ in _RESULTS if not ok)
    print(f"\n{len(_RESULTS)} test(s), {fails} failed")
    sys.exit(fails)
