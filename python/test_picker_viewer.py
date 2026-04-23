"""Tests for picker_viewer pure helpers."""
import os, sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fruit_detector import Detection
from picker_viewer import (
    _nearest_detection, _filter_by_type, _annotate, _hud_text,
)


def _mk_det(ftype, cx, cy, conf=0.8):
    return Detection(
        fruit_type=ftype,
        center_px=(cx, cy),
        center_base_m=np.array([0.3, 0.1, 0.02]),
        confidence=conf,
        area_px=500,
        bbox=(cx - 10, cy - 10, 20, 20),
    )


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


def test_filter_by_type():
    name = "filter_by_type"
    dets = [_mk_det("tomato", 1, 1), _mk_det("banana", 2, 2),
            _mk_det("tomato", 3, 3)]
    got = _filter_by_type(dets, "tomato")
    assert len(got) == 2 and all(d.fruit_type == "tomato" for d in got)
    return name, True, "keeps only matching type"


def test_annotate_returns_image_same_shape():
    name = "annotate_shape"
    img = np.full((720, 1280, 3), 50, dtype=np.uint8)
    dets = [_mk_det("tomato", 100, 100), _mk_det("banana", 300, 200)]
    out = _annotate(img, dets, residual_mm=1.2, warnings=["test warn"])
    assert out.shape == img.shape
    assert out.dtype == np.uint8
    # Annotation must modify pixels (bbox draw)
    assert not np.array_equal(out, img), "annotate left image unchanged"
    return name, True, "shape preserved, pixels modified"


def test_hud_text_contains_counts():
    name = "hud_text"
    msg = _hud_text(n_fruits=5, residual_mm=1.8, mode="idle")
    assert "5" in msg and "1.8" in msg
    return name, True, f"'{msg}'"


TESTS = [
    test_nearest_within_radius, test_nearest_outside_radius_returns_none,
    test_nearest_empty_list_returns_none, test_filter_by_type,
    test_annotate_returns_image_same_shape, test_hud_text_contains_counts,
]


def main():
    fails = 0
    for fn in TESTS:
        try:
            name, ok, msg = fn()
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {name}  {msg}")
            if not ok: fails += 1
        except Exception as ex:
            import traceback
            fails += 1
            print(f"  [FAIL] {fn.__name__}: {ex}")
            traceback.print_exc()
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed")
    return fails


if __name__ == "__main__":
    sys.exit(main())
