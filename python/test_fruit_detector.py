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


# ========================================================================
# B2. hsv_mask helper
# ========================================================================
def _solid_bgr(color_bgr, size=(480, 640)):
    """Return a size-(H, W, 3) uint8 BGR image of a solid color."""
    H, W = size
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:, :] = color_bgr
    return img


def test_hsv_mask_single_range_yellow():
    from fruit_detector import hsv_mask
    yellow = _solid_bgr((0, 220, 220))  # BGR for bright yellow
    ranges = {"h": [18, 35], "s": [80, 255], "v": [80, 255]}
    mask = hsv_mask(yellow, ranges)
    assert mask.dtype == np.uint8
    assert mask.shape == (480, 640)
    assert (mask > 0).mean() > 0.95, (
        f"yellow mask coverage {(mask > 0).mean():.2f} too low")


def test_hsv_mask_red_wrap_around():
    from fruit_detector import hsv_mask
    red = _solid_bgr((0, 0, 220))
    ranges = {"h_wrap1": [0, 10], "h_wrap2": [170, 180],
              "s": [80, 255], "v": [60, 255]}
    mask = hsv_mask(red, ranges)
    assert (mask > 0).mean() > 0.95, (
        f"red mask coverage {(mask > 0).mean():.2f} too low")


def test_hsv_mask_excludes_out_of_range():
    from fruit_detector import hsv_mask
    blue = _solid_bgr((220, 0, 0))
    ranges = {"h": [18, 35], "s": [80, 255], "v": [80, 255]}
    mask = hsv_mask(blue, ranges)
    assert (mask > 0).mean() < 0.01


# ========================================================================
# B3. banana detection
# ========================================================================
def _draw_rect_bgr(size, center, w, h, color, rotation_deg=0):
    """Render a filled rotated rect on a black size=(H,W) canvas. Returns BGR."""
    import cv2
    H, W = size
    img = np.zeros((H, W, 3), dtype=np.uint8)
    box = cv2.boxPoints(((center[0], center[1]), (w, h), rotation_deg))
    box = np.int32(box)
    cv2.fillPoly(img, [box], color)
    return img


def test_banana_contours_finds_one():
    from fruit_detector import _detect_banana_contours
    img = _draw_rect_bgr((480, 640), (320, 240), 180, 60,
                          (0, 220, 220))
    dets = _detect_banana_contours(img)
    assert len(dets) == 1, f"expected 1 banana, got {len(dets)}"
    (cx, cy), area, bbox, conf = dets[0]
    assert abs(cx - 320) < 10, f"cx={cx}"
    assert abs(cy - 240) < 10, f"cy={cy}"
    assert area > 8000, f"area={area}"
    assert conf > 0.3, f"confidence={conf}"


def test_banana_rejects_round_shape():
    import cv2
    from fruit_detector import _detect_banana_contours
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(img, (320, 240), 60, (0, 220, 220), -1)
    dets = _detect_banana_contours(img)
    assert len(dets) == 0, f"expected 0, got {len(dets)}"


def test_banana_rejects_tiny_blob():
    from fruit_detector import _detect_banana_contours
    img = _draw_rect_bgr((480, 640), (320, 240), 40, 10,
                          (0, 220, 220))
    dets = _detect_banana_contours(img)
    assert len(dets) == 0


if __name__ == "__main__":
    _section("B1 Detection fields", test_detection_fields)
    _section("B1 Detection to_dict", test_detection_to_dict)
    _section("B2 hsv_mask single (yellow)", test_hsv_mask_single_range_yellow)
    _section("B2 hsv_mask wrap (red)", test_hsv_mask_red_wrap_around)
    _section("B2 hsv_mask excludes blue", test_hsv_mask_excludes_out_of_range)
    _section("B3 banana finds one", test_banana_contours_finds_one)
    _section("B3 banana rejects round", test_banana_rejects_round_shape)
    _section("B3 banana rejects tiny", test_banana_rejects_tiny_blob)
    fails = sum(1 for _, ok, _ in _RESULTS if not ok)
    print(f"\n{len(_RESULTS)} test(s), {fails} failed")
    sys.exit(fails)
