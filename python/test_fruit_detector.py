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


# ========================================================================
# B4. tomato detection
# ========================================================================
def _draw_circle_bgr(size, center, r, color):
    import cv2
    H, W = size
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.circle(img, center, r, color, -1)
    return img


def test_tomato_finds_round_red():
    from fruit_detector import _detect_tomato_contours
    img = _draw_circle_bgr((480, 640), (320, 240), 50, (0, 0, 220))
    dets = _detect_tomato_contours(img)
    assert len(dets) == 1, f"expected 1 tomato, got {len(dets)}"
    (cx, cy), area, bbox, conf = dets[0]
    assert abs(cx - 320) < 5
    assert abs(cy - 240) < 5
    assert area > 7000, f"area={area}"
    assert conf > 0.5


def test_tomato_rejects_red_with_green_above():
    """A red circle with a green patch directly above it should be read
    as a strawberry candidate, not a tomato."""
    import cv2
    from fruit_detector import _detect_tomato_contours
    img = _draw_circle_bgr((480, 640), (320, 240), 50, (0, 0, 220))
    cv2.rectangle(img, (290, 170), (350, 185), (0, 200, 0), -1)
    dets = _detect_tomato_contours(img)
    assert len(dets) == 0, (
        f"tomato detector must reject red+green-above; got {len(dets)}")


def test_tomato_rejects_elongated_red():
    from fruit_detector import _detect_tomato_contours
    img = _draw_rect_bgr((480, 640), (320, 240), 160, 30,
                         (0, 0, 220))
    dets = _detect_tomato_contours(img)
    assert len(dets) == 0, "elongated red must not register as tomato"


# ========================================================================
# B5. strawberry detection
# ========================================================================
def _draw_strawberry_bgr(size, center, w, h_fruit, h_calyx=12):
    """A triangle-ish red body (wide top, narrow bottom) with a green band
    immediately above (the calyx). Good enough to exercise the taper score
    and the green-calyx confirm."""
    import cv2
    H, W = size
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cx, cy = center
    top_w = w
    bot_w = w // 3
    pts = np.array([
        [cx - top_w // 2, cy - h_fruit // 2],
        [cx + top_w // 2, cy - h_fruit // 2],
        [cx + bot_w // 2, cy + h_fruit // 2],
        [cx - bot_w // 2, cy + h_fruit // 2],
    ], dtype=np.int32)
    cv2.fillPoly(img, [pts], (0, 0, 220))
    gy_lo = cy - h_fruit // 2 - 20
    gy_hi = cy - h_fruit // 2 - 6
    cv2.rectangle(img, (cx - top_w // 2, gy_lo),
                   (cx + top_w // 2, gy_hi), (0, 200, 0), -1)
    return img


def test_strawberry_finds_tapered_red_with_calyx():
    from fruit_detector import _detect_strawberry_contours
    img = _draw_strawberry_bgr((480, 640), (320, 240), 60, 70)
    dets = _detect_strawberry_contours(img)
    assert len(dets) == 1, f"expected 1 strawberry, got {len(dets)}"
    (cx, cy), area, bbox, conf = dets[0]
    assert abs(cx - 320) < 10
    assert conf > 0.3


def test_strawberry_rejects_red_no_calyx():
    """Same tapered shape but no green above → not a strawberry."""
    import cv2
    from fruit_detector import _detect_strawberry_contours
    img = _draw_strawberry_bgr((480, 640), (320, 240), 60, 70)
    # Blank out the calyx band.
    img[180:240, 280:360] = 0
    cv2.fillPoly(img, [np.array([
        [290, 205], [350, 205], [340, 275], [300, 275]], dtype=np.int32)],
        (0, 0, 220))
    dets = _detect_strawberry_contours(img)
    assert len(dets) == 0, (
        f"strawberry must require green calyx; got {len(dets)}")


# ========================================================================
# B6. depth sampling
# ========================================================================
def test_depth_sample_returns_median():
    from fruit_detector import sample_depth_at_pixel
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[238:243, 318:323] = np.array([
        [100, 105, 110, 108, 102],
        [107, 113, 115, 112, 106],
        [104, 109, 120, 116, 108],
        [103, 108, 113, 110, 105],
        [101, 106, 111, 107, 102]], dtype=np.uint16)
    z = sample_depth_at_pixel(depth, (320, 240), patch=5)
    assert z is not None
    assert 105 < z < 115, f"median {z} outside expected"


def test_depth_sample_rejects_mostly_zero():
    from fruit_detector import sample_depth_at_pixel
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[238, 318:321] = 100
    z = sample_depth_at_pixel(depth, (320, 240), patch=5)
    assert z is None, f"should reject sparse depth, got {z}"


def test_depth_sample_rejects_out_of_range():
    from fruit_detector import sample_depth_at_pixel
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    z = sample_depth_at_pixel(depth, (320, 240), patch=5,
                               min_mm=10, max_mm=800)
    assert z is None


# ========================================================================
# B7. pixel -> base-frame projection
# ========================================================================
def _fake_session_cal(origin=(0.30, 0.10, 0.02),
                      cam_h_m=0.60, square_mm=30):
    """Build a SessionCal-like dict for testing projection math.
    H maps pixel (u,v) to chess-XY in mm assuming a nadir camera at
    cam_h_m above the chess plane."""
    from session_cal import SessionCal
    fx = 912.0
    fy = 912.0
    cx_p = 640.0
    cy_p = 360.0
    cam_h_mm = cam_h_m * 1000.0
    sx = cam_h_mm / fx
    sy = cam_h_mm / fy
    H = np.array([
        [sx,  0.0, -cx_p * sx],
        [0.0, sy,  -cy_p * sy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    cal = SessionCal(
        timestamp="2026-04-22T00:00:00",
        chess_origin_in_base_m=np.array(origin),
        h_pixel_to_chess_mm=H,
        survey_pose_joints_rad=np.zeros(4),
        chess_pattern={"cols": 8, "rows": 6, "square_mm": square_mm,
                        "inner_cols": 7, "inner_rows": 5},
        d415_intrinsics={"fx": fx, "fy": fy, "cx": cx_p, "cy": cy_p},
        homography_reproj_rms_px=0.5,
        camera_height_above_table_m=cam_h_m,
        image_size=(1280, 720),
    )
    return cal


def test_pixel_to_base_identity_at_principal_point():
    from fruit_detector import pixel_to_base_frame
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    xyz = pixel_to_base_frame((640, 360), fruit_top_z_mm=0.0, session_cal=cal)
    assert np.allclose(xyz[:2], [0.30, 0.10], atol=1e-3), xyz
    assert abs(xyz[2] - 0.02) < 1e-6


def test_pixel_to_base_parallax_shrinks_with_fruit_height():
    """A pixel 100 px off the principal point with a tall fruit (50 mm)
    should land closer to the principal point's base XY than the same
    pixel with a flat fruit (0 mm). That is, parallax correction moves
    the reported XY TOWARDS the optical axis as the fruit gets taller
    (since we are reporting where the fruit BASE sits)."""
    from fruit_detector import pixel_to_base_frame
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    xyz_flat = pixel_to_base_frame((740, 360), fruit_top_z_mm=0.0,
                                    session_cal=cal)
    xyz_tall = pixel_to_base_frame((740, 360), fruit_top_z_mm=50.0,
                                    session_cal=cal)
    assert xyz_flat[0] > xyz_tall[0] > 0.30, (
        f"flat={xyz_flat} tall={xyz_tall}")


def test_pixel_to_base_z_equals_origin_plus_fruit_top():
    from fruit_detector import pixel_to_base_frame
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    xyz = pixel_to_base_frame((640, 360), fruit_top_z_mm=45.0,
                               session_cal=cal)
    assert abs(xyz[2] - (0.02 + 0.045)) < 1e-6, xyz


if __name__ == "__main__":
    _section("B1 Detection fields", test_detection_fields)
    _section("B1 Detection to_dict", test_detection_to_dict)
    _section("B2 hsv_mask single (yellow)", test_hsv_mask_single_range_yellow)
    _section("B2 hsv_mask wrap (red)", test_hsv_mask_red_wrap_around)
    _section("B2 hsv_mask excludes blue", test_hsv_mask_excludes_out_of_range)
    _section("B3 banana finds one", test_banana_contours_finds_one)
    _section("B3 banana rejects round", test_banana_rejects_round_shape)
    _section("B3 banana rejects tiny", test_banana_rejects_tiny_blob)
    _section("B4 tomato finds round red", test_tomato_finds_round_red)
    _section("B4 tomato rejects green-above", test_tomato_rejects_red_with_green_above)
    _section("B4 tomato rejects elongated", test_tomato_rejects_elongated_red)
    _section("B5 strawberry tapered with calyx", test_strawberry_finds_tapered_red_with_calyx)
    _section("B5 strawberry rejects no-calyx", test_strawberry_rejects_red_no_calyx)
    _section("B6 depth sample median", test_depth_sample_returns_median)
    _section("B6 depth sample rejects sparse", test_depth_sample_rejects_mostly_zero)
    _section("B6 depth sample rejects range", test_depth_sample_rejects_out_of_range)
    _section("B7 px->base principal point", test_pixel_to_base_identity_at_principal_point)
    _section("B7 px->base parallax", test_pixel_to_base_parallax_shrinks_with_fruit_height)
    _section("B7 px->base z", test_pixel_to_base_z_equals_origin_plus_fruit_top)
    fails = sum(1 for _, ok, _ in _RESULTS if not ok)
    print(f"\n{len(_RESULTS)} test(s), {fails} failed")
    sys.exit(fails)
