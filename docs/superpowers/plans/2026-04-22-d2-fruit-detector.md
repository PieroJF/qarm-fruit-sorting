# D2 Fruit Detector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite `python/fruit_detector.py` per spec §3.2. New `Detection` dataclass + `detect_fruits(color, depth, session_cal)` that returns banana/tomato/strawberry detections with base-frame XYZ coordinates. Pure OpenCV (no ML). Consumes the `session_cal.json` produced by D1.

**Architecture:** Single new module `python/fruit_detector.py` with internal sections (HSV mask helper, per-class detectors, depth sampling, parallax correction, top-level orchestrator). Accompanied by `python/test_fruit_detector.py` with offline tests on synthetic RGB + depth frames. The old `fruit_detector.py` is replaced wholesale — downstream consumers (`main_final.py`, `preflight.py`, Simulink `.m` glue, legacy tests) will break temporarily and get fixed in D3 / D8.

**Tech Stack:** Python 3.13 (`py -3.13`), NumPy, OpenCV 4.13. `SessionCal` dataclass from D1 (`python/session_cal.py`, commit `615e801`). Test style matches the repo idiom (script-based runner, `_section` helper, exit code = fail count).

**Spec reference:** `docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md` §3.2.
**D1 output (input to this plan):** `python/session_cal.py` — `SessionCal` with `h_pixel_to_chess_mm`, `chess_origin_in_base_m`, `camera_height_above_table_m`, `d415_intrinsics`.

---

## File structure

One new test file; one module wholesale rewritten; one legacy-test surgery:

| Path | Responsibility |
|---|---|
| `python/fruit_detector.py` (rewrite) | `Detection` dataclass, HSV+shape+calyx detection, depth sampling, parallax-corrected base-frame projection, top-level `detect_fruits(color, depth, session_cal) -> list[Detection]`. |
| `python/test_fruit_detector.py` (new) | Offline unit tests using synthetic BGR + depth frames. No hardware. |
| `python/test_integration.py` (surgery) | Remove the three vision sections that import the legacy `FruitDetection` / `detection_depth_mm` / `CIRCULARITY_THRESH` symbols. Keep FSM / trajectory / controller sections intact. |

Old `fruit_detector.py` is not preserved — `git log` has the history. No `fruit_detector_legacy.py`.

---

## Task B1: `Detection` dataclass + module skeleton

**Files:**
- Create: `python/fruit_detector.py` (new content — will replace existing file later in B9; for now we build in parallel under a temp import path)
- Create: `python/test_fruit_detector.py`

**Approach for this task:** Because the existing `fruit_detector.py` is still imported by running code (`preflight.py`, etc.), we build the new module in-place by OVERWRITING `python/fruit_detector.py` now. This breaks downstream imports — that's expected and fixed by D3 / D8 per the plan header. Confirm this is acceptable before proceeding.

- [ ] **Step 1: Write the failing test**

Create `python/test_fruit_detector.py`:

```python
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: FAIL — `ImportError: cannot import name 'Detection' from 'fruit_detector'` (because old file has `FruitDetection`, not `Detection`).

- [ ] **Step 3: Replace `python/fruit_detector.py` contents**

Overwrite the existing `python/fruit_detector.py` (do NOT rename or preserve the old one — git has the history):

```python
"""
Fruit detector for the overhead-vision pipeline.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.2.

Consumes a D415 RGB frame, a D415 depth frame, and a SessionCal (from D1).
Returns a list of `Detection` objects, each carrying the fruit type,
pixel + base-frame coordinates, and confidence.

Pure OpenCV 4.x; no hardware imports, no ML.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import numpy as np
import cv2


@dataclass
class Detection:
    fruit_type: str              # "banana" | "tomato" | "strawberry"
    center_px: tuple             # (cx, cy) in image pixels
    center_base_m: np.ndarray    # shape (3,) XYZ in robot base frame, metres
    confidence: float            # 0..1
    area_px: int
    bbox: tuple                  # (x, y, w, h) — opencv convention

    def to_dict(self) -> dict[str, Any]:
        return {
            "fruit_type": self.fruit_type,
            "center_px": list(self.center_px),
            "center_base_m": np.asarray(self.center_base_m).tolist(),
            "confidence": float(self.confidence),
            "area_px": int(self.area_px),
            "bbox": list(self.bbox),
        }
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 2 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): Detection dataclass + new fruit_detector skeleton

D2-B1: start fresh rewrite of fruit_detector.py per spec §3.2.
Detection dataclass carries pixel + base-frame position + confidence.
Old HSV+shape detector removed; downstream consumers (preflight,
main_final, test_integration) will break until D3/D8 cleanups."
```

---

## Task B2: HSV mask helper

**Files:**
- Modify: `python/fruit_detector.py` (append HSV_RANGES constants + `hsv_mask` function)
- Modify: `python/test_fruit_detector.py` (append B2 section)

Handles both single-range hues (yellow, green) and wrap-around hues (red spans 0 and 180). Applies morphological open 5×5 then close 9×9 to clean up the mask.

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py` (ABOVE the runner block, BELOW the B1 functions):

```python
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
    # Almost the entire image should be masked in.
    assert (mask > 0).mean() > 0.95, (
        f"yellow mask coverage {(mask > 0).mean():.2f} too low")


def test_hsv_mask_red_wrap_around():
    from fruit_detector import hsv_mask
    # Pure red in BGR: low H (0-10) in HSV.
    red = _solid_bgr((0, 0, 220))
    ranges = {"h_wrap1": [0, 10], "h_wrap2": [170, 180],
              "s": [80, 255], "v": [60, 255]}
    mask = hsv_mask(red, ranges)
    assert (mask > 0).mean() > 0.95, (
        f"red mask coverage {(mask > 0).mean():.2f} too low")


def test_hsv_mask_excludes_out_of_range():
    from fruit_detector import hsv_mask
    blue = _solid_bgr((220, 0, 0))  # BGR blue → hue ≈ 120
    ranges = {"h": [18, 35], "s": [80, 255], "v": [80, 255]}  # yellow range
    mask = hsv_mask(blue, ranges)
    # Blue image, yellow range → mask should be essentially zero.
    assert (mask > 0).mean() < 0.01
```

Add runner calls after the B1 ones:

```python
    _section("B2 hsv_mask single (yellow)", test_hsv_mask_single_range_yellow)
    _section("B2 hsv_mask wrap (red)", test_hsv_mask_red_wrap_around)
    _section("B2 hsv_mask excludes blue", test_hsv_mask_excludes_out_of_range)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B2 sections FAIL with `ImportError: cannot import name 'hsv_mask'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
# ------------------------------------------------------------------------
# Default HSV ranges (OpenCV convention: H=0-179, S=0-255, V=0-255).
# Starting values for lab tuning; hsv_tuner.py (D4-AM) can override.
# ------------------------------------------------------------------------
HSV_RANGES = {
    "banana":      {"h": [18, 35],  "s": [80, 255], "v": [80, 255]},
    "tomato":      {"h_wrap1": [0, 10],  "h_wrap2": [170, 180],
                    "s": [80, 255], "v": [60, 255]},
    "strawberry":  {"h_wrap1": [0, 10],  "h_wrap2": [170, 180],
                    "s": [80, 255], "v": [60, 255]},
    "green_calyx": {"h": [35, 85],  "s": [60, 255], "v": [40, 255]},
}

_OPEN_KERNEL = np.ones((5, 5), np.uint8)
_CLOSE_KERNEL = np.ones((9, 9), np.uint8)


def hsv_mask(bgr: np.ndarray, ranges: dict) -> np.ndarray:
    """
    Build a cleaned binary mask from a BGR image using an HSV ranges dict.

    Accepts two shapes:
      single range:    {"h": [lo, hi], "s": [lo, hi], "v": [lo, hi]}
      wrap-around hue: {"h_wrap1": [..], "h_wrap2": [..], "s": [..], "v": [..]}

    Applies MORPH_OPEN 5x5 then MORPH_CLOSE 9x9 to suppress speckle and
    fill small holes.

    Returns uint8 mask of the same HxW as the input; 0 or 255.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s_lo, s_hi = ranges["s"]
    v_lo, v_hi = ranges["v"]
    if "h_wrap1" in ranges and "h_wrap2" in ranges:
        lo1 = np.array([ranges["h_wrap1"][0], s_lo, v_lo], dtype=np.uint8)
        hi1 = np.array([ranges["h_wrap1"][1], s_hi, v_hi], dtype=np.uint8)
        lo2 = np.array([ranges["h_wrap2"][0], s_lo, v_lo], dtype=np.uint8)
        hi2 = np.array([ranges["h_wrap2"][1], s_hi, v_hi], dtype=np.uint8)
        mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1),
                               cv2.inRange(hsv, lo2, hi2))
    else:
        lo = np.array([ranges["h"][0], s_lo, v_lo], dtype=np.uint8)
        hi = np.array([ranges["h"][1], s_hi, v_hi], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _OPEN_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _CLOSE_KERNEL)
    return mask
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 5 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): HSV mask helper with wrap-around hue support

D2-B2: hsv_mask supports both single-range hues (yellow, green) and
wrap-around hues (red spans 0 and 180 in OpenCV's HSV). Applies
MORPH_OPEN 5x5 then MORPH_CLOSE 9x9. Default HSV_RANGES table bakes
in starting values for banana/tomato/strawberry/green_calyx; the D4
hsv_tuner.py will override these in lab lighting."
```

---

## Task B3: Banana detection

**Files:**
- Modify: `python/fruit_detector.py` (append `_detect_banana_contours`)
- Modify: `python/test_fruit_detector.py` (append B3 section)

Banana is the easiest: yellow + elongated (aspect > 1.8). This task adds the contour iteration + shape gating that tomato/strawberry will follow.

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py`:

```python
# ========================================================================
# B3. banana detection
# ========================================================================
def _draw_rect_bgr(size, center, w, h, color, rotation_deg=0):
    """Render a filled rotated rect on a black size=(H,W) canvas. Returns BGR."""
    H, W = size
    img = np.zeros((H, W, 3), dtype=np.uint8)
    box = cv2.boxPoints(((center[0], center[1]), (w, h), rotation_deg))
    box = np.int32(box)
    cv2.fillPoly(img, [box], color)
    return img


def test_banana_contours_finds_one():
    from fruit_detector import _detect_banana_contours
    # A single yellow elongated rect (aspect 3) in the middle of the frame.
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
    from fruit_detector import _detect_banana_contours
    # A ROUND yellow disc — should NOT pass the aspect filter.
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(img, (320, 240), 60, (0, 220, 220), -1)
    dets = _detect_banana_contours(img)
    assert len(dets) == 0, f"expected 0, got {len(dets)}"


def test_banana_rejects_tiny_blob():
    from fruit_detector import _detect_banana_contours
    # Aspect OK but area too small.
    img = _draw_rect_bgr((480, 640), (320, 240), 40, 10,
                          (0, 220, 220))
    dets = _detect_banana_contours(img)
    assert len(dets) == 0
```

Runner calls:

```python
    _section("B3 banana finds one", test_banana_contours_finds_one)
    _section("B3 banana rejects round", test_banana_rejects_round_shape)
    _section("B3 banana rejects tiny", test_banana_rejects_tiny_blob)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B3 sections FAIL with `cannot import name '_detect_banana_contours'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
# ------------------------------------------------------------------------
# Per-class detection. Each _detect_* returns a list of raw hits:
#   [((cx, cy), area_px, (bx, by, bw, bh), confidence), ...]
# The top-level detect_fruits() wraps these into Detection objects with
# base-frame coordinates.
# ------------------------------------------------------------------------
_BANANA_MIN_AREA = 800
_BANANA_MAX_AREA = 30000
_BANANA_MIN_ASPECT = 1.8


def _detect_banana_contours(bgr: np.ndarray) -> list:
    """Find banana blobs (yellow + elongated). Pure 2D; no depth, no base
    coords. Each hit: ((cx, cy), area_px, (x, y, w, h), confidence)."""
    mask = hsv_mask(bgr, HSV_RANGES["banana"])
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    hits = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < _BANANA_MIN_AREA or area > _BANANA_MAX_AREA:
            continue
        (cx, cy), (w_r, h_r), _ = cv2.minAreaRect(c)
        if w_r < 1 or h_r < 1:
            continue
        aspect = max(w_r, h_r) / min(w_r, h_r)
        if aspect < _BANANA_MIN_ASPECT:
            continue
        x, y, w_b, h_b = cv2.boundingRect(c)
        aspect_score = min(1.0, (aspect - _BANANA_MIN_ASPECT) / 2.0)
        hits.append(((int(cx), int(cy)), int(area),
                     (int(x), int(y), int(w_b), int(h_b)),
                     float(max(0.3, aspect_score))))
    return hits
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 8 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): banana detector (yellow + aspect gating)

D2-B3: _detect_banana_contours — yellow HSV mask + contour area gate
[800, 30000] px² + minAreaRect aspect >= 1.8. Rejects round yellow
discs and tiny speckles. Confidence scales with aspect ratio excess."
```

---

## Task B4: Tomato detection

**Files:**
- Modify: `python/fruit_detector.py` (append `_detect_tomato_contours`)
- Modify: `python/test_fruit_detector.py` (append B4 section)

Tomato = red + round (circularity > 0.7) + NO green region directly above the blob's top edge (which would indicate a strawberry calyx).

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py`:

```python
# ========================================================================
# B4. tomato detection
# ========================================================================
def _draw_circle_bgr(size, center, r, color):
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
    from fruit_detector import _detect_tomato_contours
    img = _draw_circle_bgr((480, 640), (320, 240), 50, (0, 0, 220))
    # Draw a green band 10-20 px above the blob's top.
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
```

Runner calls:

```python
    _section("B4 tomato finds round red", test_tomato_finds_round_red)
    _section("B4 tomato rejects green-above", test_tomato_rejects_red_with_green_above)
    _section("B4 tomato rejects elongated", test_tomato_rejects_elongated_red)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B4 sections FAIL — `cannot import name '_detect_tomato_contours'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
_TOMATO_MIN_AREA = 400
_TOMATO_MAX_AREA = 15000
_TOMATO_MIN_CIRCULARITY = 0.7
_GREEN_ABOVE_BAND_PX = (10, 20)   # band (min, max) px above blob top
_GREEN_ABOVE_RATIO = 0.05          # green pixels / band area threshold


def _has_green_above(bgr: np.ndarray, bbox: tuple) -> float:
    """Return the fraction of green pixels in a band 10-20 px above bbox top.
    0 if bbox hits the image top."""
    x, y, w, h = bbox
    y_lo = max(0, y - _GREEN_ABOVE_BAND_PX[1])
    y_hi = max(0, y - _GREEN_ABOVE_BAND_PX[0])
    if y_hi <= y_lo:
        return 0.0
    band = bgr[y_lo:y_hi, x:x + w]
    if band.size == 0:
        return 0.0
    green_mask = hsv_mask(band, HSV_RANGES["green_calyx"])
    return float((green_mask > 0).mean())


def _detect_tomato_contours(bgr: np.ndarray) -> list:
    mask = hsv_mask(bgr, HSV_RANGES["tomato"])
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    hits = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < _TOMATO_MIN_AREA or area > _TOMATO_MAX_AREA:
            continue
        perim = float(cv2.arcLength(c, True))
        if perim < 1:
            continue
        circ = 4 * np.pi * area / (perim * perim)
        if circ < _TOMATO_MIN_CIRCULARITY:
            continue
        x, y, w_b, h_b = cv2.boundingRect(c)
        green_ratio = _has_green_above(bgr, (x, y, w_b, h_b))
        if green_ratio > _GREEN_ABOVE_RATIO:
            # Red blob with green on top → strawberry, not tomato.
            continue
        M = cv2.moments(c)
        if M["m00"] <= 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        conf = float(min(1.0, circ) * (1.0 - green_ratio))
        hits.append(((cx, cy), int(area),
                     (int(x), int(y), int(w_b), int(h_b)), conf))
    return hits
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 11 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): tomato detector (red + round + no green above)

D2-B4: _detect_tomato_contours — red HSV + circularity >= 0.7 + area
[400, 15000] + green-above-band rejection. If a 10-20 px band above
the blob's top has >=5% green pixels, this is a strawberry candidate
and the tomato detector skips it. Confidence scales with circularity
discounted by the green fraction."
```

---

## Task B5: Strawberry detection

**Files:**
- Modify: `python/fruit_detector.py` (append `_detect_strawberry_contours`)
- Modify: `python/test_fruit_detector.py` (append B5 section)

Strawberry = red + smaller than tomato + green calyx above (inverse of the tomato rejection) + taper score (top wider than bottom).

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py`:

```python
# ========================================================================
# B5. strawberry detection
# ========================================================================
def _draw_strawberry_bgr(size, center, w, h_fruit, h_calyx=12):
    """A triangle-ish red body (wide top, narrow bottom) with a green band
    immediately above (the calyx). Good enough to exercise the taper score
    and the green-calyx confirm."""
    H, W = size
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cx, cy = center
    # Body: triangle wide at top, tapering to point below.
    top_w = w
    bot_w = w // 3
    pts = np.array([
        [cx - top_w // 2, cy - h_fruit // 2],
        [cx + top_w // 2, cy - h_fruit // 2],
        [cx + bot_w // 2, cy + h_fruit // 2],
        [cx - bot_w // 2, cy + h_fruit // 2],
    ], dtype=np.int32)
    cv2.fillPoly(img, [pts], (0, 0, 220))
    # Calyx: a green band 10-20 px above the body top.
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
```

Runner calls:

```python
    _section("B5 strawberry tapered with calyx", test_strawberry_finds_tapered_red_with_calyx)
    _section("B5 strawberry rejects no-calyx", test_strawberry_rejects_red_no_calyx)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B5 sections FAIL — `cannot import name '_detect_strawberry_contours'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
_STRAWBERRY_MIN_AREA = 200
_STRAWBERRY_MAX_AREA = 4000
_STRAWBERRY_MIN_CALYX = 0.05     # same threshold as tomato's rejection band


def _taper_score(contour, bbox) -> float:
    """Return width_at_top_20% / width_at_bottom_80% of the contour.

    Images use image-coord convention (y grows DOWN), so the "top" band
    corresponds to the blob's upper edge (where the strawberry's calyx
    attaches) and is the WIDER part. A taper > 1.0 means wider top,
    narrower bottom (strawberry shape). ~1.0 means tomato. Our
    _draw_strawberry_bgr test fixture produces taper ≈ 1.9."""
    x, y, w, h = bbox
    if h < 10:
        return 1.0
    top_band_y = y + int(h * 0.2)
    bot_band_y = y + int(h * 0.8)
    mask = np.zeros((y + h + 2, x + w + 2), dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
    top_cols = np.where(mask[top_band_y] > 0)[0]
    bot_cols = np.where(mask[bot_band_y] > 0)[0]
    top_w = (top_cols.max() - top_cols.min()) if top_cols.size >= 2 else 1
    bot_w = (bot_cols.max() - bot_cols.min()) if bot_cols.size >= 2 else 1
    return float(top_w) / float(max(1, bot_w))


def _detect_strawberry_contours(bgr: np.ndarray) -> list:
    mask = hsv_mask(bgr, HSV_RANGES["strawberry"])
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    hits = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < _STRAWBERRY_MIN_AREA or area > _STRAWBERRY_MAX_AREA:
            continue
        x, y, w_b, h_b = cv2.boundingRect(c)
        calyx_ratio = _has_green_above(bgr, (x, y, w_b, h_b))
        if calyx_ratio < _STRAWBERRY_MIN_CALYX:
            continue
        taper = _taper_score(c, (x, y, w_b, h_b))
        # Accept near-round (1.0) to strongly tapered (~3.0). Below 0.9
        # means narrower top (upside-down, unlikely in overhead view);
        # above 3.5 means the contour is bizarrely thin at the bottom,
        # probably a segmentation artifact.
        if taper < 0.9 or taper > 3.5:
            continue
        M = cv2.moments(c)
        if M["m00"] <= 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        # Confidence driven mostly by calyx; taper gate is binary.
        conf = float(min(1.0, calyx_ratio * 3.0))
        hits.append(((cx, cy), int(area),
                     (int(x), int(y), int(w_b), int(h_b)),
                     max(0.3, conf)))
    return hits
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 13 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): strawberry detector (red + calyx + taper)

D2-B5: _detect_strawberry_contours — red HSV + area [200, 4000] +
green-calyx confirm (>=5% green pixels in band above) + taper score
(width@top20% / width@bottom80% in [0.3, 1.3], peaks at 0.7).
Shares _has_green_above with tomato path — tomato uses it as a
REJECT signal, strawberry as a CONFIRM signal."
```

---

## Task B6: Depth sampling with outlier rejection

**Files:**
- Modify: `python/fruit_detector.py` (append `sample_depth_at_pixel`)
- Modify: `python/test_fruit_detector.py` (append B6 section)

D415 depth is uint16 in mm. Invalid samples are 0, and edges of objects often return 0. Sample a 5×5 patch at the blob center, filter to valid range, require ≥ 8 valid samples, return median. Else reject (return None).

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py`:

```python
# ========================================================================
# B6. depth sampling
# ========================================================================
def test_depth_sample_returns_median():
    from fruit_detector import sample_depth_at_pixel
    depth = np.zeros((480, 640), dtype=np.uint16)
    # Paint a 5x5 patch of mixed depths around (320, 240).
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
    # Only 3 valid samples in a 25-pixel patch.
    depth[238, 318:321] = 100
    z = sample_depth_at_pixel(depth, (320, 240), patch=5)
    assert z is None, f"should reject sparse depth, got {z}"


def test_depth_sample_rejects_out_of_range():
    from fruit_detector import sample_depth_at_pixel
    depth = np.full((480, 640), 2000, dtype=np.uint16)  # 2 m, too far
    z = sample_depth_at_pixel(depth, (320, 240), patch=5,
                               min_mm=10, max_mm=800)
    assert z is None
```

Runner calls:

```python
    _section("B6 depth sample median", test_depth_sample_returns_median)
    _section("B6 depth sample rejects sparse", test_depth_sample_rejects_mostly_zero)
    _section("B6 depth sample rejects range", test_depth_sample_rejects_out_of_range)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B6 sections FAIL — `cannot import name 'sample_depth_at_pixel'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
_DEPTH_MIN_VALID = 8       # min valid samples in the patch
_DEPTH_MIN_MM = 10
_DEPTH_MAX_MM = 800


def sample_depth_at_pixel(depth_mm: np.ndarray, center_px: tuple,
                           patch: int = 5,
                           min_mm: int = _DEPTH_MIN_MM,
                           max_mm: int = _DEPTH_MAX_MM):
    """Return median depth (in mm) of a patch around (cx, cy), or None
    if the patch has fewer than _DEPTH_MIN_VALID samples inside
    [min_mm, max_mm]."""
    cx, cy = int(center_px[0]), int(center_px[1])
    half = patch // 2
    y_lo = max(0, cy - half)
    y_hi = min(depth_mm.shape[0], cy + half + 1)
    x_lo = max(0, cx - half)
    x_hi = min(depth_mm.shape[1], cx + half + 1)
    patch_vals = depth_mm[y_lo:y_hi, x_lo:x_hi].astype(np.int32).ravel()
    valid = patch_vals[(patch_vals >= min_mm) & (patch_vals <= max_mm)]
    if valid.size < _DEPTH_MIN_VALID:
        return None
    return float(np.median(valid))
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 16 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): depth sampling with outlier + sparsity rejection

D2-B6: sample_depth_at_pixel — 5x5 median of a patch, filtered to
valid range [10, 800] mm. Returns None if fewer than 8 valid samples
(D415 depth at object edges is usually 0 — this guards against edge-
of-fruit samples confusing the parallax correction downstream)."
```

---

## Task B7: Pixel → base-frame projection with parallax correction

**Files:**
- Modify: `python/fruit_detector.py` (append `pixel_to_base_frame`)
- Modify: `python/test_fruit_detector.py` (append B7 section)

Given a pixel `(u, v)` and a fruit-top depth in mm, and the SessionCal, produce the fruit's XYZ in robot base frame. Applies parallax correction using the nadir-camera formula derived in spec §3.2, then applies H to go pixel → chess-XY-mm, then adds `chess_origin_in_base_m` to get base frame, with Z = table height + fruit_top_z.

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py`:

```python
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
    # Pixel-to-world (mm): world_xy = (pixel - principal) * (cam_h / focal)
    # Build H as a homogeneous matrix that does exactly that for z=1.
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
    # A pixel exactly at the principal point with 0 fruit height should
    # map to chess-frame origin.
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
    # Base-frame X should decrease (toward principal point's X = 0.30)
    # when the fruit is taller.
    assert xyz_flat[0] > xyz_tall[0] > 0.30, (
        f"flat={xyz_flat} tall={xyz_tall}")


def test_pixel_to_base_z_equals_origin_plus_fruit_top():
    from fruit_detector import pixel_to_base_frame
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    xyz = pixel_to_base_frame((640, 360), fruit_top_z_mm=45.0,
                               session_cal=cal)
    assert abs(xyz[2] - (0.02 + 0.045)) < 1e-6, xyz
```

Runner calls:

```python
    _section("B7 px->base principal point", test_pixel_to_base_identity_at_principal_point)
    _section("B7 px->base parallax", test_pixel_to_base_parallax_shrinks_with_fruit_height)
    _section("B7 px->base z", test_pixel_to_base_z_equals_origin_plus_fruit_top)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B7 sections FAIL — `cannot import name 'pixel_to_base_frame'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
def pixel_to_base_frame(center_px: tuple, fruit_top_z_mm: float,
                         session_cal) -> np.ndarray:
    """
    Convert a pixel + fruit-top height into a 3-vector XYZ in the robot
    base frame (metres).

    Applies a nadir-pinhole parallax correction so the returned XY is the
    fruit's BASE on the table (not the projection of its top). Then uses
    H_pixel_to_chess_mm to get chess-frame XY in mm and adds the stored
    chess origin to land in base frame.

    Assumes the camera is approximately nadir at session_cal.camera_height_above_table_m
    above the table. Error is <2% nadir, <15% at 30 deg tilt.
    """
    intr = session_cal.d415_intrinsics
    cx_p, cy_p = float(intr["cx"]), float(intr["cy"])
    h_mm = float(session_cal.camera_height_above_table_m) * 1000.0
    u, v = float(center_px[0]), float(center_px[1])
    # Parallax correction: scale pixel offset from principal point by
    # (h - z) / h to get where the BASE of the fruit projects.
    if h_mm <= 0:
        raise ValueError("camera height must be positive")
    z = float(fruit_top_z_mm)
    u_base = cx_p + (u - cx_p) * (h_mm - z) / h_mm
    v_base = cy_p + (v - cy_p) * (h_mm - z) / h_mm
    # Apply H (pixel -> chess-plane XY in mm).
    H = np.asarray(session_cal.h_pixel_to_chess_mm, dtype=np.float64)
    pt = np.array([u_base, v_base, 1.0])
    w = H @ pt
    if abs(w[2]) < 1e-9:
        raise ValueError("homography degenerate at this pixel")
    chess_x_mm = w[0] / w[2]
    chess_y_mm = w[1] / w[2]
    # Base-frame XYZ: origin + chess offset (mm->m) + fruit top z (mm->m).
    origin = np.asarray(session_cal.chess_origin_in_base_m, dtype=float)
    x = origin[0] + chess_x_mm / 1000.0
    y = origin[1] + chess_y_mm / 1000.0
    z_base = origin[2] + z / 1000.0
    return np.array([x, y, z_base], dtype=float)
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 19 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): pixel -> base-frame projection with parallax

D2-B7: pixel_to_base_frame applies the nadir-pinhole parallax formula
from spec §3.2 to report the fruit's BASE XY (not the tip's projection).
Applies session_cal's H to go pixel -> chess-XY-mm, adds chess_origin
(metres) + fruit_top_z (mm->m). Tested with a synthetic identity-scale
SessionCal: principal-point pixel lands at chess origin, off-axis
pixels shift toward the optical axis as fruit_top_z grows."
```

---

## Task B8: Top-level `detect_fruits` orchestrator

**Files:**
- Modify: `python/fruit_detector.py` (append `detect_fruits`)
- Modify: `python/test_fruit_detector.py` (append B8 section)

The orchestrator wires the per-class detectors + depth + projection together. Runs banana, tomato, strawberry detectors in order; each produces raw hits; for each hit, samples depth; for each valid depth, projects to base frame; packages into a `Detection`. Drops anything below `CONFIDENCE_MIN`.

- [ ] **Step 1: Add failing tests**

Append to `python/test_fruit_detector.py`:

```python
# ========================================================================
# B8. top-level detect_fruits
# ========================================================================
def _compose_test_scene():
    """Render a scene with one banana, one tomato, and one strawberry at
    known pixel locations. Return (bgr, depth, expected_pixels_by_type)."""
    import cv2
    bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.zeros((720, 1280), dtype=np.uint16)
    # Banana at (300, 360), yellow, elongated.
    cv2.rectangle(bgr, (240, 335), (360, 385), (0, 220, 220), -1)
    depth[325:395, 230:370] = 500  # uniform 500 mm to simplify
    # Tomato at (640, 360), round red.
    cv2.circle(bgr, (640, 360), 40, (0, 0, 220), -1)
    depth[315:405, 595:685] = 510
    # Strawberry at (960, 360), tapered red + calyx above.
    pts = np.array([[930, 320], [990, 320], [975, 400], [945, 400]],
                    dtype=np.int32)
    cv2.fillPoly(bgr, [pts], (0, 0, 220))
    cv2.rectangle(bgr, (930, 300), (990, 315), (0, 200, 0), -1)
    depth[310:410, 920:1000] = 505
    return bgr, depth, {
        "banana": (300, 360),
        "tomato": (640, 360),
        "strawberry": (960, 360),
    }


def test_detect_fruits_end_to_end():
    from fruit_detector import detect_fruits
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    bgr, depth, expected = _compose_test_scene()
    dets = detect_fruits(bgr, depth, cal)
    types = [d.fruit_type for d in dets]
    assert "banana" in types, f"no banana among {types}"
    assert "tomato" in types, f"no tomato among {types}"
    assert "strawberry" in types, f"no strawberry among {types}"
    # Each detection should have a valid base-frame XYZ.
    for d in dets:
        assert d.center_base_m.shape == (3,)
        assert np.isfinite(d.center_base_m).all()
        assert 0.0 < d.confidence <= 1.0


def test_detect_fruits_skips_invalid_depth():
    """Fruit blobs with no valid depth (all-zero patch) must be dropped."""
    from fruit_detector import detect_fruits
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    bgr, depth, _ = _compose_test_scene()
    # Zero out the depth channel entirely.
    depth = np.zeros_like(depth)
    dets = detect_fruits(bgr, depth, cal)
    assert len(dets) == 0, f"expected 0 detections with null depth, got {len(dets)}"


def test_detect_fruits_empty_scene():
    from fruit_detector import detect_fruits
    cal = _fake_session_cal(origin=(0.30, 0.10, 0.02), cam_h_m=0.60)
    bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.full((720, 1280), 500, dtype=np.uint16)
    dets = detect_fruits(bgr, depth, cal)
    assert dets == []
```

Runner calls:

```python
    _section("B8 detect end-to-end", test_detect_fruits_end_to_end)
    _section("B8 detect skips null depth", test_detect_fruits_skips_invalid_depth)
    _section("B8 detect empty scene", test_detect_fruits_empty_scene)
```

- [ ] **Step 2: Run to verify fails**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: B8 sections FAIL — `cannot import name 'detect_fruits'`.

- [ ] **Step 3: Implement — append to `python/fruit_detector.py`**

```python
CONFIDENCE_MIN = 0.35


def detect_fruits(color_bgr: np.ndarray, depth_mm: np.ndarray,
                   session_cal) -> list[Detection]:
    """
    Top-level detector. Returns every fruit found with a valid depth
    sample and a confidence >= CONFIDENCE_MIN.

    Processing order: banana -> tomato -> strawberry. This order does
    not matter for correctness — each detector runs independently on
    its own HSV mask — but keeps the returned list deterministic.
    """
    results: list[Detection] = []
    per_class = [
        ("banana", _detect_banana_contours),
        ("tomato", _detect_tomato_contours),
        ("strawberry", _detect_strawberry_contours),
    ]
    for fruit_type, fn in per_class:
        for (cx, cy), area, bbox, conf in fn(color_bgr):
            if conf < CONFIDENCE_MIN:
                continue
            top_z_mm = sample_depth_at_pixel(depth_mm, (cx, cy))
            if top_z_mm is None:
                continue
            try:
                xyz_base = pixel_to_base_frame(
                    (cx, cy), top_z_mm, session_cal)
            except ValueError:
                continue
            results.append(Detection(
                fruit_type=fruit_type,
                center_px=(int(cx), int(cy)),
                center_base_m=xyz_base,
                confidence=float(conf),
                area_px=int(area),
                bbox=tuple(int(v) for v in bbox),
            ))
    return results
```

- [ ] **Step 4: Run to verify passes**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 22 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/fruit_detector.py python/test_fruit_detector.py
git commit -m "feat(det): top-level detect_fruits orchestrator

D2-B8: detect_fruits wires the per-class detectors + depth sampling +
base-frame projection. Drops detections below CONFIDENCE_MIN=0.35
and drops any blob with no valid depth patch. End-to-end test renders
one banana + tomato + strawberry into a synthetic RGB+depth frame
and asserts all three are recovered with finite base-frame XYZ."
```

---

## Task B9: `test_integration.py` surgery

**Files:**
- Modify: `python/test_integration.py` (remove vision-related test sections)

The old `test_integration.py` imports `from fruit_detector import (detect_fruits, detection_depth_mm, CIRCULARITY_THRESH)`. The first symbol still exists (new signature); the other two are gone. It also tests FSM / trajectory / controller which do not depend on the new detector. Surgery:

1. Remove the `detection_depth_mm` and `CIRCULARITY_THRESH` imports.
2. Remove the old vision test sections (they test the legacy API and are superseded by `test_fruit_detector.py`).
3. Keep the FSM / trajectory / controller tests intact.

- [ ] **Step 1: Read `python/test_integration.py` fully.**

Before editing, skim the file to identify every section that uses legacy detector symbols. Expected legacy call sites: the import line at the top + any section-level tests that call `detect_fruits` with the 2-arg signature (color + depth only; no session_cal).

- [ ] **Step 2: Remove legacy imports and vision sections.**

Edit out:
- `from fruit_detector import (detect_fruits, detection_depth_mm, CIRCULARITY_THRESH)` → replace with the bare module import only if some non-vision test still uses it; otherwise delete entirely.
- Any `_section("... fruit detection ...", ...)` calls in the `__main__` block that feed a synthetic BGR+depth into the old 2-arg `detect_fruits` signature.
- Their corresponding `def test_...` function definitions.

Keep:
- `MockQArm` class.
- FSM / Controller / trajectory tests.
- The runner infrastructure (`_section`, `_RESULTS`, `if __name__ == "__main__":`).

- [ ] **Step 3: Run the trimmed test_integration to verify FSM tests still pass.**

Run: `py -3.13 python/test_integration.py`
Expected: exit 0. Fewer sections than before is EXPECTED (vision sections moved to `test_fruit_detector.py`).

- [ ] **Step 4: Confirm test_fruit_detector.py still passes.**

Run: `py -3.13 python/test_fruit_detector.py`
Expected: 22 sections PASS, exit 0.

- [ ] **Step 5: Commit**

```bash
git add python/test_integration.py
git commit -m "refactor(test): drop legacy fruit_detector tests from integration suite

D2-B9: the rewritten fruit_detector.py no longer exports
detection_depth_mm or CIRCULARITY_THRESH, and detect_fruits now
requires a SessionCal. Removed the vision sections from
test_integration.py; equivalent coverage now lives in
test_fruit_detector.py (22 sections, all offline). FSM / controller /
trajectory tests remain in test_integration.py as before."
```

---

## Done-criteria for D2

When all of the above commits are on `yichang_branch`:

1. `py -3.13 python/test_fruit_detector.py` returns exit code 0 with **22 PASS**:
   - B1 × 2 (Detection dataclass + to_dict)
   - B2 × 3 (HSV mask helper: single range + wrap-around + excludes out-of-range)
   - B3 × 3 (banana: finds + rejects round + rejects tiny)
   - B4 × 3 (tomato: finds + rejects green-above + rejects elongated)
   - B5 × 2 (strawberry: finds tapered+calyx + rejects no-calyx)
   - B6 × 3 (depth: median + sparsity reject + out-of-range reject)
   - B7 × 3 (pixel->base: principal point + parallax direction + z)
   - B8 × 3 (detect_fruits: end-to-end + null depth + empty scene)
2. `py -3.13 python/test_integration.py` still returns exit 0 (FSM / controller / trajectory intact; vision sections removed).
3. `python/fruit_detector.py` has been completely replaced; no legacy `FruitDetection` class or `detection_depth_mm` / `CIRCULARITY_THRESH` constants remain.
4. No hardware was touched during D2.

## Handoff to D3

The D3 plan (to be written) will add `picker_viewer.py`, `survey_capture.py`, and `pick_single` to `FruitSortingController`. D3 consumes the new `detect_fruits` and `Detection` from this plan plus the `SessionCal` from D1.

D3 is also offline-testable until the D4 lab session.

## Risks (D2-specific)

1. **Synthetic tests pass, lab tests may not.** HSV defaults are educated guesses; lab lighting may require significant retuning. Mitigated by D4-AM `hsv_tuner.py` (deferred task).
2. **Downstream fragility.** Other files in the repo still import `FruitDetection`, `detection_depth_mm`, or `CIRCULARITY_THRESH`: `main_final.py`, `preflight.py`, `hover_test.py`, `analyze_detections.py`, `analyze_static.py`, `test_auto_pick.py`, `recal_from_pose.py`, `main_autonomous.py`, plus the Simulink glue (`py_detect_test.m`, `py_detect_live.m`, `build_slice_vision.m`). These will break on import until D3 / D8 cleans them up. The smoke test for D2 is `test_fruit_detector.py` PLUS `test_integration.py` (both offline); other files are not expected to run between D2 and D3.
3. **`_taper_score` is hot-loop with a rasterised mask.** It allocates a mask per contour. For scenes with ~15 contours per frame at survey pose, this is negligible; if we later pick at higher frame rates we may need to optimise. Not blocking.
4. **Green-calyx detection is sensitive to the chosen band (10-20 px above the blob).** In the lab, the band may need to move up or down depending on the distance from the camera to the strawberry. Tunable via `_GREEN_ABOVE_BAND_PX`.
