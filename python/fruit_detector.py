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
