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
