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
