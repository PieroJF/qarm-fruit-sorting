"""
Operator-facing picker UI: click-to-pick + category-batch autonomous loop.

See docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md §3.5.

Event loop lives in run_picker_loop(). Everything above it is pure and
unit-tested.
"""
from __future__ import annotations
import os
import sys

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fruit_detector import Detection

_TYPE_COLORS = {
    "banana":     (0, 255, 255),    # yellow (BGR)
    "tomato":     (0, 0, 255),      # red
    "strawberry": (200, 100, 255),  # pink-magenta
}
_WARN_COLOR = (0, 180, 220)         # amber
_HUD_COLOR = (255, 255, 255)
_RESIDUAL_OK = (0, 200, 0)
_RESIDUAL_WARN = (0, 180, 220)
_RESIDUAL_BAD = (0, 0, 255)

CLICK_MAX_R_PX = 50


def _nearest_detection(detections, click_xy, max_r_px=CLICK_MAX_R_PX):
    """Return the detection with smallest pixel distance to click_xy,
    within max_r_px. None if no detection qualifies."""
    if not detections:
        return None
    cx, cy = float(click_xy[0]), float(click_xy[1])
    best = None
    best_d2 = (max_r_px + 1) ** 2
    for d in detections:
        dx = d.center_px[0] - cx
        dy = d.center_px[1] - cy
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best = d
    if best_d2 > max_r_px * max_r_px:
        return None
    return best


def _filter_by_type(detections, fruit_type):
    """Keep only detections whose fruit_type matches."""
    return [d for d in detections if d.fruit_type == fruit_type]


def _residual_color(residual_mm):
    if residual_mm is None or residual_mm < 3.0:
        return _RESIDUAL_OK
    if residual_mm < 10.0:
        return _RESIDUAL_WARN
    return _RESIDUAL_BAD


def _annotate(color_bgr, detections, residual_mm, warnings):
    """Draw bboxes + labels + HUD on a copy of color_bgr."""
    out = color_bgr.copy()
    for d in detections:
        color = _TYPE_COLORS.get(d.fruit_type, (255, 255, 255))
        x, y, w, h = d.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        label = f"{d.fruit_type} {d.confidence:.2f}"
        cv2.putText(out, label, (x, max(18, y - 6)),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        cv2.circle(out, tuple(int(v) for v in d.center_px), 4, color, -1)
    hud = _hud_text(len(detections), residual_mm, "idle")
    cv2.putText(out, hud, (10, 28),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, _HUD_COLOR, 2, cv2.LINE_AA)
    # residual chip (top-right)
    if residual_mm is not None:
        rc = _residual_color(residual_mm)
        cv2.circle(out, (out.shape[1] - 30, 28), 10, rc, -1)
    for i, w in enumerate(warnings or []):
        cv2.putText(out, w, (10, 56 + i * 24),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, _WARN_COLOR, 1,
                     cv2.LINE_AA)
    return out


def _hud_text(n_fruits, residual_mm, mode):
    r = f"{residual_mm:.1f}" if residual_mm is not None else "n/a"
    return (f"{n_fruits} fruits  |  residual {r} mm  |  "
            f"mode: {mode}  |  click / b t s / r / ESC")
