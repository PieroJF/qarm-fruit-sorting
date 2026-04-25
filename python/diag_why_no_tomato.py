"""
One-shot diagnosis: load the last diag_detector_raw.png, run the tomato
HSV+shape gates by hand, print which gate each red contour fails.
"""
import os, sys
import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fruit_detector import (
    HSV_RANGES, hsv_mask, _has_green_above, _has_green_in_top_strip,
    _TOMATO_MIN_AREA, _TOMATO_MAX_AREA,
    _TOMATO_MIN_CIRCULARITY, _TOMATO_MAX_ASPECT,
    _GREEN_ABOVE_RATIO, _STRAWBERRY_MIN_CALYX,
)

repo = os.path.dirname(_HERE)
raw_path = os.path.join(repo, "logs", "diag_detector_raw.png")
img = cv2.imread(raw_path)
if img is None:
    sys.exit(f"missing {raw_path} — run diag_detector.py first")

mask = hsv_mask(img, HSV_RANGES["tomato"])
cv2.imwrite(os.path.join(repo, "logs", "debug_tomato_mask.png"), mask)

contours, _ = cv2.findContours(
    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f"found {len(contours)} red contour(s)\n")

overlay = img.copy()
for i, c in enumerate(contours):
    area = cv2.contourArea(c)
    perim = cv2.arcLength(c, True)
    x, y, w, h = cv2.boundingRect(c)
    circ = 4 * np.pi * area / (perim * perim) if perim > 0 else 0.0
    aspect = max(w, h) / max(1, min(w, h))
    ga = _has_green_above(img, (x, y, w, h))
    gt = _has_green_in_top_strip(img, (x, y, w, h))
    fails = []
    if area < _TOMATO_MIN_AREA:        fails.append(f"AREA<{_TOMATO_MIN_AREA}")
    if area > _TOMATO_MAX_AREA:        fails.append(f"AREA>{_TOMATO_MAX_AREA}")
    if circ < _TOMATO_MIN_CIRCULARITY: fails.append(f"CIRC<{_TOMATO_MIN_CIRCULARITY}")
    if aspect > _TOMATO_MAX_ASPECT:    fails.append(f"ASPECT>{_TOMATO_MAX_ASPECT}")
    if ga > _GREEN_ABOVE_RATIO:        fails.append(f"GREEN_ABOVE>{_GREEN_ABOVE_RATIO}")
    verdict = "REJECT(" + ",".join(fails) + ")" if fails else "ACCEPT as tomato"
    straw_ok = (max(ga, gt) > _STRAWBERRY_MIN_CALYX)
    print(f"  [{i}] area={area:6.0f}  bbox=({x:4d},{y:4d},{w:3d},{h:3d})  "
          f"circ={circ:.3f}  aspect={aspect:.2f}  "
          f"green_above={ga:.3f}  top_strip={gt:.3f}")
    print(f"      tomato: {verdict}")
    print(f"      strawberry calyx gate: "
          f"{'PASS' if straw_ok else 'FAIL'} "
          f"(needs max(green_above, top_strip) > {_STRAWBERRY_MIN_CALYX})\n")
    col = (0, 255, 0) if not fails else (0, 0, 255)
    cv2.rectangle(overlay, (x, y), (x + w, y + h), col, 2)
    cv2.putText(overlay, f"#{i}", (x, max(20, y - 6)),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)

out_path = os.path.join(repo, "logs", "debug_tomato_overlay.png")
cv2.imwrite(out_path, overlay)
print(f"mask    saved -> logs/debug_tomato_mask.png")
print(f"overlay saved -> logs/debug_tomato_overlay.png "
      f"(green=accept, red=reject)")
