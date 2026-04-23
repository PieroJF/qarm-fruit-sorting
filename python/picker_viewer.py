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


# ------------------------------------------------------------------------
# Pick dispatchers
# ------------------------------------------------------------------------

MAX_CATEGORY_PICKS = 20
RETRY_LIMIT_PER_TARGET = 2


def _pick_one(controller, detection) -> bool:
    """Dispatch one synchronous pick. Returns True on success."""
    print(f"  [picker] picking {detection.fruit_type} at "
          f"{detection.center_base_m.round(3)} "
          f"(conf={detection.confidence:.2f})")
    try:
        return bool(controller.pick_single(
            detection.center_base_m, detection.fruit_type))
    except Exception as ex:
        print(f"  [picker] pick_single raised: {ex}")
        return False


def _nearest_to_home(detections, home_xy_m):
    if not detections:
        return None
    hx, hy = float(home_xy_m[0]), float(home_xy_m[1])
    return min(detections, key=lambda d: (
        (d.center_base_m[0] - hx) ** 2 +
        (d.center_base_m[1] - hy) ** 2))


def _pixel_key(detection, bucket_px=5):
    cx, cy = detection.center_px
    return (int(cx) // bucket_px * bucket_px,
            int(cy) // bucket_px * bucket_px)


def _pick_category(driver, camera, session_cal, controller,
                     fruit_type, should_abort_fn):
    """Category batch: re-capture, pick nearest-to-home, repeat until
    none left, limit hit, or abort signalled. Returns count picked."""
    from survey_capture import capture_fruits
    picks_done = 0
    retries = {}
    skipped_keys = set()
    while picks_done < MAX_CATEGORY_PICKS:
        # Service OpenCV events so ESC can abort between picks (spec §7.5).
        if (cv2.waitKey(1) & 0xFF) == 27:
            print("  [picker] ESC pressed — aborting batch")
            return picks_done
        if should_abort_fn():
            print("  [picker] batch aborted")
            break
        try:
            dets, diag = capture_fruits(driver, camera, session_cal)
        except Exception as ex:
            print(f"  [picker] capture failed mid-batch: {ex}")
            break
        matches = [d for d in _filter_by_type(dets, fruit_type)
                   if _pixel_key(d) not in skipped_keys]
        if not matches:
            break
        target = _nearest_to_home(matches, controller.HOME_POS[:2])
        key = _pixel_key(target)
        success = _pick_one(controller, target)
        if success:
            picks_done += 1
            retries.pop(key, None)
        else:
            retries[key] = retries.get(key, 0) + 1
            if retries[key] >= RETRY_LIMIT_PER_TARGET:
                print(f"  [picker] marking stuck target {key} as skipped")
                skipped_keys.add(key)
    return picks_done


# ------------------------------------------------------------------------
# Event loop
# ------------------------------------------------------------------------

def run_picker_loop(driver, camera, session_cal, controller) -> None:
    """Interactive OpenCV picker. Blocks until ESC.

    Parameters
    ----------
    driver     : QArmDriver (connected)
    camera     : QArmCamera (opened)
    session_cal: SessionCal
    controller : FruitSortingController
    """
    from survey_capture import capture_fruits
    window = "Picker"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)

    state = {"click": None, "abort": False}

    def _on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    cv2.setMouseCallback(window, _on_mouse)

    def _refresh():
        dets, diag = capture_fruits(driver, camera, session_cal)
        frame = _annotate(diag["color_frame"], dets,
                            diag.get("chessboard_residual_mm"),
                            diag.get("warnings", []))
        cv2.imshow(window, frame)
        return dets, diag

    try:
        dets, diag = _refresh()
    except Exception as ex:
        print(f"  [picker] initial capture failed: {ex}")
        cv2.destroyWindow(window)
        return

    while True:
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            print("  [picker] ESC — exiting")
            break
        if state["click"] is not None:
            click = state["click"]; state["click"] = None
            target = _nearest_detection(dets, click)
            if target is None:
                print(f"  [picker] no fruit near {click}")
            else:
                _pick_one(controller, target)
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key in (ord('b'), ord('t'), ord('s')):
            ftype = {ord('b'): 'banana',
                       ord('t'): 'tomato',
                       ord('s'): 'strawberry'}[key]
            matches = _filter_by_type(dets, ftype)
            if not matches:
                print(f"  [picker] no {ftype} visible")
            else:
                print(f"  [picker] category batch: {ftype}")
                n = _pick_category(driver, camera, session_cal,
                                    controller, ftype,
                                    lambda: state["abort"])
                print(f"  [picker] {ftype} batch done: {n} picked")
                try:
                    dets, diag = _refresh()
                except Exception as ex:
                    print(f"  [picker] re-capture failed: {ex}")
        elif key == ord('r'):
            print("  [picker] refresh")
            try:
                dets, diag = _refresh()
            except Exception as ex:
                print(f"  [picker] re-capture failed: {ex}")

    cv2.destroyWindow(window)
