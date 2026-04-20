"""
Grab one frame from the UGreen camera (DirectShow idx=3, MSMF backend).
Used for gripper/fruit analysis and calibration loops.
"""
import os
import sys
import cv2

UGREEN_IDX = 3
DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "logs",
    "ugreen_latest.png",
)


def capture(path=None, width=1280, height=720, warmup=10):
    path = path or DEFAULT_OUT
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cap = cv2.VideoCapture(UGREEN_IDX, cv2.CAP_MSMF)
    if not cap.isOpened():
        raise RuntimeError("UGreen camera did not open (idx=3 MSMF)")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    frame = None
    for _ in range(warmup):
        ok, f = cap.read()
        if ok and f is not None and f.size > 0 and f.mean() > 5:
            frame = f
    cap.release()
    if frame is None:
        raise RuntimeError("UGreen opened but no valid frame after warmup")
    cv2.imwrite(path, frame)
    return path, frame


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else None
    path, frame = capture(out)
    print(f"saved {frame.shape} mean={frame.mean():.1f} -> {path}")
