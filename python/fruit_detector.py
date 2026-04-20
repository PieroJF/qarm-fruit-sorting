"""
Fruit detection and classification using HSV color segmentation.
Detects strawberries (red/elongated), tomatoes (red/round), and bananas (yellow).
"""

import numpy as np
import cv2


# HSV ranges (OpenCV convention: H=0-179, S=0-255, V=0-255)
# Tuned for lab lighting — wider ranges to catch washed-out / bright fruit.
HSV_RANGES = {
    'banana': {
        'lower': np.array([20, 70, 100]),
        'upper': np.array([35, 255, 255]),
    },
    'red': {  # Shared by strawberry and tomato
        # Range 1: red-to-orange (covers bright strawberries under lab light)
        'lower1': np.array([0, 50, 50]),
        'upper1': np.array([15, 255, 255]),
        # Range 2: deep red / magenta wrap-around
        'lower2': np.array([165, 50, 50]),
        'upper2': np.array([179, 255, 255]),
    },
}

MIN_AREA = 300           # Minimum blob area in pixels (lowered for distant fruit)
CIRCULARITY_THRESH = 0.75  # Above = tomato, below = strawberry. Raised
                           # from 0.65 because small strawberries measured
                           # ~0.70 in lab lighting; real tomatoes sit at
                           # 0.85-0.95 so margin stays healthy.
SATURATION_THRESH = 140    # Higher saturation = tomato


class FruitDetection:
    """Result of a single fruit detection."""
    def __init__(self, fruit_type, centroid, bbox, area, confidence,
                 contour=None):
        self.fruit_type = fruit_type
        self.centroid = centroid        # (row, col) in pixels
        self.bbox = bbox                # (x, y, w, h)
        self.area = area
        self.confidence = confidence
        self.contour = contour          # cv2 contour array (optional)

    def __repr__(self):
        return (f"FruitDetection('{self.fruit_type}', "
                f"centroid={self.centroid}, conf={self.confidence:.2f})")


# Plausible workspace depth band from the camera (lab setup, D415 at the
# pickhome1 pose). Values outside this band are IR shadow or background.
DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 700


def detection_depth_mm(det, depth_image,
                       dmin=DEPTH_MIN_MM, dmax=DEPTH_MAX_MM):
    """Robust depth in mm for a detection using the blob's contour interior.

    Builds a mask from det.contour (falls back to bbox if no contour),
    intersects with valid depth pixels in [dmin, dmax], and returns the
    median. Returns None if not enough valid pixels remain.
    """
    if depth_image is None:
        return None
    h, w = depth_image.shape[:2]

    mask = np.zeros((h, w), dtype=np.uint8)
    if det.contour is not None:
        cv2.drawContours(mask, [det.contour], -1, 255, -1)
    else:
        x, y, bw, bh = det.bbox
        x0 = max(0, x); y0 = max(0, y)
        x1 = min(w, x + bw); y1 = min(h, y + bh)
        mask[y0:y1, x0:x1] = 255

    depth_masked = depth_image[mask > 0]
    valid = depth_masked[(depth_masked >= dmin) & (depth_masked <= dmax)]
    if valid.size < 20:
        return None
    return float(np.median(valid))


def detect_fruits(bgr_image, depth_image=None, min_area=MIN_AREA):
    """
    Detect and classify fruits in an image.

    Parameters
    ----------
    bgr_image : np.ndarray
        HxWx3 BGR image (OpenCV format, as captured by Video3D).
    depth_image : np.ndarray, optional
        HxW depth map (uint16, mm).
    min_area : int
        Minimum blob area to consider.

    Returns
    -------
    list[FruitDetection]
        List of detected fruits with type, centroid, and bounding box.
    """
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    detections = []

    # --- Banana detection (yellow) ---
    banana_mask = cv2.inRange(hsv, HSV_RANGES['banana']['lower'],
                              HSV_RANGES['banana']['upper'])
    banana_mask = _clean_mask(banana_mask, min_area)
    _extract_blobs(banana_mask, hsv, 'banana', detections, min_area)

    # --- Red fruit detection (strawberry + tomato) ---
    red_mask1 = cv2.inRange(hsv, HSV_RANGES['red']['lower1'],
                            HSV_RANGES['red']['upper1'])
    red_mask2 = cv2.inRange(hsv, HSV_RANGES['red']['lower2'],
                            HSV_RANGES['red']['upper2'])
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    red_mask = _clean_mask(red_mask, min_area)
    _extract_red_blobs(red_mask, hsv, detections, min_area)

    return detections


def _clean_mask(mask, min_area):
    """Morphological cleanup."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Remove small blobs
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    clean = np.zeros_like(mask)
    for c in contours:
        if cv2.contourArea(c) >= min_area:
            cv2.drawContours(clean, [c], -1, 255, -1)
    return clean


def _extract_blobs(mask, hsv, fruit_type, detections, min_area):
    """Extract blob detections from a binary mask."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        x, y, w, h = cv2.boundingRect(c)
        conf = min(1.0, area / 5000)

        detections.append(FruitDetection(
            fruit_type=fruit_type,
            centroid=(cy, cx),  # (row, col)
            bbox=(x, y, w, h),
            area=area,
            confidence=conf,
            contour=c,
        ))


def _extract_red_blobs(mask, hsv, detections, min_area):
    """Extract and classify red blobs as strawberry or tomato."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue

        # Circularity: 1.0 = perfect circle
        circularity = 4 * np.pi * area / (perimeter ** 2)

        # Mean saturation in this blob
        blob_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(blob_mask, [c], -1, 255, -1)
        mean_sat = cv2.mean(hsv[:, :, 1], mask=blob_mask)[0]

        # Classify: round + high saturation = tomato, else strawberry
        if circularity > CIRCULARITY_THRESH and mean_sat > SATURATION_THRESH:
            fruit_type = 'tomato'
        else:
            fruit_type = 'strawberry'

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(c)
        conf = min(1.0, area / 4000)

        detections.append(FruitDetection(
            fruit_type=fruit_type,
            centroid=(cy, cx),
            bbox=(x, y, w, h),
            area=area,
            confidence=conf,
            contour=c,
        ))


FRUIT_TYPE_ID = {'strawberry': 1, 'tomato': 2, 'banana': 3}
N_MAX_DEFAULT = 16  # max rows in the fixed-size matrix returned to MATLAB


def detect_and_project(bgr_image, depth_image, intrinsics, T_cam_to_base,
                        n_max=N_MAX_DEFAULT):
    """Detect fruits in a single BGR+depth frame, project each to the robot
    base frame, and return a fixed-shape matrix suitable for a Simulink
    MATLAB Function block.

    Returns
    -------
    out : np.ndarray, shape (n_max, 5), dtype float64
        Each row is [type_id, wx, wy, wz, confidence]. type_id:
        1=strawberry, 2=tomato, 3=banana, 0=empty. Unused rows are zeros.
    count : int
        Number of valid detections (0 <= count <= n_max).

    Detections with no plausible depth (blob median outside [200,700] mm)
    are silently skipped so the MATLAB side doesn't get garbage positions.
    """
    out = np.zeros((int(n_max), 5), dtype=np.float64)
    if bgr_image is None or depth_image is None:
        return out, 0
    fx = float(intrinsics['fx']); fy = float(intrinsics['fy'])
    cx = float(intrinsics['cx']); cy = float(intrinsics['cy'])
    T = np.asarray(T_cam_to_base, dtype=float).reshape(4, 4)

    dets = detect_fruits(bgr_image, depth_image)
    i = 0
    for d in dets:
        if i >= n_max:
            break
        dmm = detection_depth_mm(d, depth_image)
        if dmm is None:
            continue
        row, col = d.centroid
        Z = dmm / 1000.0
        X = (col - cx) * Z / fx
        Y = (row - cy) * Z / fy
        p_cam = np.array([X, Y, Z, 1.0])
        p_base = T @ p_cam
        tid = FRUIT_TYPE_ID.get(d.fruit_type, 0)
        out[i, :] = [tid, p_base[0], p_base[1], p_base[2],
                     float(d.confidence)]
        i += 1
    return out, i


def draw_detections(bgr_image, detections):
    """Draw bounding boxes and labels on an image (for visualization)."""
    colors = {
        'strawberry': (0, 0, 255),   # Red
        'tomato': (255, 0, 255),     # Magenta
        'banana': (0, 255, 255),     # Yellow
    }
    img = bgr_image.copy()
    for det in detections:
        x, y, w, h = det.bbox
        color = colors.get(det.fruit_type, (255, 255, 255))
        cv2.rectangle(img, (x, y), (x+w, y+h), color, 2)
        label = f"{det.fruit_type} ({det.confidence:.0%})"
        cv2.putText(img, label, (x, y-8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
    return img
