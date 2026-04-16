"""
Fruit detection and classification using HSV color segmentation.
Detects strawberries (red/elongated), tomatoes (red/round), and bananas (yellow).
"""

import numpy as np
import cv2


# HSV ranges (OpenCV convention: H=0-179, S=0-255, V=0-255)
HSV_RANGES = {
    'banana': {
        'lower': np.array([20, 100, 120]),
        'upper': np.array([35, 255, 255]),
    },
    'red': {  # Shared by strawberry and tomato
        'lower1': np.array([0, 100, 80]),
        'upper1': np.array([10, 255, 255]),
        'lower2': np.array([170, 100, 80]),
        'upper2': np.array([179, 255, 255]),
    },
}

MIN_AREA = 500           # Minimum blob area in pixels
CIRCULARITY_THRESH = 0.65  # Above = tomato, below = strawberry
SATURATION_THRESH = 140    # Higher saturation = tomato


class FruitDetection:
    """Result of a single fruit detection."""
    def __init__(self, fruit_type, centroid, bbox, area, confidence):
        self.fruit_type = fruit_type
        self.centroid = centroid        # (row, col) in pixels
        self.bbox = bbox                # (x, y, w, h)
        self.area = area
        self.confidence = confidence

    def __repr__(self):
        return (f"FruitDetection('{self.fruit_type}', "
                f"centroid={self.centroid}, conf={self.confidence:.2f})")


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
            confidence=conf
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
            confidence=conf
        ))


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
