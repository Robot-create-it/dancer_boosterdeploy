"""Extract the robot-frame ball position from RoboCup demo detections."""

import math


def select_ball(detections, min_confidence: float = 60.0):
    """Return (x, y) in metres from the best valid ``Ball``, or None."""
    best = None
    best_confidence = min_confidence
    for obj in detections.detected_objects:
        if obj.label != "Ball" or not math.isfinite(obj.confidence):
            continue
        if obj.confidence < best_confidence or len(obj.position_projection) < 2:
            continue
        x, y = map(float, obj.position_projection[:2])
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        # Match Brain::detectProcessBalls' forward-distance filter.
        if x < -0.5 or x > 15.0:
            continue
        best, best_confidence = (x, y), obj.confidence
    return best
