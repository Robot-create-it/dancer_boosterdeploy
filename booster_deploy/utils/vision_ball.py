"""Extract the robot-frame ball position from RoboCup demo detections."""

import math
from dataclasses import dataclass


class MotionBallMemory:
    """Last valid raw XY for a kick actor, matching demo SimMotion's latch.

    Missing observations do not expire the reference. Keep this separate from
    fresh BallObservation/bbox data used by the head tracker. Store coordinates
    before model offsets/normalisation so those are applied only once per frame.
    """

    def __init__(self):
        self.position: tuple[float, float] | None = None

    def update(self, ball: tuple[float, float] | None):
        if ball is not None and all(math.isfinite(value) for value in ball):
            self.position = (float(ball[0]), float(ball[1]))
        return self.position


@dataclass(frozen=True)
class BallObservation:
    x: float
    y: float
    confidence: float
    bbox: tuple[float, float, float, float] | None
    stamp: float  # monotonic time corresponding to image acquisition
    image_stamp: float


def select_ball_observation(detections, received_at: float, ros_now: float,
                            max_age: float = 0.5):
    """Select one ball for both the policy and tracker; reject stale images."""
    header = detections.header.stamp
    image_stamp = float(header.sec) + float(header.nanosec) * 1e-9
    age = ros_now - image_stamp
    if not math.isfinite(age) or image_stamp <= 0 or age < -0.05 or age > max_age:
        return None
    obj = _select_object(detections)
    if obj is None:
        return None
    bbox = tuple(float(getattr(obj, name, float("nan")))
                 for name in ("xmin", "ymin", "xmax", "ymax"))
    if not all(map(math.isfinite, bbox)) or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        bbox = None
    return BallObservation(float(obj.position_projection[0]),
                           float(obj.position_projection[1]), float(obj.confidence),
                           bbox, received_at - max(0.0, age), image_stamp)


def select_ball(detections, min_confidence: float = 60.0):
    """Return (x, y) in metres from the best valid ``Ball``, or None."""
    obj = _select_object(detections, min_confidence)
    return tuple(map(float, obj.position_projection[:2])) if obj is not None else None


def _select_object(detections, min_confidence=60.0):
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
        best, best_confidence = obj, obj.confidence
    return best
