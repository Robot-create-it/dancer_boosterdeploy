"""Image decoding and timestamp matching for the read-only browser viewer."""

import cv2
import numpy as np


def image_stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def decode_image(message):
    """Decode ROS color images, including the K1 camera's padded NV12 rows."""
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0:
        raise ValueError("Empty camera image")
    encoding = message.encoding.lower()
    data = np.frombuffer(message.data, dtype=np.uint8)
    if encoding == "nv12":
        if width % 2 or height % 2 or step < width:
            raise ValueError("Invalid NV12 dimensions/stride")
        rows = height * 3 // 2
        if data.size < rows * step:
            raise ValueError("Truncated NV12 image")
        planes = data[:rows * step].reshape(rows, step)[:, :width].copy()
        return cv2.cvtColor(planes, cv2.COLOR_YUV2BGR_NV12)
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {encoding}")
    if step < width * channels or data.size < height * step:
        raise ValueError("Invalid image data/stride")
    image = data[:height * step].reshape(height, step)[:, :width * channels].copy()
    image = image.reshape(height, width, channels)
    conversions = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR,
                   "bgra8": cv2.COLOR_BGRA2BGR, "mono8": cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(image, conversions[encoding]) if encoding in conversions else image


def choose_frame(frames, detection, now, detection_received, tolerance=0.05):
    """Return (image, matched detection); never draw old boxes on a new frame."""
    if not frames:
        return None, None
    if detection is not None and 0 <= now - detection_received < 0.5:
        closest = min(frames, key=lambda frame: abs(image_stamp(frame) - image_stamp(detection)))
        if abs(image_stamp(closest) - image_stamp(detection)) <= tolerance:
            return closest, detection
    return frames[-1], None


def draw_detections(image, detection):
    output = image.copy()
    if detection is None:
        return output
    height, width = output.shape[:2]
    for obj in detection.detected_objects:
        if not np.isfinite(obj.confidence):
            continue
        x0, x1 = [max(0, min(width - 1, int(v))) for v in (obj.xmin, obj.xmax)]
        y0, y1 = [max(0, min(height - 1, int(v))) for v in (obj.ymin, obj.ymax)]
        if x1 <= x0 or y1 <= y0:
            continue
        color = (0, 255, 0) if obj.label == "Ball" else (255, 180, 80)
        text = f"{obj.label} {obj.confidence:.0f}%"
        if obj.label == "Ball" and len(obj.position_projection) >= 2:
            x, y = obj.position_projection[:2]
            text += f" ({x:.2f},{y:.2f})m"
        cv2.rectangle(output, (x0, y0), (x1, y1), color, 2)
        cv2.putText(output, text, (x0, max(14, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(output, text, (x0, max(14, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
    return output
