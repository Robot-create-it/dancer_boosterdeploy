from types import SimpleNamespace as NS
import unittest

try:
    import cv2
except ModuleNotFoundError:
    raise unittest.SkipTest('Viewer tests require system OpenCV; run with /usr/bin/python3')
import numpy as np

from booster_deploy.utils.vision_view import decode_image, choose_frame, draw_detections


def stamped(t):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9))))


class VisionViewTests(unittest.TestCase):
    def test_nv12_with_row_padding(self):
        planes = np.array([[80, 90], [100, 110], [128, 128]], dtype=np.uint8)
        padded = np.pad(planes, ((0, 0), (0, 2)), constant_values=255)
        msg = NS(width=2, height=2, step=4, encoding='nv12', data=padded.tobytes())
        np.testing.assert_array_equal(decode_image(msg), cv2.cvtColor(planes, cv2.COLOR_YUV2BGR_NV12))
        msg.data = b''
        with self.assertRaises(ValueError):
            decode_image(msg)

    def test_rgb_to_bgr_with_row_padding(self):
        msg = NS(width=1, height=1, step=4, encoding='rgb8', data=bytes([255, 0, 0, 99]))
        self.assertEqual(decode_image(msg).tolist(), [[[0, 0, 255]]])

    def test_timestamp_alignment_and_stale_detection(self):
        old, current = stamped(10.), stamped(10.2)
        det = stamped(10.)
        self.assertEqual(choose_frame([old, current], det, 50., 49.9), (old, det))
        self.assertEqual(choose_frame([old, current], det, 50., 49.), (current, None))
        self.assertEqual(choose_frame([current], det, 50., 49.9), (current, None))

    def test_overlay_does_not_modify_original(self):
        raw = np.zeros((100, 200, 3), dtype=np.uint8)
        det = NS(detected_objects=[NS(label='Ball', confidence=65., xmin=20, ymin=40,
                                      xmax=60, ymax=80, position_projection=[1., 2.])])
        overlay = draw_detections(raw, det)
        self.assertGreater(overlay.sum(), 0)
        self.assertEqual(raw.sum(), 0)


if __name__ == '__main__':
    unittest.main()
