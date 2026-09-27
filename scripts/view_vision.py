"""Read-only ROS camera/detection viewer, served on localhost for SSH forwarding."""

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
from booster_deploy.utils.vision_config import load_vision_config, camera_topics
from booster_deploy.utils.vision_view import choose_frame, decode_image, draw_detections


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>K1 相机与识别画面</title><style>
body{font:16px system-ui;background:#14212d;color:#eee;margin:24px auto;max-width:1100px;padding:0 16px}
h1{font-size:24px}img{width:100%;max-height:78vh;object-fit:contain;background:#080e14}
button{padding:8px 16px;margin:0 12px 12px 0;cursor:pointer}#status{line-height:1.6;margin:12px 0}
</style><h1>K1 相机与识别画面 · 只读</h1>
<button id="toggle" onclick="toggle()">隐藏识别框</button>
<span>绿色：球；蓝色：其他目标。球框显示置信度及机器人坐标（米）。</span>
<div id="status">正在连接…</div><img id="video" src="/stream.mjpg">
<script>
let boxes=true;
function toggle(){boxes=!boxes;document.getElementById('video').src='/stream.mjpg?raw='+(boxes?'0':'1');
document.getElementById('toggle').textContent=boxes?'隐藏识别框':'显示识别框';}
async function update(){try{const r=await fetch('/status',{cache:'no-store'});const s=await r.json();
const age=x=>x===null?'尚未收到':x.toFixed(2)+' 秒前';
document.getElementById('status').textContent='图像：'+age(s.image_age)+' ｜ 检测：'+age(s.detection_age)+
' ｜ '+(s.matched?'识别框已按时间戳匹配':'当前无匹配识别框')+(s.error?' ｜ '+s.error:'');}
catch(e){document.getElementById('status').textContent='连接已断开，请检查机器人上的查看程序和 SSH 转发。';}}
setInterval(update,1000);update();</script></html>"""


class Viewer:
    def __init__(self, fps, quality):
        self.fps, self.quality = fps, quality
        self.lock = threading.Condition()
        self.stop = threading.Event()
        self.frames = deque(maxlen=30)
        self.detection = None
        self.image_received = self.detection_received = 0.
        self.jpegs = (None, None)
        self.sequence = 0
        self.matched = False
        self.error = ""

    def on_image(self, msg):
        with self.lock:
            self.frames.append(msg)
            self.image_received = time.monotonic()

    def on_detection(self, msg):
        with self.lock:
            self.detection = msg
            self.detection_received = time.monotonic()

    def status(self):
        with self.lock:
            now = time.monotonic()
            return {"image_age": now-self.image_received if self.image_received else None,
                    "detection_age": now-self.detection_received if self.detection_received else None,
                    "matched": self.matched and now-self.detection_received < .5,
                    "error": self.error}

    def render(self):
        while not self.stop.is_set():
            start = time.monotonic()
            with self.lock:
                frame, detection = choose_frame(list(self.frames), self.detection,
                                                 start, self.detection_received)
            if frame is not None:
                try:
                    raw = decode_image(frame)
                    overlay = draw_detections(raw, detection)
                    encoded = []
                    for picture in (overlay, raw):
                        ok, jpg = cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                        if not ok:
                            raise ValueError('JPEG encoding failed')
                        encoded.append(jpg.tobytes())
                    with self.lock:
                        self.jpegs = tuple(encoded)
                        self.sequence += 1
                        self.matched = detection is not None
                        self.error = ""
                        self.lock.notify_all()
                except (ValueError, cv2.error) as exc:
                    with self.lock:
                        self.error = str(exc)
            self.stop.wait(max(0., 1/self.fps - (time.monotonic()-start)))


def handler_for(viewer):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/status'):
                data = PAGE.encode() if path == '/' else json.dumps(viewer.status()).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8' if path == '/' else 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)
            elif path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                raw = 'raw=1' in self.path
                previous = -1
                self.connection.settimeout(5.)
                try:
                    while not viewer.stop.is_set():
                        with viewer.lock:
                            viewer.lock.wait_for(lambda: viewer.sequence != previous or viewer.stop.is_set(), timeout=1.)
                            if viewer.sequence == previous:
                                continue
                            previous = viewer.sequence
                            jpg = viewer.jpegs[int(raw)]
                        if jpg is not None:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                                             str(len(jpg)).encode()+b'\r\n\r\n'+jpg+b'\r\n')
                            self.wfile.flush()
                except (OSError, TimeoutError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision-config', default='/opt/booster')
    parser.add_argument('--image-topic', help='Override camera topic, e.g. for RealSense')
    parser.add_argument('--detection-topic', default='/booster_vision/detection')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--fps', type=float, default=10.)
    parser.add_argument('--quality', type=int, default=80)
    args = parser.parse_args()
    if not (1 <= args.fps <= 30 and 1 <= args.quality <= 100 and 1 <= args.port <= 65535):
        parser.error('Require fps=1..30, quality=1..100 and port=1..65535')
    topic = args.image_topic or camera_topics(load_vision_config(args.vision_config))[0]
    viewer = Viewer(args.fps, args.quality)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(viewer))
    server.daemon_threads = True
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from vision_interface.msg import Detections
    rclpy.init()
    node = rclpy.create_node('k1_readonly_vision_viewer')
    node.create_subscription(Image, topic, viewer.on_image, qos_profile_sensor_data)
    node.create_subscription(Detections, args.detection_topic, viewer.on_detection, qos_profile_sensor_data)
    def spin():
        try:
            while not viewer.stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
        except ExternalShutdownException:
            pass
    threads = [threading.Thread(target=spin, daemon=True),
               threading.Thread(target=viewer.render, daemon=True)]
    for thread in threads:
        thread.start()
    print(f'Read-only viewer: http://127.0.0.1:{args.port} | camera={topic}', flush=True)
    print(f'On your computer: ssh -N -L {args.port}:127.0.0.1:{args.port} booster@ROBOT_IP', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.stop.set()
        with viewer.lock:
            viewer.lock.notify_all()
        server.server_close()
        for thread in threads:
            thread.join(timeout=2.)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
