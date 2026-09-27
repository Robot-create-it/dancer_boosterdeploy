"""Demo CamTrackBall/CamFindBall adapted to deploy's joint target loop."""

import math


class HeadBallTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        if (not all(math.isfinite(v) for v in (cfg.fx, cfg.fy, cfg.max_speed,
                cfg.smoother, cfg.scan_interval_s, cfg.yaw_min, cfg.yaw_max,
                cfg.pitch_min, cfg.pitch_max, cfg.center_tolerance,
                cfg.detection_max_age, cfg.lost_hold_s))
                or min(cfg.fx, cfg.fy, cfg.max_speed, cfg.smoother,
                       cfg.scan_interval_s, cfg.detection_max_age) <= 0
                or cfg.lost_hold_s < 0 or not 0 <= cfg.center_tolerance < 0.5
                or cfg.yaw_min >= cfg.yaw_max or cfg.pitch_min >= cfg.pitch_max):
            raise ValueError("Invalid head tracking configuration; load calibrated camera intrinsics")
        self.target = None
        self.command = None
        self.last_image = None
        self.last_seen = None
        self.scan_started = None
        self.scan_index = 0
        self.state = "hold"
        self.reason = "initializing"

    def update(self, now, dt, measured, ball, image_size):
        c = self.cfg
        if not all(math.isfinite(v) for v in measured):
            raise ValueError("Non-finite measured head angles")
        if self.command is None:
            self.command = list(measured)
            self.target = list(measured)
            self.last_seen = now
        width, height = image_size
        # No scan until live image geometry is available.
        if width <= 0 or height <= 0:
            self.state = "hold"
            self.reason = "image_stale"
            return tuple(self.command)
        visible = (ball is not None and ball.bbox is not None
                   and 0 <= now - ball.stamp <= c.detection_max_age)
        if visible:
            self.state = "track"
            self.last_seen = ball.stamp
            self.scan_started = None
            x0, y0, x1, y1 = ball.bbox
            dx = (x0 + x1) / 2 - width / 2
            dy = (y0 + y1) / 2 - height / 2
            outside = abs(dx) >= width * c.center_tolerance or abs(dy) >= height * c.center_tolerance
            self.reason = "pixel_error" if outside else "within_deadband"
            if ball.image_stamp != self.last_image:
                self.last_image = ball.image_stamp
                if outside:
                    fov_x = 2 * math.atan(width / (2 * c.fx))
                    fov_y = 2 * math.atan(height / (2 * c.fy))
                    self.target = [measured[0] - dx / width * fov_x / c.smoother,
                                   measured[1] + dy / height * fov_y / c.smoother]
                else:
                    # Cancel an outstanding scan target as soon as the ball is centred.
                    self.target = list(measured)
        elif now - self.last_seen <= c.lost_hold_s:
            self.state = "hold"
            self.reason = "ball_missing_short"
        else:
            self.state = "scan"
            self.reason = "ball_missing_scan"
            if self.scan_started is None:
                self.scan_started = now
                self.scan_index = 0
            sequence = [(c.yaw_max, c.pitch_max), (0., c.pitch_max),
                        (c.yaw_min, c.pitch_max), (c.yaw_min, c.pitch_min),
                        (0., c.pitch_min), (c.yaw_max, c.pitch_min)]
            waypoint = sequence[self.scan_index]
            # The demo high-level RPC moves faster than our limited low-level
            # targets. Wait for arrival before advancing so every waypoint is visited.
            if (now - self.scan_started >= c.scan_interval_s
                    and max(abs(self.command[i] - waypoint[i]) for i in range(2)) < 0.03):
                self.scan_index = (self.scan_index + 1) % len(sequence)
                self.scan_started = now
            self.target = list(sequence[self.scan_index])
        self.target[0] = max(c.yaw_min, min(c.yaw_max, self.target[0]))
        self.target[1] = max(c.pitch_min, min(c.pitch_max, self.target[1]))
        step = c.max_speed * max(0., min(dt, 0.1))
        for i in range(2):
            self.command[i] += max(-step, min(step, self.target[i] - self.command[i]))
        return tuple(self.command)
