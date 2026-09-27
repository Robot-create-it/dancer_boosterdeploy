"""K1 passing in MuJoCo, with a physical ball and privileged ball position."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch

from booster_deploy.utils.head_ball_tracker import HeadBallTracker
from booster_deploy.utils.vision_ball import BallObservation

from .mujoco_controller import MujocoController


SOCCER_SCENE = Path(__file__).resolve().parents[2] / "assets" / "soccer" / "world.xml"


def make_pass_scene(robot_xml: Path, ball_xy: tuple[float, float]) -> str:
    """Merge the supplied soccer world into the K1 MJCF without changing assets."""
    if len(ball_xy) != 2 or not np.isfinite(ball_xy).all():
        raise ValueError("ball_init_xy must contain two finite coordinates")
    robot = ET.parse(robot_xml).getroot()
    soccer = ET.parse(SOCCER_SCENE).getroot()
    for visual_default in soccer.findall("./default/default"):
        robot.find("default").append(visual_default)

    # from_xml_string has no source directory: resolve every file explicitly.
    robot_compiler = robot.find("compiler")
    mesh_dir = robot_xml.parent / robot_compiler.get("meshdir", "")
    for mesh in robot.findall("./asset/mesh"):
        mesh.set("file", str((mesh_dir / mesh.get("file")).resolve()))
    robot_compiler.attrib.pop("meshdir", None)
    for texture in soccer.findall("./asset/texture"):
        for key in ("file", "fileup", "filedown", "filefront", "fileback",
                    "fileleft", "fileright"):
            if key in texture.attrib:
                texture.set(key, str((SOCCER_SCENE.parent / "assets" / texture.get(key)).resolve()))
        robot.find("asset").append(texture)
    for material in soccer.findall("./asset/material"):
        robot.find("asset").append(material)

    # The soccer pitch replaces the robot model's checkerboard ground.
    world = robot.find("worldbody")
    for geom in list(world.findall("geom")):
        if geom.get("name") == "ground":
            world.remove(geom)
    for child in soccer.find("worldbody"):
        if child.tag == "body" and child.get("name") == "ball":
            child.set("pos", f"{ball_xy[0]} {ball_xy[1]} 0.11")
        world.append(child)
    return ET.tostring(robot, encoding="unicode")


class K1PassMujocoController(MujocoController):
    """Run pass with MuJoCo truth gated by the moving head camera's FOV."""

    def __init__(self, cfg):
        super().__init__(cfg)
        camera = self.cfg.mujoco.pass_camera
        if (camera.width <= 0 or camera.height <= 0
                or not np.isfinite([camera.fx, camera.fy, camera.cx, camera.cy]).all()
                or camera.fx <= 0 or camera.fy <= 0
                or not 0 <= camera.cx < camera.width
                or not 0 <= camera.cy < camera.height):
            raise ValueError("Invalid MuJoCo pass camera intrinsics or image size")
        self._head_tracker = None

    def start(self):
        super().start()
        if self.cfg.booster.head_tracking.enabled:
            camera = self.cfg.mujoco.pass_camera
            tracker_cfg = self.cfg.booster.head_tracking.replace(fx=camera.fx, fy=camera.fy)
            self._head_tracker = HeadBallTracker(tracker_cfg)
        else:
            self._head_tracker = None

    def _load_model(self):
        robot_xml = Path(self._expand_assets_placeholder(self.robot.cfg.mjcf_path))
        xml = make_pass_scene(robot_xml, tuple(self.cfg.mujoco.ball_init_xy))
        model = mujoco.MjModel.from_xml_string(xml)
        self._ball_body_id = model.body("ball").id
        self._trunk_body_id = model.body("trunk").id
        self._camera_body_id = model.body("head_booster_stereo_rgb_link").id
        return model

    def _ball_in_robot_frame(self, origin_w: np.ndarray) -> np.ndarray:
        delta_w = self.mj_data.xpos[self._ball_body_id] - origin_w
        return self.mj_data.xmat[self._trunk_body_id].reshape(3, 3).T @ delta_w

    def _visible_ball_pixels(self):
        """Project the ball centre into the current camera image, or return None."""
        camera = self.cfg.mujoco.pass_camera
        delta_w = self.mj_data.xpos[self._ball_body_id] - self.mj_data.xpos[self._camera_body_id]
        optical = self.mj_data.xmat[self._camera_body_id].reshape(3, 3).T @ delta_w
        # The K1 head_booster_stereo_rgb_link uses optical +Z forward, +X right,
        # +Y down. Its xmat includes both head joints and the robot base pose.
        if not np.isfinite(optical).all() or optical[2] <= 0:
            return None
        u = camera.fx * optical[0] / optical[2] + camera.cx
        v = camera.fy * optical[1] / optical[2] + camera.cy
        if not (0 <= u < camera.width and 0 <= v < camera.height):
            return None
        return float(u), float(v), float(optical[2])

    def get_ball_position(self, max_age: float):
        if self._visible_ball_pixels() is None:
            return None
        local = self._ball_in_robot_frame(self.mj_data.xpos[self._trunk_body_id])
        return (float(local[0]), float(local[1])) if np.isfinite(local).all() else None

    def ctrl_step(self, dof_targets: torch.Tensor):
        if self._head_tracker is not None:
            camera = self.cfg.mujoco.pass_camera
            now = float(self.mj_data.time)
            pixels = self._visible_ball_pixels()
            ball = None
            if pixels is not None:
                u, v, depth = pixels
                radius = self.mj_model.geom("ball").size[0]
                rx, ry = camera.fx * radius / depth, camera.fy * radius / depth
                relative = self._ball_in_robot_frame(self.mj_data.xpos[self._trunk_body_id])
                ball = BallObservation(
                    float(relative[0]), float(relative[1]),
                    100.0, (u - rx, v - ry, u + rx, v + ry), now, now,
                )
            names = ("aahead_yaw_joint", "aahead_pitch_joint")
            indices = [self.robot.cfg.joint_names.index(name) for name in names]
            measured = [float(self.robot.data.joint_pos[index]) for index in indices]
            target = self._head_tracker.update(
                now, self.cfg.policy_dt, measured, ball, (camera.width, camera.height))
            dof_targets = dof_targets.clone()
            for index, value in zip(indices, target):
                dof_targets[index] = value
        super().ctrl_step(dof_targets)
