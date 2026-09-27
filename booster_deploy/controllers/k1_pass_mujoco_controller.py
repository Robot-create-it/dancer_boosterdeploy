"""K1 passing in MuJoCo, with a physical ball and privileged ball position."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch

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
    """Run the same pass policy with MuJoCo truth replacing ROS vision."""

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

    def get_ball_position(self, max_age: float):
        local = self._ball_in_robot_frame(self.mj_data.xpos[self._trunk_body_id])
        return (float(local[0]), float(local[1])) if np.isfinite(local).all() else None

    def ctrl_step(self, dof_targets: torch.Tensor):
        if self.cfg.booster.head_tracking.enabled:
            local = self._ball_in_robot_frame(self.mj_data.xpos[self._camera_body_id])
            if np.isfinite(local).all():
                head = self.cfg.booster.head_tracking
                target_yaw = float(np.clip(math.atan2(local[1], local[0]), head.yaw_min, head.yaw_max))
                target_pitch = float(np.clip(
                    math.atan2(-local[2], math.hypot(local[0], local[1])),
                    head.pitch_min, head.pitch_max,
                ))
                dof_targets = dof_targets.clone()
                step = head.max_speed * self.cfg.policy_dt
                for name, target in (("aahead_yaw_joint", target_yaw),
                                     ("aahead_pitch_joint", target_pitch)):
                    index = self.robot.cfg.joint_names.index(name)
                    measured = float(self.robot.data.joint_pos[index])
                    dof_targets[index] = measured + np.clip(target - measured, -step, step)
        super().ctrl_step(dof_targets)
