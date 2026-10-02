"""Student on the same MuJoCo soccer scene and camera as pass/shoot."""

from .k1_pass_mujoco_controller import K1PassMujocoController


class K1StudentMujocoController(K1PassMujocoController):
    """Run the slow student actor with pass/shoot ball visibility and head scan."""
