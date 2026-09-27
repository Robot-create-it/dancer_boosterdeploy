"""K1 shooting in the same MuJoCo soccer and camera scene as passing."""

from .k1_pass_mujoco_controller import K1PassMujocoController


class K1ShootMujocoController(K1PassMujocoController):
    """Run the shoot actor with the pass scene's ball visibility and head scan."""
