"""Read the same base/local calibration merge as the demo vision node."""

from pathlib import Path
import math
import yaml


def load_vision_config(directory):
    directory = Path(directory).resolve()
    def merge(base, override):
        for key, value in override.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                merge(base[key], value)
            else:
                base[key] = value
    with (directory / "vision.yaml").open() as f:
        config = yaml.safe_load(f)
    local = directory / "vision_local.yaml"
    if local.exists():
        with local.open() as f:
            merge(config, yaml.safe_load(f) or {})
    intr = config["camera"]["intrin"]
    if any(not math.isfinite(float(intr[k])) or float(intr[k]) <= 0 for k in ("fx", "fy")):
        raise ValueError("Camera fx/fy must be finite and positive")
    return config


def camera_topics(config):
    # The platform camera publishes these topics regardless of sensor type.
    return "/boostercamera/head/rgb", "/boostercamera/head/depth"
