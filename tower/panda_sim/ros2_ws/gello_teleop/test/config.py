"""The package's YAML parameters, for the tests."""

from pathlib import Path

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

with open(CONFIG_DIR / "gello_panda.yaml") as config_file:
    _params = yaml.safe_load(config_file)
with open(CONFIG_DIR / "panda_real.yaml") as config_file:
    _real = yaml.safe_load(config_file)

DRIVER = _params["gello_driver"]["ros__parameters"]
CALIBRATOR = _params["gello_calibrator"]["ros__parameters"]

# What 'robot:=real' overrides, and the calibrator's parameters with it
REAL_OVERRIDES = _real["gello_calibrator"]["ros__parameters"]
REAL_CALIBRATOR = {**CALIBRATOR, **REAL_OVERRIDES}

# The simulator's home pose (the fake GELLO's too)
HOME = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71]
