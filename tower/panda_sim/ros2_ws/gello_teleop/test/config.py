"""The package's YAML parameters, for the tests."""

from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parent.parent / "config" / "gello_panda.yaml"

with open(CONFIG) as config_file:
    _params = yaml.safe_load(config_file)

DRIVER = _params["gello_driver"]["ros__parameters"]
CALIBRATOR = _params["gello_calibrator"]["ros__parameters"]

# The simulator's home pose (the fake GELLO's too)
HOME = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71]
