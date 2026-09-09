"""Import installed ROS MPC modules without ROS initialization or publication."""
import hashlib
import json
import math
from pathlib import Path

import yaml
import multi_purpose_mpc_ros.mpc_controller as controller
import importlib

core=importlib.import_module('multi_purpose_mpc_ros.core.MPC')
root=Path('/v4')
installed=Path('/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros/config/config.yaml')
config=yaml.safe_load(installed.read_text())
assert config['mpc']['v_max']==20.0 and config['mpc']['ay_max']==3.0
assert hasattr(core.MPC, '_reject_control')
records={}
for name, module in [('MPC.py',core),('mpc_controller.py',controller)]:
    path=Path(module.__file__)
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    expected=hashlib.sha256((root/'expected'/name).read_bytes()).hexdigest()
    assert actual==expected, (name,actual,expected)
    records[name]={'path':str(path),'sha256':actual}
limits={'raw_limit_rad':math.radians(config['mpc']['delta_max_deg']),
        'gain':config['mpc']['steering_tire_angle_gain_var']}
(root/'limits.json').write_text(json.dumps(limits))
print(json.dumps({'installed':records,'config_sha256':hashlib.sha256(installed.read_bytes()).hexdigest(),
                  'limits':limits,'ros_initialized':False}))
