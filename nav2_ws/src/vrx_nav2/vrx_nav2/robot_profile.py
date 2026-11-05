"""Local navigation camera-rate profile; never edits installed VRX source."""
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


def set_camera_rate(urdf, rate):
    if not math.isfinite(rate) or rate <= 0 or rate > 30:
        raise ValueError('camera_rate must be greater than 0 and at most 30 Hz')
    root = ET.fromstring(urdf)
    cameras = [s for s in root.iter('sensor') if s.get('type') == 'camera']
    if not cameras:
        raise ValueError('No camera sensors found in the installed VRX model')
    for sensor in cameras:
        update = sensor.find('update_rate')
        if update is None:
            raise ValueError('Camera lacks explicit update_rate: '+sensor.get('name', ''))
        update.text = str(rate)
    return ET.tostring(root, encoding='unicode')


def prepare_robot(directory, rate):
    xacro = Path(get_package_share_directory('wamv_gazebo'))/'urdf/wamv_gazebo.urdf.xacro'
    # Same arguments used by the installed VRX model generator.
    generated = subprocess.run(['xacro', str(xacro), 'namespace:=wamv', 'locked:=true',
        'vrx_sensors_enabled:=true', 'thruster_config:=H'], check=True, capture_output=True, text=True)
    target = Path(directory)/'navigation_robot.urdf'
    target.write_text(set_camera_rate(generated.stdout, rate))
    return str(target)
