import xml.etree.ElementTree as ET
import pytest
from vrx_nav2.robot_profile import set_camera_rate


URDF = '''<robot name="test"><link name="hull"><inertial><mass value="180"/></inertial></link>
<gazebo><sensor name="camera" type="camera"><update_rate>30.0</update_rate><camera><image><width>1280</width></image></camera></sensor>
<sensor name="lidar" type="gpu_ray"><update_rate>10</update_rate><ray><scan><horizontal><samples>1875</samples></horizontal></scan></ray></sensor>
<plugin name="dynamics" filename="unchanged"><xU>100</xU></plugin></gazebo></robot>'''


def test_camera_profile_changes_only_camera_rate():
    before = ET.fromstring(URDF)
    after = ET.fromstring(set_camera_rate(URDF, 2.))
    assert after.find('.//sensor[@type="camera"]/update_rate').text == '2.0'
    assert after.find('.//sensor[@type="gpu_ray"]/update_rate').text == '10'
    after.find('.//sensor[@type="camera"]/update_rate').text = '30.0'
    assert ET.tostring(after) == ET.tostring(before)


@pytest.mark.parametrize('rate', [0, -1, float('nan'), float('inf'), 31])
def test_invalid_camera_rate_rejected(rate):
    with pytest.raises(ValueError):
        set_camera_rate(URDF, rate)
