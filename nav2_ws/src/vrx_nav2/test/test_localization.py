import math
import numpy as np
import pytest
from vrx_nav2.localization_math import ecef, enu_rotation, rotation, antenna_to_hull, InputFreshness, GyroLowPass


def test_wgs84_enu_axes_and_metre_scale():
    origin = ecef(0., 0., 0.)
    r = enu_rotation(0., 0.)
    np.testing.assert_allclose(r @ (ecef(0., 0., 1.)-origin), [0, 0, 1], atol=1e-7)
    east = r @ (ecef(0., math.degrees(1/6378137.), 0.)-origin)
    assert east[0] == pytest.approx(1., abs=1e-6)
    assert abs(east[1]) < 1e-6


def test_antenna_lever_correction_during_turn():
    lever = np.array([-.85, 0., 1.3])
    start = rotation([0, 0, 0, 1]) @ lever
    q = [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]
    # GPS moves as the boat rotates in place; hull must stay at the origin.
    np.testing.assert_allclose(antenna_to_hull([.85, -.85, 0], start, q, lever, np.eye(3)),
                               [0, 0, 0], atol=1e-10)
    # After a turn plus 3 m north, local +X (north) must be 3 m, not antenna travel.
    north_frame = rotation([0, 0, -math.sin(math.pi/4), math.cos(math.pi/4)])
    np.testing.assert_allclose(antenna_to_hull([.85, 2.15, 0], start, q, lever, north_frame),
                               [3, 0, 0], atol=1e-10)


def test_freshness_requires_both_advancing_inputs():
    f = InputFreshness()
    assert f.accept('imu', 10., 100., 10.)
    assert not f.ready(100., 10.)
    assert f.accept('gps', 10., 100., 10.)
    assert f.ready(100., 10.)
    assert not f.accept('gps', 10., 100.4, 10.4)  # replay cannot refresh wall time
    assert not f.ready(100.8, 10.4)
    assert not f.ready(100.2, 10.6)
    assert not f.accept('gps', 9., 101., 10.)
    assert not f.accept('gps', 12., 101., 10.)


@pytest.mark.parametrize('q', [[0, 0, 0, 0], [math.nan, 0, 0, 1]])
def test_invalid_orientation_rejected(q):
    with pytest.raises(ValueError):
        rotation(q)


def test_gyro_filter_attenuates_fast_oscillation_and_tracks_turn():
    f = GyroLowPass()
    f.update([0, 0, 0], 0.)
    for i in range(1, 201):
        value = f.update([0, 0, (-1)**i], i*.01)
        if i > 100:
            assert abs(value[2]) < .03
    for i in range(201, 401):
        value = f.update([0, 0, .1], i*.01)
    assert value[2] == pytest.approx(.1, abs=.001)
