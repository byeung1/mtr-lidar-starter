import math
import numpy as np
import pytest
from vrx_nav2.core import (Sample, ThrustControl, filter_points, rotation,
                           quat_mul, yaw_quat, imminent_obstacle, LatestCloudQueue)


def test_mount_axes_and_self_water_filter():
    xyz = np.array([[0, 0, 0], [5, 0, 0], [5, 0, -1.8], [np.inf, 0, 0]])
    sensor, base = filter_points(xyz, [.7, 0, 1.8], [0, 0, 0, 1])
    assert sensor.shape == (1, 3)
    np.testing.assert_allclose(base[0], [5.7, 0, 1.8])


def test_local_frame_rotation():
    np.testing.assert_allclose(rotation(yaw_quat(math.pi/2)) @ [1, 0, 0], [0, 1, 0], atol=1e-7)
    np.testing.assert_allclose(quat_mul(yaw_quat(-1), yaw_quat(1)), [0, 0, 0, 1], atol=1e-7)


def test_duplicate_or_stale_stamp_cannot_refresh_watchdog():
    s = Sample()
    assert s.update(1, 10, 10)
    assert not s.update(2, 10, 10.5)
    assert not s.fresh(3, 10.6, 1.5)
    assert not s.update(3, 9, 11)
    assert not s.update(3, 12, 11)


def test_paused_simulation_wall_watchdog():
    s = Sample()
    assert s.update(10, 1, 1)
    assert s.fresh(10.1, 1, .75)
    assert not s.fresh(11, 1, .75)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_invalid_input_neutral(value):
    c = ThrustControl()
    assert c.step(value, 0, 0, 0, .05) == (0, 0)


def test_positive_yaw_increases_right_thrust():
    c = ThrustControl()
    for _ in range(100):
        left, right = c.step(.3, .08, .3, 0, .05)
    assert right > left


def test_saturation_and_slew():
    c = ThrustControl()
    previous = np.zeros(2)
    for _ in range(100):
        current = np.array(c.step(100, 100, -100, -100, .05))
        assert np.max(current) <= 180
        assert np.min(current) >= -100
        assert np.max(current-previous) <= 4.000001
        assert np.min(current-previous) >= -6.000001
        previous = current


def test_spin_and_reverse_are_rejected():
    c = ThrustControl()
    assert c.step(0, .1, 0, 0, .1) == (0, 0)
    assert c.step(-.2, 0, 0, 0, .1) == (0, 0)


def test_emergency_neutral_is_not_slew_limited():
    c = ThrustControl()
    for _ in range(100):
        c.step(.4, 0, 0, 0, .1)
    assert c.left > 0
    assert c.zero() == (0, 0)


def test_repeated_clock_does_not_reset_acceleration_ramp():
    c = ThrustControl()
    force = c.step(.4, 0, 0, 0, .1)
    assert c.step(.4, 0, 0, 0, 0) == force
    assert c.target_speed > 0
    assert c.step(.4, 0, 0, 0, -.1) == (0, 0)


def test_swept_footprint_catches_ahead_not_side_obstacle():
    assert imminent_obstacle(np.array([[4., 0., 1.]]), .45, 0)
    assert not imminent_obstacle(np.array([[4., 3., 1.]]), .45, 0)
    assert not imminent_obstacle(np.empty((0, 3)), .45, 0)


def test_cloud_waits_for_clock_without_restamping():
    q = LatestCloudQueue()
    assert q.offer(10.2, 1., 'cloud')
    assert q.ready(1.01, 10., lambda _: True) is None
    ready = q.ready(1.1, 10.2, lambda _: True)
    assert ready.payload == 'cloud'
    assert ready.stamp == 10.2 and ready.wall == 1.


def test_pending_tf_is_nonblocking_and_newest_ready_cloud_wins():
    q = LatestCloudQueue()
    q.offer(10., 1., 'old')
    assert q.ready(1.01, 10., lambda _: False) is None
    q.offer(10.1, 1.1, 'new')
    ready = q.ready(1.2, 10.1, lambda _: True)
    assert ready.payload == 'new'
    assert not q.pending


def test_queue_is_bounded_and_duplicate_cannot_extend_expiry():
    q = LatestCloudQueue()
    q.offer(10., 1., 'a')
    assert not q.offer(10., 2., 'duplicate')
    assert q.ready(2.6, 10., lambda _: True) is None
    for i in range(10):
        q.offer(11.+i*.01, 3.+i*.01, i)
    assert len(q.pending) == 8
    assert q.dropped >= 2


def test_queue_drops_old_acquisition_time_even_if_just_received():
    q = LatestCloudQueue()
    q.offer(10., 2., 'stale')
    assert q.ready(2.01, 10.9, lambda _: True) is None


def test_nav2_and_safety_outputs_are_the_same_physical_points():
    points = np.array([[8., 2., 0.], [5., -3., .5], [0., 0., 0.]])
    translation, quaternion = [.7, 0., 1.8], yaw_quat(.2)
    sensor, base = filter_points(points, translation, quaternion)
    np.testing.assert_allclose(base, sensor @ rotation(quaternion).T + translation)
