"""ROS-independent geometry and fail-safe control. Units: metres, seconds, N."""
import math
from dataclasses import dataclass
from collections import deque
import numpy as np

BASE_FRAME = 'nav_base'
HULL_FRAME = 'wamv/wamv/base_link'
LIDAR_FRAME = HULL_FRAME + '/lidar_wamv_sensor'
# Envelope includes aft motors/propellers, hulls and a conservative allowance.
FOOTPRINT = [(-3.1, -1.4), (2.7, -1.4), (2.7, 1.4), (-3.1, 1.4)]


@dataclass(frozen=True)
class QueuedCloud:
    stamp: float
    wall: float
    payload: object


class LatestCloudQueue:
    """Bounded TF/clock ordering buffer; never re-stamps or refreshes old data."""
    def __init__(self):
        self.pending = deque(maxlen=8)
        self.latest_stamp = -math.inf
        self.dropped = 0

    def offer(self, stamp, wall, payload):
        if not math.isfinite(stamp) or not math.isfinite(wall) or stamp <= self.latest_stamp:
            return False
        if len(self.pending) == self.pending.maxlen:
            self.dropped += 1
        self.pending.append(QueuedCloud(stamp, wall, payload))
        self.latest_stamp = stamp
        return True

    def ready(self, wall, sim_now, has_transform):
        valid = []
        for item in self.pending:
            if wall-item.wall > 1.5 or sim_now-item.stamp > .8:
                self.dropped += 1
            else:
                valid.append(item)
        self.pending = deque(valid, maxlen=8)
        for item in reversed(valid):
            if item.stamp <= sim_now+.05 and has_transform(item.payload):
                self.dropped += sum(v.stamp < item.stamp for v in valid)
                self.pending = deque([v for v in valid if v.stamp > item.stamp], maxlen=8)
                return item
        return None


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_mul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def yaw_quat(yaw):
    return np.array([0., 0., math.sin(yaw/2), math.cos(yaw/2)])


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def filter_points(sensor_xyz, translation, quaternion):
    """Keep real above-water returns; never fabricate max-range clearing rays."""
    sensor_xyz = np.asarray(sensor_xyz, dtype=np.float32)
    sensor_xyz = sensor_xyz[np.isfinite(sensor_xyz).all(axis=1)]
    base = sensor_xyz @ rotation(quaternion).T + np.asarray(translation)
    finite = np.isfinite(base).all(axis=1)
    # Nav_base is the horizontal water-plane projection, not the rocking hull.
    height = (base[:, 2] >= .35) & (base[:, 2] <= 3.5)
    self_return = ((base[:, 0] >= -3.2) & (base[:, 0] <= 2.8)
                   & (np.abs(base[:, 1]) <= 1.5))
    keep = finite & height & ~self_return
    return sensor_xyz[keep], base[keep]


def imminent_obstacle(points, speed, yaw_rate, pivot=False):
    """Conservative swept hull over a braking horizon, using only real returns."""
    points = np.asarray(points)
    if not len(points):
        return False
    speed = max(0., speed)
    if pivot:
        # Check the padded hull through the next three seconds of rotation.
        for angle in np.linspace(0., yaw_rate*3., 12):
            x = points[:, 0]*math.cos(angle)+points[:, 1]*math.sin(angle)
            y = -points[:, 0]*math.sin(angle)+points[:, 1]*math.cos(angle)
            if np.any((x > -3.3) & (x < 2.9) & (np.abs(y) < 1.6)):
                return True
        if speed <= .05:
            return False
    # 2 m minimum forward stopping reserve; no instantaneous-stop assumption.
    distance = max(2., speed*1.5 + speed*speed/(2*.08))
    for distance_along in np.linspace(0., distance, 12):
        angle = np.clip(yaw_rate * distance_along / max(speed, .1), -.6, .6)
        if abs(yaw_rate) > .001 and speed > .05:
            radius = speed / yaw_rate
            cx, cy = radius*math.sin(angle), radius*(1-math.cos(angle))
        else:
            cx, cy = distance_along, 0.
        x = (points[:, 0]-cx)*math.cos(angle)+(points[:, 1]-cy)*math.sin(angle)
        y = -(points[:, 0]-cx)*math.sin(angle)+(points[:, 1]-cy)*math.cos(angle)
        if np.any((x > -3.3) & (x < 2.9) & (np.abs(y) < 1.6)):
            return True
    return False


def obstacle_limited_command(points, speed, command):
    """Keep the requested curvature while reducing speed near a blocked braking horizon."""
    forward, yaw_rate = command
    if 0. <= forward < .01 and abs(yaw_rate) > .01:
        if imminent_obstacle(points, abs(speed), yaw_rate, pivot=True):
            return None
        return command
    if not imminent_obstacle(points, max(speed, forward), yaw_rate):
        return command
    if imminent_obstacle(points, speed, yaw_rate):
        return None
    for scale in (.5, .25):
        candidate = forward*scale, yaw_rate*scale
        if not imminent_obstacle(points, max(speed, candidate[0]), candidate[1]):
            return candidate
    return None


@dataclass
class Sample:
    wall: float = -math.inf
    stamp: float = -math.inf

    def update(self, wall, stamp, sim_now):
        if not all(math.isfinite(v) for v in (wall, stamp, sim_now)):
            return False
        if stamp <= self.stamp or not -.05 <= sim_now-stamp <= .8:
            return False
        self.wall, self.stamp = wall, stamp
        return True

    def fresh(self, wall, sim_now, timeout, max_sim_age=.8):
        return (0 <= wall-self.wall <= timeout
                and -.05 <= sim_now-self.stamp <= max_sim_age)


class ThrustControl:
    """Feedback around VRX drag; NOT a physical PWM/ESC controller."""
    def __init__(self):
        self.left = self.right = self.target_speed = 0.

    def zero(self):
        self.left = self.right = self.target_speed = 0.
        return 0., 0.

    def step(self, v_cmd, w_cmd, speed, yaw_rate, dt):
        if not all(math.isfinite(v) for v in (v_cmd, w_cmd, speed, yaw_rate, dt)) or dt < 0:
            return self.zero()
        # Opposing thrust can pivot the simulated boat. Translational reverse is disabled.
        if v_cmd < 0:
            return self.zero()
        if dt == 0:
            return self.left, self.right  # Hold between clock messages; wall watchdog still runs.
        turn_limit = .22 if v_cmd < .01 else .12
        v_cmd, w_cmd = min(v_cmd, 1.), max(-turn_limit, min(turn_limit, w_cmd))
        dt = min(dt, .1)
        if v_cmd < .01:
            self.target_speed = 0.
        else:
            self.target_speed += np.clip(v_cmd-self.target_speed, -.12*dt, .08*dt)
        force = 100*self.target_speed + 150*self.target_speed**2 + 300*(self.target_speed-speed)
        torque = 800*w_cmd + 800*w_cmd*abs(w_cmd) + 700*(w_cmd-yaw_rate)
        desired = np.array([(force-torque/1.027135)/2, (force+torque/1.027135)/2])
        # Reduced 180 N limit, versus the model's > 2300 N positive limit.
        desired = np.clip(desired, -100., 180.)
        current = np.array([self.left, self.right])
        current += np.clip(desired-current, -120*dt, 80*dt)
        self.left, self.right = map(float, current)
        return self.left, self.right
