"""WGS84 and rigid-body conversions, independent of ROS."""
import math
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < .5 or norm > 1.5:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def ecef(latitude, longitude, altitude):
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)) or abs(latitude) > 90 or abs(longitude) > 180:
        raise ValueError('invalid GPS fix')
    lat, lon = math.radians(latitude), math.radians(longitude)
    e2 = 6.69437999014e-3
    n = 6378137. / math.sqrt(1-e2*math.sin(lat)**2)
    return np.array([(n+altitude)*math.cos(lat)*math.cos(lon),
                     (n+altitude)*math.cos(lat)*math.sin(lon),
                     (n*(1-e2)+altitude)*math.sin(lat)])


def enu_rotation(latitude, longitude):
    lat, lon = math.radians(latitude), math.radians(longitude)
    return np.array([[-math.sin(lon), math.cos(lon), 0.],
                     [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
                     [math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)]])


def antenna_to_hull(enu, initial_lever, orientation, lever, local_rotation):
    return local_rotation @ (np.asarray(enu)+initial_lever-rotation(orientation) @ lever)


class InputFreshness:
    """An EKF heartbeat must never stand in for advancing sensor observations."""
    def __init__(self):
        self.samples = {}

    def accept(self, name, stamp, wall, now):
        if not all(math.isfinite(v) for v in (stamp, wall, now)) or not -.1 <= now-stamp <= .5:
            return False
        if name in self.samples and stamp <= self.samples[name][0]:
            return False
        self.samples[name] = stamp, wall
        return True

    def ready(self, wall, now):
        return all(name in self.samples and 0 <= wall-self.samples[name][1] <= .75
                   and -.1 <= now-self.samples[name][0] <= .5 for name in ('imu', 'gps'))


class GyroLowPass:
    def __init__(self, tau=.3):
        self.tau, self.value, self.stamp = tau, None, None

    def update(self, value, stamp):
        value = np.asarray(value, dtype=float)
        if self.value is None or stamp-self.stamp > .5:
            self.value = value.copy()
        elif stamp > self.stamp:
            self.value += (1.-math.exp(-(stamp-self.stamp)/self.tau))*(value-self.value)
        self.stamp = stamp
        return self.value.copy()
