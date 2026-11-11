"""Coordinate contract shared with simulation ground-truth localization."""
import math


def world_to_map(x, y, origin_x=-532., origin_y=162., origin_yaw=1.):
    if not all(math.isfinite(v) for v in (x, y, origin_x, origin_y, origin_yaw)):
        raise ValueError('Coordinates must be finite')
    dx, dy = x-origin_x, y-origin_y
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    return c*dx+s*dy, -s*dx+c*dy
