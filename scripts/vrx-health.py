"""Observe real PointCloud2 and clock progress without recording cloud payloads."""
import argparse
import json
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=600)
args = parser.parse_args()
rclpy.init()
node = Node('vrx_stability_observer')
start = last_report = time.monotonic()
count = clocks = regressions = 0
first_stamp = last_stamp = first_clock = last_clock = None
last_arrival = None
max_gap = 0.0
finite = occupied = roi_points = roi_cells = 0

def on_cloud(msg):
    global count, first_stamp, last_stamp, last_arrival, max_gap, regressions, finite, occupied, roi_points, roi_cells
    now = time.monotonic()
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if first_stamp is None:
        first_stamp = stamp
    if last_stamp is not None and stamp <= last_stamp:
        regressions += 1
    if last_arrival is not None:
        max_gap = max(max_gap, now - last_arrival)
    last_arrival, last_stamp = now, stamp
    count += 1
    points = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'])
    xyz = np.column_stack([points[k].reshape(-1) for k in ('x', 'y', 'z')])
    finite = int(np.isfinite(xyz).all(axis=1).sum())
    # Run the existing costmap code unchanged on actual sensor returns.
    from mtr_lidar_starter.processing import Processor
    result = Processor().process(xyz, stamp_seconds=stamp, frame_id=msg.header.frame_id)
    occupied = int((result.costs == 100).sum())
    roi_points = int(((xyz[:, 0] > 4) & (xyz[:, 0] < 9)
                      & (np.abs(xyz[:, 1]) < 3) & (np.abs(xyz[:, 2]) < 0.2)).sum())
    roi_cells = int((result.costs[35:65, 70:95] == 100).sum())

def on_clock(msg):
    global clocks, first_clock, last_clock
    last_clock = msg.clock.sec + msg.clock.nanosec * 1e-9
    if first_clock is None:
        first_clock = last_clock
    clocks += 1

subscriptions = [node.create_subscription(PointCloud2, '/lidar/points', on_cloud, qos_profile_sensor_data),
                 node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)]

def report():
    now = time.monotonic()
    print(json.dumps(dict(wall_seconds=round(now-start, 3), clouds=count,
        cloud_sim_first=first_stamp, cloud_sim_last=last_stamp, clocks=clocks,
        clock_first=first_clock, clock_last=last_clock,
        max_cloud_gap=round(max_gap, 3),
        seconds_since_cloud=None if last_arrival is None else round(now-last_arrival, 3),
        nonincreasing_stamps=regressions, finite_points=finite, occupied_cells=occupied,
        forward_roi_points=roi_points, forward_roi_cells=roi_cells)), flush=True)

try:
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() - last_report >= 10:
            report()
            last_report = time.monotonic()
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
