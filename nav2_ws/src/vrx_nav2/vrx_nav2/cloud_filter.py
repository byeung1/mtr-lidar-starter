"""One nonblocking, timestamped transform for Nav2 and the safety adapter."""
import json
import time
from copy import deepcopy
import numpy as np
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from .core import BASE_FRAME, LIDAR_FRAME, LatestCloudQueue, filter_points


class CloudFilter(Node):
    def __init__(self):
        super().__init__('nav_cloud_filter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PointCloud2, '/nav/points', qos_profile_sensor_data)
        self.safety_pub = self.create_publisher(PointCloud2, '/nav/safety_points', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/nav/cloud_status', 10)
        self.sub = self.create_subscription(PointCloud2, '/lidar/points', self.receive, qos_profile_sensor_data)
        self.queue = LatestCloudQueue()
        self.received = self.processed = self.invalid = 0
        self.last_received = self.last_processed = None
        self.last_error = ''
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.process_pending, clock=steady)
        self.create_timer(1., self.report, clock=steady)

    def receive(self, msg):
        if msg.header.frame_id != LIDAR_FRAME:
            self.invalid += 1
            self.last_error = 'unexpected LiDAR frame: '+msg.header.frame_id
            return
        self.received += 1
        self.last_received = time.monotonic()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.queue.offer(stamp, self.last_received, msg)

    def has_transform(self, msg):
        ready, error = self.buffer.can_transform(BASE_FRAME, msg.header.frame_id,
            Time.from_msg(msg.header.stamp), return_debug_tuple=True)
        if not ready:
            self.last_error = error
        return ready

    def process_pending(self):
        item = self.queue.ready(time.monotonic(), self.get_clock().now().nanoseconds*1e-9,
                                self.has_transform)
        if item is None:
            return
        msg = item.payload
        try:
            tf = self.buffer.lookup_transform(BASE_FRAME, msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp)).transform
            points = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'])
            xyz = np.column_stack([points[k].reshape(-1) for k in ('x', 'y', 'z')])
            if np.isfinite(xyz).all(axis=1).sum() < 10:
                self.invalid += 1
                self.last_error = 'fewer than 10 finite raw points'
                return
            q, p = tf.rotation, tf.translation
            sensor_points, base_points = filter_points(xyz, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            # Nav2 keeps the real LiDAR frame/origin for obstacle ray clearing.
            self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, sensor_points))
            header = deepcopy(msg.header)
            header.frame_id = BASE_FRAME
            self.safety_pub.publish(point_cloud2.create_cloud_xyz32(header, base_points))
            self.processed += 1
            self.last_processed = time.monotonic()
            self.last_error = ''
        except (TransformException, ValueError, AssertionError, KeyError) as error:
            self.invalid += 1
            self.last_error = str(error)

    def report(self):
        now = time.monotonic()
        self.status_pub.publish(String(data=json.dumps(dict(
            received=self.received, processed=self.processed, invalid=self.invalid,
            dropped=self.queue.dropped, pending=len(self.queue.pending),
            raw_wall_age=None if self.last_received is None else round(now-self.last_received, 3),
            processed_wall_age=None if self.last_processed is None else round(now-self.last_processed, 3),
            last_error=self.last_error))))


def main():
    from .runtime import spin
    spin(CloudFilter)
