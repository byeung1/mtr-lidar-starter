"""ROS adapter: best-effort points -> NumPy -> your Processor."""
import time
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from .processing import Processor

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_starter')
        topic = self.declare_parameter('input_topic', '/lidar/points').value
        self.processor = Processor()
        self.previous_stamp = None
        self.last_log = 0.0
        self.count = 0
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f'Waiting for {topic}; edit processing.py to add your algorithm')

    def on_cloud(self, cloud):
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        try:
            points = point_cloud2.read_points(cloud, field_names=['x', 'y', 'z'])
            xyz = np.column_stack([points[k].reshape(-1) for k in ('x', 'y', 'z')])
            xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32, copy=False)
        except (ValueError, AssertionError, KeyError) as error:
            self.get_logger().error(f'Cannot decode XYZ: {error}')
            return
        if self.previous_stamp is not None and stamp < self.previous_stamp:
            self.processor.reset()
            self.get_logger().info('Input time moved backwards; processor reset for replay')
        self.previous_stamp = stamp
        result = self.processor.process(xyz, stamp_seconds=stamp, frame_id=cloud.header.frame_id)
        # Add publishers or pass result to your chosen visualization here.
        self.count += 1
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.last_log = now
            self.get_logger().info(f'Processed scan {self.count}: {len(xyz):,} finite points, frame={cloud.header.frame_id}')

def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
