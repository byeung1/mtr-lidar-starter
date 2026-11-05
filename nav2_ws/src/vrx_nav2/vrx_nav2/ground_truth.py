"""Simulation-only localization from native Gazebo Pose_V (preserves names)."""
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from .core import BASE_FRAME, HULL_FRAME, quat_mul, quat_yaw, yaw_quat, wrap


class GroundTruth(Node):
    def __init__(self):
        super().__init__('sim_ground_truth')
        self.declare_parameter('origin_x', -532.)
        self.declare_parameter('origin_y', 162.)
        self.declare_parameter('origin_yaw', 1.)
        self.origin = np.array([self.get_parameter('origin_x').value,
                                self.get_parameter('origin_y').value])
        self.yaw0 = self.get_parameter('origin_yaw').value
        self.queue = deque(maxlen=1)
        self.previous = None
        self.velocity = np.zeros(3)
        self.odom_pub = self.create_publisher(Odometry, '/nav/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/nav/ground_truth', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.rotation.w = 1.
        self.static_tf.sendTransform(t)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, '/world/sydney_regatta/pose/info', self.receive)
        self.timer = self.create_timer(.02, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warn('SIMULATION ONLY: Gazebo ground truth, not a real localization solution')

    def receive(self, msg):
        for pose in msg.pose:
            if pose.name == 'wamv':
                p, q = pose.position, pose.orientation
                self.queue.append((msg.header.stamp.sec, msg.header.stamp.nsec,
                                   (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
                break

    def publish(self):
        if not self.queue:
            return
        sec, nsec, position, quaternion = self.queue.pop()
        stamp = sec+nsec*1e-9
        if self.previous and stamp <= self.previous[0]:
            return  # Repeated/rewound time must not refresh pose watchdogs.
        if not np.isfinite([*position, *quaternion]).all():
            return
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        dx, dy = np.array(position[:2])-self.origin
        x, y = c*dx+s*dy, -s*dx+c*dy
        qlocal = quat_mul(yaw_quat(-self.yaw0), quaternion)
        yaw = quat_yaw(qlocal)
        qflat = yaw_quat(yaw)
        residual = quat_mul(yaw_quat(-yaw), qlocal)
        if self.previous:
            prev_stamp, px, py, pyaw = self.previous
            dt = stamp-prev_stamp
            if .001 < dt < .5:
                vx, vy = (x-px)/dt, (y-py)/dt
                measurement = np.array([math.cos(yaw)*vx+math.sin(yaw)*vy,
                                        -math.sin(yaw)*vx+math.cos(yaw)*vy,
                                        wrap(yaw-pyaw)/dt])
                self.velocity = .8*self.velocity+.2*measurement
        self.previous = stamp, x, y, yaw
        odom = Odometry()
        odom.header.stamp.sec, odom.header.stamp.nanosec = sec, nsec
        odom.header.frame_id, odom.child_frame_id = 'odom', BASE_FRAME
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = map(float, qflat)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = map(float, self.velocity)
        self.odom_pub.publish(odom)
        flat = TransformStamped()
        flat.header, flat.child_frame_id = odom.header, BASE_FRAME
        flat.transform.translation.x, flat.transform.translation.y = float(x), float(y)
        flat.transform.rotation = odom.pose.pose.orientation
        hull = TransformStamped()
        hull.header.stamp = odom.header.stamp
        hull.header.frame_id, hull.child_frame_id = BASE_FRAME, HULL_FRAME
        hull.transform.translation.z = float(position[2])
        hull.transform.rotation.x, hull.transform.rotation.y, hull.transform.rotation.z, hull.transform.rotation.w = map(float, residual)
        self.tf.sendTransform([flat, hull])
        pose = PoseStamped()
        pose.header = odom.header
        pose.header.frame_id = 'map'
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)


def main():
    from .runtime import spin
    spin(GroundTruth)
