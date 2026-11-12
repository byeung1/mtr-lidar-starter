"""VRX sensor preprocessing and freshness-gated EKF -> navigation TF.

No Gazebo transport, model pose, commanded thrust or ground truth is read here.
The local datum is the first accepted GPS fix and IMU heading after startup.
"""
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster, TransformException
from .core import BASE_FRAME, HULL_FRAME, quat_mul, quat_yaw, yaw_quat
from .localization_math import rotation, ecef, enu_rotation, antenna_to_hull, InputFreshness, GyroLowPass


def quaternion(msg):
    return np.array([msg.x, msg.y, msg.z, msg.w])


def set_quaternion(msg, q):
    msg.x, msg.y, msg.z, msg.w = map(float, q)


class SensorLocalization(Node):
    def __init__(self):
        super().__init__('sensor_localization')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        fixed = TransformStamped()
        fixed.header.frame_id, fixed.child_frame_id = 'map', 'odom'
        fixed.transform.rotation.w = 1.
        self.static_tf.sendTransform(fixed)
        self.fresh = InputFreshness()
        self.imu = None
        self.gyro = GyroLowPass()
        self.yaw0 = None
        self.datum = None
        self.last_output = -math.inf
        self.error = 'waiting for IMU, GPS and mounting TF'
        self.gps_pub = self.create_publisher(Odometry, '/nav/localization/gps', 10)
        self.imu_pub = self.create_publisher(Imu, '/nav/localization/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/nav/odom', 10)
        self.status_pub = self.create_publisher(String, '/nav/localization/status', 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/nav/localization/filtered', self.on_filtered, 10)
        self.create_timer(.5, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('GPS + IMU EKF localization; first fix/heading defines local origin. No ground-truth input.')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def mounting(self, frame):
        # Relative mounting TF is available without a global boat pose.
        return self.buffer.lookup_transform(HULL_FRAME, frame, Time()).transform

    def on_imu(self, msg):
        try:
            if msg.orientation_covariance[0] < 0 or msg.angular_velocity_covariance[0] < 0:
                raise ValueError('IMU must provide orientation and angular velocity')
            mount = self.mounting(msg.header.frame_id)
            qm = quaternion(mount.rotation)
            rm = rotation(qm)
            q = quaternion(msg.orientation)
            rotation(q)
            q = q/np.linalg.norm(q)
            qh = quat_mul(q, qm*np.array([-1., -1., -1., 1.]))
            angular = rm @ np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            if not np.isfinite(angular).all():
                raise ValueError('invalid IMU angular velocity')
            if not self.fresh.accept('imu', self.stamp(msg), time.monotonic(), self.now()):
                return
            # The thrust feedback loop must not chase high-frequency gyro
            # oscillations. Use acquisition-time low-pass filtering, comparable
            # to the velocity smoothing used by the former ground-truth node.
            stamp = self.stamp(msg)
            filtered_gyro = self.gyro.update(angular, stamp)
            if self.yaw0 is None:
                self.yaw0 = quat_yaw(qh)
            local_q = quat_mul(yaw_quat(-self.yaw0), qh)
            self.imu = (self.stamp(msg), qh, local_q)
            out = copy.deepcopy(msg)
            out.header.frame_id = HULL_FRAME
            set_quaternion(out.orientation, local_q)
            out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, filtered_gyro)
            # VRX bridge may report zero (unknown) covariance. Conservative,
            # explicit baseline floors, not a calibrated real sensor model.
            out.orientation_covariance = list(np.diag([.01**2, .01**2, .02**2]).ravel())
            out.angular_velocity_covariance = list(np.diag([.02**2]*3).ravel())
            out.linear_acceleration_covariance[0] = -1.  # deliberately not fused
            self.imu_pub.publish(out)
        except (TransformException, ValueError) as exc:
            self.error = str(exc)

    def on_gps(self, msg):
        try:
            stamp = self.stamp(msg)
            if msg.status.status < 0 or self.imu is None or abs(stamp-self.imu[0]) > .15:
                return
            mount = self.mounting(msg.header.frame_id)
            lever = np.array([mount.translation.x, mount.translation.y, mount.translation.z])
            point = ecef(msg.latitude, msg.longitude, msg.altitude)
            if not self.fresh.accept('gps', stamp, time.monotonic(), self.now()):
                return
            if self.datum is None:
                self.earth_to_enu = enu_rotation(msg.latitude, msg.longitude)
                self.enu_to_local = rotation(yaw_quat(-self.yaw0))
                # Origin at initial hull, not at the GPS antenna.
                self.initial_lever = rotation(self.imu[1]) @ lever
                self.datum = point
                self.get_logger().info('Local GPS datum initialized; +x = initial IMU heading, +y = left')
            enu = self.earth_to_enu @ (point-self.datum)
            hull = antenna_to_hull(enu, self.initial_lever, self.imu[1], lever, self.enu_to_local)
            out = Odometry()
            out.header = copy.deepcopy(msg.header)
            out.header.frame_id, out.child_frame_id = 'odom', HULL_FRAME
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, hull)
            out.pose.pose.orientation.w = 1.  # EKF only consumes XYZ from this input
            cov = np.array(msg.position_covariance).reshape(3, 3)
            if msg.position_covariance_type == 0 or not np.isfinite(cov).all():
                cov = np.diag([.15**2, .15**2, .3**2])
            else:
                cov = (cov+cov.T)/2
                cov += np.eye(3)*max(0., .15**2-float(np.linalg.eigvalsh(cov).min()))
            cov = self.enu_to_local @ cov @ self.enu_to_local.T
            full = np.eye(6)*1e6
            full[:3, :3] = cov
            out.pose.covariance = list(full.ravel())
            self.gps_pub.publish(out)
        except (TransformException, ValueError) as exc:
            self.error = str(exc)

    def on_filtered(self, msg):
        wall, now, stamp = time.monotonic(), self.now(), self.stamp(msg)
        if self.datum is None or not self.fresh.ready(wall, now) or not -.1 <= now-stamp <= .5 or stamp <= self.last_output:
            return  # No fresh TF/odometry on an EKF prediction heartbeat alone.
        if msg.header.frame_id != 'odom' or msg.child_frame_id != HULL_FRAME:
            self.error = 'unexpected EKF frame names'
            return
        p, q = msg.pose.pose.position, quaternion(msg.pose.pose.orientation)
        v = msg.twist.twist
        try:
            rotation(q)
            if not np.isfinite([p.x, p.y, p.z, v.linear.x, v.linear.y, v.linear.z,
                                v.angular.x, v.angular.y, v.angular.z]).all():
                return
        except ValueError:
            return
        yaw = quat_yaw(q)
        flat_q = yaw_quat(yaw)
        residual = quat_mul(yaw_quat(-yaw), q)
        # EKF velocity is in the full hull frame; nav_base is horizontal.
        velocity = rotation(residual) @ np.array([v.linear.x, v.linear.y, v.linear.z])
        angular = rotation(residual) @ np.array([v.angular.x, v.angular.y, v.angular.z])
        out = copy.deepcopy(msg)
        out.child_frame_id = BASE_FRAME
        out.pose.pose.position.z = 0.
        set_quaternion(out.pose.pose.orientation, flat_q)
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, velocity)
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, angular)
        out.twist.twist.linear.z = 0.
        out.twist.twist.angular.x = out.twist.twist.angular.y = 0.
        # Rotate velocity uncertainty into the horizontal navigation frame.
        block = np.zeros((6, 6))
        block[:3, :3] = block[3:, 3:] = rotation(residual)
        out.twist.covariance = list((block @ np.array(msg.twist.covariance).reshape(6, 6) @ block.T).ravel())
        self.odom_pub.publish(out)
        flat = TransformStamped()
        flat.header, flat.child_frame_id = out.header, BASE_FRAME
        flat.transform.translation.x, flat.transform.translation.y = p.x, p.y
        flat.transform.rotation = out.pose.pose.orientation
        hull = TransformStamped()
        hull.header = copy.deepcopy(out.header)
        hull.header.frame_id, hull.child_frame_id = BASE_FRAME, HULL_FRAME
        hull.transform.translation.z = p.z
        set_quaternion(hull.transform.rotation, residual)
        self.tf.sendTransform([flat, hull])
        self.last_output = stamp
        self.error = ''

    def status(self):
        wall, now = time.monotonic(), self.now()
        self.status_pub.publish(String(data=json.dumps({
            'source': 'gps_imu_ekf', 'inputs_fresh': self.fresh.ready(wall, now),
            'datum_initialized': self.datum is not None, 'error': self.error,
            'input_wall_ages': {k: wall-v[1] for k, v in self.fresh.samples.items()}})))


def main():
    from .runtime import spin
    spin(SensorLocalization)
