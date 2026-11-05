"""Only simulated VRX thrust. Wall-clock watchdogs survive paused simulation."""
import json
import math
import time
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from .core import BASE_FRAME, Sample, ThrustControl, imminent_obstacle


class ThrustAdapter(Node):
    def __init__(self):
        super().__init__('sim_thrust_adapter')
        self.declare_parameter('initially_armed', False)
        self.armed = self.get_parameter('initially_armed').value
        self.pose = Sample()
        self.lidar = Sample()
        self.command_wall = -math.inf
        self.command = (0., 0.)
        self.velocity = (0., 0.)
        self.points = np.empty((0, 3))
        self.control = ThrustControl()
        self.previous_sim = None
        self.left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.steering = [self.create_publisher(Float64, f'/wamv/thrusters/{side}/pos', 10) for side in ('left', 'right')]
        self.status = self.create_publisher(String, '/nav/safety_status', 10)
        self.create_subscription(Twist, '/nav/cmd_vel', self.on_command, 10)
        self.create_subscription(Odometry, '/nav/odom', self.on_pose, 10)
        self.create_subscription(PointCloud2, '/nav/safety_points', self.on_cloud, qos_profile_sensor_data)
        self.cloud_pipeline = {}
        self.create_subscription(String, '/nav/cloud_status', self.on_cloud_status, 10)
        self.create_service(SetBool, '/nav/arm', self.on_arm)
        self.reason = 'disarmed'
        self.last_fault = ''
        self.last_fault_details = {}
        self.cloud_seen_wall = -math.inf
        self.last_cloud_error = ''
        self.good_clouds = 0
        self.stable_since = None
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def sim_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-6 for v in values[1:5]):
            self.command_wall = -math.inf
            return
        self.command, self.command_wall = (msg.linear.x, msg.angular.z), time.monotonic()

    def on_pose(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != BASE_FRAME:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if all(math.isfinite(x) for x in (v, w)) and self.pose.update(time.monotonic(), stamp, self.sim_now()):
            self.velocity = v, w

    def on_cloud(self, msg):
        if msg.header.frame_id != BASE_FRAME:
            return
        self.cloud_seen_wall = time.monotonic()
        try:
            p = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'])
            points = np.column_stack([p[k].reshape(-1) for k in ('x', 'y', 'z')])
            if not np.isfinite(points).all():
                return
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if self.lidar.update(time.monotonic(), stamp, self.sim_now()):
                self.points = points
                self.last_cloud_error = ''
                self.good_clouds += 1
            else:
                self.last_cloud_error = 'cloud timestamp did not pass freshness/advancement checks'
        except (ValueError, KeyError, AssertionError) as error:
            self.last_cloud_error = str(error)
            return

    def on_cloud_status(self, msg):
        try:
            self.cloud_pipeline = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def input_fault(self):
        now, sim = time.monotonic(), self.sim_now()
        if not self.pose.fresh(now, sim, 1.0):
            return 'stale_pose'
        if not self.lidar.fresh(now, sim, 3.0, max_sim_age=1.5):
            return 'stale_lidar'
        return ''

    def on_arm(self, req, response):
        fault = self.input_fault()
        if req.data and not fault and (self.good_clouds < 3 or self.stable_since is None
                                      or time.monotonic()-self.stable_since < 1.):
            fault = 'waiting for three fresh clouds and one second of stable inputs'
        self.armed = bool(req.data and not fault)
        if self.armed:
            self.last_fault = ''
            self.last_fault_details = {}
        if not req.data:
            self.neutral()
        response.success = not req.data or self.armed
        response.message = fault if req.data and fault else ('armed' if self.armed else 'disarmed')
        return response

    def publish_force(self, left, right):
        self.left.publish(Float64(data=float(left)))
        self.right.publish(Float64(data=float(right)))
        for pub in self.steering:
            pub.publish(Float64(data=0.))  # Keep both azimuth joints straight.

    def neutral(self):
        self.control.zero()
        self.publish_force(0., 0.)

    def tick(self):
        now, sim = time.monotonic(), self.sim_now()
        fault = self.input_fault()
        if fault:
            self.good_clouds = 0
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        if self.armed and fault:
            self.last_fault = fault
            def age(value):
                return round(value, 3) if math.isfinite(value) else None
            self.last_fault_details = dict(
                sim_time=round(sim, 3),
                pose_wall_age=age(now-self.pose.wall),
                pose_sim_age=age(sim-self.pose.stamp),
                lidar_accepted_wall_age=age(now-self.lidar.wall),
                lidar_accepted_sim_age=age(sim-self.lidar.stamp),
                lidar_received_wall_age=age(now-self.cloud_seen_wall),
                last_cloud_error=self.last_cloud_error,
                cloud_pipeline=self.cloud_pipeline)
            self.get_logger().warn('Disarmed: '+fault+' '+json.dumps(self.last_fault_details))
            self.armed = False  # Sensor loss latches; fresh commands cannot re-arm.
        reason = fault or ('disarmed' if not self.armed else '')
        if not reason and now-self.command_wall > 1.0:
            reason = 'command_timeout'
        if not reason and imminent_obstacle(self.points, max(self.velocity[0], self.command[0]), self.command[1]):
            reason = 'obstacle_stop'
        dt = 0. if self.previous_sim is None else sim-self.previous_sim
        self.previous_sim = sim
        if reason:
            self.neutral()
        else:
            self.publish_force(*self.control.step(*self.command, *self.velocity, dt))
        self.reason = reason or 'running'
        self.status.publish(String(data=json.dumps(dict(armed=self.armed, reason=self.reason,
            left_N=self.control.left, right_N=self.control.right,
            speed=self.velocity[0], yaw_rate=self.velocity[1],
            command_v=self.command[0], command_w=self.command[1], last_fault=self.last_fault,
            last_fault_details=self.last_fault_details))))


def main():
    from .runtime import spin
    spin(ThrustAdapter)
