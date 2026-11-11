"""Gazebo GUI world-plane clicks -> simulation-only NavigateToPose goals."""
import math
import queue
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy
from gz.transport13 import Node as GzNode
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.stringmsg_pb2 import StringMsg

from .click_geometry import world_to_map


class ClickGoal(Node):
    def __init__(self):
        super().__init__('gazebo_click_goal')
        for key, value in [('origin_x', -532.), ('origin_y', 162.), ('origin_yaw', 1.)]:
            self.declare_parameter(key, value)
        self.origin = [self.get_parameter(k).value for k in ('origin_x', 'origin_y', 'origin_yaw')]
        self.events = queue.Queue(maxsize=16)
        self.gz = GzNode()
        self.status_pub = self.gz.advertise('/nav/click_goal/status', StringMsg)
        self.gz.subscribe(Vector3d, '/nav/click_goal/world', self.on_click)
        self.gz.subscribe(Boolean, '/nav/click_goal/stop', self.on_stop)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.arm = self.create_client(SetBool, '/nav/arm')
        self.pose = None
        self.pose_wall = -math.inf
        self.grid = None
        self.goal_handle = None
        self.busy = False
        self.generation = 0
        self.text = 'Ready: enable Click to navigate, then click the water.'
        self.create_subscription(PoseStamped, '/nav/ground_truth', self.on_pose, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1., self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def put(self, value):
        try:
            self.events.put_nowait(value)
        except queue.Full:
            pass

    def on_click(self, msg):
        self.put(('goal', (msg.x, msg.y)))

    def on_stop(self, msg):
        if msg.data:
            # Stop takes priority over queued clicks.
            while True:
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    break
            self.put(('stop', None))

    def on_pose(self, msg):
        self.pose, self.pose_wall = msg, time.monotonic()

    def on_map(self, msg):
        self.grid = msg

    def status(self, text):
        self.text = text
        self.get_logger().info(text)
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(StringMsg(data=self.text))

    def tick(self):
        try:
            kind, value = self.events.get_nowait()
        except queue.Empty:
            return
        if kind == 'stop':
            self.generation += 1
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            if self.arm.service_is_ready():
                self.arm.call_async(SetBool.Request(data=False))
            self.busy = False
            self.status('Stopped: disarmed boat and requested goal cancellation.')
            return
        if self.busy:
            self.status('Waiting for Nav2 response. Click again in a moment, or Stop.')
            return
        try:
            x, y = world_to_map(*value, *self.origin)
        except ValueError as error:
            self.status(str(error))
            return
        if not self.pose or time.monotonic()-self.pose_wall > 2.:
            self.status('Cannot navigate: fresh boat position unavailable.')
            return
        if self.grid is None or self.grid.header.frame_id != 'map':
            self.status('Cannot navigate: water map unavailable.')
            return
        info = self.grid.info
        ix = math.floor((x-info.origin.position.x)/info.resolution)
        iy = math.floor((y-info.origin.position.y)/info.resolution)
        if not (0 <= ix < info.width and 0 <= iy < info.height) or self.grid.data[iy*info.width+ix] != 0:
            self.status('Destination outside navigable water. Choose a closer water location.')
            return
        if not self.action.server_is_ready() or not self.arm.service_is_ready():
            self.status('Cannot navigate: Nav2 or arming service unavailable.')
            return
        self.generation += 1
        generation = self.generation
        self.busy = True
        if self.goal_handle:
            previous, self.goal_handle = self.goal_handle, None
            future = previous.cancel_goal_async()
            future.add_done_callback(lambda f: self.after_cancel(f, x, y, generation))
        else:
            self.start(x, y, generation)

    def after_cancel(self, future, x, y, generation):
        if generation != self.generation:
            return
        if not future.result().goals_canceling:
            self.busy = False
            self.status('Could not cancel previous goal. Press Stop before trying again.')
            return
        self.start(x, y, generation)

    def start(self, x, y, generation):
        if generation != self.generation:
            return
        self.status(f'Requesting destination ({x:.1f}, {y:.1f}) m; checking boat inputs.')
        future = self.arm.call_async(SetBool.Request(data=True))
        future.add_done_callback(lambda f: self.armed(f, x, y, generation))

    def armed(self, future, x, y, generation):
        if generation != self.generation:
            # A Stop during arming must leave the boat disarmed.
            self.arm.call_async(SetBool.Request(data=False))
            return
        response = future.result()
        if not response.success:
            self.busy = False
            self.status('Cannot arm: '+response.message)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        position = self.pose.pose.position
        yaw = math.atan2(y-position.y, x-position.x)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        future = self.action.send_goal_async(goal)
        future.add_done_callback(lambda f: self.accepted(f, x, y, generation))

    def accepted(self, future, x, y, generation):
        handle = future.result()
        if generation != self.generation:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.busy = False
        if not handle.accepted:
            self.status('Nav2 rejected destination.')
            self.arm.call_async(SetBool.Request(data=False))
            return
        self.goal_handle = handle
        self.status(f'Navigating to ({x:.1f}, {y:.1f}) m. Click to redirect; Stop to cancel.')
        handle.get_result_async().add_done_callback(lambda f: self.finished(f, generation))

    def finished(self, future, generation):
        if generation != self.generation:
            return
        self.goal_handle = None
        response = future.result()
        self.arm.call_async(SetBool.Request(data=False))
        if response.status == 4 and response.result.error_code == 0:
            self.status('Arrived. Click another water destination to continue.')
        else:
            self.status(f'Navigation ended: status={response.status}, {response.result.error_msg}. Boat disarmed.')


def main():
    from .runtime import spin
    spin(ClickGoal)
