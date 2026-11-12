"""Gazebo GUI world-plane clicks -> simulation-only NavigateToPose goals."""
import math
import queue
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy
from gz.transport13 import Node as GzNode
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.pose_v_pb2 import Pose_V

from .click_geometry import aligned_world_to_map
from .core import quat_yaw, BASE_FRAME


class ClickGoal(Node):
    def __init__(self):
        super().__init__('gazebo_click_goal')
        self.events = queue.Queue(maxsize=16)
        self.gz = GzNode()
        self.status_pub = self.gz.advertise('/nav/click_goal/status', StringMsg)
        self.gz.subscribe(Vector3d, '/nav/click_goal/world', self.on_click)
        self.gz.subscribe(Boolean, '/nav/click_goal/stop', self.on_stop)
        # GUI-coordinate alignment only: never publishes navigation pose/TF.
        self.world_pose = None
        self.gz.subscribe(Pose_V, '/world/sydney_regatta/pose/info', self.on_world_pose)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tree = str(Path(get_package_share_directory('vrx_nav2')) / 'behavior_trees' / 'click_to_pose.xml')
        self.arm = self.create_client(SetBool, '/nav/arm')
        self.pose = None
        self.pose_wall = -math.inf
        self.grid = None
        self.goal_handle = None
        self.busy = False
        self.generation = 0
        self.text = 'Ready: enable Click to navigate, then click the water.'
        self.create_subscription(Odometry, '/nav/odom', self.on_pose, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_client = self.create_client(GetMap, '/map_server/map')
        self.map_request = None
        self.map_requested_at = 0.
        self.create_timer(1., self.fetch_map, clock=Clock(clock_type=ClockType.STEADY_TIME))
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
        if msg.header.frame_id != 'odom' or msg.child_frame_id != BASE_FRAME:
            return
        pose = PoseStamped()
        pose.header, pose.pose = msg.header, msg.pose.pose
        self.pose, self.pose_wall = pose, time.monotonic()

    def on_world_pose(self, msg):
        for pose in msg.pose:
            if pose.name == 'wamv':
                q = pose.orientation
                self.world_pose = (msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                                   time.monotonic(), (pose.position.x, pose.position.y,
                                   quat_yaw((q.x, q.y, q.z, q.w))))
                break

    def on_map(self, msg):
        if (msg.header.frame_id == 'map' and msg.info.resolution > 0 and msg.info.width and msg.info.height
                and len(msg.data) == msg.info.width * msg.info.height):
            self.grid = msg

    def fetch_map(self):
        # Request the active grid if initial transient-local delivery was missed.
        if self.grid is not None or not self.map_client.service_is_ready():
            return
        now = time.monotonic()
        if self.map_request is not None:
            if not self.map_request.done():
                if now-self.map_requested_at < 5.:
                    return
                self.map_client.remove_pending_request(self.map_request)
            else:
                response = self.map_request.result()
                if response is not None:
                    self.on_map(response.map)
                    if self.grid is not None:
                        self.get_logger().info('Loaded the water map from the map server service.')
                        return
        self.map_requested_at = now
        self.map_request = self.map_client.call_async(GetMap.Request())

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
        if not self.pose or time.monotonic()-self.pose_wall > 2.:
            self.status('Cannot navigate: fresh boat position unavailable.')
            return
        stamp = self.pose.header.stamp.sec+self.pose.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        world = self.world_pose
        if (world is None or time.monotonic()-world[1] > 1.
                or abs(stamp-world[0]) > .15 or not -.1 <= now-stamp <= .8):
            self.status('Cannot navigate: fresh synchronized GUI/boat alignment unavailable.')
            return
        p, q = self.pose.pose.position, self.pose.pose.orientation
        try:
            x, y = aligned_world_to_map(*value, world[2],
                                       (p.x, p.y, quat_yaw((q.x, q.y, q.z, q.w))))
        except ValueError as error:
            self.status(str(error))
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
        goal.behavior_tree = self.tree
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
            reasons = {
                ComputePathToPose.Result.NO_VALID_PATH: 'Nav2 could not find a clear route to the destination.',
                ComputePathToPose.Result.TIMEOUT: 'Nav2 reached its route search limit.',
                ComputePathToPose.Result.GOAL_OCCUPIED: 'The destination is blocked by an obstacle.',
                FollowPath.Result.NO_VALID_CONTROL: 'The controller stopped because it could not follow the path safely.',
                FollowPath.Result.FAILED_TO_MAKE_PROGRESS: 'The boat could not make progress along the route.',
            }
            reason = response.result.error_msg or reasons.get(
                response.result.error_code, f'Navigation ended (status {response.status}).')
            self.status(f'{reason} Boat disarmed.')


def main():
    from .runtime import spin
    spin(ClickGoal)
