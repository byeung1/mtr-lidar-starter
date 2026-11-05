"""Live motion evidence; run in a sourced ROS shell alongside navigation.launch.py."""
import argparse
import json
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=120)
p.add_argument('--goal', nargs=3, type=float)
p.add_argument('--waypoints', help='JSON array of [x,y,yaw]')
p.add_argument('--command', nargs=2, type=float)
p.add_argument('--arm', action='store_true')
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('boat_live_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
start = last_print = time.monotonic()
state = dict(raw_clouds=0, filtered_clouds=0, odom_messages=0)
trajectory = []
def odom(m):
    q, pos = m.pose.pose.orientation, m.pose.pose.position
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    state.update(x=pos.x, y=pos.y, yaw=yaw, speed=m.twist.twist.linear.x,
                 sim_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    state['odom_messages'] += 1
    if not trajectory or time.monotonic()-start-trajectory[-1][0] > .2:
        trajectory.append((time.monotonic()-start, pos.x, pos.y, yaw))
def safety(m):
    state['safety'] = json.loads(m.data)
def cloud(key):
    def cb(m):
        state[key] += 1
    return cb
subs = [n.create_subscription(Odometry, '/nav/odom', odom, 10),
        n.create_subscription(String, '/nav/safety_status', safety, 10),
        n.create_subscription(PointCloud2, '/lidar/points', cloud('raw_clouds'), qos_profile_sensor_data),
        n.create_subscription(PointCloud2, '/nav/points', cloud('filtered_clouds'), qos_profile_sensor_data)]
cmd = n.create_publisher(Twist, '/nav/cmd_vel', 10)
arm = n.create_client(SetBool, '/nav/arm')
arm_future = goal_future = goal_handle = result_future = None
points = json.loads(a.waypoints) if a.waypoints else ([a.goal] if a.goal else [])
kind = FollowWaypoints if len(points) > 1 else NavigateToPose
action = ActionClient(n, kind, '/follow_waypoints' if len(points)>1 else '/navigate_to_pose')
sent = False
armed = not a.arm
try:
    while rclpy.ok() and time.monotonic()-start < a.seconds:
        rclpy.spin_once(n, timeout_sec=.05)
        now = time.monotonic()
        if a.arm and not armed and arm_future is None and arm.service_is_ready() and state.get('filtered_clouds', 0)>2:
            arm_future = arm.call_async(SetBool.Request(data=True))
        if arm_future is not None and arm_future.done():
            response = arm_future.result()
            armed = response.success
            print(json.dumps(dict(event='arm', success=response.success, message=response.message)), flush=True)
            arm_future = None
            if not armed:
                rclpy.spin_once(n, timeout_sec=.3)
        if armed and points and not sent and action.server_is_ready():
            goal = kind.Goal()
            poses=[]
            for x,y,yaw in points:
                pose=PoseStamped()
                pose.header.frame_id='map'
                pose.header.stamp=n.get_clock().now().to_msg()
                pose.pose.position.x,pose.pose.position.y=float(x),float(y)
                pose.pose.orientation.z,pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
                poses.append(pose)
            if len(points)>1:goal.poses=poses
            else:goal.pose=poses[0]
            goal_future=action.send_goal_async(goal)
            sent=True
        if goal_future is not None and goal_future.done() and goal_handle is None:
            goal_handle=goal_future.result()
            print(json.dumps(dict(event='goal',accepted=goal_handle.accepted)),flush=True)
            if goal_handle.accepted:result_future=goal_handle.get_result_async()
            else:break
        if result_future is not None and result_future.done():
            result=result_future.result()
            print(json.dumps(dict(event='result', status=result.status, result=str(result.result))),flush=True)
            break
        if a.command and armed:
            msg=Twist();msg.linear.x,msg.angular.z=a.command;cmd.publish(msg)
        if now-last_print>2:
            print(json.dumps(dict(wall_seconds=round(now-start,3),**state)),flush=True)
            last_print=now
finally:
    if rclpy.ok():
        if goal_handle and goal_handle.accepted and (result_future is None or not result_future.done()):
            cancel=goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(n,cancel,timeout_sec=2)
        if arm.service_is_ready():
            future=arm.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(n,future,timeout_sec=2)
        cmd.publish(Twist())
    print(json.dumps(dict(event='final',wall_seconds=round(time.monotonic()-start,3),
                          trajectory=trajectory,**state)),flush=True)
    n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
