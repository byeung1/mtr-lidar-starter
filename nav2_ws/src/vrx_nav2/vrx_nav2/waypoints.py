"""Jazzy NavigateToPose / FollowWaypoints client, coordinates in local map metres."""
import argparse
import json
import math
import time
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('points', help='JSON [[x,y,yaw_radians], ...] in map')
    args, ros_args = parser.parse_known_args()
    points = json.loads(args.points)
    if not points or any(len(p) != 3 or not all(math.isfinite(x) for x in p) for p in points):
        parser.error('Expected one or more finite [x,y,yaw] triples')
    rclpy.init(args=ros_args)
    node = rclpy.create_node('boat_waypoint_client', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    action_type = NavigateToPose if len(points) == 1 else FollowWaypoints
    client = ActionClient(node, action_type, '/navigate_to_pose' if len(points) == 1 else '/follow_waypoints')
    if not client.wait_for_server(timeout_sec=30):
        raise RuntimeError('Nav2 action server unavailable')
    goal = action_type.Goal()
    poses = []
    for x, y, yaw in points:
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        poses.append(pose)
    if len(points) == 1:
        goal.pose = poses[0]
    else:
        goal.poses = poses
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle or not handle.accepted:
        raise RuntimeError('Goal rejected')
    result = handle.get_result_async()
    try:
        while rclpy.ok() and not result.done():
            rclpy.spin_once(node, timeout_sec=.2)
        if result.done():
            print(result.result(), flush=True)
    except KeyboardInterrupt:
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
