"""Live smoke test; requires the simulator, Nav2 and click_goal already running.

Moves the simulated boat four metres along its current heading, then tests Stop.
Run explicitly with python3; this is deliberately not a pytest collection file.
"""
import json
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.stringmsg_pb2 import StringMsg


def main():
    rclpy.init()
    node = rclpy.create_node('click_goal_probe')
    poses, safety, statuses = [], [], []
    node.create_subscription(PoseStamped, '/nav/ground_truth', lambda m: poses.append(m), 10)
    node.create_subscription(String, '/nav/safety_status', lambda m: safety.append(json.loads(m.data)), 10)
    gz = GzNode()
    gz.subscribe(StringMsg, '/nav/click_goal/status', lambda m: statuses.append(m.data))
    goals = gz.advertise('/nav/click_goal/world', Vector3d)
    stop = gz.advertise('/nav/click_goal/stop', Boolean)

    def until(predicate, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                return
        raise AssertionError(f'Timed out. Last status: {statuses[-3:]}')

    try:
        until(lambda: poses and safety and statuses, 15)
        goals.publish(Vector3d(x=0., y=0., z=0.))
        until(lambda: 'outside navigable water' in statuses[-1], 5)
        print('PASS: out-of-map click rejected', flush=True)
        p = poses[-1].pose
        yaw = 2*math.atan2(p.orientation.z, p.orientation.w)
        x, y = p.position.x+4*math.cos(yaw), p.position.y+4*math.sin(yaw)
        c, s = math.cos(1.), math.sin(1.)
        world = Vector3d(x=-532+c*x-s*y, y=162+s*x+c*y, z=0.)
        goals.publish(world)
        until(lambda: statuses[-1].startswith('Navigating'), 10)
        print(f'Navigating to {x:.2f}, {y:.2f}', flush=True)
        until(lambda: statuses[-1].startswith('Arrived'), 75)
        p = poses[-1].pose.position
        error = math.hypot(p.x-x, p.y-y)
        assert error < 1., error
        print(f'PASS: reached goal, error {error:.3f} m', flush=True)
        # Send a second goal, then stop while it is active.
        goals.publish(Vector3d(x=world.x+4*c, y=world.y+4*s, z=0.))
        until(lambda: statuses[-1].startswith('Navigating'), 10)
        stop.publish(Boolean(data=True))
        until(lambda: statuses[-1].startswith('Stopped') and not safety[-1]['armed'], 10)
        print('PASS: Stop canceled goal and disarmed thrust', flush=True)
    finally:
        stop.publish(Boolean(data=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
