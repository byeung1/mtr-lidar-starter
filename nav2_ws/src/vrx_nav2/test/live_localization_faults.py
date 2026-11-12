"""Isolated ROS fail-safe test in DDS domain 87; NEVER reaches boat thrust.

Run from a sourced shell: python3 nav2_ws/src/vrx_nav2/test/live_localization_faults.py
All sensor/control/TF interfaces are remapped under /localization_test. Synthetic
EKF output keeps advancing while each raw sensor is withheld independently.
"""
import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool
from vrx_nav2.core import HULL_FRAME, BASE_FRAME
from vrx_nav2.sensor_localization import SensorLocalization
from vrx_nav2.thrust_adapter import ThrustAdapter


def main():
    topics = ['/tf', '/tf_static', '/wamv/sensors/imu/imu/data', '/wamv/sensors/gps/gps/fix',
              '/nav/localization/imu', '/nav/localization/gps', '/nav/localization/filtered',
              '/nav/localization/status', '/nav/odom', '/nav/cmd_vel', '/nav/safety_points',
              '/nav/safety_status', '/nav/cloud_status', '/nav/arm']
    topics += [f'/wamv/thrusters/{side}/{kind}' for side in ('left', 'right') for kind in ('thrust', 'pos')]
    # Include argv[0]: rclpy treats the first element as the executable name.
    # A separate DDS domain provides isolation even if a remap is malformed.
    args = ['localization_fault_probe', '--ros-args']
    for topic in topics:
        args += ['-r', f'{topic}:=/localization_test{topic}']
    rclpy.init(args=args, domain_id=87)
    sensor, adapter = SensorLocalization(), ThrustAdapter()
    probe = rclpy.create_node('localization_fault_probe')
    executor = SingleThreadedExecutor()
    for node in (sensor, adapter, probe):
        executor.add_node(node)
    for name, lever in [('test_imu', (.3, -.2, 1.3)), ('test_gps', (-.85, 0., 1.3))]:
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = HULL_FRAME, name
        t.transform.rotation.w = 1.
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = lever
        sensor.buffer.set_transform_static(t, 'fixture')
    pubs = {k: probe.create_publisher(c, topic, 10) for k, c, topic in [
        ('imu', Imu, '/wamv/sensors/imu/imu/data'), ('gps', NavSatFix, '/wamv/sensors/gps/gps/fix'),
        ('ekf', Odometry, '/nav/localization/filtered'), ('cloud', PointCloud2, '/nav/safety_points'),
        ('cmd', Twist, '/nav/cmd_vel')]}
    state = {}
    probe.create_subscription(String, '/nav/safety_status', lambda m: state.update(json.loads(m.data)), 10)
    arm = probe.create_client(SetBool, '/nav/arm')

    def step(drop=None):
        stamp = probe.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            executor.spin_once(timeout_sec=.05)
            return
        imu = Imu()
        imu.header = Header(stamp=stamp, frame_id='test_imu')
        imu.orientation.w = 1.
        gps = NavSatFix()
        gps.status.status = 0
        gps.header = Header(stamp=stamp, frame_id='test_gps')
        gps.latitude, gps.longitude, gps.altitude = -33., 150., 1.3
        if drop != 'imu':
            pubs['imu'].publish(imu)
        if drop != 'gps':
            pubs['gps'].publish(gps)
        ekf = Odometry()
        ekf.header = Header(stamp=stamp, frame_id='odom')
        ekf.child_frame_id = HULL_FRAME
        ekf.pose.pose.orientation.w = 1.
        pubs['ekf'].publish(ekf)
        pubs['cloud'].publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=BASE_FRAME), [[50., 50., 1.]]))
        cmd = Twist()
        cmd.linear.x = .1
        pubs['cmd'].publish(cmd)
        end = time.monotonic()+.05
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)

    def run(seconds, drop=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            step(drop)

    start = time.monotonic()
    try:
        for missing in ('gps', 'imu'):
            run(3.)
            assert sensor.datum is not None, (sensor.error, sensor.fresh.samples,
                                              probe.get_topic_names_and_types())
            assert arm.service_is_ready(), 'isolated arm service unavailable'
            future = arm.call_async(SetBool.Request(data=True))
            while not future.done():
                step()
            assert future.result().success, future.result().message
            run(.5)
            assert state.get('armed') and abs(state.get('left_N', 0))+abs(state.get('right_N', 0)) > 0, state
            run(2.5, missing)
            assert not state.get('armed') and state.get('last_fault') == 'stale_pose', state
            assert state.get('left_N') == 0 and state.get('right_N') == 0, state
            run(1.)
            assert not state.get('armed'), 'sensor recovery must not auto-arm'
            print(json.dumps({'missing_sensor': missing, 'passed': True, 'state': state}), flush=True)
        print(json.dumps({'duration_wall_seconds': time.monotonic()-start, 'passed': True}))
    finally:
        executor.shutdown()
        for node in (sensor, adapter, probe):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
