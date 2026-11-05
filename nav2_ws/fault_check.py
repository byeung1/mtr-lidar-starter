"""Bounded live adapter tests. Run only after navigation goals have finished."""
import json
import os
from pathlib import Path
import signal
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool

rclpy.init()
n = rclpy.create_node('adapter_fault_test')
state = {}
def status(msg):
    state.clear()
    state.update(json.loads(msg.data))
sub = n.create_subscription(String, '/nav/safety_status', status, 10)
pub = n.create_publisher(Twist, '/nav/cmd_vel', 10)
arm = n.create_client(SetBool, '/nav/arm')
results = []
paused = None

def spin_for(seconds, command=False):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        if command:
            msg = Twist()
            msg.linear.x = .2
            pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=.05)

def set_arm(value):
    if not arm.wait_for_service(timeout_sec=3):
        raise RuntimeError('No arm service')
    f = arm.call_async(SetBool.Request(data=value))
    rclpy.spin_until_future_complete(n, f, timeout_sec=3)
    if not f.done():
        raise RuntimeError('Arm service timed out')
    return f.result()

def process_for(executable):
    found = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            args = (proc/'cmdline').read_bytes().decode(errors='replace')
            if '/lib/vrx_nav2/'+executable+'\0' in args:
                found.append(int(proc.name))
        except OSError:
            pass
    if len(found)!=1:
        raise RuntimeError('Expected one '+executable+', found '+str(found))
    return found[0]

try:
    spin_for(3)
    for kind, executable, wait in [('lidar', 'cloud_filter', 4.2),
                                    ('pose', 'ground_truth', 2.2),
                                    ('command', None, 1.8)]:
        set_arm(False)
        deadline = time.monotonic()+10
        while not set_arm(True).success:
            if time.monotonic()>deadline:
                raise RuntimeError('Inputs did not become ready')
            spin_for(.3)
        spin_for(2., command=True)
        before = dict(state)
        if abs(before.get('left_N', 0))+abs(before.get('right_N', 0))<.1:
            raise RuntimeError('No positive actuation before fault injection')
        if executable:
            paused = process_for(executable)
            os.kill(paused, signal.SIGSTOP)
        spin_for(wait, command=bool(executable))
        after = dict(state)
        expected = {'lidar': 'stale_lidar', 'pose': 'stale_pose', 'command': 'command_timeout'}[kind]
        passed = (after.get('reason')==expected and after.get('left_N')==0.
                  and after.get('right_N')==0. and (kind=='command' or not after.get('armed')))
        result = dict(test=kind, passed=passed, before=before, after=after)
        results.append(result)
        print(json.dumps(result), flush=True)
        if paused is not None:
            os.kill(paused, signal.SIGCONT)
            paused = None
        spin_for(2.5)
        set_arm(False)
        if not passed:
            raise RuntimeError('Fail-safe test failed: '+kind)
finally:
    if paused is not None:
        os.kill(paused, signal.SIGCONT)
    if rclpy.ok():
        try:
            set_arm(False)
            pub.publish(Twist())
        except Exception:
            pass
    Path('nav2_ws/results/fault-injection.json').write_text(json.dumps(results, indent=2)+'\n')
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
