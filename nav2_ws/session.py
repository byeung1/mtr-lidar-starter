"""Own one launch process group and clean up descendants even if ros2 exits first."""
import argparse
import os
import signal
import subprocess
import time

p=argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=0)
p.add_argument('command', nargs=argparse.REMAINDER)
a=p.parse_args()
if not a.command:p.error('command required')
proc=subprocess.Popen(a.command, start_new_session=True)
start=time.monotonic()
try:
    while proc.poll() is None and (not a.seconds or time.monotonic()-start<a.seconds):
        time.sleep(.2)
except KeyboardInterrupt:
    pass
finally:
    # Let ros2 launch shut down its children first, instead of interrupting every
    # rclpy node and then having launch send a second simultaneous interruption.
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:pass
    for sig,delay in [(signal.SIGTERM,2),(signal.SIGKILL,0)]:
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:break
        end=time.monotonic()+delay
        while time.monotonic()<end:
            try:os.killpg(proc.pid,0)
            except ProcessLookupError:break
            time.sleep(.1)
    proc.wait()
