"""Read-only wall/sim timing and per-process CPU measurements."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=60)
p.add_argument('--warmup', type=float, default=15)
p.add_argument('--output', required=True)
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('pipeline_profiler')
started = time.monotonic()
data = {'clock': [], 'raw': [], 'filtered': [], 'safety': []}

def clock(msg):
    data['clock'].append((time.monotonic(), msg.clock.sec+msg.clock.nanosec*1e-9))

def cloud(key, msg):
    data[key].append((time.monotonic(), msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9))

subs = [n.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
        n.create_subscription(PointCloud2, '/lidar/points', lambda m: cloud('raw', m), qos_profile_sensor_data),
        n.create_subscription(PointCloud2, '/nav/points', lambda m: cloud('filtered', m), qos_profile_sensor_data),
        n.create_subscription(String, '/nav/safety_status', lambda m: data['safety'].append(json.loads(m.data)), 10)]

def processes():
    result = {}
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            cmd = (proc/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
            name = (proc/'comm').read_text().strip()
            if '/lib/vrx_nav2/' not in cmd and name != 'rviz2' and not cmd.startswith('gz sim '):
                continue
            # Linux stat's comm can contain spaces; fields after its final ')' are stable.
            stat = (proc/'stat').read_text().rsplit(')', 1)[1].split()
            result[proc.name] = dict(command=cmd, cpu_ticks=int(stat[11])+int(stat[12]),
                                     rss_kb=int(stat[21])*os.sysconf('SC_PAGE_SIZE')//1024)
        except (OSError, ValueError):
            pass
    return result

cpu_start = None
try:
    while time.monotonic()-started < a.seconds:
        rclpy.spin_once(n, timeout_sec=.1)
        if cpu_start is None and time.monotonic()-started >= a.warmup:
            cpu_start, cpu_time = processes(), time.monotonic()
finally:
    end = time.monotonic()
    cpu_end = processes()
    result = {'wall_seconds': end-started, 'warmup_seconds': a.warmup, 'processes': []}
    if cpu_start is not None:
        for pid, proc in cpu_end.items():
            if pid in cpu_start:
                proc['cpu_percent'] = 100*(proc['cpu_ticks']-cpu_start[pid]['cpu_ticks'])/os.sysconf('SC_CLK_TCK')/(end-cpu_time)
                result['processes'].append(proc)
    for key in ('raw', 'filtered'):
        values = [v for v in data[key] if v[0]-started >= a.warmup]
        result[key] = dict(messages=len(values),
                          max_wall_gap=max([b[0]-x[0] for x, b in zip(values, values[1:])], default=None),
                          sim_first=values[0][1] if values else None,
                          sim_last=values[-1][1] if values else None)
    clocks = [v for v in data['clock'] if v[0]-started >= a.warmup]
    if len(clocks)>1:
        result['real_time_factor'] = (clocks[-1][1]-clocks[0][1])/(clocks[-1][0]-clocks[0][0])
    result['safety_latest'] = data['safety'][-1] if data['safety'] else None
    result['safety_reasons'] = sorted({s['reason'] for s in data['safety']})
    Path(a.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
