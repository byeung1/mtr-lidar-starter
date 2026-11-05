"""Bounded test; only stops its own process groups. Run in a sourced ROS shell."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

p = argparse.ArgumentParser()
p.add_argument('--mode', choices=['gui', 'headless'], default='gui')
p.add_argument('--adapter', default='NVIDIA')
p.add_argument('--seconds', type=int, default=600)
p.add_argument('--obstacle', action='store_true')
p.add_argument('--rviz-at', type=int, help='Add RViz after this many wall seconds')
p.add_argument('--output', required=True)
a = p.parse_args()
root = Path(__file__).resolve().parent.parent
out = root / a.output
out.mkdir(parents=True, exist_ok=False)
env = os.environ.copy()
env['VRX_ADAPTER'] = a.adapter
env['PYTHONPATH'] = str(root / 'src/mtr_lidar_starter') + ':' + env.get('PYTHONPATH', '')
children = []
files = []
start = time.monotonic()
stop_reason = 'duration reached'

def launch(cmd, filename):
    f = (out / filename).open('w')
    files.append(f)
    proc = subprocess.Popen(cmd, cwd=root, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(proc)
    return proc

def service(name, reqtype, request, filename):
    with (out / filename).open('w') as f:
        return subprocess.run(['gz', 'service', '-s', '/world/sydney_regatta/' + name,
            '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
            '--req', request], env=env, stdout=f, stderr=subprocess.STDOUT, timeout=10).returncode

def resources():
    processes = []
    for path in Path('/proc').glob('[0-9]*'):
        try:
            if os.getpgid(int(path.name)) != sim.pid:
                continue
            cmd = (path / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
            if cmd.startswith('gz sim '):
                status = (path / 'status').read_text().splitlines()
                rss = next(line for line in status if line.startswith('VmRSS:'))
                processes.append(dict(pid=int(path.name), command=cmd, rss=rss))
        except (OSError, StopIteration):
            pass
    memory = {line.split(':')[0]: line.split(':')[1].strip()
              for line in Path('/proc/meminfo').read_text().splitlines()
              if line.split(':')[0] in ('MemAvailable', 'SwapFree', 'SwapTotal')}
    with (out / 'resources.jsonl').open('a') as f:
        f.write(json.dumps(dict(wall_seconds=time.monotonic()-start,
                               processes=processes, memory=memory)) + '\n')

try:
    sim = launch(['bash', 'scripts/vrx-wsl.sh', a.mode], 'simulation.log')
    bridge = launch(['bash', 'scripts/vrx-wsl.sh', 'bridge'], 'bridge.log')
    monitor = launch(['/usr/bin/python3', 'scripts/vrx-health.py', '--seconds', str(a.seconds)], 'health.jsonl')
    inserted = removed = rviz_started = False
    last_resources = -30
    while time.monotonic() - start < a.seconds:
        elapsed = time.monotonic() - start
        if elapsed - last_resources >= 30:
            resources()
            last_resources = elapsed
        if a.rviz_at is not None and elapsed > a.rviz_at and not rviz_started:
            launch(['bash', 'scripts/vrx-wsl.sh', 'rviz'], 'rviz.log')
            rviz_started = True
        if a.obstacle and elapsed > 60 and not inserted:
            sdf = '<sdf version="1.9"><model name="wsl_stability_box"><static>true</static><pose>-528 168 2 0 0 0</pose><link name="box"><collision name="collision"><geometry><box><size>2 2 4</size></box></geometry></collision><visual name="visual"><geometry><box><size>2 2 4</size></box></geometry></visual></link></model></sdf>'
            service('create', 'gz.msgs.EntityFactory', 'sdf: ' + json.dumps(sdf), 'insert.log')
            inserted = True
        if inserted and not removed and elapsed > a.seconds - 60:
            service('remove', 'gz.msgs.Entity', 'name: "wsl_stability_box", type: MODEL', 'remove.log')
            removed = True
        if sim.poll() is not None or monitor.poll() is not None:
            stop_reason = 'simulation or monitor exited'
            break
        time.sleep(1)
except KeyboardInterrupt:
    stop_reason = 'interrupted by operator'
finally:
    duration = time.monotonic() - start
    (out / 'result.json').write_text(json.dumps(dict(duration_seconds=duration, mode=a.mode,
        adapter=a.adapter, stop_reason=stop_reason,
        sim_returncode=sim.poll(), monitor_returncode=monitor.poll()), indent=2) + '\n')
    for proc in reversed(children):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
    deadline = time.monotonic() + 15
    for proc in children:
        try:
            proc.wait(timeout=max(0.1, deadline-time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    for f in files:
        f.close()
