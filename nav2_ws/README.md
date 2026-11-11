# Start VRX, Nav2, waypoints and the viewers

These instructions start from **no running simulation processes**. Use a separate
Ubuntu/WSL terminal for each numbered section. Every block includes its own
working directory and environment setup.

After updating the code, rebuild once with all simulations stopped:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
cd nav2_ws
colcon build --base-paths src --symlink-install
```

Recommended startup order: **Terminal 1 → Terminal 4 → Terminal 5 → Terminal 2**.
Leave Terminal 3 (Pygame) closed unless you need it. Let both GUI windows finish
loading, then wait about five seconds before arming in Terminal 2.

The current navigation profile renders the three cameras at 2 Hz instead of
30 Hz. LiDAR remains at 10 Hz. A measured comparison improved headless progress
from about 0.51× to 1.00× real time; the combined Intel Gazebo + NVIDIA RViz test
ran at about 0.96×. These are bounded tests on this laptop, not a universal rate
guarantee. See [the performance diagnosis and results](results/PERFORMANCE.md).

## Click a destination directly in Gazebo

The **Boat navigation** panel sends water-surface clicks to Nav2. It uses the
same planner, LiDAR costmaps and guarded thruster adapter as the waypoint client.
The boat travels to a fixed clicked destination; it does not continuously follow
the mouse. Click again to cancel the old goal and request a new destination.

Build the ROS package after pulling these changes. From the repository root,
source ROS and VRX, then run `colcon build --symlink-install --packages-select vrx_nav2`.
If using the nested workspace build described above, build there instead.
Start `navigation.launch.py` as usual (Terminal 1); it now includes the
`click_goal` bridge. For a simulation that was already running before the update,
source its install environment and run this in a separate terminal:

```bash
ros2 run vrx_nav2 click_goal --ros-args -p use_sim_time:=true
```

Open the new viewer **instead of Terminal 4's viewer**:

```bash
# From the repository root, in Ubuntu/WSL:
bash nav2_ws/click_viewer.sh
```

The script compiles the Gazebo GUI plugin using the installed Gazebo 8 / Qt 5
development libraries. If development packages are missing, install
`ros-jazzy-gz-gui-vendor`, `ros-jazzy-gz-rendering-vendor`,
`ros-jazzy-gz-transport-vendor`, `qtdeclarative5-dev`, `cmake` and `g++`.
The viewer uses the simpler Ogre renderer on Intel; the sky effect is omitted.
The existing server retains its original rendering and sensor configuration.

1. Enable **Click to navigate** in the Boat navigation panel.
2. Left-click water near the boat. Dragging still moves the camera.
3. Read the destination and navigation result in the panel.
4. Press **Stop boat** to request cancellation and disarm thrust.

Each new destination requests arming through `/nav/arm`; stale sensors or other
arming faults are reported in the panel. Arrival and navigation failure disarm
the boat. Do not send waypoint or RViz goals at the same time as GUI goals.
Keep the bridge terminal running if it was started separately.

Clicks are camera-ray intersections with mean water level `z=0`, transformed
using the same default origin as simulation localization: world
`(-532, 162)`, heading `1` radian. The bridge rejects points outside the static
water map or on its occupied/unknown cells. Nav2 checks its LiDAR obstacle
costmaps during planning and control; a blocked destination may fail instead of
being reachable. Waves, camera projection and the 0.8 m goal tolerance mean
arrival is approximate. This tool is for this simulated Sydney world only.
If changing localization's origin parameters, set identical values on
`click_goal` as well.

Validation: the automated coordinate/navigation suite is under `src/vrx_nav2/test`.
`test/click_goal_probe.py` is an explicit live smoke test that moves the boat
four metres, checks arrival, and tests cancellation/disarming.

## Terminal 1 — simulation and Nav2

Paste:

```bash
cd ~/mtr-lidar-starter
bash nav2_ws/run.sh
```

Leave this terminal running. It starts **one headless Sydney Regatta simulation**,
the LiDAR and simulation-clock bridges, localization/TF, Nav2 and the guarded
thruster adapter. No separate LiDAR bridge is needed.

Wait for:

```text
Managed nodes are active
```

The boat starts disarmed. The Gazebo window is opened separately in Terminal 4.
The launch uses a temporary navigation model; it does not edit the installed VRX
source. All sensors remain present, and the LiDAR configuration is unchanged.
If you need the original 30 Hz camera feeds, start with
`bash nav2_ws/run.sh camera_rate:=30.0`; expect higher rendering load.
Do not also launch `competition.launch.py` or another simulator.

## Terminal 2 — arm and send waypoints

When Terminal 1 is ready, paste:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 service call /nav/arm std_srvs/srv/SetBool '{data: true}'
```

Check that the response says `success=True` and `message='armed'`. Arming now
requires three fresh clouds and one second of stable inputs. If it reports
`waiting for three fresh clouds` or stale pose/LiDAR, wait for the inputs and
retry once after the windows have finished loading; do not bypass the gate or
keep re-arming repeatedly if a fault returns.

For a first goal four metres forward, paste:

```bash
ros2 run vrx_nav2 waypoints '[[4,0,0]]'
```

After that completes, try two more waypoints:

```bash
ros2 run vrx_nav2 waypoints '[[7,0,0],[10,1,0]]'
```

Alternatively, immediately after a fresh simulation start, this is the tested
short two-waypoint sequence:

```bash
ros2 run vrx_nav2 waypoints '[[4,0,0],[7,0,0]]'
```

Run one waypoint command at a time. Each triple is `[x, y, yaw]`, in **metres and
radians**, relative to the local map origin at the boat's nominal starting pose.
+X is its initial forward direction; +Y is left. Coordinates are absolute in this
local map, not relative to the boat's current position. Start with forward goals.

`status=4`, `error_code=0` means the navigation action succeeded. For multiple
waypoints, also check `missed_waypoints=[]`. The configured position tolerance is
0.8 m. The boat can coast after thrust reaches zero.

To inspect the safety gate in this terminal when no waypoint client is running:

```bash
ros2 topic echo /nav/safety_status
```

Ctrl+C stops that status display. A sensor fault disarms the adapter; once the
fault is resolved, explicitly arm again before sending another goal.

## Terminal 3 — optional Pygame costmap viewer (normally leave closed)

Paste:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
source install/local_setup.bash
ros2 run mtr_lidar_starter costmap_viewer --ros-args -p use_sim_time:=true
```

This opens your existing Pygame viewer and uses the live `/lidar/points` stream.
It does **not** replay a recording. Leave the window open while navigating.

The Pygame grid is your original custom sensor-frame costmap: red is obstacle
returns, green is ray-observed space, and gray is unknown. It is **separate from
Nav2's costmaps**, which are displayed in RViz. Its height filtering also differs
from the Nav2 filter. Press Escape or close the Pygame window to stop this viewer.

Do not use the starter's top-level `run.sh` here: that workflow plays a bag by
default, which would introduce recorded points and another simulation clock.

## Terminal 4 — Gazebo simulation window

Paste:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
env -u LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER=d3d12 \
  MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel gz sim -g
```

`-g` opens **only the GUI** and attaches to the simulator already running in
Terminal 1. It does not start another physics server. You can watch the boat move
while Nav2 and the other viewers run.

This selects **Intel acceleration only for the GUI**. Terminal 1 retains NVIDIA
for the simulation's GPU LiDAR; Terminal 5 uses NVIDIA for RViz. Ogre2 and the
sensors are unchanged. Pygame is optional and can be left closed.

The NVIDIA GUI command crashed on this laptop. The llvmpipe software fallback
used about four CPU cores and sensor delays triggered a safety stop. The separate
Intel GUI is the tested choice for this navigation profile. After the processing
and camera-rate fixes, the combined-viewer test ran at about 0.96× real time,
with a maximum observed filtered-cloud gap of 0.62 seconds, and completed the
(4,0) and (7,0) waypoint mission. Long-term driver stability is not established.
Avoid opening/restarting the GUI during a mission: initialize both viewers
before arming. If a fault returns, keep the warning details and diagnose it
rather than repeatedly re-arming.

## Terminal 5 — RViz with Nav2 costmaps

Paste:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  rviz2 -d nav2_ws/src/vrx_nav2/config/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

The supplied configuration uses `map` as the fixed frame and includes:

- WAM-V model and TF axes.
- Filtered LiDAR from `/nav/points`.
- Bounded-water map prior.
- Global and local Nav2 costmaps.
- Global path, controller path, lookahead point and boat footprint.

The boat must be armed from Terminal 2 before moving. Use the Nav2 goal tool in
RViz to select a position and forward heading, or use the waypoint commands in
Terminal 2. Do not send both at once. No AMCL initial-pose step is needed for this
simulation ground-truth baseline.

## Stop everything

First cancel an active waypoint client with Ctrl+C in Terminal 2. Then disarm:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 service call /nav/arm std_srvs/srv/SetBool '{data: false}'
```

Close Pygame, Gazebo GUI and RViz, or Ctrl+C their respective terminals.
Finally, Ctrl+C **Terminal 1** and allow several seconds for the wrapper to stop
Nav2 and its Gazebo process group. Start the next session from Terminal 1 again.

## Rebuild only after changing package files

With the simulation stopped:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
cd nav2_ws
colcon build --base-paths src --symlink-install
```

You do not need to rebuild each time you start the simulation. The warning about
`.deps/root/opt/ros/jazzy` lacking `local_setup.*` files is expected for our locally
extracted Nav2 dependencies. A successful build ends with `1 package finished`.

## Understanding slow motion and stops

The boat is intentionally limited to **0.45 m/s** and slows further near a goal.
At real-time simulation speed that is about one metre every 2.2 seconds. A low
simulation rate makes this look even slower; the fixes target simulation
throughput, not a higher boat-speed limit.

Read the gate from Terminal 2:

```bash
ros2 topic echo /nav/safety_status
```

- `running`: fresh command and required inputs; thrust control is active.
- `command_timeout`: no fresh velocity command; zero thrust. This is expected
  after a goal finishes, and does not itself latch disarmed.
- `stale_lidar` / `stale_pose`: required data was too old; zero thrust and latched
  disarm. Even if the stream recovers, the boat requires explicit re-arming.
- `disarmed`: not armed. Check `last_fault` and `last_fault_details` to see whether
  a sensor fault caused it. Successful manual re-arming acknowledges/clears that
  fault from the current status.

The warning `Disarmed: ... {...}` in Terminal 1 now records wall-time and
simulation-time ages and the cloud-pipeline diagnostic. Copy that whole line if
another unexplained stop occurs. More detail is available on `/nav/cloud_status`.

The improved cloud node queues up to eight recent samples while clock/TF data
catches up; it never blocks waiting for a transform, restamps a cloud or reuses
one as a fresh heartbeat. Nav2 keeps the LiDAR-frame points for correct clearing.
The adapter uses `/nav/safety_points`, the same points already transformed to
`nav_base`, so it does not perform a second TF wait/transform.

**Safety timeouts were not increased:** pose is limited to 1 wall second or
0.8 simulated seconds, LiDAR to 3 wall seconds or 1.5 simulated seconds, and
commands to 1 wall second. Stopping thrust is not an instantaneous physical stop;
the boat can coast or drift.

## What is validated

The baseline completed the two forward waypoints `(4,0)` and `(7,0)` in simulation,
and 24 automated conversion/control/queue/profile tests passed. The revised
mission also completed with both GUI windows open. Deliberately pausing the
LiDAR-processing node and pose node, and interrupting velocity commands, each
produced the expected zero-thrust response in live tests. Obstacle detours,
narrow gaps and full collision-clearance acceptance tests are **not yet
validated**. The map's
free interior is an assumed bounded-water test course, not surveyed free space.

See [the baseline test report](results/BASELINE.md) for actual measurements, and
[architecture and implementation details](ARCHITECTURE.md) for frames, sensors,
costmap assumptions, dependencies and safety limits.

Successful simulation is a development milestone, not evidence of readiness for
autonomous operation on the physical boat.
