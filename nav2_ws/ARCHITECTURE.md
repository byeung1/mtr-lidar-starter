# VRX + Nav2 baseline (simulation only)

This is a separate ROS workspace and package. It leaves the starter's NumPy
Processor, Pygame viewer, recordings, and VRX source unchanged. This first version
defaults to simulated GPS + IMU localization through a Jazzy robot_localization
EKF. Ground truth is an explicit fallback (`localization:=ground_truth`). It is a development baseline,
not a validated autonomous boat controller.

## Start

Only run one simulator and one LiDAR bridge. Stop earlier VRX launch/bridge
terminals first. From `~/mtr-lidar-starter`:

```bash
source nav2_ws/env.sh
cd nav2_ws
colcon build --base-paths src --symlink-install
cd ..
bash nav2_ws/run.sh
```

The wrapper launches headless Sydney Regatta, LiDAR and world-clock bridges,
GPS/IMU localization, cloud filtering, the thrust adapter, map server,
planner, controller, navigators and waypoint follower. It starts DISARMED.
Ctrl+C the wrapper to shut down its process group, including Gazebo descendants.

In another terminal, wait for Nav2's `Managed nodes are active` message and sensor
delivery, then arm explicitly:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 service call /nav/arm std_srvs/srv/SetBool '{data: true}'
ros2 run vrx_nav2 waypoints '[[4,0,0]]'
```

Several waypoints (all distances in metres, yaw in radians):

```bash
ros2 run vrx_nav2 waypoints '[[4,0,0],[8,1,0],[12,1,0]]'
```

The one-goal client uses Jazzy `NavigateToPose`; multiple points use
`FollowWaypoints`. The client prints the action result. An accepted goal is not
proof of reaching it: check final pose and result status. Ctrl+C requests cancel.

RViz, optional due to the laptop's graphics/memory issues:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  rviz2 -d nav2_ws/src/vrx_nav2/config/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

The configuration includes robot model, TF, filtered LiDAR, map, both costmaps,
global path, controller's transformed path, lookahead and footprint. Use the
Nav2 goal tool for a position and forward heading. The Nav2 panel supports queued
waypoints. No AMCL initial-pose step is needed for this GPS/IMU baseline.
Behind-the-boat goals, sharp headings and final-yaw alignment are deliberately
limited by forward-only operation; choose achievable forward goals first.

Stop and disarm:

```bash
ros2 service call /nav/arm std_srvs/srv/SetBool '{data: false}'
```

Do not also run `scripts/vrx-wsl.sh bridge`; this integration owns that bridge.
`start_sim:=false` can attach to an existing compatible simulation, but the
existing launch must not publish a competing ROS clock or localization TF.

## Dependencies and reversibility

ROS 2 Jazzy and the existing VRX installation remain the underlays. Nav2 was
missing and passwordless sudo was unavailable, so official binary packages were
downloaded with `apt-get download` and extracted with `dpkg-deb -x` into
`.deps/root`. No package scripts were run and no system packages were installed.
The exact versions are in `results/dependency-versions.txt` (Nav2 1.3.13).
`env.sh` activates these local libraries/packages only in the sourcing terminal.

The normal alternative, in your own terminal with sudo, is:

```bash
sudo apt-get install --no-install-recommends \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

If switching to system Nav2, rename `.deps` out of the way and start a fresh
terminal before sourcing `env.sh`. The package's runtime dependencies are also
declared in `src/vrx_nav2/package.xml`. The local binary cache is ignored by Git.
Remove this new workspace to undo the integration; first stop its processes.
Nothing was committed or pushed.

## November 2 performance revision

The current launch uses a temporary, generated navigation URDF with all three
camera update rates reduced from 30 to 2 Hz. This pipeline does not use camera
images. Sensors, camera resolution, dynamics, geometry, thrusters and the GPU
LiDAR settings are retained; installed VRX source is untouched. Set
`camera_rate:=30.0` to restore the stock camera update rate for a run.

The cloud processor now uses a bounded eight-sample pending queue to handle
clock/TF message ordering without blocking its executor. It publishes the
original LiDAR-frame `/nav/points` for Nav2 and matching, already transformed
`/nav/safety_points` in `nav_base` for the adapter. This removes the adapter's
second TF listener, lookup wait and matrix transform. The custom nodes use a
single-thread executor with nonblocking callbacks. Raw, processed, pending,
dropped and invalid counts are published on `/nav/cloud_status`.

All original freshness limits are retained. Arming additionally requires three
fresh clouds and one second of stable inputs. Fault snapshots include the last
pipeline diagnostic; successful re-arming clears the acknowledged status fault.
The main README is the current startup guide; historical results below are kept
for context. See [performance results](results/PERFORMANCE.md).

## Architecture and actual interfaces

```text
GPS + IMU -> local measurement conversion -> sensor EKF -> freshness gate
                                                         -> /nav/odom + TF
                                                 |
GPU LiDAR -> /lidar/points -> nav_cloud_filter -> /nav/points
                                                 |
map prior + obstacle/inflation layers -> Smac Hybrid planner -> RPP controller
                                                 |
                              /nav/cmd_vel (Twist)
                                                 |
                         guarded simulation thrust adapter
                                                 |
                   Float64 newtons -> VRX left/right thrusters
```

The inspected ROS sensor interfaces include
`/wamv/sensors/gps/gps/fix` (`NavSatFix`) and
`/wamv/sensors/imu/imu/data` (`Imu`). They are now fused in the default baseline.
VRX's default launch had sensor TF and joint states but no ROS odometry.

The following describes only the optional ground-truth fallback; the default
sensor pipeline, datum, safety gate and limitations are documented in
[LOCALIZATION.md](results/LOCALIZATION.md).

Ground truth subscribes natively to Gazebo
`/world/sydney_regatta/pose/info` and selects the model named `wamv`. This avoids
losing the model name when the world Pose_V is converted to ROS TFMessage.
The Gazebo message's simulation timestamp is retained; body velocity is estimated
from successive poses and smoothed. This is perfect-position simulation data,
not a realistic noise model.

The local map origin is the inspected spawn position `(-532,162)` in Gazebo,
rotated by the spawn yaw of 1 radian. Thus local +X is the original forward
direction; +Y is left. The chain is:

```text
map -> odom -> nav_base -> wamv/wamv/base_link
                        -> existing sensor TF ->
                           wamv/wamv/base_link/lidar_wamv_sensor
```

`map -> odom` is identity for this non-drifting ground-truth baseline.
`odom -> nav_base` is horizontal pose; `nav_base ->` the real hull frame retains
heave, roll and pitch. Existing VRX robot_state_publisher/sensor broadcasters
remain responsible for their original transforms. This package does not publish
a duplicate LiDAR mounting transform. The live mounting TF was verified as
`(0.7,0,1.8)` with no rotation: +X forward, +Y left, +Z up.

The launch disables VRX's competition-topic bridge and bridges the always-named
`/world/sydney_regatta/clock` to ROS `/clock`. Gazebo's global `/clock` may be
suppressed if another simulator was present at startup, so bridging it blindly
caused a real zero-clock failure during development. All ROS nodes use sim time.

For a real robot, validate and calibrate the GPS/IMU pipeline, heading source,
covariances and outage behavior, and provide continuous local odometry where
required. Do not use the ground-truth fallback. Latitude/longitude waypoint conversion is a
separate later integration. Do not add AMCL without its map/sensor prerequisites.

## Costmaps and boat motion assumptions

The existing custom costmap is not a ROS OccupancyGrid and is not fed into Nav2.
Standard Nav2 ObstacleLayer consumes `/nav/points`, derived from `/lidar/points`.
The filter uses the timestamped mounting/hull TF, rejects nonfinite points and
self returns, and keeps heights 0.35–3.5 m above the horizontal water plane.
This excludes water and most hull clutter. **Low, submerged, or poorly reflecting
obstacles can be missed; those require a later perception design.**

Marking and clearing are enabled. Clearing rays end only at finite real returns;
no synthetic max-range points are invented. Missing returns do not clear space.
Old marks can therefore persist when the sensor cannot provide contrary evidence.
Local window: 24 x 24 m, 0.25 m cells. Global map: 50 x 40 m, 0.25 m cells.
Outside the map cannot be planned through; unknown traversal is disabled.

`water.pgm` is an explicit **assumed-navigable bounded-water prior**, with an
occupied border. It is not a surveyed map or a LiDAR-generated free-space map,
and not a map of all Sydney terrain. Its free interior is a supervised test-course
assumption, not evidence that unobserved water is safe. LiDAR adds obstacles to
both costmaps. A general initially-unknown exploration system is not implemented.

The model has 4 m main hull collisions plus bow and aft motor/propeller geometry,
with thrusters at y = +/-1.027135 m. A conservative rectangular envelope is
x=[-3.1,2.7], y=[-1.4,1.4] m, plus 0.2 m padding. Inflation is 4 m; it is a graded
cost field, not four metres of guaranteed hard clearance. Smac Hybrid checks the
full footprint with Dubins forward-only paths and a 4 m minimum turning radius.

Regulated Pure Pursuit now follows these paths at at most 0.25 m/s, with a
1.25–2.5 m lookahead, curvature/cost regulation and collision checking. The
November 9 tuning disables endpoint extrapolation and retains the existing
position/heading goal tolerances; see results/ARRIVAL.md. It has no boat
hydrodynamic model. The adapter closes a simple surge/yaw velocity loop using
the inspected drag coefficients, differential thrust, and conservative limits.
It is not an ideal differential-drive mapping, but it is still an initial
approximation; stopping distance, current/waves and turning need calibration.
No spinning/reversing BT recoveries are launched. Only wait-at-waypoint is enabled.

The installed Jazzy TwistPublisher defaults to `Twist`; the configuration sets
`enable_stamped_cmd_vel: false` explicitly. Later Nav2 releases can differ.
Commands go to `/nav/cmd_vel`, never directly to a physical actuator.

## Actuation and fail-safe behavior

The inspected VRX topics are `/wamv/thrusters/{left,right}/thrust`, Float64
**newtons**, and `/wamv/thrusters/{left,right}/pos`, Float64 **radians**. The model
uses force-command mode despite `velocity_control=true` (that setting directly
controls propeller rotation internally). This package holds steering angles at
zero, mixes total surge force and yaw moment using the actual thruster spacing,
and limits each thruster to [-100,180] N, far below the model's positive limit.
Negative thrust is permitted for braking/steering; negative-speed navigation and
pure-spin commands are rejected. No ESP32 PWM values are used.

Target acceleration is limited to +0.08/-0.12 m/s² and thrust slew to
+80/-120 N/s in simulation time. Repeated simulation-clock values hold the
controller state rather than resetting its acceleration ramp. Wall-time timers
still enforce safety during pauses.

Independent freshness gates require advancing sensor timestamps, correct frames,
and valid data. Pose timeout: 1 wall second. LiDAR timeout: 3 wall seconds or
1.5 simulated seconds. Command timeout: 1 wall second. The LiDAR limit accounts
for observed WSL delivery gaps, not a high-speed operational requirement.
Sensor faults latch disarmed and require a successful explicit `/nav/arm` call;
fresh Nav2 commands cannot bypass stale sensors. Command loss outputs neutral.
A conservative swept-footprint check reserves at least 2 m forward for stopping.
Emergency/fault output goes straight to zero thrust rather than slewing down.

Zero thrust is **not an instantaneous physical stop**: the simulated boat coasts
and can drift. This code does not provide station keeping, independently verified
emergency braking, or a separate watchdog if the adapter process itself crashes.
The Gazebo thruster may retain its last command if its publisher is killed
abruptly; use the session wrapper and disarm before stopping individual nodes.

Inspect `/nav/safety_status` for the current gate, force, command and last fault.

## Validation and current limits

```bash
source nav2_ws/env.sh
PYTHONPATH="$PWD/nav2_ws/src/vrx_nav2:$PYTHONPATH" \
  python3 -m pytest nav2_ws/src/vrx_nav2/test/test_core.py -q
```

Automated checks cover quaternion/axis conversion, self/water/nonfinite filtering,
stale/repeated/future timestamp rejection, paused-simulation wall timeout,
thrust sign and saturation, slew limits, reverse/spin rejection, emergency neutral,
swept obstacle detection and repeated-clock acceleration behavior.

The `results/` directory retains actual launch and trajectory logs, including
failed trials. Overlapping-server and silent-clock trials are not valid navigation
successes. Nav2 lifecycle activation, sensor TF, local odometry and cloud filtering
have been exercised live. The current final motion result is recorded separately
in `results/BASELINE.md` after the last bounded test.

Full stationary-obstacle detours, left/right offsets, passable/narrow gaps,
fully blocked route, injected LiDAR/pose loss and measured stopping/clearance
tests remain a later iteration unless explicitly marked passed in that report.
No path visualization or accepted action is reported as successful boat motion.

Successful simulation is a development milestone. It does not establish readiness
for autonomous operation on the physical boat.

## Official references used

- [Nav2 Jazzy configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/)
- [Jazzy costmaps](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)
- [Jazzy Regulated Pure Pursuit](https://api.nav2.org/nav2-jazzy/html/md_nav2_regulated_pure_pursuit_controller_README.html)
- [VRX jazzy source and support](https://github.com/osrf/vrx/tree/jazzy)
- [Harmonic Thruster implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/thruster/Thruster.cc)

Actual launch options, TF, sensor pose, dynamics, units and action message layouts
were checked against local VRX source and extracted Jazzy headers/interfaces.
