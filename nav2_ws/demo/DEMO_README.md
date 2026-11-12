# VRX + Nav2 demonstration guide

Use this guide to start **one simulation**, open Gazebo and RViz2, click a water
destination or send local waypoints, and insert repeatable stationary obstacles.
No Pygame viewer is needed.

**Already demonstrated:** forward waypoint navigation, simulated LiDAR/TF/Nav2
integration, and zero-thrust responses to missing sensor/pose/command inputs.
**Experiments in this guide:** detouring around a box, rejecting a blocked route,
and choosing whether a gap is passable. Their obstacle layouts are provided here;
successful avoidance and physical clearance have not yet been validated.
The merged clicking feature builds and the combined automated suite passes
36 tests; actual GUI clicking has not yet been tested. GPS/IMU localization
completed a single four-metre goal; the recorded two-waypoint/combined-viewer
tests used the earlier ground-truth baseline.

## Terminal layout

| Terminal | Purpose | Leave running? |
|---|---|---|
| **1** | Headless physics, sensors, bridges and Nav2 | Yes |
| **2** | Optional terminal waypoints; cancel and disarm | Only while its command runs |
| **3** | Spawn/remove obstacles; inspect status | Available for commands |
| **4** | Clickable Gazebo GUI, rendered on Intel | Yes, while viewing |
| **5** | RViz2, rendered on NVIDIA | Yes, while viewing |

Start in this order: **1 → 4 → 5 → 3 (if adding obstacles)**, then choose either
Gazebo clicks or Terminal 2 waypoints. Wait until the viewers finish loading
before requesting movement. Open viewers before movement to avoid
startup load interrupting the sensor stream. These terminal numbers retain the
main README's roles, except Terminal 3 now handles obstacles instead of Pygame.

## First time after a code change — build once

With all previous simulations stopped, paste into any terminal:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
cd nav2_ws
colcon build --base-paths src --symlink-install
```

The `.deps` / `local_setup.*` warning is expected for the local dependencies;
check that the build ends with `1 package finished`. The obstacle SDF files in
`nav2_ws/demo/` are read directly; adding them does not require rebuilding.

## Terminal 1 — start simulation and Nav2

```bash
cd ~/mtr-lidar-starter
bash nav2_ws/run.sh
```

Wait for `Managed nodes are active`. This command starts Sydney Regatta, the boat,
GPU LiDAR, clock/point-cloud bridges, simulation localization, Nav2 and the guarded
thruster adapter. The boat starts **disarmed**. Localization now uses simulated
GPS and IMU through an extended Kalman filter, not Gazebo model pose. The first
fix/heading defines the local waypoint origin, so start with a fresh stationary
simulation. See [localization details](../results/LOCALIZATION.md).

Keep this terminal running. Do not also launch `competition.launch.py`, another
`run.sh`, another LiDAR bridge, or bag playback. The navigation model retains
10 Hz LiDAR and reduces the three camera feeds to 2 Hz to reduce rendering load.

## Terminal 4 — open the clickable Gazebo viewer

```bash
cd ~/mtr-lidar-starter
bash nav2_ws/click_viewer.sh
```

The script builds the click plugin, then attaches a GUI to Terminal 1's simulator
with `gz sim -g`. It does not start a second physics server. It uses Intel and the
simpler Ogre renderer for this GUI; the server keeps its NVIDIA GPU LiDAR and
sensor rendering configuration. The required development libraries were available
and the plugin built successfully on this laptop.

In Gazebo, use the entity tree to find `wamv` and any `demo_*` obstacle. Move the
camera to keep both the boat and obstacle in view. Closing this GUI leaves the
headless simulation running. Use this viewer instead of also running a plain
`gz sim -g` window. Keep RViz in Terminal 5 open alongside it.

To move by clicking, after both viewers have loaded:

1. Enable **Click to navigate** in Gazebo's **Boat navigation** panel.
2. Left-click nearby water ahead of the boat; dragging still moves the camera.
3. Watch the panel for the destination, navigation result or input-fault message.
4. Press **Stop boat** to request cancellation and disarm thrust.

Each destination requests arming through `/nav/arm`; manual arming in Terminal 2
is unnecessary for clicks. Arrival/failure disarms the boat. A new click requests
cancellation of the previous click goal before sending a replacement. Do not use
clicks, terminal waypoint missions and RViz goals at the same time.

Clicks intersect mean water level `z=0`. The bridge aligns Gazebo coordinates
with the GPS/IMU navigation frame using synchronized world and navigation boat
poses. Gazebo pose is used for GUI coordinate conversion only, not localization
or TF. Stale alignment or destinations outside the bounded water map are rejected;
obstacles are checked by Nav2. The current bridge assumes identity `map -> odom`.
If a fault is reported, inspect Terminal 3 instead of repeatedly clicking to arm.

## Terminal 5 — open RViz2

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  rviz2 -d nav2_ws/src/vrx_nav2/config/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

Keep **Fixed Frame = `map`**. Enable these supplied displays:

- **WAM-V** and **TF**: the boat and coordinate frames.
- **Filtered LiDAR**: obstacle returns on `/nav/points`.
- **Global costmap**, **Local costmap**, and **Footprint**: obstacle costs and
  the boat's occupied area.
- **Global path**, **Controller path**, and **Controller lookahead**: the planned
  route and what the controller is following.

The global/local costmaps are different from the old Pygame grid. If RViz reports
a GLSL/indexed-map shader error or the maps are blank, do not interpret a blank
display as obstacle-free space; that rendering issue was observed previously.
Use the LiDAR display and terminal status, and retain the error for diagnosis.

You can also use RViz's Nav2 goal tool after arming, but do not send RViz goals
while a Terminal 2 waypoint client or Gazebo click goal is active. No AMCL
initial-pose step is needed for the current GPS/IMU setup.

## Coordinates, speed and success

Waypoints are absolute **`[x, y, yaw]`** triples in the local `map` frame:

- x/y are metres; yaw is radians.
- `(0,0)` is the initial GPS-derived hull location, with +X set by the initial
  IMU heading and +Y left. Restarting localization alone can change this datum.
- `(4,0,0)` means four metres forward from that origin, not four metres from
  wherever the boat is now.
- Use forward goals; reversing and spinning recoveries are disabled.
- Returning to `(0,0)` does not reset momentum, obstacles or costmaps. In
  `[0,0,4]`, the 4 is a heading in radians. For a fresh demo, follow the full reset
  procedure instead of driving back to the origin.
- The arrival tuning uses shorter lookahead and a lower speed to reduce repeated
  overshooting. It still requires the configured position and heading tolerances;
  it does not guarantee station keeping after arrival.

Gazebo has a different world origin. The spawns below explicitly convert local
nominal spawn-relative `(12,0)` into world `(-525.516372,172.097652)` and use world
yaw **1 radian**. The measured GPS/IMU datum makes the local placement approximate.
Do not enter `(12,0)` directly as Gazebo world coordinates. Obstacles are static,
4 m tall and centred at world z=2 m, extending from the water plane to z=4 m.

The Nav2 cruise limit is **0.25 m/s**, with slower approaches. A long obstacle course
can take a few minutes. `status=4` and `error_code=0` mean action success; a
multi-waypoint result should also have `missed_waypoints=[]`. Position tolerance
is 0.8 m. A visible path alone does not prove the boat passed the obstacle safely.

## Demo A — choose clicks or terminal waypoints

For a click demonstration, start fresh with no obstacles and use Terminal 4's
panel to click nearby water ahead of the boat. Observe motion and the planned
path in both viewers. **Stop boat** cancels/disarms; confirm zero thrust through
Terminal 3. Actual click-to-arrival behavior remains a live acceptance test.

For reproducible numerical goals, use the terminal workflow below instead.
Disable **Click to navigate** while running it.

Start a fresh simulation with no added obstacles. Once both viewers are loaded,
paste into **Terminal 2**:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 service call /nav/arm std_srvs/srv/SetBool '{data: true}'
```

Continue only when the response says `success=True` and `armed`. If it says it is
waiting for three fresh clouds or stable inputs, wait a few seconds and retry.

For the two-waypoint mission previously tested with ground truth:

```bash
ros2 run vrx_nav2 waypoints '[[4,0,0],[7,0,0]]'
```

Watch the boat actually move in Gazebo and RViz. The path and lookahead should
advance, and the action should finish with no missed waypoints.

Alternatively, from a fresh start, use the single goal tested with GPS/IMU
localization (37.1 seconds in the recorded run):

```bash
ros2 run vrx_nav2 waypoints '[[4,0,0]]'
```

After completing the two-waypoint mission, an optional further forward mission
is `[[10,0,0],[14,1,0]]`. This is an additional trial, not the recorded regression
test. Avoid sending the original `(4,0)` goal again once the boat is beyond it.

## Demo B — detect a box and attempt a detour (experiment)

**Reset to a fresh simulation first**, following the reset section below. Leave
the boat disarmed while inserting the obstacle. In **Terminal 3** paste:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 run ros_gz_sim create -world sydney_regatta \
  -file "$PWD/nav2_ws/demo/box.sdf" \
  -x -525.516372 -y 172.097652 -z 2 -Y 1
```

The orange `demo_box` is 2 × 2 × 4 m, centred 12 m ahead in the local map. Check
the creation command reports success. Wait at least five seconds and look for:

1. The box in Gazebo.
2. Returns from it in RViz's filtered LiDAR display.
3. An obstacle mark and inflated costs in the Nav2 costmaps.

Then, in **Terminal 2**, arm and request a goal beyond the box. Leave Gazebo's
**Click to navigate** disabled for this reproducible waypoint test:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 service call /nav/arm std_srvs/srv/SetBool '{data: true}'
ros2 run vrx_nav2 waypoints '[[22,0,0]]'
```

The intended demonstration is a curved path around the box followed by actual
boat motion with clearance. Watch the whole footprint, including the stern.
If no feasible route is found, an abort/stop is preferable to a collision and
does not establish a successful detour. If the boat clips the box or stalls,
cancel/disarm and record the result; obstacle-avoidance acceptance is unfinished.

## Demo C — reject a fully blocked route (experiment)

Reset first. In **Terminal 3**, spawn the red wall:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 run ros_gz_sim create -world sydney_regatta \
  -file "$PWD/nav2_ws/demo/wall.sdf" \
  -x -525.516372 -y 172.097652 -z 2 -Y 1
```

`demo_wall` spans the 40 m local test-course width at x=12. Wait for LiDAR and
the costmaps to show it. Then use the same Terminal 2 arm/goal block from Demo B.

Expected safe behavior is refusal to plan or stopping before the wall. An action
abort (`status=6`) can be the correct outcome here. Confirm the boat does not
cross or collide with the wall and inspect thrust/status in Terminal 3. This is
a blocked-route experiment; it is not a command to push through the obstacle.

## Optional Demo D — compare wide and narrow gaps (experiments)

Run each layout in a **separate fresh simulation**, not together. Both layouts
use two wall sections at local x=12, with the opening centred on y=0.

For a **5.5 m wide opening**, paste in **Terminal 3**:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 run ros_gz_sim create -world sydney_regatta \
  -file "$PWD/nav2_ws/demo/wide_gap.sdf" \
  -x -525.516372 -y 172.097652 -z 2 -Y 1
```

Wait for obstacle observations, then arm/send `[[22,0,0]]` from Terminal 2 using
the Demo B commands. The opening is geometrically wider than the configured
footprint; actual passage and clearance still need to be observed and validated.

After another fresh reset, try the **2.6 m narrow opening**:

```bash
cd ~/mtr-lidar-starter
source nav2_ws/env.sh
ros2 run ros_gz_sim create -world sydney_regatta \
  -file "$PWD/nav2_ws/demo/narrow_gap.sdf" \
  -x -525.516372 -y 172.097652 -z 2 -Y 1
```

The configured hull envelope is 2.8 m wide plus 0.2 m padding per side, about
3.2 m total. The narrow opening should be rejected. Request the same goal beyond
it and verify that no route squeezes the footprint through. Inflation is a cost
field, so gap width alone is not a full physical-clearance guarantee.

## Terminal 3 — inspect progress or a stop

After sourcing the environment in Terminal 3, run one monitor at a time:

```bash
ros2 topic echo /nav/safety_status
```

Ctrl+C stops only this monitor. `running` means the adapter has fresh commands and
inputs. `command_timeout` with zero thrust is normal after a goal completes.
`stale_lidar` or `stale_pose` causes zero thrust and latched disarm. `obstacle_stop`
means the adapter's near-obstacle gate is withholding thrust. `disarmed` means
arming is required; inspect `last_fault` and its details.

To inspect the input pipeline:

```bash
ros2 topic echo /nav/cloud_status
```

To inspect GPS/IMU freshness:

```bash
ros2 topic echo /nav/localization/status
```

`inputs_fresh` and `datum_initialized` should be true. Loss of GPS or IMU stops
the navigation pose feed, causing the adapter's stale-pose disarm even if the
EKF continues predicting. See [localization test results](../results/LOCALIZATION.md).

These monitors do not send movement commands. Do not weaken the watchdog to make
a demo continue. If an unexpected fault occurs, keep the full `Disarmed: ...`
warning from Terminal 1. Live input-loss checks were recorded in
[the performance report](../results/PERFORMANCE.md); they are separate from proving
collision-free obstacle avoidance.

## Remove an obstacle (cleanup)

In Terminal 3, this removes the box only:

```bash
gz service -s /world/sydney_regatta/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'name: "demo_box", type: MODEL'
```

For another layout, replace `demo_box` with `demo_wall`, `demo_wide_gap`, or
`demo_narrow_gap`. A successful response contains `data: true`. Re-running a spawn
with the same name can fail; these commands do not silently create renamed copies.

**Removing a model does not necessarily remove its old costmap marks.** Clearing
depends on real finite LiDAR rays; missing returns do not automatically mean free
space. For reproducible comparisons, restart the simulation between layouts.
Do not clear a costmap during a mission just to force a plan through a marked area.

## Cancel, disarm and reset between demos

1. For a Gazebo click mission, press **Stop boat** in Terminal 4 and disable
   **Click to navigate**. In **Terminal 2**, Ctrl+C an active waypoint client.
   Do not rely on closing the
   client alone to cancel every server-side goal.
2. In that terminal, disarm, then explicitly cancel pending waypoint/navigation
   goals while Nav2 is still running:

   ```bash
   cd ~/mtr-lidar-starter
   source nav2_ws/env.sh
   ros2 service call /nav/arm std_srvs/srv/SetBool '{data: false}'
   ros2 service call /follow_waypoints/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
   ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
   ```

   An empty cancellation request means all goals on that action server. An empty
   `goals_canceling` list can simply mean no goal is active. These calls are for
   this single-boat demo; do not issue them against somebody else's mission.

3. Observe zero thrust; the boat can still coast. Close viewers with Ctrl+C in
   **Terminals 4 and 5**.
4. Ctrl+C **Terminal 1**, then wait several seconds for shutdown.
5. Repeat startup in Terminals **1, 4, 5**. This resets the boat, costmaps and
   spawned models. In Terminal 3, choose exactly one obstacle layout, then arm
   and send the appropriate goal from Terminal 2.

Models inserted by these commands disappear when the simulation exits; no world
file or recordings are overwritten. The existing map is an assumed bounded-water
prior, not a surveyed map of all Sydney terrain. Keep goals within this local
course and record whether the actual boat, not just the path, clears obstacles.

For implementation details see [README.md](../README.md) and
[ARCHITECTURE.md](../ARCHITECTURE.md). Successful simulation is a development
milestone and does not establish physical-boat autonomy readiness.
