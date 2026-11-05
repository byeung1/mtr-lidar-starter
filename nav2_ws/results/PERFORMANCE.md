# Performance and stop diagnosis — 2026-11-02

## Findings and changes

The previous live stops showed `stale_lidar` with both thrusts zero and a latched disarm. That was the watchdog response, not proof the simulator had crashed. Old status snapshots cannot establish which transient caused every past fault; the original code did not record enough event timing.

Confirmed avoidable work in the original code:

1. Each cloud was transformed twice: once by the filter, then again by a separate TF listener in the adapter. Both could block for 100 ms waiting for TF and then discard data. The filter also rejected a newly arriving stamp immediately if the ROS clock was momentarily behind it. These ordering cases are now covered by tests.
2. The custom nodes used three-thread Python executors despite most work being sequential. The revised nonblocking pipeline uses a single-thread executor per custom node.
3. The default VRX model renders three unused camera streams at 30 Hz. A generated, temporary navigation model now sets those cameras to 2 Hz. Camera resolution, geometry, physics, thrusters, other sensors and GPU LiDAR remain unchanged. The installed VRX source is not edited. `camera_rate:=30.0` restores full-rate cameras for a run.
4. The llvmpipe GUI previously used roughly four CPU cores. NVIDIA GUI initialization also crashed. The startup guide selects Intel for only Gazebo's GUI, keeping NVIDIA on the sensor server and RViz. This does not claim to fix the underlying Windows/WSLg driver defect.

The filter now queues at most eight recent raw clouds until the acquisition-time transform and clock are available. Calls never block the executor. Samples expire after 1.5 wall seconds or 0.8 simulated seconds in the pending queue; duplicates cannot refresh them. The original acquisition timestamp is preserved.

Nav2 still receives `/nav/points` in the true LiDAR frame, preserving the clearing origin. The adapter consumes matching `/nav/safety_points` already transformed into `nav_base`. No transforms or physical geometry are invented.

Safety timeouts were not relaxed: pose 1 wall second / 0.8 simulated seconds, LiDAR 3 wall seconds / 1.5 simulated seconds, command 1 wall second. Arming now also requires three fresh clouds and one second of stable inputs. Sensor faults remain latched. A successful manual re-arm clears the acknowledged fault from current status, avoiding misleading historical `last_fault` values. Fault snapshots and `/nav/cloud_status` retain queue/TF/age evidence.

Shutdown now interrupts the launch supervisor first, allowing it to stop its children once; process-group termination remains the fallback for orphan cleanup.

## Controlled short comparisons

The first three measurements ran disarmed and headless, without viewers, for 55 wall seconds each. A 20-second observation warmup was excluded. The fourth used Intel Gazebo and NVIDIA RViz during/after waypoint navigation, for 50 seconds with five seconds excluded.

| Configuration | Simulated seconds / wall second | Max observed filtered-cloud gap |
|---|---:|---:|
| Original pipeline, 30 Hz cameras | 0.509 | 0.946 s |
| Nonblocking single-transform pipeline, 30 Hz cameras | 0.540 | 0.739 s |
| Revised pipeline, 2 Hz cameras | 0.999 | 0.320 s |
| Revised pipeline, 2 Hz cameras, both viewers | 0.962 | 0.621 s |

This supports camera rendering as the main throughput bottleneck in the measured headless workload. The pipeline revision improved ordering behavior and reduced redundant work, but its short-run rate improvement alone was modest. Differences between runs are not an exact CPU microbenchmark: driver caching and sampling vary, and the original profiler missed Python process CPU entries. Raw/filtered observer counts can differ because each DDS subscription has independent discovery and delivery.

The generated model was inspected: three camera update rates were 2.0 Hz and the GPU-ray LiDAR remained 10 Hz. No resolution or scan-density reduction was made.

## Actual motion and failure tests

- All five Nav2 lifecycle servers activated.
- Both viewers stayed running during the measured navigation test.
- FollowWaypoints (4,0,0), (7,0,0) succeeded: status 4, no missed waypoints, error code 0. Observation lasted 33.286 wall seconds; final observed position (6.559, 0.088) m. No sensor-latch fault occurred during the successful mission. Position tolerance remains 0.8 m.
- Navigation velocity limits were not raised. The boat still approaches slowly and coasts after thrust is zero.
- Injected lidar loss: PASS; adapter reported `stale_lidar` and both thrusts were zero.
- Injected pose loss: PASS; adapter reported `stale_pose` and both thrusts were zero.
- Injected command loss: PASS; adapter reported `command_timeout` and both thrusts were zero.

The LiDAR-loss test paused the cloud-processing node, representing loss of the adapter's required sensor pipeline while raw Gazebo LiDAR continued. The pose-loss test paused ground-truth publication. Both were resumed in finally-cleanup; fresh commands were kept publishing during those faults to verify they could not conceal stale inputs. The command-loss test stopped only velocity publication. These are live adapter tests with brief actual motion at a 0.2 m/s command limit; they are not collision-avoidance acceptance tests. The test finished disarmed.

The final simulator session was bounded to 420 wall seconds (seven minutes), including setup, combined viewers, motion and fault injections. The extra planned 45-second armed soak was NOT completed: the bounded simulator had already ended before its arm client connected; the waiting client was cleaned up. Do not count it as a pass. The successful motion observation is the duration reported above, not seven minutes of continuous navigation.

24 automated tests passed, including delayed TF/clock queuing, expiry, duplicate stamps, matching Nav2/safety coordinates, camera-only model changes, thrust conversion and safety logic. Build, Python syntax and README shell syntax were checked.

## Limits and remaining work

The code changes address measured load and concrete ordering weaknesses. They do not prove every old stop had the same cause or guarantee unlimited stability. Full obstacle detours, tight gaps, hard physical clearance, environmental extremes and independently measured braking distance remain unvalidated. The driver memory-growth issue is not declared fixed.

RViz stayed open and connected, but logged an indexed-map GLSL shader warning on NVIDIA. Navigation and data delivery were verified; this test did not visually validate every costmap pixel. That graphics limitation may need separate work if the costmap display is blank.

Data: `performance-*.json`, `final-performance-session.log`, `improved-waypoints.jsonl`, `fault-injection.json`, `fault-injection.log`, `improved-unit-tests.txt`, `final-intel-gui.log`, `final-rviz.log`. Initial profiling sessions printed shutdown-related driver/rclpy errors; they are retained and not confused with the steady-state timing measurements.

Successful simulation is a development milestone, not physical-boat autonomy certification.
