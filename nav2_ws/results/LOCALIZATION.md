# GPS + IMU localization baseline

Default startup is unchanged: `bash nav2_ws/run.sh`, after rebuilding the package.
Stop the previous simulation before rebuilding/restarting. To compare the old
baseline, use `bash nav2_ws/run.sh localization:=ground_truth` instead.
The launch choices are exclusive: the old ground-truth node is not started in
GPS/IMU mode. No ROS/Gazebo packages or VRX source were modified.

## Inputs and estimation

Installed VRX interfaces inspected:

- `/wamv/sensors/gps/gps/fix`: NavSatFix, stock GPS at 20 Hz, hull mounting
  `(-0.85, 0, 1.3)` m. The stock NavSat sensor has no configured noise model.
- `/wamv/sensors/imu/imu/data`: Imu, stock IMU at 100 Hz, mounting
  `(0.3, -0.2, 1.3)` m, aligned axes. Its xacro explicitly selects ENU orientation;
  gyro and acceleration include noise/bias. Orientation is simulator-provided,
  not proof that a real six-axis IMU supplies absolute heading.
- Mount transforms are looked up from existing VRX TF, not duplicated or assumed
  in the estimator. The LiDAR configuration and existing mounting TF are retained.

`sensor_localization.py` converts WGS84 GPS coordinates to a local Cartesian frame
using ECEF then ENU rotation. It subtracts the rotated GPS antenna lever arm so
turning does not look like hull translation. The first accepted fix/heading
defines local zero and +X; +Y is left. IMU orientation is rotated into this frame
and angular velocity into the hull frame. Initial hull altitude defines zero
height; the horizontal navigation plane is at that initial hull height, not a
measured tide/water surface. Existing height filters therefore remain a baseline
assumption and require verification in waves and on a real boat.

The installed Jazzy `robot_localization/ekf_node` fuses XYZ position plus IMU
roll/pitch/yaw and angular rates. It estimates body velocity from position changes
and its motion model. Acceleration is deliberately excluded: double integration
of the stock biased accelerometer would drift. Sway is permitted, not constrained
to zero as if the boat had differential-drive wheels. This is not the standard
navsat_transform_node pipeline; GPS preprocessing is local package code.

Angular velocity is low-pass filtered with a 0.3 s acquisition-time constant
before fusion. The unfiltered baseline produced rapid thrust feedback oscillation
and failed a waypoint test; live raw IMU showed alternating rates near 1 rad/s
while sampled orientation varied much less. Smoothing prevents the adapter from
chasing each rapid gyro fluctuation, at the cost of additional rate-feedback lag.
This is a baseline tuning choice, not a diagnosis of a faulty physical IMU.

Unknown GPS covariance receives baseline standard deviations of 0.15 m horizontal
and 0.3 m vertical; provided covariance is retained with a 0.15 m minimum standard
deviation. IMU floors are 0.01 rad roll/pitch, 0.02 rad heading, and 0.02 rad/s
angular rate. These are explicit development settings, not measured calibration.

## Frames and safety

The EKF has TF publication disabled. The sensor localization node alone publishes
`map -> odom` (identity), `odom -> nav_base` (horizontal), and
`nav_base -> wamv/wamv/base_link` (estimated heave/roll/pitch). Existing VRX TF
continues from the hull to sensors. `/nav/odom` retains the interface expected by
Nav2 and the thrust adapter. Internal measurement/output topics are:

- `/nav/localization/imu`
- `/nav/localization/gps`
- `/nav/localization/filtered`
- `/nav/localization/status` (JSON health and input wall-clock ages)

The relay requires advancing IMU and GPS timestamps, both no older than 0.5 s in
simulation time and 0.75 s in wall time. Invalid/replayed measurements and timestamps
more than 0.1 s in the future do
not refresh freshness. EKF predictions alone do not produce navigation TF or
`/nav/odom` after inputs expire. The existing independent thrust watchdog then
latches a stale-pose disarm and zero thrust. Restart the whole simulation after a
clock reset or datum change; do not silently redefine coordinates mid-mission.

GPS corrections are fused into the single local odom estimate for this bounded
simulation baseline. This does not implement a separately continuous local odom
and globally corrected map transform. Noisy GNSS and long outages need a more
complete localization design before hardware use.

## Documentation consulted

[Official Jazzy robot_localization sensor/frame requirements](https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/doc/preparing_sensor_data.rst),
the installed Jazzy `params/ekf.yaml`, and installed VRX `wamv_imu.xacro`,
`wamv_gps.xacro`, payload bridge code and generated robot were inspected.

## Validation

Tested 2026-11-09, headless Sydney Regatta on this WSL laptop:

- `colcon build --base-paths src --symlink-install`: passed. The existing rootless
  dependency `local_setup.*` warning remains; it did not fail the build.
- `python3 -m pytest nav2_ws/src/vrx_nav2/test -q`: **30 passed**, including WGS84
  axis/scale, rotated antenna correction, invalid quaternion, advancing/future/
  replayed sensor timestamps, independent freshness, and gyro filtering tests.
- Unfiltered gyro feedback failed the first waypoint run (100 s limit); a clean
  repeat was interrupted after 46 s with poor progress. Read-only comparison
  confirmed large rapidly alternating raw gyro rates. These runs are retained
  in the JSONL files and are not counted as passing navigation tests.
- The first fail-safe harness had incorrect remapping, then was corrected to
  use an entirely separate DDS domain (87). All final motion evidence below is
  from a new simulation after that harness was isolated.
- Corrected GPS/IMU mode reached `[4,0,0]` with action **status 4 / succeeded** in
  **37.149 wall seconds**, from a fresh spawn. At action completion estimated
  position was `(3.221, 0.048)` m and heading `-0.026` rad: within the unchanged
  0.8 m / 0.7 rad goal tolerances. It received 694 odometry messages and 244
  filtered-cloud messages during the probe. Subscriber discovery/loss means raw
  and filtered subscriber counts are not a one-to-one delivery audit.
- A separate read-only Gazebo pose check confirmed actual boat movement: a later
  sample was approximately `(3.38, 0.065)` m relative to nominal spawn, around
  0.62 m from the requested goal. This comparison did not feed localization or
  publish TF; it is not a synchronized trajectory-error benchmark.
- A **20.001 s** post-goal health check received 354 odometry messages, showed
  fresh GPS/IMU, and confirmed the boat was disarmed with both forces zero.
  Pipeline status showed 1,072 clouds received and processed, no invalid/dropped
  clouds at that sample. The ROS node graph had `sensor_ekf` and
  `sensor_localization`, and no `sim_ground_truth`.
- `test/live_localization_faults.py` uses separate DDS domain 87 and remapped
  actuation/TF topics. With synthetic EKF output, LiDAR and velocity commands
  continuing, withholding GPS and then IMU independently caused **stale_pose,
  disarmed, zero left/right thrust**. Restoring data did not auto-arm. This tests
  the real preprocessing gate and thrust-adapter classes using synthetic sensor
  fixtures; it does not establish real GNSS outage performance at sea.
  The final isolated run lasted **14.342 wall seconds**.

Evidence: `gps-imu-waypoint-filtered.jsonl`, `gps-imu-final-comparison.jsonl`,
`gps-imu-final-health.json`, and `gps-imu-faults.jsonl` in this directory.
No new 10-minute endurance, noisy-GNSS, multi-waypoint, obstacle-clearance, GUI or
physical-hardware test was completed for this localization change. Previous
ground-truth motion tests do not validate those cases with the new mode.

Simulation success is a development milestone; real GNSS accuracy, heading
observability, calibration, outages, thrust hardware and independent stopping
remain separate work. A basic IMU alone cannot replace absolute position or
guarantee absolute yaw on a real boat.
