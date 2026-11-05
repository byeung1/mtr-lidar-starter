# Baseline validation — 2026-11-01

Scope was reduced to a base implementation at the user's request to conserve usage.

## Passed

- Build and YAML/Python syntax checks.
- 13 automated geometry, filtering, conversion and fail-safe logic tests (`unit-tests.txt`).
- All five Nav2 lifecycle servers activate (`nav2-final.log`).
- Live map -> odom -> nav_base -> actual hull -> LiDAR TF (`tf-chain.txt`).
- Actual installed command interface: geometry_msgs/msg/Twist (`cmd-vel-type.txt`).
- Headless VRX, real simulated GPU LiDAR, filtering and ground-truth odometry.
- FollowWaypoints with local goals (4,0,0) and (7,0,0): action status 4 (SUCCEEDED), no missed waypoints, error_code 0.
- Motion observation: 64.616 wall seconds, 219 raw clouds observed, 240 filtered clouds observed, 1722 odometry messages. Independent ROS discovery means raw/filtered observer counts need not match.
- Final action position: (6.343, 0.002) m; peak sampled surge speed 0.425 m/s. Goal tolerance is 0.8 m.
- Neutral/command-timeout observed at completion, followed by explicit disarm. A separate 14.024 second observation ended at (6.509, 0.057) m and speed 0.000382 m/s, with both thrusts zero. The observation started after the action test; its wall interval is not the exact stopping time.
- No sensor latch fault reported during the successful motion run.

## Not yet validated

Obstacle-ahead detours, left/right offset obstacles, passable and narrow gaps, a fully blocked route, full swept physical clearance, dynamic environmental effects, and injected LiDAR/pose loss during motion. Command timeout and stale-data logic have unit coverage and were observed gating earlier failed/startup trials, but these are not substitutes for the full moving-boat fault-injection matrix. RViz configuration is supplied; its final Nav2 display/goal workflow was not visually exercised in this iteration.

The failed trials are retained, not presented as successes. They exposed an orphan simulator, the silent global-clock source, and the repeated-clock thrust-ramp reset. The final launch bridges the world clock and the controller now holds state on repeated stamps.

This is a working forward-navigation simulation baseline, not a completed obstacle-avoidance acceptance test or physical-boat readiness assessment.
