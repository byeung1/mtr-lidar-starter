# Arrival / circling diagnosis — November 9, 2026

The user kept Gazebo and RViz running. The current goal was read from `/plan`: x=4 m, y=0 m, yaw=0 rad. Although `[0,0,4]` would mean a 4-radian heading, that was not the goal observed in this run.

The boat was actively commanded at 0.45 m/s while turning, with healthy sensors and no latched fault. A sampled global path had length 20.457 m and included a loop. The installed RPP approach-scaling implementation only activates when remaining path length is below its threshold; closeness in Euclidean distance does not by itself force that slowdown while a long loop remains. The configuration also used fixed curvature lookahead with extrapolation past the goal. The SimpleGoalChecker required both 0.8 m position and 0.7 rad heading tolerance, while in-place rotation and reverse navigation were disabled.

The ongoing attempt was disarmed and canceled; physics and both viewers were kept running. Disabling fixed curvature lookahead and endpoint extrapolation alone did not complete a bounded retest: it ran 100.004 wall seconds and was canceled/disarmed. The closest stored trajectory sample was 1.036 m from the target, with heading 0.579 rad. That was outside position tolerance but inside heading tolerance. Thus this observation supports a tracking/overshoot issue, rather than proof that heading tolerance alone caused the circling.

## Applied tuning

- Nav2 desired cruise speed: 0.45 -> 0.25 m/s.
- Velocity-scaled lookahead: 2.0–4.5 -> 1.25–2.5 m; time 6 -> 4 s.
- Minimum approach speed: 0.08 -> 0.05 m/s.
- Path-based approach scaling distance: 3 -> 5 m.
- Fixed curvature lookahead and extrapolation after the goal: disabled.
- Position and heading tolerances, footprint, obstacle rules and sensor/command watchdog limits: unchanged.

These values were applied atomically to the running idle controller and saved in `config/nav2.yaml`. The waypoint client now logs the requested map coordinates with units. No simulator restart or physical reset was performed.

## Live result

The next `(4,0,0)` retest succeeded (status 4, error code 0) in 32.906 wall seconds. The test observer disarmed after the result. A later 10.011 second read-only observation ended at (3.794, -0.438) m, about 0.485 m from the goal, heading -0.545 rad and surge speed -0.001809 m/s. Both thrusts were zero and the adapter was disarmed. This later window is not an exact measurement of braking time or distance.

The two retests started from different stopped poses around the same target. This is evidence that the revised profile completed one previously troublesome goal, not a controlled proof for every approach direction or obstacle layout. The combination of changes was tested; individual contributions beyond the unsuccessful first change were not separately measured. Return-to-origin and arbitrary final-heading missions remain limited by the forward-only planner and available space. No station-keeping capability is claimed.

To obtain a fresh demo, restart the simulation and costmaps. Driving to the origin does not clear obstacles, reset velocity, restore the starting heading or reset localization time. The third waypoint component is heading in radians.

Files: `endpoint-tracking-retest.jsonl`, `boat-approach-retest.jsonl`, `arrival-settling.json`. GUI and simulation processes were left running; no goal or nonzero thrust was left active by these tests.

Primary implementation references: [Jazzy RPP approach regulation](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp), [Jazzy SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/simple_goal_checker.cpp). Local installed headers/plugin declarations and live parameter-service responses were also checked.
