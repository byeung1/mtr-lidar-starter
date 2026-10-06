# MTR LiDAR starter — ROS 2 Jazzy

Develop your own LiDAR processing with a real recording and the same input
topic as the boat. No hardware, boat repository clone, or visualization tool
is needed. This package provides the data-access plumbing; you choose the
algorithm, outputs, visualization, and final package structure.

## 1. Set up

Use Ubuntu 24.04 (native or WSL) with ROS 2 Jazzy already installed.
Extract this ZIP into your Linux home directory, then open a terminal there:

```bash
cd ~/mtr-lidar-starter
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-numpy \
  ros-jazzy-rclpy ros-jazzy-sensor-msgs-py ros-jazzy-launch-ros \
  ros-jazzy-ros2bag ros-jazzy-rosbag2-transport \
  ros-jazzy-rosbag2-storage-default-plugins ros-jazzy-rmw-fastrtps-cpp
bash build.sh
```

## 2. Run

```bash
bash run.sh
```

This launches your node, waits two seconds, then replays the bundled bag in
a loop. Look for `Processed scan ... finite points, frame=lidar_link`.
Ctrl+C stops both processes. Do not start a second player on the same topic.
The Fast DDS profile is applied to the launch and its child processes.

## 3. Write your code

Edit `src/mtr_lidar_starter/mtr_lidar_starter/processing.py`.
`Processor.process()` receives a finite `(N, 3)` float32 NumPy array of XYZ
points, the cloud timestamp, and its frame ID. The starter imposes no height,
range, clustering, or grid architecture. The processor returns a per-scan obstacle costmap; console scan counts confirm
the input reaches it.

Restart `bash run.sh` after Python edits. With symlink installation, existing
Python files do not need a rebuild. Rebuild after adding modules, dependencies,
entry points, or launch files. Put state cleanup in `Processor.reset()`;
the adapter calls it when cloud timestamps move backwards at a bag loop.

Choose your own outputs. Add ROS publishers in `lidar_node.py`, return results
from `process()`, or add your own nodes. For a GUI, keep its event loop responsive
and avoid blocking the ROS callback. A Pygame costmap viewer is included; no ROS occupancy-grid publisher is included.

## Input contract

| Property | Value |
|---|---|
| Topic | `/lidar/points` |
| Type | `sensor_msgs/msg/PointCloud2` |
| Subscription QoS | Sensor data: best effort, volatile |
| Frame | `lidar_link` |
| Coordinates | X forward, Y left, Z up; metres |
| Available fields | x, y, z, intensity, time |
| Adapter output | XYZ only; use the original cloud for extra fields |

The recording contains 295 clouds over approximately 29.4 seconds, only on
`/lidar/points`. It contains no TF, IMU, odometry, camera, or labels. Work in
`lidar_link`; boat/world coordinates need measured transforms and appropriate
pose information. Header timestamps were assigned at acquisition on the boat.
Playback publishes `/clock`; the starter node uses `use_sim_time=true`.
Finite-point filtering changes the array length but not coordinates.

## Just view the recording in RViz (optional)

You can look at the recorded 3D points without building the starter package or
running any processing code. Extract the ZIP and use these steps. On WSL, use
a WSLg-enabled installation so RViz can open a desktop window.

1. Install the viewer and playback tools (ROS 2 Jazzy must already be installed):

   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-rviz2 ros-jazzy-ros2bag \
     ros-jazzy-rosbag2-transport ros-jazzy-rosbag2-storage-default-plugins \
     ros-jazzy-rmw-fastrtps-cpp
   ```

2. In terminal 1, start playback from the extracted folder:

   ```bash
   cd ~/mtr-lidar-starter
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds.xml"
   ros2 bag play ./bags/obstacle_case --clock --loop --read-ahead-queue-size 30
   ```

   Keep this running. The approximately 30-second recording repeats until you
   press Ctrl+C. If `bash run.sh` is already running, it supplies playback;
   skip this step so you do not start a second player.

3. In terminal 2, open RViz with the same environment:

   ```bash
   cd ~/mtr-lidar-starter
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds.xml"
   rviz2 --ros-args -p use_sim_time:=true
   ```

4. In the left **Displays** panel, expand **Global Options** and set
   **Fixed Frame** to `lidar_link`. If the panel is missing, enable it under
   **Panels → Displays**.

5. Click **Add → By display type → PointCloud2 → OK**. Expand the new
   **PointCloud2** display and set **Topic** to `/lidar/points`.

6. Expand **Topic** using its arrow and set **Reliability Policy** to
   **Best Effort** and **Durability Policy** to **Volatile**. This matters:
   Reliable subscribers cannot receive this bag's Best Effort stream.

7. Once points arrive, set **Position Transformer** to **XYZ** and
   **Color Transformer** to **FlatColor** (choose a bright color). Use
   **Style → Points**, **Size (Pixels) → 3**, and **Decay Time → 0** for a
   simple view of each scan. Alternatively use Flat Squares with Size (m) 0.05.

You should now see the recorded scene as 3D points. Left-drag rotates the view,
middle-drag moves it, and the mouse wheel zooms. This is the original recording,
not obstacle classifications or an occupancy grid.

If nothing appears, keep playback looping, check the Topic QoS settings, and
expand the PointCloud2 **Status** for details. Empty transformer dropdowns can
mean no cloud has arrived yet. The **By topic** search box only filters topics;
typing a topic there does not create a display. No TF is recorded, so use the
cloud's own `lidar_link` frame rather than `map` or `base_link`.
Use matching ROS_DOMAIN_ID in both terminals if you set it explicitly.

## Your own visualization

RViz is just an optional way to inspect the input recording. Choose RViz,
Pygame, plots, or any other visualization for your own processing results.

## Moving toward the boat

The real repository is https://github.com/armutie/mtr-boat-core/tree/ros2-foundation
and currently targets ROS 2 Humble. This starter mirrors its standard LiDAR
topic/QoS/frame boundary, not its entire hardware stack. To use a live stream:

```bash
bash run.sh play_bag:=false use_sim_time:=false
```

Start the existing boat driver separately; it remains the hardware owner.
Before integration, build/test your package under Humble on Ubuntu 22.04.
Jazzy playback has been tested; Humble portability of future code is not assumed.

## Troubleshooting

- No scans: check that playback is running and topic/domain settings match.
- Incompatible RELIABILITY_QOS_POLICY: set the subscriber to Best Effort.
- Bag open error: point to the folder containing metadata.yaml and the .db3.
- Shared-memory errors: the supplied profile uses an 8 MB segment. If using
  separate tools, export FASTRTPS_DEFAULT_PROFILES_FILE to config/fastdds.xml
  and RMW_IMPLEMENTATION=rmw_fastrtps_cpp in their terminals too.

This archive includes source and recording only, not installed ROS or generated
build artifacts. MIT applies to the starter code; the recording is supplied by
the repository owner for this handoff.

## 2D local costmap and Pygame viewer

```bash
sudo apt install -y python3-pygame
bash build.sh
bash run.sh executable:=costmap_viewer
```

This launches the viewer and one looping bag player. Close the window or press
Escape to close the viewer; use Ctrl+C in the terminal to stop the launch and
bag player. A desktop display is required (WSLg on WSL).

`processing.py` builds a fresh 20 m square grid for every scan, at 0.1 m per
cell, in the incoming sensor frame. It keeps points with sensor-relative Z
between -1 and 2 m, excludes returns within 0.5 m horizontally, and ignores
points outside the grid. These are provisional filters, not ground detection;
tune `Processor` constructor settings for the sensor mounting and scene.

Selected XY positions mark obstacle cells (100). A circular 0.6 m inflation
buffer assigns decreasing costs (1–99) around obstacles. Other cells remain
unknown (-1): absence of a return does not establish free space. No ray
clearing, motion compensation, tracking, or temporal accumulation is included.
The inflation radius is a starting visualization setting, not a measured boat
footprint or validated clearance.

`costmap_viewer.py` reuses the ROS adapter and displays its results at up to
30 frames per second. Red means obstacle, orange means inflation, gray means
unknown, and the cyan marker shows the sensor origin. Forward (+X) points up;
left (+Y) points left. It displays the latest scan and holds it between arrivals.
The regular `bash run.sh` still computes costmaps without opening a window.
