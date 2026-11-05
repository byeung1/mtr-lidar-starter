# WSLg / VRX investigation, 2026-11-01

## What was checked

No applicable AGENTS.md was found in the workspace, VRX tree, or their parent directories. Git status was inspected before edits. VRX is on `jazzy`, commit `7609d1bd90ce7edb29d040a082f949e8b089c864`, with no source changes. The LiDAR project already had Zone.Identifier filename changes; those were left alone. Costmap code and recordings were not edited. Nothing was committed or pushed.

Local, additive changes:

- `scripts/vrx-wsl.sh`: selects Mesa's D3D12 driver and NVIDIA adapter only within the launched process tree; sources the two installed setup files. Offers GUI, server-only, bridge, and RViz commands.
- `config/vrx-lidar.rviz`: saved PointCloud2 view with the working sensor frame, Best Effort / Volatile QoS, and a modest 20 FPS display rate. The wrapper sets simulation time.
- `scripts/vrx-health.py`: monitors actual PointCloud2 arrivals and `/clock`, and runs the existing Processor on real points without recording cloud payloads.
- `scripts/vrx-stability-test.py`: bounded test runner, with cleanup restricted to process groups it starts. Optional temporary static obstacle and delayed RViz startup.

Undo: stop the helpers with Ctrl+C and remove these added files and this diagnostic directory if desired. No shell startup files, system configuration, packages, repositories, VRX source, or X11 mounts were changed.

## Confirmed findings

1. `/mnt/wslg/stderr.log` records Weston terminating with signal 11 at 19:59:20, followed by Xwayland losing its Wayland connection. This supports a real WSLg desktop crash as the explanation for the earlier broken X11 connection. It does not identify the original trigger.
2. Default GLX and adapter-name-only GLX use llvmpipe. Both `GALLIUM_DRIVER=d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` are needed here. Together they select accelerated NVIDIA OpenGL 4.6. Intel selects accelerated OpenGL 4.1 but was much too slow in VRX.
3. Mesa 25.2.8 packages come from official Ubuntu noble-updates. Configured sources are Ubuntu, ROS, and OSRF stable; no Mesa PPA was found. No persistent Mesa/Gallium override, LD_PRELOAD, or custom GL library in `/usr/local/lib` was found in the locations inspected.
4. Both OSRF system Gazebo packages and ROS vendor packages are installed. The inspected running server loaded the ROS vendor Ogre / gz-rendering libraries, system GLVND / Mesa, and WSL's `libd3d12` / `libd3d12core`. There was no observed mixed vendor/system rendering stack. Co-installation alone is not evidence of a conflict.
5. WSLg reports its own Mesa 23.1.0; the Ubuntu application uses Mesa 25.2.8. These are separate components, not proof of a conflicting library installation.
6. The read-only tmpfs at `/tmp/.X11-unix` contains X0 and worked during testing. It was left untouched. Sandboxed checks could not access the display or `/dev/dxg`; diagnostics and simulations were therefore run outside that sandbox after approval. The real WSL environment has `/dev/dxg`.
7. Windows-side read-only checks confirm WSL 3.0.3.0, WSLg 1.0.79, and NVIDIA KMD driver 621.01. See `windows-and-gui.txt` (the WSL command's initial section is UTF-16LE).
8. The installed and source competition launch files agree. Supported options include `world`, `headless`, `paused`, `extra_gz_args`, `sim_mode`, `urdf`, and model/config options. `headless:=true` appends `-s`: it removes the GUI but retains sensor rendering. The world explicitly selects Ogre2; the LiDAR xacro uses the GPU-ray sensor type. Sensors and rendering engine were preserved.
9. Gazebo's separate `--headless-rendering` option uses EGL and is available through `extra_gz_args`; it is **not** enabled by VRX's `headless` argument. It was not used or validated here. Ogre's device enumeration showed only a software EGL device, so EGL must not be assumed to provide NVIDIA acceleration on this installation.

## Interpretation and remaining uncertainty

The reported segmentation fault through Gallium and WSL D3D12 points toward the graphics path, rather than the Python costmap or ROS bridge. Its exact faulting stack was not found in retained ROS/Gazebo logs or `/var/crash`; the user-supplied stack description remains the evidence for that particular crash. A Windows GPU reset, Mesa bug, resource pressure, and a WSLg bug remain hypotheses, not established causes.

The initial 180-second NVIDIA GUI baseline printed `D3D12: Removing Device` and an EGL initialization warning, yet continued simulating and delivering LiDAR until the scheduled shutdown. Therefore that message alone is not a reliable crash detector. Check actual simulation time and cloud delivery. The initial test did not insert primitives, showing that the message does not require model insertion.

The new wrapper makes the known working graphics selection repeatable; it is a scoped mitigation, not a demonstrated repair of a particular upstream driver bug. A successful bounded run cannot prove the intermittent crash is permanently fixed.

The longer GUI trial exposed substantial continuing memory growth and swap use. At about 6:18, server RSS was 2,574,332 kB and GUI RSS was 3,810,052 kB. At about 7:44, server RSS was 2,611,604 kB and GUI RSS was 4,139,780 kB, with anonymous/private memory recorded in `resources-7min.txt`. Some pages were already being swapped, so resident memory alone understates the allocations. WSL memory fell back to about 667 MiB used after stopping the test. This makes rendering-related resource growth a concrete concern, but does not prove it caused the earlier device removal or compositor crash.

An [open upstream Gazebo report](https://github.com/gazebosim/gz-sim/issues/2873) describes memory growth under WSLg/D3D12, including with Mesa 25.2.8. It is a similar report, not a confirmed diagnosis of this installation. No package change or purported upstream fix was applied.

## Test sequence

- NVIDIA GUI baseline: scheduled 180-second run. A 65.125-second observation window received 299 clouds; cloud stamps advanced from 66.3 to 98.8 seconds. Its last few seconds overlapped the scheduled shutdown. Device-removal warning occurred without a fatal crash.
- NVIDIA server-only comparison: 120.028 seconds, clock advanced from 0.008 to 67.912 simulated seconds. Early measurements were contaminated by a leftover bridge from the first manual observation. That bridge was stopped; nonincreasing stamps stopped at 244. Do not use the whole-run cloud count as a unique-cloud rate. Later measurements show continuing delivery. No device-removal warning was found. This short test does not establish long-term headless stability.
- Intel GUI trial: stopped deliberately after 107.426 seconds because it produced only 12 clouds and about 1.25 seconds of simulated progress; maximum arrival gap 8.466 seconds. Static-box detection worked, but performance is unsuitable. It was not selected as the solution.
- NVIDIA GUI longer run: **520.809 seconds (8 min 40.8 s)**, stopped deliberately before RAM and swap exhaustion. Received 2,062 unique clouds, maximum arrival gap 1.448 s, no nonincreasing cloud stamps. Clock advanced from 0.008 to 271.780 simulated seconds. RViz ran from about 120 seconds until shutdown and used simulation time; five-second discovery confirmed its Best Effort / Volatile PointCloud2 subscription. No spontaneous segmentation fault or device-removal message occurred in this run. The original planned duration was 630 seconds; `remove-early.log` records the deliberate box removal around 500 seconds. Forward obstacle returns went from zero before insertion, to roughly 240–250 points / 25–28 occupied cells while present, then back to zero after removal. The internal Gazebo camera screenshot in `gui-nvidia-sustained/camera/` verifies actual rendering. X11 window captures (`gazebo.png`, `rviz.png`) were black and cannot establish the Windows-visible result; RViz visual appearance needs user confirmation.
- NVIDIA server-only sustained run: see `headless-nvidia-sustained/` for final duration, sensor, obstacle, RViz, and memory measurements.

For every test, scheduled interruption and launch shutdown messages must be distinguished from spontaneous crashes. Final shutdown can produce `exit code -2` because the runner intentionally sends SIGINT.

## Startup

The GUI could render, but continuing memory growth prevented recommending it for unattended work. From `~/mtr-lidar-starter`, use separate terminals for the server-only fallback:

```bash
bash scripts/vrx-wsl.sh headless
bash scripts/vrx-wsl.sh bridge
bash scripts/vrx-wsl.sh rviz
```

For a short GUI session, use this instead of the first command and watch memory/swap use:

```bash
bash scripts/vrx-wsl.sh gui
```

Run only one simulation and one `/lidar/points` bridge. VRX also has its own namespaced sensor bridge, which is expected. Ctrl+C each terminal when finished. Bridge publisher QoS is Reliable / Volatile by default, which is compatible with the Best Effort / Volatile observers used here.

The equivalent simulation command is:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

For server-only operation add `headless:=true`. This keeps GPU LiDAR active but still uses X/WSLg for the server's rendering context; it is not insulation from every WSLg or D3D12 failure.

Re-run the sustained diagnostic (requires an ordinary WSL terminal with display/GPU access and no existing simulation):

```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 scripts/vrx-stability-test.py --mode gui --adapter NVIDIA \
  --seconds 630 --obstacle --rviz-at 120 \
  --output diagnostics/my-next-vrx-test
```

The output directory must be new. The test adds a static 2 × 2 × 4 m box named `wsl_stability_box` at world position `(-528, 168, 2)`; the existing Processor is checked for occupied cells in the forward sensor region. The world is never saved, so the box does not persist after the test simulation exits.

## If the intermittent crash returns

Save the simulation terminal output, `~/.gz/rendering/ogre2.log`, and `/mnt/wslg/{stderr.log,weston.log,versions.txt}` before restarting WSL. Note whether `/clock` and `/lidar/points` stop together. A working desktop with a dead server differs from Weston restarting and taking all GUI connections down.

On Windows, open Event Viewer → Windows Logs → System and check the crash time for Display / NVIDIA / WHEA events. Record the event ID and text rather than assuming a GPU timeout. This is a Windows-side check; it was not performed by the agent.

If WSLg remains broken, first save work in all WSL distributions, close their applications, then run `wsl --shutdown` in Windows PowerShell. This terminates all WSL processes. Reopen Ubuntu and retry the helper. `wsl --version` and `wsl --status` are read-only version checks. No further driver update, package downgrade, Linux NVIDIA display driver, or broad reinstall is justified by the current evidence alone.

## Primary references consulted

- [WSLg GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg): adapter selection operates within Mesa's D3D12 backend.
- [Mesa D3D12 documentation](https://docs.mesa3d.org/drivers/d3d12.html): adapter-name matching and D3D12 diagnostics.
- [Gazebo Harmonic troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/): Ogre2 OpenGL requirements and graphics diagnosis.
- [Gazebo Sim 8 headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html): EGL option, Ogre2 requirement, distinction from server-only mode.
- [Gazebo Harmonic sensors](https://gazebosim.org/docs/harmonic/sensors/): GPU LiDAR and Ogre2 sensor-system configuration.
- [VRX jazzy README](https://github.com/osrf/vrx/blob/jazzy/README.md): ROS 2 Jazzy / Gazebo Harmonic support. Launch details were checked against the actual installed source, not inferred from other branches.
- [Microsoft WSL GUI applications](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps): WSL GUI architecture and Windows-side restart procedure.
