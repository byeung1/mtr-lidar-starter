# Navigation map boundary fix — 2026-11-11

The previous map was 200 x 160 cells at 0.25 m, origin (-10,-20):
x [-10,40], y [-20,20] m. Its occupied border was loaded by both static costmap
layers. The global costmap is non-rolling and follows the static map geometry.
The click bridge also rejects destinations outside `/map`. This configuration
explains a boundary stop; no log from the user's specific stop was collected.
RViz's viewport and decorative grid do not constrain boat motion.

Default launch now selects `config/open_water.yaml`: 800 x 800 cells at 0.25 m,
origin (-100,-100), covering 200 x 200 m. Footprint, inflation, LiDAR sources,
unknown-space handling, planner/controller settings, and safety timeouts remain
unchanged. The original files are retained. Launch argument `map` selects another
map, including the original `config/water.yaml`. Restart the map server and Nav2
by restarting Terminal 1; the active simulation was not modified during testing.

This is a larger finite assumed-water area, not unlimited navigation, SLAM, or
a surveyed shoreline map. Goals near the outer border still require clearance.
The larger grid uses more memory and can increase planning cost. The old demo
wall/gap layouts no longer span the default map: select the original map to
test blocked routes rather than routing around the wall ends.

Validation:

- ROS package build passed; 36 existing automated tests passed.
- Real Jazzy map_server, planner_server (Smac Hybrid) and lifecycle manager ran
  in isolated ROS domain 89 with synthetic pose/TF and PointCloud2 input.
- The loaded map reported 800 x 800 cells, resolution 0.25 m.
- ComputePathToPose from (38,0,0) to (45,0,0) returned action status 4, with
  20 path poses ending at x=45. This crosses the previous x=40 boundary.
- Test duration was 7.046 wall seconds, including server startup. Test processes
  were stopped afterward. No thrust was published, and no user simulation was
  restarted. Actual boat travel, long-distance endurance and GUI behavior have
  not been retested with the larger map.

The installed-version behavior was checked against the
[official Jazzy static-layer documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/).
