# Collaborative UAV-UGV Disaster Navigation

> CSC 591 (010): Software for Robots — Spring 2026
> Pranav Bhagwat, Rachit Gupta, Rameez Malik

## Project Overview

This project explores a two-agent collaborative-robotics architecture: a small Unmanned Aerial Vehicle (UAV) flies first and produces a 2D occupancy map of an unknown environment, then hands that map off to an Unmanned Ground Vehicle (UGV), which uses it to localize and plan paths to a goal — without ever running SLAM itself. To validate the value of the handoff, we built two UGV variants in parallel: one that runs on-board SLAM (`slam_toolbox`) as a baseline, and one that consumes a pre-built map only. The UAV is built on PX4-Autopilot in ROS 2 Humble; both UGVs are built on Nav2 in ROS 2 Jazzy.

## Team & Branches

| Member          | Workstream         | Feature Branch   | ROS 2 Distro |
|-----------------|--------------------|------------------|--------------|
| Pranav Bhagwat  | UGV with SLAM      | `pranav_dev`     | Jazzy        |
| Rameez Malik    | UGV without SLAM   | `rameez_dev_v3`  | Jazzy        |
| Rachit Gupta    | UAV (PX4 SITL)     | `rachit_dev`     | Humble       |

`main` is the consolidation point. Each component's source and run instructions live on its feature branch.

## System Architecture

The UAV builds a map of the environment from its downward-facing depth camera and exports it as a Nav2-compatible `.pgm` + `.yaml` pair. The UGV-without-SLAM consumes that file via `map_server`, localizes against it with AMCL, and plans/executes paths with Nav2. The UGV-with-SLAM is the standalone baseline that does its own mapping for comparison.

```
   ┌─────────────────┐                    ┌──────────────────────────┐
   │ UAV (PX4 SITL)  │ ── .pgm/.yaml ──►  │ UGV without SLAM         │
   │ Rachit / Humble │     handoff        │ Rameez / Jazzy           │
   └─────────────────┘                    │ map_server + AMCL + Nav2 │
                                          └──────────────────────────┘

   ┌─────────────────────────────────┐
   │ UGV with SLAM (baseline)        │
   │ Pranav / Jazzy                  │
   │ slam_toolbox + Nav2             │
   └─────────────────────────────────┘
```

## The Three Workstreams

### 4.1 UGV with SLAM (Pranav)

A containerized ROS 2 Jazzy stack that runs the Turtlebot3 Waffle in modern Gazebo, generates a live 2D occupancy grid via `slam_toolbox` from simulated lidar + wheel odometry, and uses `nav2_simple_commander` to drive the robot to parameterized goals while logging the actual trajectory and the dynamically built map for post-run evaluation. Branch: `pranav_dev`. Key files: `workspace_folder/src/solo_ugv_project/{commander_node.py, launch/ugv_tracker.launch.py, config/params.yaml}`. Setup and run: `README-pranav.md` on `pranav_dev`.

### 4.2 UGV without SLAM (Rameez)

Same simulation infrastructure (Turtlebot3 Waffle, headless Gazebo, RViz2, Nav2) with SLAM intentionally disabled. The UGV reads a pre-built `.pgm` / `.yaml` map from disk via `nav2_map_server`, localizes via AMCL, and plans / executes with Nav2 (NavFn global planner + MPPI controller). The static map stands in for what the UAV branch hands off — simulating the perception-to-action handoff. Branch: `rameez_dev_v3`. Key files mirror Pranav's, plus `analysis/results_viz.py` for report-figure generation. Setup and run: `README-rameez.md` on `rameez_dev_v3`.

### 4.3 UAV (Rachit)

A modified PX4-Autopilot `x500_depth` model (camera rotated downward) launched via PX4 SITL inside a Gazebo world that imports the same Turtlebot3 hexagonal sandbox the UGVs use, so all three robots share the environment. ROS 2 Humble bridges PX4's uORB messages into the ROS graph through `Micro-XRCE-DDS-Agent`, QGroundControl provides the GCS, and a custom `offboard_control.py` drives the drone to a target pose. The downward depth camera publishes a `PointCloud2` visualized in RViz2. Branch: `rachit_dev`. Setup and run: `README.md` on `rachit_dev`.

## Journey & Lessons Learned

- **Docker-first development paid off.** The `turtleslam` image (ROS Jazzy + Nav2 + Gazebo Harmonic + RViz + slam_toolbox) is the only environment in which the UGV stack reliably builds and runs across our laptops and the NCSU VCL VM. Native installs were not pursued.
- **`nav2_minimal_tb3_sim` ships its models without registering `GZ_SIM_RESOURCE_PATH`.** `gz sim` cannot resolve `model://turtlebot3_world` and dies silently with `Error Code 14` until you mutate `os.environ['GZ_SIM_RESOURCE_PATH']` directly inside the launch file. The declarative `SetEnvironmentVariable` action did not reliably propagate into `ExecuteProcess` actions inside an included launch description.
- **`--network host` + gz-transport requires `GZ_IP=127.0.0.1`.** On a multi-NIC host (e.g. an NCSU VCL VM) gz-transport binds to all interfaces and its in-container nodes can't discover each other; pinning to loopback fixes it. `--network host` itself is needed because SSH-forwarded `DISPLAY=:N.0` resolves to `localhost:6010`, only reachable when the container shares the host's network namespace.
- **`/odom` is published in the odom frame, anchored at the robot's spawn point** — not the world frame. Visualizing the trajectory on the static map requires adding the spawn offset to every odom sample first.
- **Map `origin` in the YAML must align with where Gazebo places the world.** A 6-meter Y-offset between the UAV-supplied map and the simulated world manifests as the robot driving "outside" the map even though it is inside the actual environment.
- **AMCL is not SLAM.** AMCL only localizes within an externally-provided map; it never updates or builds one. Confusing the two led to wasted debugging time on a system that wasn't running SLAM in the first place.
- **TF tree completeness gates everything in Nav2.** A missing `map → odom` (because AMCL has not activated) makes RViz silently render only the grid, with no error. Always check the TF tree first when "nothing shows up". Static-TF publishers are a fine debugging crutch but conflict with live TF once the simulator works — remove them before relying on AMCL or sensor-driven `/tf`.
- **Docker containers leak DDS state under `--network host`.** When RViz crashed mid-launch, the next `ros2 launch` inside the same container would silently fail because zombie DDS participants from the previous run still squatted on host network resources. Recovery: `exit` the container fully and start a fresh one.
- **PX4 + ROS 2 distro split between teams matters.** The UGV is on Jazzy (for Gazebo Harmonic + Nav2), the UAV is on Humble (for PX4 + Micro-XRCE-DDS-Agent). End-to-end UAV→UGV integration will require either a bridge between the two distros or migrating one side.
- **Drone TF tree height bug surfaces in visualization.** Until the drone's altitude is correctly propagated through its static TF chain, the depth camera's point cloud renders below ground level in RViz. Solving this is open work on `rachit_dev`.

## Where to go next

| Looking for...                         | Go to                                                |
|----------------------------------------|------------------------------------------------------|
| UGV-with-SLAM source + setup           | branch `pranav_dev`, file `README-pranav.md`         |
| UGV-without-SLAM source + setup        | branch `rameez_dev_v3`, file `README-rameez.md`      |
| UAV source + setup                     | branch `rachit_dev`, file `README.md`                |
| Plot-generation script (report figs)   | `analysis/results_viz.py` (on `rameez_dev_v3`)       |
