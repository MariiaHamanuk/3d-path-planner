# 3D Path Planner

### Team

Very cool team

Very cool mentor

---

### What we've used
- **Gazebo** - simulation
- **ROS 2** -  Robotics middleware
- **Foxglove** - visualization
- **Docker** - put it all together

---

### Usage

**1. Build Docker Image**
```bash
./docker/build.sh
```

**2. Run Container**
```bash
./docker/run.sh
```

`run.sh` limits the container to two thirds of the cores and 8 GB of RAM (no swap). Change it with
`DOCKER_CPUS=6 DOCKER_MEMORY=10g ./docker/run.sh`; it applies when the container starts.

**3. Start Simulation & Services**

Inside the running container
```bash
./scripts/start.sh [-w <world>] [config-path]
```

Worlds live in `simulation/worlds/<name>.world`: `default` (flat ground, default) and `city` (200x200 m town).
Pick one with `-w city`, `WORLD=city ./scripts/start.sh`, or `WORLD=city` in the config file.

This launches a tmux session defined in `scripts/tmux.yml` with:
- `gz`- Gazebo simulation
- `ardupilot` - ArduCopter SITL + MAVProxy console
- `bridge`- ROS-Gz bridge
- `foxglove`- Foxglove WebSocket bridge
- `planner` - Shell pane for path-planning (wip)

---

### Controlling the drone (ArduPilot)

The drone runs ArduCopter SITL, connected to Gazebo through `ardupilot_gazebo`.

**Console** - in the `ardupilot` tmux window (MAVProxy):
```
mode guided
arm throttle
takeoff 5
guided 10 0 -5       # fly to local NED x y z (m)
mode land
```

**QGroundControl** - on the host, with QGC closed/open, it autoconnects on UDP `14550`
(MAVProxy sends there via `host.docker.internal`). If it doesn't, add a Comm Link:
type UDP, listening port `14550`. On Linux, allow UDP 14550 through the host firewall.
Use `QGC_HOST=<ip> ./scripts/start.sh` if QGC runs on another machine.

Depth: `/depth_camera` (32FC1, meters, 320x240, hfov 1.047 rad, range 0.1-200 m), `/depth_camera/camera_info` and `/depth_camera/points` (PointCloud2), same viewpoint as `/camera`. Gazebo's depth sensor always advertises its info as `/camera_info` (it ignores `<camera_info_topic>`), so the bridge remaps it to `/depth_camera/camera_info`.

Extra ArduCopter params live in `scripts/bebop.parm` (starting values, not tuned).

---

### Reading the simulation from C++ (`3dpath/`)

`path3d::SensorTopics` (`3dpath/include/io/reader.hpp`) holds one topic for each sensor stream: the drone
pose, color and depth images, camera calibrations and base-link -> camera mounts. A `path3d::Reader`
publishes into these topics as data arrives. `path3d::SimReader` (`3dpath/include/io/sim_reader.hpp`) is
the Gazebo implementation; a real drone or a replay would be another. Every ROS callback converts its
message to a project type from `core/types.hpp` (Eigen poses, `cv::Mat` images, `cv::Matx` calibration)
and publishes it straight away. No ROS types appear outside `io/sim_reader.hpp`/`.cpp`.

```bash
# inside the container, with the sim running
colcon build --base-paths 3dpath && source install/setup.bash
ros2 run path3d sim_reader_demo default     # arg = world name (default | city)
ros2 run path3d path3d_main default         # the pipeline so far: Reader (prints stamps)
colcon test --base-paths 3dpath             # unit tests
```

`core/communication.hpp` has the in-process pub/sub used between pipeline stages:
`Topic<T>` (publisher), `LatestSlot<T>` (newest value only), `BoundedQueue<T>` (bounded stream,
drops oldest), `HistoryBuffer<T>` (last N messages by timestamp: `closest(t)`, `bracket(t)`,
`wait_until_newest(t)`). `core/threading.hpp` has `Stage` (thread lifecycle) and `Rate` (fixed-period loop).
Pipeline interfaces: `Reader` (publishes into `SensorTopics`, `io/reader.hpp`), `Mapper<M>` (builds a map and hands out
updates for it, `mapping/mapper.hpp`), `GlobalPlanner<M, R, P>` (answers replanning requests `R`
with plans `P`) and `LocalPlanner<M, P, R>` (follows plans `P`, sends requests `R`). Both are in
`planning/global_planner.hpp` and `planning/local_planner.hpp`. The two planners talk through `Pipe<P, R>`
(`core/communication.hpp`): plans go down, requests come back up. Planners read the map and the pose through `MapClient<M>` and `UavClient` (`pipeline/clients.hpp`), passed to
them at construction. A local planner returns its requests from `plan()`, and the stage publishes them
up the pipe. The stages own
the threads and publish plans. Request protocol (`planning/plan_protocol.hpp`): the local stage sends one
request at a time, numbered by `id`. The global planner answers every request once, with
`status = Failed` if there is no path, and the stage echoes the id in `request_id`. Answers to older
requests are dropped. Consumers synchronize the streams they combine: `FrameSynchronizer`
(`pipeline/sync.hpp`) pairs each depth image with the pose interpolated at its stamp, using a pose
`HistoryBuffer`, and adds the newest calibration and mount, giving a `SensorFrame` for the mapper.
`UavClient` reads the pose topic directly.
Maps are copied through updates: `MappingStage<M>` publishes `M::Update` messages (a keyframe with the
whole map first and then every 5 s, diffs in between, at most every 0.2 s of sim time), and each
`MapClient<M>` keeps its own copy of `M` and applies them in `sync()`, which a planner calls from its
own thread through `prepare()`. A copy therefore never changes during `plan()`, and reads need no
locks. A diff applies only on top of the version it was made from; after a lost update or a late
start the copy waits for the next keyframe. A map type joins by satisfying `ReplicatedMap`
(`mapping/map_view.hpp`): a nested `Update` with `keyframe`, `from_version`, `to_version` and `stamp`, and an
`apply(update)` method. Incremental maps send only what changed; any other copyable map sends a full
copy in every update. Loops are paced in wall time, so planners take time steps from message
stamps. Views derive from `MapView` (`mapping/map_view.hpp`);
concepts such as `GridMap` and `DistanceMap` say which queries an algorithm needs.
All settings (topics, rates, queue sizes, periods, timeouts) are in one nested `PipelineConfig`
(`core/config.hpp`): `sim_reader`, `mapping` (with `sync`), `global_planning` and `local_planning` (each
with its `map_client`). Each stage takes its own part, and `apps/main.cpp` fills the whole tree.
`pipeline/stages.hpp` has `MappingStage<M>`, `GlobalPlanningStage<M, P, R>` and
`LocalPlanningStage<M, P, R>`, and `apps/main.cpp`
is the only place that names concrete implementations (currently only the reader; no mapper or
planner is implemented yet).

### Flying the drone from C++

`path3d::Vehicle` (`io/vehicle.hpp`) is the interface to the autopilot. It takes world-frame (ENU)
`Setpoint`s and generic `FlightMode`s, so planners never see an autopilot protocol.
`path3d::MavsdkVehicle` (`io/mavsdk_vehicle.hpp`) implements it with [MAVSDK](https://mavsdk.mavlink.io)
(`Action`, `Offboard` and `Telemetry`), so it also works with PX4. MAVProxy forwards the SITL link to
`127.0.0.1:14560` (see `scripts/tmux.yml`), where MAVSDK listens (`MavsdkConfig::connection_url`).
MAVSDK is installed in the docker image (see `docker/Dockerfile`); rebuild the image with
`./docker/build.sh` after pulling this change.

- **Frames:** the autopilot works in local NED around its EKF origin. That origin is found at startup
  from the Gazebo pose, so the drone must stay on the ground until `VehicleState::frames_ready`.
- **Modes:** MAVSDK reports generic flight modes, which map to `FlightMode`. ArduPilot's GUIDED is
  offboard control in MAVSDK, so it is `FlightMode::External`.
- **Takeoff:** `takeoff()` uses the autopilot's own takeoff mode. Call `set_mode(External)` after
  it to hand control to the `CommanderStage`.
- **Safety note:** while offboard control is active MAVSDK keeps re-sending the newest setpoint, so a
  dead `CommanderStage` leaves the drone flying to its last commanded position.

The local planner returns a short `Trajectory` with each `plan()`. `CommanderStage` samples the newest
trajectory at 50 Hz and sends it to the vehicle. It sends nothing unless the vehicle is armed and in
`External` mode, and it holds position when the trajectory or the pose goes stale.

```bash
# inside the container, with the sim running (./scripts/start.sh)
ros2 run path3d vehicle_demo default   # arm, take off to 5 m, fly a 10 m square, land
```

---

### How to use tmux?

As configured in `docker/.tmux.conf` (prefix is `Ctrl + b`):

**Switching between windows**

(`gz`, `ardupilot`, `bridge`, `foxglove`, `planner`):
  - `Ctrl + b` then `0` .. `4`
  - `Ctrl + b` then `n` (next) / `p` (previous)

**Ending the Session**
- `Ctrl + b` then `q`

*the sky in the simulation is pink*

---

### Seeing the map in Foxglove

`path3d_main` builds a log-odds occupancy grid (`LogOddsMapper`) from the depth camera and
`LogOddsMapVisualizer` (`3dpath/include/visualization/`) publishes it, in the world frame
(`default` or `city`), at 2 Hz and only when the map changed:

- `/map/occupied_voxels` - `visualization_msgs/Marker`, occupied voxels as cubes colored by height
- `/map/occupied_cloud` - `sensor_msgs/PointCloud2`, the same voxels as points (`intensity` = log-odds)
- `/map/free_cloud` - `PointCloud2` of the free voxels, only with `map_visualizer.publish_free`

Run it in the `planner` tmux window once the simulation is up: `./scripts/run_mapping.sh [-n] [world]`
(builds `path3d` first, `-n` skips the build; the world defaults to the one `start.sh` was started with).
Add a 3D panel, set the display frame to the world name and turn the topics on. The topics are
latched, so a viewer that connects late still gets the last map.
