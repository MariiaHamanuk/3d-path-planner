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

Depth: `/depth_camera` (32FC1, meters, 320x240, hfov 1.047 rad, range 0.1-200 m) and `/depth_camera/points` (PointCloud2), same viewpoint as `/camera`.

Extra ArduCopter params live in `scripts/bebop.parm` (starting values, not tuned).

---

### Reading the simulation from C++ (`3dpath/`)

`path3d::SimReader` (`3dpath/include/sim_reader.hpp`) subscribes to the sim and returns only
project types from `types.hpp` (Eigen poses, `cv::Mat` images, `cv::Matx` calibration) - no ROS types
outside `sim_reader.cpp`. It gives the drone pose in the world, color + depth images, camera
calibration and the base-link -> camera transforms.

```bash
# inside the container, with the sim running
colcon build --base-paths 3dpath && source install/setup.bash
ros2 run path3d sim_reader_demo default     # arg = world name (default | city)
colcon test --base-paths 3dpath             # unit tests
```

`communication.hpp` has the in-process pub/sub used between pipeline stages:
`Topic<T>` (publisher), `LatestSlot<T>` (newest value only), `BoundedQueue<T>` (bounded stream,
drops oldest), `HistoryBuffer<T>` (last N messages by timestamp: `closest(t)`, `bracket(t)`,
`wait_until_newest(t)`). `threading.hpp` has `Stage` (thread lifecycle) and `Rate` (fixed-period loop).

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
