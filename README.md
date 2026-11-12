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

**Flying from QGroundControl:** open QGC on the host (it autoconnects on UDP 14550; `QGC_HOST=<ip> ./scripts/start.sh` if it runs elsewhere). In the Fly view: *Takeoff* (slide to confirm), right-click the map -> *Go to location*, then *Land* or *RTL*.

**Comparing the global planners:** inside the container (no simulation needed)
```bash
./scripts/run_benchmark.sh
```
prints a table per test scene (path length, climb, time, explored nodes) for A* with each heuristic, Dijkstra, Lazy Theta*, RRT, RRT-Connect and RRT*, and writes `benchmark.csv`. See `docs/planning_algorithms.md`.

---

