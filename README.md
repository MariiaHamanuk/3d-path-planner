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
./src/scripts/start.sh <config-path>
```

This launches a tmux session defined in `src/scripts/tmux.yml` with:
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
Use `QGC_HOST=<ip> ./src/scripts/start.sh` if QGC runs on another machine.

Extra ArduCopter params live in `src/scripts/bebop.parm` (starting values, not tuned).

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
