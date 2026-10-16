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
- `bridge`- ROS-Gz bridge
- `foxglove`- Foxglove WebSocket bridge
- `planner` - Shell pane for path-planning (wip)

---

### How to use tmux?

As configured in `docker/.tmux.conf` (prefix is `Ctrl + b`):

**Switching between windows**

(`gz`, `bridge`, `foxglove`, `planner`):
  - `Ctrl + b` then `0` .. `3`
  - `Ctrl + b` then `n` (next) / `p` (previous)

**Ending the Session**
- `Ctrl + b` then `q`

*the sky in the simulation is pink*
