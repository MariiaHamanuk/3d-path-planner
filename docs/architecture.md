## Architecture of the system

[TOC]

### Problem setup

System described tries to model different planning tasks (go to location, exploration of are) with UAV. Drone is equipped with LIDAR and navigation system, so pose and depth information is available to the system. Generally, we aim to solve navigating to known positon and exploration in previously unobserved area.

### Outline {#outline}

The system is a pipeline built around five interfaces. Each one can be replaced by another implementation without changing the others, which is what makes the pipeline flexible: a `path3d::Reader` for a real drone instead of the simulator, a `path3d::GlobalPlanner` that explores an enclosed area instead of flying to a point, or a different map in the `path3d::Mapper`. Click a box in the diagram to open its description.

@dot
digraph overview {
  rankdir=TB; nodesep=1.1; ranksep=0.5;
  fontname="Helvetica"; fontsize=11;
  node [shape=box, style="rounded,filled", fillcolor="#e8f0fe", color="#5b7bb5", fontname="Helvetica", fontsize=11, width=1.9, height=0.6];
  edge [fontname="Helvetica", fontsize=10, color="#555555"];

  sim [label="Simulator / real drone", shape=ellipse, fillcolor="#f1f1f1", color="#999999", height=0.4];
  autopilot [label="Autopilot", shape=ellipse, fillcolor="#f1f1f1", color="#999999", height=0.4];

  reader [label=<Reader<BR/><FONT POINT-SIZE="9" COLOR="#666666">the library's own thread</FONT>>, URL="\ref path3d::Reader"];
  mapper [label=<Mapper<BR/><FONT POINT-SIZE="9" COLOR="#666666">MappingThread</FONT>>, URL="\ref path3d::Mapper"];
  global [label=<GlobalPlanner<BR/><FONT POINT-SIZE="9" COLOR="#666666">GlobalPlanningThread</FONT>>, URL="\ref path3d::GlobalPlanner"];
  local [label=<LocalPlanner<BR/><FONT POINT-SIZE="9" COLOR="#666666">LocalPlanningThread</FONT>>, URL="\ref path3d::LocalPlanner"];
  commander [label=<Commander<BR/><FONT POINT-SIZE="9" COLOR="#666666">CommanderThread</FONT>>, URL="\ref path3d::CommanderThread"];
  vehicle [label=<Vehicle>, URL="\ref path3d::Vehicle"];

  { rank=same; global; local; }

  sim -> reader;
  reader -> mapper [label=" sensor data"];
  mapper -> global [label=" map"];
  mapper -> local [label=" map"];
  global -> local [dir=both, label="GlobalPath  →\n←  ReplanRequest"];
  local -> commander [label=" Trajectory"];
  commander -> vehicle [label=" Setpoint"];
  vehicle -> autopilot;
}
@enddot

Arrows carry data between threads through topics (see @ref threading). Not drawn: the planners and the commander also read the drone pose from the reader, and the commander reads the status of the vehicle.

- **Reader**: responsible for reading messages from external sources (`path3d::Reader`)
- **Mapper**: responsible for creating maps (`path3d::Mapper`)
- **GlobalPlanner** sets the general direction of flight (`path3d::GlobalPlanner`)
- **LocalPlanner** takes the path created by the global planner and optimizes it in a local horizon. The resulting trajectory is fed directly into the next interface (`path3d::LocalPlanner`)
- **Vehicle** acts as the interface for controlling the UAV (takeoff, land, setting velocity/acceleration at a certain point) (`path3d::Vehicle`)

`path3d::CommanderThread` and `path3d::LogOddsMapVisualizer` belong to no interface. The commander is the safety layer between the local planner and the vehicle, and the visualizer publishes the map for Foxglove.

To see the pieces working together, follow the step-by-step @ref tutorial_line, where you write a global and a local planner and fly the drone with them.

### Time and frames {#time_frames}

Two conventions are used everywhere in the system, and mixing them up is the most likely source of bugs.

#### Time {#time}

There are two clocks:

- **Simulation time** (`path3d::TimeSec`, seconds). This is the clock of the data. Every message has a `stamp` in sim time, and everything that compares times (the age of a trajectory, the pose at an image's time, throttling of map updates) uses stamps.
- **Wall time** (`std::chrono`). This is the clock of the threads: loop periods (`path3d::Rate`), waits, timeouts and queue pops.

The two differ when the simulation does not run at real-time speed (a real-time factor below 1, or a paused simulator): a wall-time loop then runs more often per simulated second.

@warning Take every time step from message stamps. Never derive a time step from a loop rate or from the wall clock, and never compare a stamp with the wall clock.

The settings in `path3d::PipelineConfig` follow the same rule: fields of type `TimeSec` are sim time, `std::chrono` fields and rates in Hz are wall time.

| Setting | Clock | Meaning |
|---|---|---|
| `MappingConfig::publish_period` | sim | minimum time between map diffs |
| `CommanderConfig::max_trajectory_age` | sim | how old a trajectory may be, measured from its stamp to the pose stamp |
| `MappingConfig::keyframe_period`, `idle_flush` | wall | when to repeat a keyframe and when to flush unpublished changes |
| `SyncConfig::pose_wait` | wall | how long to wait for a pose after an image |
| `LocalPlanningConfig::rate_hz`, `request_timeout` | wall | control cycle rate and the timeout of an unanswered request |
| `CommanderConfig::rate_hz`, `max_pose_silence` | wall | setpoint rate and how long the pose may stay unchanged |

@note `path3d::VehicleState::stamp` is wall time since the vehicle was created, not sim time. It tells how fresh the status is. It is not comparable with the stamps of poses and maps.

#### Frames {#frames}

Positions and poses always say which coordinate frame they are in (`path3d::FrameId`). Readers convert their own frame names (ROS TF strings, for example) to this enum, so no string travels with a message.

@dot
digraph frames {
  rankdir=LR; nodesep=0.4; ranksep=0.7;
  node [shape=box, style="rounded,filled", fillcolor="#e8f0fe", color="#5b7bb5", fontname="Helvetica", fontsize=10];
  edge [fontname="Helvetica", fontsize=9, color="#555555"];
  world [label="World (ENU)\nx east, y north, z up"];
  base [label="Base\nthe drone body"];
  depth [label="DepthCamera"];
  color [label="ColorCamera"];
  ned [label="Autopilot local frame (NED)\nx north, y east, z down", style="rounded,dashed", fillcolor="#f1f1f1", color="#999999"];
  world -> base [label="pose topic\nStampedPose, T_world_base"];
  base -> depth [label="base_to_depth\nstatic mount"];
  base -> color [label="base_to_color\nstatic mount"];
  world -> ned [style=dashed, label="NedOrigin\nfound at startup"];
}
@enddot

- **World frame (ENU).** The frame of the map, the planners, the trajectories and every setpoint: x east, y north, z up, in meters. It is the frame of the simulator.
- A `path3d::StampedPose` is the pose of a *child* frame in its *parent* frame, `T_parent_child`. The drone pose is `T_world_base`, and a camera mount is `T_base_camera`. A camera pose in the world is `T_world_base * T_base_camera`, which is how the mapper projects a depth image.
- **Camera frames.** Camera axes follow the Gazebo convention: x forward, y left, z up. The calibration matrix `K` is expressed for the image, where u runs right and v runs down. Depth is the distance along the optical axis.
- **Autopilot frame (NED).** ArduPilot works in its own local frame: x north, y east, z down, with the origin where its estimator started. Only `path3d::MavsdkVehicle` knows about it. It finds the origin at startup by pairing the autopilot's local position with the world pose of the drone (see `path3d::NedOrigin`), and converts every setpoint on the way out. Until it is found (`path3d::VehicleState::frames_ready`), setpoints are not sent, and the drone has to stand still.
- **Yaw.** In the world frame, yaw is counter-clockwise from east. In NED it is clockwise from north. The conversion is done in the vehicle layer too.

### Threading & communication {#threading}

In running system there are four threads running embodied by the types: 

 - `path3d::MappingThread`
 - `path3d::GlobalPlanningThread`
 - `path3d::LocalPlanningThread` 
 - `path3d::CommanderThread`
 
All they are descendants of `path3d::Thread`. First three serves as executors for corresponding interfaces described in the previous section (`path3d::Mapper`, `path3d::GlobalPlanner`, `path3d::LocalPlanner`). `path3d::Reader` has no dedicated runner, because usually data reading already runs in its separate thread (for example spin thread in case of ROS client).


#### Communication between threads

To have a fully working system he following data communication lines must be impelmented:

- `path3d::Mapper` must get synchronized data from the `path3d::Reader` to be able to make make updates
- `path3d::GlobalMapper` and `path3d::LocalMapper` must have a communication line between them as first must process requests for replanning and later must get created global plans to optimize. 
- Apart from that aforementioned components must have access to the current map and current pose of the UAV.

Threads exchange communicate with immutable messages -- `path3d::Msg<T>`. A `path3d::Topic<T>` is the publisher side. `publish()` runs on the publisher's thread (i.e `path3d::MappingThread` or any of threads discussed before) and puts the message into the buffer of every subscriber. Each subscriber gets its own buffer and chooses its policy when it subscribes. There are following subscribing policies:

- `path3d::LatestSlot` (created with `subscribe_latest()`)
  - **Keeps:** only the newest message.
  - **If the reader is slow:** older messages are overwritten and lost.
  - **Reader can block:** no, it reads the current value.
  - **Used for:** state, such as the drone pose, the trajectory and the vehicle status.
- `path3d::BoundedQueue` (created with `subscribe_queue(n)`)
  - **Keeps:** the last `n` messages, in order.
  - **If the reader is slow:** the oldest is dropped (and counted).
  - **Reader can block:** yes, `pop()` waits up to a timeout.
  - **Used for:** streams where order matters: depth images, map updates, replan requests.
- `path3d::HistoryBuffer` (created with `subscribe_history(n)`)
  - **Keeps:** the last `n` messages, sorted by stamp.
  - **If the reader is slow:** the oldest is evicted.
  - **Reader can block:** yes, until a message at or after a given time has arrived.
  - **Used for:** looking up the message closest to a time, such as the pose at a depth image's stamp.

Higher-level pieces are built from these:

- `path3d::Pipe` is two topics in opposite directions between a pair of threads. The global and
  local planner use one: plans go down, replan requests go up. Both sides read with a queue of
  one, so only the newest unanswered message counts.
- `path3d::FrameSynchronizer` combines a queue of depth images with a history of poses to build a
  `path3d::SensorFrame`. The mapper sees one frame with everything needed to project the image.
- `path3d::MapClient` turns the stream of map updates (a queue) into a private copy of the map for
  one thread. The copy changes only inside `sync()`, so the planner reads it without locks and it
  never changes in the middle of a plan. If an update is lost, the copy waits for the next
  keyframe, which holds the whole map.
- `path3d::UavClient` is a `LatestSlot` over the pose topic.

Time in the messages is simulation time (the `stamp` field). Loops are paced in wall-clock time with
`path3d::Rate`, so any time step must be computed from message stamps, never from the loop rate.


### path3d::Reader

First part of pipeline is digesting messages into system. Apart from reading messages from communication line (ROS etc.) reader converts messages to standard types used throught the codebase.


- `path3d::StampedPose`: a pose (`path3d::Pose`: position and orientation as Eigen types) with a timestamp and the two frames it relates. It is published twice:
  - `pose`: the drone's base link in the world frame, at the pose rate;
  - `base_to_color`, `base_to_depth`: where each camera is mounted on the drone, republished with the calibration.
- `path3d::ImageFrame`: one camera image as a `cv::Mat` with its stamp and frame. Color is 8-bit BGR. Depth is 32-bit float in meters along the optical axis, and inf/NaN means no return. Published as `color` and `depth`.
- `path3d::CameraCalibration`: the pinhole model of one camera (`K`, `R`, `P`, distortion, image size). Published as `color_calibration` and `depth_calibration`, republished regularly so a late subscriber still gets it.

Small types that go with them:

- `path3d::TimeSec`: time in seconds on the simulation clock, used for every `stamp`.
- `path3d::FrameId`: which coordinate frame a message is in (`World`, `Base`, `ColorCamera`, `DepthCamera`). The reader converts its own frame names, such as ROS TF strings, to this enum.
- `path3d::CameraId`: `Color` or `Depth`.

Subscriber (for example mapper) can use `path3d::FrameSynchronizer` to syncronize a depth image with the pose at its stamp, the calibration and the camera mount into a `path3d::SensorFrame`, which is what the mapper receives.


### path3d::Mapper

A `path3d::Mapper` turns sensor observations into a map. The rest of the system depends only on this interface, so a map representation (an occupancy grid, a TSDF with an ESDF, etc.) can be replaced without touching the threads around it. `path3d::Mapper<M>` is a template on the map type `M` that its readers keep copies of.

#### API

- `integrate(frame)` fuses one `path3d::SensorFrame` into the map.
- `version()` is a counter that grows by one every time `integrate()` changed the map. It is 0 for an empty map.
- `make_update(keyframe)` returns the update for the readers (see Output), or nothing when there is nothing to send.

A mapper is single-threaded: `path3d::MappingThread` calls all three from one thread, and `integrate()` receives frames in stamp order. The mapper itself needs no locks.

**The map type: `path3d::MapView`.** `path3d::MapView` is the base class of every map type: a snapshot of a map at one moment. It carries three fields:

- `stamp`: simulation time of the newest observation in this snapshot;
- `version`: the mapper version the snapshot was taken at;
- `frame`: the coordinate frame of all positions in it (`path3d::FrameId`, the world frame by default).

`MapView` has no queries, and its constructor is protected, so only derived maps exist. A derived map adds its own queries (for example `path3d::LogOddsGridMap` has `occupancy()`). Because the queries differ between maps, the code that uses a map is templated on it and states what it needs as a C++20 concept:

- `path3d::MapViewType`: any class derived from `MapView`.
- `path3d::ReplicatedMap`: a map that can be copied through updates (see Output). A mapper, the threads and `path3d::MapClient` need only this.
- `path3d::OccupancyMap`: has `occupancy(p)` at a world position, answering with a `path3d::Occupancy`: `Free`, `Occupied` or `Unknown`.
- `path3d::GridMap`: an occupancy map that is a regular voxel grid, with `resolution()`, `index_of()`, `center_of()` and `occupancy_at(index)`. Grid search algorithms (A*, Theta*, JPS) require this.
- `path3d::DistanceMap` adds `distance(p)` to the nearest obstacle, and `path3d::GradientMap` adds `gradient(p)` too, for gradient-based trajectory optimization.

A planner template picks the weakest concept it can work with, and then runs with any map that satisfies it.

#### Input

The mapper does not read topics. It receives a `path3d::SensorFrame`: one depth observation with everything needed to project it into the world.

- the depth image and its calibration,
- the camera mount (base link to camera),
- the drone pose interpolated at the image's stamp, so the image is projected from where it was taken.

`path3d::FrameSynchronizer` builds the frame from the reader's topics (see `path3d::Reader`). An image that cannot be paired (no calibration or mount yet, or it is older than the pose history) is dropped as *unsynced*. When the mapper is too slow, the short depth queue drops the oldest waiting image as *dropped*. Both are counted. Dropping is intentional: the map should follow the newest data instead of falling behind.

#### Output

Every reader of the map (each planner, the visualizer) keeps its own copy of it. The mapper does not share its working map. This costs memory, but a reader needs no locks, and its copy cannot change in the middle of a plan.

To keep the copies current, the mapper publishes updates of type `M::Update`, and a copy applies them with `M::apply(update)`, which must be deterministic. The transport reads only four fields of an update (the concept `path3d::MapUpdate`); everything else is up to the map:

- `stamp`: time of the update;
- `keyframe`: `true` if the update holds the whole map;
- `from_version`: the version a diff builds on (unused for a keyframe);
- `to_version`: the version of the map after the update is applied.

There are two kinds of updates:

- **Keyframe** (`make_update(true)`): the whole map. It can be applied to any copy, including an empty one, and it is never null.
- **Diff** (`make_update(false)`): only what changed since the previous `make_update()` call of either kind, with `from_version` set to the version at that call and `to_version` to `version()`. It can be applied only to a copy at exactly `from_version`. It is null when nothing changed.

The mapper keeps track of what it has already handed out, so a diff after a keyframe builds on that keyframe. Maps that are cheap to copy may skip diffs and make every update a keyframe.

**Receiving: `path3d::MapClient`.** `path3d::MapClient<M>` is a reader's copy of the map. It subscribes to the update topic with a queue, and applies the updates in `sync()`, which its owner calls from its own thread. Because the copy changes only inside `sync()`, the owner sees one consistent map between two calls. If a diff does not fit (an update was lost), the copy waits for the next keyframe (`ready()` tells whether it holds a full map).

#### Execution: path3d::MappingThread

`path3d::MappingThread` runs the loop around a mapper. It takes frames from the `FrameSynchronizer`, integrates each one, and decides when to publish a map update (`path3d::MappingConfig`):

- the first update is a keyframe, and a keyframe is repeated every `keyframe_period` (5 s of wall time), so a reader that joined late or lost an update can catch up;
- diffs are published at most every `publish_period` of sim time, because building an update is not free;
- when no frame has come for `idle_flush` (100 ms), changes that are still unpublished are flushed.

The thread counts integrated frames, dropped and unsynced images, and published updates and keyframes.


### path3d::GlobalPlanner

A `path3d::GlobalPlanner<M, R, P>` sets the general direction of flight: from where the drone is to wherever it should go, on the whole map. A global planner can be thought of as the manager of a task: going to a certain position behaves differently than exploring an enclosed area. It receives a request of type `R` (`path3d::ReplanRequest`) and returns a plan of type `P` (for example `path3d::GlobalPath`).

The search-based global planners (A*, Dijkstra, Lazy Theta*, RRT, RRT-Connect, RRT*), their settings and their comparison are described in @ref planning_algorithms.

#### API

- `prepare()` brings the planner's map copy up to date. It returns true once a plan can be made, that is, once the copy holds a full map. The thread calls it before every `plan()`.
- `plan(request)` computes the response to one request.
- **It must always answer.** When no path exists, `plan()` returns a plan that says so (`status = PlanStatus::Failed` and no waypoints), so the local planner is never left waiting.

#### Input

- `path3d::ReplanRequest` (local to global): "plan again from here". It has the drone `position`, a `reason` (`NoPath`, `Blocked` or `Deviated`), a `stamp` and a numeric `id`.
- The planner is built with two clients, and reads everything else through them:
  - `path3d::MapClient<M>`: its private copy of the map (see `path3d::MapClient`). It changes only inside `prepare()`, so the whole plan is made on one consistent map.
  - `path3d::UavClient`: the newest pose of the drone (`m_uav.pose()`, `m_uav.position()`), with its stamp.

The map type is a template parameter, so a planner states the weakest map concept it needs (`path3d::GridMap` for a grid search, for example) and works with any mapper that produces such a map.

#### Output

- `path3d::GlobalPath` (global to local): the answer. It has the `request_id` of the request it answers, a `status` (`Ok` or `Failed`), the `waypoints` in the world frame (the first is the start and the last the goal), the `frame` and a `stamp`. The stamp is the time of the map or state the path was planned from.
- The thread fills in `request_id`. The planner does not.

Planner and thread are templated on the request and plan types and constrain them with the concepts `path3d::PlanRequest` (a stamp and a writable `id`) and `path3d::PlanAnswer` (a stamp and a writable `request_id`), so another pair of message types can be used.

#### Execution: path3d::GlobalPlanningThread

The thread is event-driven. It waits for a request on a queue (for up to `request_wait`, 50 ms), and then:

- It keeps only the newest unanswered request. If a new one arrives before the old one is answered, the old one is replaced.
- It calls `prepare()` on every wake-up, also while idle, so the map copy keeps up with the mapper.
- When there is a request and `prepare()` returned true, it calls `plan(request)`, sets `request_id` to the id of the request, and publishes the plan down the `path3d::Pipe`. A request is held until the map is ready.
- It can start with an *initial* request (id 0), so a planner can plan once at start without waiting for the local planner.

All times are simulation time, taken from message stamps. Never use the wall clock.


### path3d::LocalPlanner

A `path3d::LocalPlanner<M, P, R>` takes the path of the global planner and turns it into the near-future motion of the drone: a short trajectory, a few seconds ahead, that the commander flies. It runs continuously, so it is also the part that reacts: it sees new obstacles in the map, notices that the drone has drifted away from the path, and asks the global planner for a new route when the current one can no longer be followed.

#### API

- `prepare()` brings the map copy up to date, and returns true when a map and a pose both exist. The thread calls `plan()` only then.
- `plan(new_route)` is one control cycle. It returns a `path3d::LocalStep` with two optional parts: a `trajectory` and a replan `request` (see Output).

#### Input

- `new_route`: the newest plan that arrived since the last cycle (it answers the newest request), or null. The planner keeps its own copy of the current plan between cycles.
- The same two clients as in the global planner: `path3d::MapClient<M>` (`m_map.map()`) and `path3d::UavClient` (`m_uav.pose()`). The map copy changes only inside `prepare()`, never during `plan()`. Since `prepare()` guarantees a pose, `plan()` can use it without checking.

#### Output

- `trajectory`: the drone's next states. If it is empty, the commander keeps flying the last trajectory. A `path3d::Trajectory` is a list of `path3d::Setpoint`s with increasing stamps, in the world frame (ENU, the frame of the map):
  - `position`, and optionally `velocity` and `acceleration` as feed-forward terms (zero means just fly to or hold the position);
  - `yaw`: the heading of the body x axis, counter-clockwise from world x.
- `request`: a `path3d::ReplanRequest`, for when the current plan can no longer be followed (no path, blocked, or the drone has deviated).

The commander samples the trajectory at its own rate, so the planner may be slower or irregular. It has to refresh the trajectory every cycle: when the trajectory is older than `max_trajectory_age` (0.5 s of sim time) or has run out, the commander holds the position.

#### Execution: path3d::LocalPlanningThread

The thread runs at a fixed rate (`rate_hz`, 20 Hz of wall time). Each cycle it:

1. Takes the newest plan from the pipe, if there is one. A plan that answers an *older* request than the newest one sent is dropped as stale and counted.
2. Calls `prepare()` and, if it returned true, `plan()` with the plan it got (or null).
3. Publishes the trajectory, if there is one, on the trajectory topic.
4. Sends the request, if there is one, up the pipe.


### path3d::CommanderThread

The commander is the last link between planning and the drone. It takes the trajectory of the local planner and sends it, as one setpoint at a time and at a steady rate, to the `path3d::Vehicle`. It is also the only place that decides *whether* anything may be sent. Planners can stall, crash or send old data, and the autopilot may be disarmed, in a pilot's hands or in a failsafe. The commander makes sure none of this turns into a command that the drone should not follow.

#### What it reads

All three sources are read as "newest value" (`path3d::LatestSlot`), because only the current state matters:

- the newest `path3d::Trajectory` from the local planner;
- the `path3d::VehicleState` published by the vehicle: `connected`, `armed`, `frames_ready` (the world frame and the autopilot's own frame are known, so a setpoint can be converted) and the generic `path3d::FlightMode`;
- the drone pose in the world frame. Its stamp is the commander's clock: everything is compared in simulation time, never with the wall clock.

The thresholds below are in `path3d::CommanderConfig`.

#### What it does each cycle

The thread runs at a fixed rate (`rate_hz`, 50 Hz of wall time), regardless of how fast the planner produces trajectories. Each cycle it takes one of three decisions:

1. **Send nothing.** This is the case unless the vehicle is connected, armed, has its frames ready and is in `FlightMode::External`, or while no trajectory has arrived yet. This way the commander never fights a pilot or an autopilot failsafe, and manual control works until a planner takes over.
2. **Follow the trajectory.** The trajectory is sampled at the stamp of the newest pose, interpolating between its points, and the result is sent with `Vehicle::send()`.
3. **Hold position.** If the trajectory cannot be followed, the commander sends the position the drone had when it noticed the problem. The hold setpoint is used once, at the current pose, and not updated while the problem lasts, so the drone does not slide along with a drifting pose. The trajectory cannot be followed when:
   - it is too old: its stamp is more than `max_trajectory_age` (0.5 s of sim time) behind the pose. This means the planner stopped refreshing it;
   - it has run out (the pose time is after its last point);
   - the pose is stale: its stamp has not advanced for `max_pose_silence` (200 ms of wall time).

When the trajectory is valid again, following resumes at once. When the vehicle stops being controllable, the hold is forgotten.

This is why the local planner must refresh the trajectory every cycle: the commander treats silence as failure and stops the drone.

#### The vehicle

`path3d::Vehicle` is the interface to the autopilot. Only project types cross it: setpoints are in the world frame (ENU) and modes are generic, so neither the planners nor the commander see an autopilot protocol or frame.

- `set_mode()`, `arm()`, `takeoff()` and `land()` block until the autopilot acknowledges the command or the timeout expires, and return whether it was accepted. They are called by the application, from one thread at a time, not by the commander.
- `send(setpoint)` never blocks, is thread-safe, and is ignored while the vehicle is not ready for setpoints.
- The implementation publishes `path3d::VehicleState` on a topic given at construction.

The commander sends setpoints only. Getting the drone ready is the application's job: arm, take off, then `set_mode(FlightMode::External)`. After that the commander takes over. `path3d::MavsdkVehicle` implements the interface with MAVSDK: it converts the setpoints from ENU to the autopilot's NED frame and sends them through MAVSDK's offboard control.

Note that while offboard control is active MAVSDK keeps re-sending the newest setpoint by itself. If the commander thread dies, the drone keeps flying to the last commanded position instead of stopping.
