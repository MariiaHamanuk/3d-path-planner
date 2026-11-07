# Tutorial: your first planners (a straight line) {#tutorial_line}

In this tutorial you write two small planners and make the drone fly with them:

- the **global planner** makes a plan that has only two points: where the drone is and where it should go;
- the **local planner** follows that line.

We build it in small steps. After every step the program compiles and you can run it and see something. We start with empty classes and add the logic one piece at a time.

The finished code is already in the repository, so you can always look at it:

| What | Where |
|---|---|
| the two planners | `3dpath/include/planning/line_planner.hpp` |
| a program that flies the drone with them | `3dpath/apps/line_planner_demo.cpp` |
| the script that runs it | `scripts/run_line_planner.sh` |
| tests of the planners | `3dpath/test/line_planner_test.cpp` |

To follow along, make your own files (we use the names `my_planners.hpp` and `my_demo.cpp` below) and do not copy the finished ones.

[TOC]

## Before you start {#tutorial_before}

You need the simulation running, as in the README:

@code{.sh}
./scripts/start.sh -w city        # inside the container
@endcode

The planners need a map to start, and the map is built from the depth camera. If the depth camera does not render in your simulation, the planners will wait forever.

You should also know the picture of the whole system. The short version is in @ref outline. Here is the part we use:

- The **mapper** builds a map from the depth camera. Every planner keeps its own copy of the map.
- The **global planner** answers questions: "plan a path from here". It runs only when somebody asks.
- The **local planner** runs all the time, about 20 times per second. Each time it says where the drone should be during the next two seconds. That is a `path3d::Trajectory`. If it has no path, or the path does not work any more, it asks the global planner for a new one.
- The **commander** takes the newest trajectory and sends it to the drone. We add it in step 5.

## The four rules {#tutorial_rules}

Planners must follow four rules. We will see each of them in the code.

1. **Always answer.** The global planner must answer every question. If there is no path, it answers "failed". It must never stay silent, or the local planner waits forever.
2. **Keep asking.** The local planner asks for a path by returning a request. The system sends only one question at a time, so the planner returns the request again on every cycle until the path arrives.
3. **Refresh the trajectory every cycle.** The commander stops the drone if the trajectory is older than half a second. This protects the drone when a planner crashes.
4. **Use simulation time.** Take every time from message stamps, for example `pose.stamp`. Never use the computer clock.

## Step 1: two empty classes {#tutorial_step1}

Both planners are classes that inherit from a base class. The base class does the plumbing. We only write the `plan()` function.

Create `3dpath/include/planning/my_planners.hpp`:

@code{.cpp}
#pragma once

#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/clients.hpp"
#include "planning/global_planner.hpp"
#include "planning/local_planner.hpp"

using namespace path3d;

class LineGlobalPlanner : public GlobalPlanner<LogOddsGridMap, ReplanRequest, GlobalPath> {
public:
	LineGlobalPlanner(MapClient<LogOddsGridMap>& map, UavClient& uav)
	    : GlobalPlanner(map, uav) {}

	GlobalPath plan(const ReplanRequest& request) override {
		GlobalPath path;
		path.status = PlanStatus::Failed;  // we have no path yet
		return path;
	}
};

class LineLocalPlanner : public LocalPlanner<LogOddsGridMap, GlobalPath, ReplanRequest> {
public:
	LineLocalPlanner(MapClient<LogOddsGridMap>& map, UavClient& uav)
	    : LocalPlanner(map, uav) {}

	LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
		return {};  // do nothing yet
	}
};
@endcode

What is going on here:

- `GlobalPlanner<LogOddsGridMap, ReplanRequest, GlobalPath>` says: this planner works with the log-odds map, it receives `ReplanRequest`s and answers with `GlobalPath`s.
- `LocalPlanner<LogOddsGridMap, GlobalPath, ReplanRequest>` has the same three types, but in another order: it receives `GlobalPath`s and sends `ReplanRequest`s. Take care not to mix the order up.
- A planner is created with two helpers, passed to the base class:
  - a `MapClient`, which is the planner's own copy of the map. It is available as `m_map.map()`.
  - a `UavClient`, which tells where the drone is. It is available as `m_uav.pose()` and `m_uav.position()`.
- The global planner's `plan()` gets a question and returns an answer. For now it says "failed".
- The local planner's `plan()` is called once per cycle. It returns a `LocalStep`, which can hold a trajectory and a request. Both are empty for now.

This compiles, but it does nothing. Let us run it.

## Step 2: wire them in main() {#tutorial_step2}

A planner does nothing alone. A **thread** calls it again and again. We create the planners, the threads, and the things they need, in `main()`.

Create `3dpath/apps/my_demo.cpp`:

@code{.cpp}
#include "io/sim_reader.hpp"
#include "mapping/log_odds_mapper.hpp"
#include "pipeline/clients.hpp"
#include "pipeline/threads.hpp"
#include "planning/my_planners.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

int main() {
	using Map = LogOddsGridMap;
	PipelineConfig config;

	// 1. The topics everything talks through.
	SensorTopics sensors;
	Topic<Map::Update> map_updates;
	Topic<Trajectory> trajectories;

	// 2. The map: a mapper and the thread that runs it.
	LogOddsMapper mapper(config.log_odds_mapper);
	MappingThread<Map> mapping(sensors, mapper, map_updates, config.mapping);
	mapping.start();

	// 3. The planners. Each one gets its own copy of the map and its own view of the drone.
	MapClient<Map> global_map(map_updates);
	MapClient<Map> local_map(map_updates);
	UavClient global_uav(sensors.pose);
	UavClient local_uav(sensors.pose);
	LineGlobalPlanner global_planner(global_map, global_uav);
	LineLocalPlanner local_planner(local_map, local_uav);

	// 4. The threads that run the planners, and the link between them.
	Pipe<GlobalPath, ReplanRequest> pipe;
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(global_planner, pipe, std::nullopt);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories);
	global_thread.start();
	local_thread.start();

	// 5. The simulation reader goes last: data starts to flow when it is created.
	SimReader reader(config.sim_reader, sensors);

	while (true) {
		std::this_thread::sleep_for(1s);
		std::cout << "local planner cycles: " << local_thread.cycle_count() << "\n";
	}
}
@endcode

Add it to `3dpath/CMakeLists.txt`, next to the other programs:

@code{.cmake}
add_executable(my_demo apps/my_demo.cpp)
target_link_libraries(my_demo path3d)
@endcode

Build and run it:

@code{.sh}
colcon build --base-paths 3dpath --packages-select path3d
source install/setup.bash
ros2 run path3d my_demo
@endcode

You should see the number of cycles grow by about 20 every second, once the map is ready. It stays at 0 for the first seconds: the local planner does not run until it has a map and a drone position.

Things worth knowing about `main()`:

- **Order matters.** A topic gives a message only to those who are already listening. So we create the listeners first (the mapping thread, the clients, the planner threads) and the simulation reader last. When the reader starts, data starts to flow.
- Every planner gets its **own** map client and its own `UavClient`. Do not share them between two planners: each one runs on its own thread.
- The `Pipe` is the link between the two planners: plans go one way, requests go the other way.
- `std::nullopt` means "do not start with a question". The local planner will ask.

## Step 3: the global planner makes a line {#tutorial_step3}

Now the global planner gets some logic. It needs to know where to go, so we give it the goal when we create it. Its answer is the two points: the drone's position (which is in the question) and the goal.

Replace the global planner in `my_planners.hpp` with:

@code{.cpp}
class LineGlobalPlanner : public GlobalPlanner<LogOddsGridMap, ReplanRequest, GlobalPath> {
public:
	LineGlobalPlanner(MapClient<LogOddsGridMap>& map, UavClient& uav, Eigen::Vector3d goal)
	    : GlobalPlanner(map, uav), m_goal(goal) {}

	GlobalPath plan(const ReplanRequest& request) override {
		GlobalPath path;
		path.stamp = m_map.map().stamp;
		path.waypoints = {request.position, m_goal};  // from where the drone is, to the goal
		return path;
	}

private:
	Eigen::Vector3d m_goal;
};
@endcode

That is all. A few notes:

- `request.position` is where the drone was when the question was asked.
- `path.waypoints` is a list of points in the world frame. The first is the start and the last is the goal.
- `path.stamp` says when the plan was made. We use the time of our map copy.
- The base class sets `request_id` for us.

Change `main()` to pass a goal, 10 meters east and 5 meters up:

@code{.cpp}
const Eigen::Vector3d goal(10, 0, 5);
LineGlobalPlanner global_planner(global_map, global_uav, goal);
@endcode

Nothing new happens when you run it, because nobody asks the global planner yet. That comes next.

## Step 4: the local planner follows the line {#tutorial_step4}

The local planner does three things in each cycle:

1. If a new path has arrived, remember its two ends.
2. If it has no path yet, ask for one.
3. Otherwise find where the drone is on the line and plan the next two seconds from there.

Replace the local planner in `my_planners.hpp` with:

@code{.cpp}
class LineLocalPlanner : public LocalPlanner<LogOddsGridMap, GlobalPath, ReplanRequest> {
public:
	LineLocalPlanner(MapClient<LogOddsGridMap>& map, UavClient& uav, double speed = 2.0)
	    : LocalPlanner(map, uav), m_speed(speed) {}

	LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
		const StampedPose pose = *m_uav.pose();  // prepare() guarantees there is a pose

		// A new path arrived: remember its two ends.
		if (new_route && new_route->status == PlanStatus::Ok && new_route->waypoints.size() >= 2) {
			m_start = new_route->waypoints.front();
			m_goal = new_route->waypoints.back();
			m_has_route = true;
		}

		// No path yet: ask for one.
		if (!m_has_route) {
			ReplanRequest request;
			request.stamp = pose.stamp;
			request.reason = ReplanReason::NoPath;
			request.position = pose.pose.position;
			return {request, std::nullopt};
		}

		// Where is the drone on the line, and which way does the line go?
		const Eigen::Vector3d line = m_goal - m_start;
		const double length = line.norm();
		const Eigen::Vector3d direction = line / length;
		const double progress = std::clamp((pose.pose.position - m_start).dot(direction), 0.0, length);

		// Plan the next two seconds: a point every 0.1 s along the line.
		Trajectory trajectory;
		trajectory.stamp = pose.stamp;
		for (double t = 0.0; t <= 2.0; t += 0.1) {
			const double travelled = std::min(progress + m_speed * t, length);
			Setpoint point;
			point.stamp = pose.stamp + t;
			point.position = m_start + direction * travelled;
			if (travelled < length) point.velocity = direction * m_speed;
			point.yaw = std::atan2(direction.y(), direction.x());
			trajectory.points.push_back(point);
		}
		return {std::nullopt, trajectory};
	}

private:
	double m_speed;
	bool m_has_route = false;
	Eigen::Vector3d m_start = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_goal = Eigen::Vector3d::Zero();
};
@endcode

Add `#include <algorithm>` and `#include <cmath>` at the top of the file.

Let us look at the interesting parts.

**Asking for a path.** A `ReplanRequest` says where the drone is and why we ask (`NoPath`). The planner returns it as the first part of the result. It returns it again on the next cycle if the path has not arrived yet. This is rule 2.

**Finding the drone on the line.** `direction` is the unit vector from the start to the goal. The dot product `(position - start) . direction` tells how many meters along the line the drone is, even if it has drifted to the side. We call this `progress`.

**Planning from there.** Every 0.1 seconds for the next two seconds we compute a point: it is `progress + speed * t` meters along the line, but not further than the goal. Because we start from the drone's real progress, a drone that drifted off the line is pulled back to it. The points have increasing stamps that start at the stamp of the pose (rule 4), and we make a new trajectory on every cycle (rule 3).

**Velocity and yaw.** `velocity` is a hint for the controller: it is the speed along the line, and zero at the goal. `yaw` is the direction the drone looks: along the line.

To see it work, add this to `main()` before the reader, and print the trajectory instead of the cycles:

@code{.cpp}
const auto latest_trajectory = trajectories.subscribe_latest();
@endcode

@code{.cpp}
while (true) {
	std::this_thread::sleep_for(1s);
	if (const auto trajectory = latest_trajectory->latest()) {
		std::cout << "first point: " << trajectory->points.front().position.transpose()
		          << ", last point: " << trajectory->points.back().position.transpose() << "\n";
	}
}
@endcode

Run it. The first point should be where the drone is (the drone does not move yet). The last point is 4 meters further along the line: two seconds at 2 m/s.

## Step 5: make the drone fly {#tutorial_step5}

So far the trajectory goes nowhere. To fly, we need two more things:

- a **vehicle**, which is the connection to the autopilot (`path3d::MavsdkVehicle`);
- a **commander** (`path3d::CommanderThread`), which sends the newest trajectory to the vehicle 50 times per second. It sends nothing unless the drone is armed and in external control mode. It also holds the position if the trajectory gets old.

Add them to `main()`, after the simulation reader:

@code{.cpp}
Topic<VehicleState> vehicle_state;
const auto latest_state = vehicle_state.subscribe_latest();   // create this BEFORE the vehicle
MavsdkVehicle vehicle(config.mavsdk, sensors.pose, vehicle_state);
CommanderThread commander(vehicle, trajectories, vehicle_state, sensors.pose, config.commander);
commander.start();
@endcode

Now the commander is ready, but the drone is on the ground. The commander does not start the drone. We do it, in the right order:

First a small helper that waits until something is true, and gives up after a timeout:

@code{.cpp}
template <class Done>
bool wait_until(Done done, std::chrono::seconds timeout) {
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (!done()) {
		if (std::chrono::steady_clock::now() > deadline) return false;
		std::this_thread::sleep_for(100ms);
	}
	return true;
}
@endcode

Then the start, in the right order:

@code{.cpp}
UavClient uav(sensors.pose);   // create it early, before the reader, like the other listeners

// the autopilot is found and the drone knows its frames (the drone must stand still)
wait_until([&] {
	const auto state = latest_state->latest();
	return state && state->connected && state->frames_ready;
}, 120s);

vehicle.arm(5s);                                   // start the motors
vehicle.takeoff(5.0, 20s);                         // climb to 5 m
wait_until([&] { return uav.position()->z() > 4.5; }, 60s);
vehicle.set_mode(FlightMode::External, 5s);        // from now on the commander controls the drone
@endcode

After that we know where the drone hovers, and we can choose the goal. This is how the demo makes a line of a given length in a given direction:

@code{.cpp}
const Eigen::Vector3d start = *uav.position();
const Eigen::Vector3d goal = start + distance * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
@endcode

The yaw is the direction of the line in the world frame, in radians, counter-clockwise from east: 0 is east and pi/2 is north. The goal has the same height as the start, so the drone flies a horizontal line.

Create the planners and their threads **after** this point, with this goal, and start them. Then wait until the drone is near the goal and land:

@code{.cpp}
while ((*uav.position() - goal).norm() > 0.5) {
	std::this_thread::sleep_for(100ms);
}
vehicle.land(10s);
@endcode

You can read the whole program in `3dpath/apps/line_planner_demo.cpp`.

## Step 6: do not fly into things {#tutorial_step6}

Our drone has a map, so let us use it. A planner reads its copy of the map with `m_map.map()`. The most useful question is `occupancy(point)`. It answers `Free`, `Occupied` or `Unknown`.

In the local planner, check each point of the trajectory. If a point is inside an obstacle, do not publish the trajectory. Ask for a new path instead:

@code{.cpp}
if (m_map.map().occupancy(point.position) == Occupancy::Occupied) {
	ReplanRequest request;
	request.stamp = pose.stamp;
	request.reason = ReplanReason::Blocked;
	request.position = pose.pose.position;
	return {request, std::nullopt};
}
@endcode

Put it inside the loop that makes the points. Because we return no trajectory, the commander sees an old trajectory after half a second and holds the position: the drone stops in front of the obstacle. This is rule 3 working as a safety net.

In the global planner, we can refuse a goal that is inside an obstacle (rule 1: answer "failed" instead of staying silent):

@code{.cpp}
if (m_map.map().occupancy(m_goal) == Occupancy::Occupied) {
	path.status = PlanStatus::Failed;
	return path;
}
@endcode

Our global planner is not clever: if the line is blocked, it makes the same line again, and the drone waits in front of the obstacle. Teaching it to go around is the next exercise.

Unknown space is not treated as an obstacle here. A more careful planner may avoid it too.

The finished planners in `line_planner.hpp` have these checks.

## Run the finished demo {#tutorial_run}

The script builds the package and runs the demo:

@code{.sh}
./scripts/run_line_planner.sh [yaw] [distance] [world]
@endcode

- `yaw` is the direction of the line, in degrees, counter-clockwise from east. 0 is east, 90 is north, 180 is west. Default 0.
- `distance` is the length of the line in meters. Default 10.
- `world` is `default` or `city`. By default it is the world that `start.sh` started.

Examples, in the `planner` window of tmux:

@code{.sh}
./scripts/run_line_planner.sh                # 10 m east
./scripts/run_line_planner.sh 90 20          # 20 m north
./scripts/run_line_planner.sh 45 15 city     # 15 m north-east in the city
./scripts/run_line_planner.sh -n 0 5         # do not rebuild, 5 m east
@endcode

What happens: the program waits for the simulation and the autopilot, arms the drone and takes off to 5 meters, then flies the line at 2 m/s and prints the distance to the goal every second. It lands at the end. Press Ctrl-C to stop it and land at once.

The map is published for Foxglove as in `path3d_main` (`/map/occupied_voxels`), so you can watch the drone and the obstacles.


