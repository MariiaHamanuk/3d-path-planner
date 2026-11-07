#pragma once

#include "core/communication.hpp"
#include "pipeline/clients.hpp"
#include "core/config.hpp"
#include "planning/global_planner.hpp"
#include "planning/local_planner.hpp"
#include "mapping/mapper.hpp"
#include "planning/plan_protocol.hpp"
#include "io/reader.hpp"
#include "pipeline/sync.hpp"
#include "core/threading.hpp"
#include "io/vehicle.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

namespace path3d {

/**
 * @brief Stage that turns synchronized sensor frames into map updates.
 *
 * Integrates every frame into the Mapper, then publishes a keyframe first and periodically after
 * that, with diffs (throttled by sim time) in between. Subscribes in the constructor, so create it
 * before the Reader.
 * @tparam M Map type the readers keep copies of.
 */
template <ReplicatedMap M>
class MappingStage : public Stage {
public:
	using Update = typename M::Update;

	/**
	 * @param in Sensor streams to synchronize.
	 * @param mapper Mapper that builds the map and produces the updates.
	 * @param out Topic the updates are published on.
	 * @param config Stage settings (see MappingConfig).
	 */
	MappingStage(SensorTopics& in, Mapper<M>& mapper, Topic<Update>& out,
	             const MappingConfig& config = {})
	    : m_sync(in, config.sync), m_mapper(mapper), m_config(config), m_out(out) {}

	~MappingStage() override { stop(); }

	std::size_t integrated_count() const { return m_integrated_count; }
	std::size_t dropped_count() const { return m_sync.dropped_count(); }
	std::size_t unsynced_count() const { return m_sync.unsynced_count(); }
	std::size_t published_update_count() const { return m_published_update_count; }
	std::size_t published_keyframe_count() const { return m_published_keyframe_count; }

private:
	void run() override {
		std::uint64_t published_version = 0;
		TimeSec published_stamp = -std::numeric_limits<TimeSec>::infinity();
		bool sent_keyframe = false;
		std::chrono::steady_clock::time_point last_keyframe_time;

		while (running()) {
			const Msg<SensorFrame> frame = m_sync.next(m_config.idle_flush);
			if (frame) {
				m_mapper.integrate(*frame);
				++m_integrated_count;
			}
			if (m_mapper.version() == 0) continue;

			const auto now = std::chrono::steady_clock::now();

			const bool keyframe_due = !sent_keyframe || now - last_keyframe_time >= m_config.keyframe_period;

			const bool changed = m_mapper.version() != published_version;

			const bool diff_due = changed && (!frame || frame->stamp - published_stamp >= m_config.publish_period);

			if (!keyframe_due && !diff_due) continue;

			if (auto update = m_mapper.make_update(keyframe_due)) {
				published_version = m_mapper.version();
				if (frame) published_stamp = frame->stamp;
				if (keyframe_due) {
					sent_keyframe = true;
					last_keyframe_time = now;
					++m_published_keyframe_count;
				}
				m_out.publish(std::move(*update));
				++m_published_update_count;
			}
		}
	}

	void on_stop() override { m_sync.close(); }

	FrameSynchronizer m_sync;
	Mapper<M>& m_mapper;
	MappingConfig m_config;

	std::atomic<std::size_t> m_integrated_count{0};
	std::atomic<std::size_t> m_published_update_count{0};
	std::atomic<std::size_t> m_published_keyframe_count{0};

	Topic<Update>& m_out;
};

/**
 * @brief Stage that answers replanning requests with plans.
 *
 * Event-driven: waits for requests, calls the planner once its map is ready, and publishes each
 * plan with the id of the request it answers. Only the newest unanswered request is kept.
 * @tparam M Map type the planner reads.
 * @tparam P Plan message published down the pipe.
 * @tparam R Replanning request received from the pipe.
 */
template <ReplicatedMap M, PlanAnswer P, PlanRequest R>
class GlobalPlanningStage : public Stage {
public:
	/**
	 * @param planner Planner that computes the plans.
	 * @param pipe Link to the local planner: requests come up, plans go down.
	 * @param initial First request, handled at start (use id 0); empty for none.
	 * @param config Stage settings (see GlobalPlanningConfig).
	 */
	GlobalPlanningStage(GlobalPlanner<M, R, P>& planner, Pipe<P, R>& pipe, std::optional<R> initial,
	                    const GlobalPlanningConfig& config = {})
	    : m_planner(planner),
	      m_requests(pipe.up().subscribe_queue(1)),
	      m_down(pipe.down()),
	      m_initial_request(initial ? std::make_shared<const R>(*initial) : nullptr),
	      m_wait_for_plan(config.request_wait) {}

	~GlobalPlanningStage() override { stop(); }

	std::size_t plan_call_count() const { return m_plan_call_count; }

private:
	void run() override {
		Msg<R> pending = m_initial_request;
		while (running()) {
			if (Msg<R> request = m_requests->pop(m_wait_for_plan)) pending = std::move(request);
			/// also while idle, so the map copy keeps up
			const bool ready = m_planner.prepare();
			if (pending && ready) {
				++m_plan_call_count;
				P plan = m_planner.plan(*pending);
				plan.request_id = pending->id;
				m_down.publish(std::move(plan));
				pending.reset();
			}
		}
	}

	void on_stop() override { m_requests->close(); }

	GlobalPlanner<M, R, P>& m_planner;

	std::shared_ptr<BoundedQueue<R>> m_requests;
	Topic<P>& m_down;

	Msg<R> m_initial_request;
	std::chrono::milliseconds m_wait_for_plan;
	std::atomic<std::size_t> m_plan_call_count{0};
};

/**
 * @brief Stage that runs the local planner at a fixed rate.
 *
 * Each cycle hands the newest global plan to the planner and publishes the replan request it
 * returns. Requests are numbered and sent one at a time; an unanswered request times out, and
 * answers to older requests are dropped as stale.
 * @tparam M Map type the planner reads.
 * @tparam P Plan message received from the pipe.
 * @tparam R Replanning request published up the pipe.
 */
template <ReplicatedMap M, PlanAnswer P, PlanRequest R>
class LocalPlanningStage : public Stage {
public:
	/**
	 * @param planner Planner that follows the plans.
	 * @param pipe Link to the global planner: plans come down, requests go up.
	 * @param trajectories Topic the planner's trajectories are published on (for the CommanderStage).
	 * @param config Stage settings (see LocalPlanningConfig).
	 */
	LocalPlanningStage(LocalPlanner<M, P, R>& planner, Pipe<P, R>& pipe,
	                   Topic<Trajectory>& trajectories, const LocalPlanningConfig& config = {})
	    : m_planner(planner),
	      m_config(config),
	      m_up(pipe.up()),
	      m_routes(pipe.down().subscribe_queue(1)),
	      m_trajectories(trajectories) {}

	~LocalPlanningStage() override { stop(); }

	std::size_t cycle_count() const { return m_cycle_count; }
	std::size_t sent_request_count() const { return m_sent_request_count; }
	std::size_t stale_route_count() const { return m_stale_route_count; }
	std::size_t request_timeout_count() const { return m_request_timeout_count; }
	std::size_t published_trajectory_count() const { return m_published_trajectory_count; }

private:
	void run() override {
		Rate rate(m_config.rate_hz);
		Msg<P> pending_global_plan;
		std::uint64_t last_request_id = 0;
		bool awaiting = false;
		std::chrono::steady_clock::time_point sent_at;
		while (running()) {
			if (awaiting && std::chrono::steady_clock::now() - sent_at >= m_config.request_timeout) {
				/// lost: allow a new request (its answer will supersede the old one)
				awaiting = false;
				++m_request_timeout_count;
			}

			if (Msg<P> route = m_routes->pop(std::chrono::milliseconds::zero())) {
				if (route->request_id < last_request_id) {
					++m_stale_route_count;
				} else {
					if (route->request_id == last_request_id) awaiting = false;
					pending_global_plan = std::move(route);
				}
			}

			if (m_planner.prepare()) {
				++m_cycle_count;
				LocalStep<R> step = m_planner.plan(pending_global_plan.get());
				pending_global_plan.reset();
				if (step.trajectory) {
					m_trajectories.publish(std::move(*step.trajectory));
					++m_published_trajectory_count;
				}
				std::optional<R>& request = step.request;
				if (request && !awaiting) {
					request->id = ++last_request_id;
					awaiting = true;
					sent_at = std::chrono::steady_clock::now();
					m_up.publish(std::move(*request));
					++m_sent_request_count;
				}
			}

			rate.sleep();
		}
	}

	LocalPlanner<M, P, R>& m_planner;
	LocalPlanningConfig m_config;

	Topic<R>& m_up;
	std::shared_ptr<BoundedQueue<P>> m_routes;
	Topic<Trajectory>& m_trajectories;

	std::atomic<std::size_t> m_cycle_count{0};
	std::atomic<std::size_t> m_sent_request_count{0};
	std::atomic<std::size_t> m_stale_route_count{0};
	std::atomic<std::size_t> m_request_timeout_count{0};
	std::atomic<std::size_t> m_published_trajectory_count{0};
};

/**
 * @brief Stage that streams the newest trajectory to the vehicle at a fixed rate.
 *
 * The single place for the rules that keep the drone safe when something upstream misbehaves:
 *  - sends nothing unless the vehicle is connected, armed, ready for setpoints and in
 *    FlightMode::External, so it never fights a pilot or an autopilot failsafe;
 *  - sends nothing before the first trajectory, so manual control works until a planner takes over;
 *  - samples the trajectory at the time of the newest pose (sim time);
 *  - holds position (a setpoint latched once at the current pose) when the pose is stale, the
 *    trajectory is too old (the planner stopped refreshing it) or has run out.
 */
class CommanderStage : public Stage {
public:
	/**
	 * @param vehicle Vehicle the setpoints are sent to.
	 * @param trajectories Trajectories from the local planner (the newest one is followed).
	 * @param state Vehicle status, as published by the vehicle.
	 * @param poses Drone pose in the world frame (its stamp is the commander's clock).
	 * @param config Stage settings (see CommanderConfig).
	 */
	CommanderStage(Vehicle& vehicle, Topic<Trajectory>& trajectories, Topic<VehicleState>& state,
	               Topic<StampedPose>& poses, const CommanderConfig& config = {})
	    : m_vehicle(vehicle),
	      m_config(config),
	      m_trajectories(trajectories.subscribe_latest()),
	      m_state(state.subscribe_latest()),
	      m_poses(poses.subscribe_latest()) {}

	~CommanderStage() override { stop(); }

	std::size_t sent_count() const { return m_sent_count; }
	std::size_t hold_count() const { return m_hold_count; }

private:
	void run() override {
		Rate rate(m_config.rate_hz);
		std::optional<Setpoint> hold;
		TimeSec last_pose_stamp = -std::numeric_limits<TimeSec>::infinity();
		std::chrono::steady_clock::time_point pose_changed_at = std::chrono::steady_clock::now();

		while (running()) {
			const auto wall_now = std::chrono::steady_clock::now();
			const Msg<VehicleState> state = m_state->latest();
			const Msg<StampedPose> pose = m_poses->latest();
			const Msg<Trajectory> trajectory = m_trajectories->latest();
			if (pose && pose->stamp != last_pose_stamp) {
				last_pose_stamp = pose->stamp;
				pose_changed_at = wall_now;
			}

			const bool controllable = state && state->connected && state->armed && state->frames_ready &&
			                          state->mode == FlightMode::External;
			if (!controllable || !trajectory) {
				hold.reset();
				rate.sleep();
				continue;
			}

			const bool pose_fresh = pose && wall_now - pose_changed_at < m_config.max_pose_silence;
			std::optional<Setpoint> target;
			if (pose_fresh && pose->stamp - trajectory->stamp <= m_config.max_trajectory_age) {
				target = trajectory->sample(pose->stamp);
			}

			if (target) {
				hold.reset();
				m_vehicle.send(*target);
				++m_sent_count;
			} else if (hold || pose) {
				if (!hold) hold = Setpoint{.stamp = pose->stamp, .position = pose->pose.position, .yaw = yaw_of(pose->pose.orientation)};
				if (pose) hold->stamp = pose->stamp;
				m_vehicle.send(*hold);
				++m_hold_count;
			}

			rate.sleep();
		}
	}

	Vehicle& m_vehicle;
	CommanderConfig m_config;

	std::shared_ptr<LatestSlot<Trajectory>> m_trajectories;
	std::shared_ptr<LatestSlot<VehicleState>> m_state;
	std::shared_ptr<LatestSlot<StampedPose>> m_poses;

	std::atomic<std::size_t> m_sent_count{0};
	std::atomic<std::size_t> m_hold_count{0};
};

}
