#include "io/mavsdk_vehicle.hpp"

#include <future>
#include <mutex>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace path3d {
namespace {

/// Runs a MAVSDK `*_async` call and waits up to `timeout` for its result; true on Success.
/// MAVSDK may call the callback more than once (progress updates, retries) or after the wait has
/// ended, so only the first result counts and the shared state outlives this function.
template <class Plugin, class Start>
bool run(const std::shared_ptr<Plugin>& plugin, Start start, std::chrono::milliseconds timeout) {
	if (!plugin) return false;
	struct Answer {
		std::promise<bool> success;
		std::once_flag first;
	};
	auto answer = std::make_shared<Answer>();
	std::future<bool> success = answer->success.get_future();
	start(*plugin, [answer](auto result) {
		std::call_once(answer->first, [&] { answer->success.set_value(result == decltype(result)::Success); });
	});
	return success.wait_for(timeout) == std::future_status::ready && success.get();
}

/// ENU yaw (rad, counter-clockwise from east) -> MAVSDK yaw (deg, clockwise from north).
float to_ned_yaw_deg(double enu_yaw) {
	return static_cast<float>(enu_to_ned_yaw(enu_yaw) * 180.0 / std::numbers::pi);
}

}

MavsdkVehicle::MavsdkVehicle(MavsdkConfig config, Topic<StampedPose>& world_pose,
                             Topic<VehicleState>& state_out)
    : Vehicle(state_out),
      m_config(std::move(config)),
      m_world_pose(world_pose.subscribe_latest()),
      m_started(std::chrono::steady_clock::now()),
      m_mavsdk(mavsdk::Mavsdk::Configuration(mavsdk::ComponentType::CompanionComputer)),
      m_origin(m_config.origin_samples) {
	const mavsdk::ConnectionResult result = m_mavsdk.add_any_connection(m_config.connection_url);
	if (result != mavsdk::ConnectionResult::Success) {
		std::ostringstream message;
		message << "MavsdkVehicle: cannot connect to " << m_config.connection_url << ": " << result;
		throw std::runtime_error(message.str());
	}
	m_thread = std::jthread([this](std::stop_token stop) { connect(stop); });
}

MavsdkVehicle::~MavsdkVehicle() {
	m_thread.request_stop();
	m_thread.join();
	// Unsubscribe before anything is destroyed, so no callback runs on a half-destroyed object.
	if (m_system) m_system->unsubscribe_is_connected(m_connected_handle);
	if (m_telemetry) {
		m_telemetry->unsubscribe_armed(m_armed_handle);
		m_telemetry->unsubscribe_flight_mode(m_mode_handle);
		m_telemetry->unsubscribe_position_velocity_ned(m_position_handle);
	}
}

template <class Change>
void MavsdkVehicle::update_state(Change&& change) {
	VehicleState state;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		change(m_state);
		m_state.stamp = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_started).count();
		state = m_state;
	}
	m_state_out.publish(state);
}

void MavsdkVehicle::connect(std::stop_token stop) {
	const double attempt_s = std::chrono::duration<double>(m_config.discovery_attempt).count();
	std::optional<std::shared_ptr<mavsdk::System>> found;
	while (!stop.stop_requested() && !(found = m_mavsdk.first_autopilot(attempt_s))) {
	}
	if (!found) return;

	const std::shared_ptr<mavsdk::System> system = *found;
	const auto telemetry = std::make_shared<mavsdk::Telemetry>(system);
	const auto action = std::make_shared<mavsdk::Action>(system);
	const auto offboard = std::make_shared<mavsdk::Offboard>(system);
	telemetry->set_rate_position_velocity_ned(m_config.position_rate_hz);

	const auto connected_handle = system->subscribe_is_connected(
	    [this](bool connected) { update_state([&](VehicleState& s) { s.connected = connected; }); });
	const auto armed_handle = telemetry->subscribe_armed(
	    [this](bool armed) { update_state([&](VehicleState& s) { s.armed = armed; }); });
	const auto mode_handle = telemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode mode) {
		update_state([&](VehicleState& s) { s.mode = to_flight_mode(mode); });
	});
	const auto position_handle = telemetry->subscribe_position_velocity_ned(
	    [this](mavsdk::Telemetry::PositionVelocityNed position) { on_position(position); });
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_system = system;
		m_telemetry = telemetry;
		m_action = action;
		m_offboard = offboard;
		m_connected_handle = connected_handle;
		m_armed_handle = armed_handle;
		m_mode_handle = mode_handle;
		m_position_handle = position_handle;
	}
	update_state([&](VehicleState& s) { s.connected = system->is_connected(); });
}

void MavsdkVehicle::on_position(const mavsdk::Telemetry::PositionVelocityNed& position) {
	const Eigen::Vector3d ned(position.position.north_m, position.position.east_m, position.position.down_m);
	if (!ned.allFinite()) return;
	const Msg<StampedPose> world = m_world_pose->latest();
	bool became_ready = false;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_ned_position = position.position;
		if (world && !m_origin.ready()) {
			m_origin.add(world->pose.position, ned);
			became_ready = m_origin.ready();
		}
	}
	if (became_ready) update_state([](VehicleState& s) { s.frames_ready = true; });
}

bool MavsdkVehicle::set_mode(FlightMode mode, std::chrono::milliseconds timeout) {
	std::shared_ptr<mavsdk::Action> action;
	std::shared_ptr<mavsdk::Offboard> offboard;
	std::optional<mavsdk::Telemetry::PositionNed> position;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		action = m_action;
		offboard = m_offboard;
		position = m_ned_position;
	}
	switch (mode) {
		case FlightMode::External: {
			if (!offboard || !position) return false;
			// MAVSDK refuses to start offboard control without a setpoint: hold the current position
			// until the CommanderThread sends its own.
			const Msg<StampedPose> world = m_world_pose->latest();
			const double yaw = world ? yaw_of(world->pose.orientation) : 0.0;
			const mavsdk::Offboard::PositionNedYaw hold{position->north_m, position->east_m,
			                                            position->down_m, to_ned_yaw_deg(yaw)};
			if (offboard->set_position_ned(hold) != mavsdk::Offboard::Result::Success) return false;
			return run(offboard, [](mavsdk::Offboard& o, auto done) { o.start_async(done); }, timeout);
		}
		case FlightMode::Hold:
			return run(action, [](mavsdk::Action& a, auto done) { a.hold_async(done); }, timeout);
		case FlightMode::Land:
			return run(action, [](mavsdk::Action& a, auto done) { a.land_async(done); }, timeout);
		case FlightMode::ReturnHome:
			return run(action, [](mavsdk::Action& a, auto done) { a.return_to_launch_async(done); }, timeout);
		case FlightMode::Manual:
		case FlightMode::Unknown:
			return false;
	}
	return false;
}

bool MavsdkVehicle::arm(std::chrono::milliseconds timeout) {
	std::shared_ptr<mavsdk::Action> action;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		action = m_action;
	}
	return run(action, [](mavsdk::Action& a, auto done) { a.arm_async(done); }, timeout);
}

bool MavsdkVehicle::takeoff(double altitude_m, std::chrono::milliseconds timeout) {
	std::shared_ptr<mavsdk::Action> action;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		action = m_action;
	}
	const bool altitude_set = run(action, [altitude_m](mavsdk::Action& a, auto done) { a.set_takeoff_altitude_async(static_cast<float>(altitude_m), done); }, timeout);
	return altitude_set && run(action, [](mavsdk::Action& a, auto done) { a.takeoff_async(done); }, timeout);
}

bool MavsdkVehicle::land(std::chrono::milliseconds timeout) {
	return set_mode(FlightMode::Land, timeout);
}

void MavsdkVehicle::send(const Setpoint& setpoint) {
	std::shared_ptr<mavsdk::Offboard> offboard;
	Eigen::Vector3d position;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_offboard || !m_origin.ready()) {
			++m_dropped_setpoint_count;
			return;
		}
		offboard = m_offboard;
		position = m_origin.to_ned(setpoint.position);
	}
	const Eigen::Vector3d velocity = enu_to_ned(setpoint.velocity);
	const Eigen::Vector3d acceleration = enu_to_ned(setpoint.acceleration);
	const float yaw = to_ned_yaw_deg(setpoint.yaw);
	const mavsdk::Offboard::Result result = offboard->set_position_velocity_acceleration_ned(
	    {static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()), yaw},
	    {static_cast<float>(velocity.x()), static_cast<float>(velocity.y()), static_cast<float>(velocity.z()), yaw},
	    {static_cast<float>(acceleration.x()), static_cast<float>(acceleration.y()), static_cast<float>(acceleration.z())});
	if (result == mavsdk::Offboard::Result::Success) {
		++m_sent_setpoint_count;
	} else {
		++m_dropped_setpoint_count;
	}
}

std::optional<Eigen::Vector3d> MavsdkVehicle::ned_origin() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	if (!m_origin.ready()) return std::nullopt;
	return m_origin.offset();
}

}
