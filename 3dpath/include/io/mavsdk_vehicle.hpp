#pragma once

#include "core/communication.hpp"
#include "core/config.hpp"
#include "core/ned.hpp"
#include "io/vehicle.hpp"

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/offboard/offboard.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace path3d {

/// Generic meaning of a MAVSDK flight mode; modes with no generic meaning are Unknown.
constexpr FlightMode to_flight_mode(mavsdk::Telemetry::FlightMode mode) {
	using Mode = mavsdk::Telemetry::FlightMode;
	switch (mode) {
		case Mode::Offboard:
			return FlightMode::External;
		case Mode::Hold:
			return FlightMode::Hold;
		case Mode::Manual:
		case Mode::Stabilized:
		case Mode::Altctl:
		case Mode::Posctl:
		case Mode::Acro:
		case Mode::Rattitude:
			return FlightMode::Manual;
		case Mode::ReturnToLaunch:
			return FlightMode::ReturnHome;
		case Mode::Land:
			return FlightMode::Land;
		default:
			return FlightMode::Unknown;
	}
}

/**
 * @brief Vehicle backed by an autopilot reached through MAVSDK (ArduPilot SITL or real, PX4).
 *
 * MAVSDK handles the MAVLink link, discovery, commands and their retries. This class keeps the
 * system's view of the vehicle: it publishes VehicleState, turns lifecycle calls into MAVSDK
 * `Action` and `Offboard` calls, and converts setpoints from the world frame (ENU) to the
 * autopilot's local NED frame.
 *
 * The NED origin (the autopilot's EKF origin) is found at startup from pairs of the world pose and
 * the autopilot's local position (see NedOrigin), so the drone must stand still until
 * VehicleState::frames_ready. A background thread waits for the autopilot, so the constructor
 * returns at once. This is the MAVSDK-facing I/O layer, so its header exposes MAVSDK types.
 *
 * Note: while offboard control is active MAVSDK keeps re-sending the newest setpoint by itself, so
 * if the CommanderThread dies the vehicle keeps flying to the last commanded position.
 */
class MavsdkVehicle : public Vehicle {
public:
	/**
	 * @param config Connection and frame settings.
	 * @param world_pose Drone pose in the world frame (used to find the autopilot's frame origin).
	 * @param state_out Topic the vehicle status is published on.
	 * @throws std::runtime_error if the connection cannot be set up (e.g. the port is taken).
	 */
	MavsdkVehicle(MavsdkConfig config, Topic<StampedPose>& world_pose, Topic<VehicleState>& state_out);
	~MavsdkVehicle() override;

	/// External starts offboard control (from the current position); Hold, Land and ReturnHome use
	/// the matching MAVSDK action. Manual and Unknown cannot be commanded and return false.
	bool set_mode(FlightMode mode, std::chrono::milliseconds timeout) override;
	bool arm(std::chrono::milliseconds timeout) override;
	/// Takes off to `altitude_m`; MAVSDK puts the vehicle into its takeoff mode itself. Afterwards
	/// call set_mode(External) to hand control to the CommanderThread.
	bool takeoff(double altitude_m, std::chrono::milliseconds timeout) override;
	bool land(std::chrono::milliseconds timeout) override;
	/// Converts the setpoint to NED and hands it to MAVSDK's offboard control; dropped until the
	/// autopilot is found and the frames are known.
	void send(const Setpoint& setpoint) override;

	/// World position of the autopilot's NED origin; nullopt until it is known.
	std::optional<Eigen::Vector3d> ned_origin() const;
	std::size_t sent_setpoint_count() const { return m_sent_setpoint_count; }
	std::size_t dropped_setpoint_count() const { return m_dropped_setpoint_count; }

private:
	void connect(std::stop_token stop);
	/// Pairs the autopilot's local position with the world pose to find the NED origin.
	void on_position(const mavsdk::Telemetry::PositionVelocityNed& position);
	/// Changes the state under the lock and publishes the result.
	template <class Change>
	void update_state(Change&& change);

	MavsdkConfig m_config;
	std::shared_ptr<LatestSlot<StampedPose>> m_world_pose;
	std::chrono::steady_clock::time_point m_started;
	mavsdk::Mavsdk m_mavsdk;

	mutable std::mutex m_mutex;
	std::shared_ptr<mavsdk::System> m_system;
	std::shared_ptr<mavsdk::Action> m_action;
	std::shared_ptr<mavsdk::Offboard> m_offboard;
	std::shared_ptr<mavsdk::Telemetry> m_telemetry;
	mavsdk::System::IsConnectedHandle m_connected_handle{};
	mavsdk::Telemetry::ArmedHandle m_armed_handle{};
	mavsdk::Telemetry::FlightModeHandle m_mode_handle{};
	mavsdk::Telemetry::PositionVelocityNedHandle m_position_handle{};
	VehicleState m_state;
	/// Newest local position of the autopilot (NED); empty until the first one.
	std::optional<mavsdk::Telemetry::PositionNed> m_ned_position;
	/// The autopilot's NED origin in the world frame.
	NedOrigin m_origin;

	std::atomic<std::size_t> m_sent_setpoint_count{0};
	std::atomic<std::size_t> m_dropped_setpoint_count{0};
	std::jthread m_thread;
};

}
