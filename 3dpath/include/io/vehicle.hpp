#pragma once

#include "core/communication.hpp"
#include "core/types.hpp"

#include <chrono>

namespace path3d {

/**
 * @brief The drone's autopilot, as the rest of the system sees it.
 *
 * Implementations (e.g. MavsdkVehicle) talk to a real or simulated autopilot and publish its
 * status as VehicleState into the topic given at construction. Only project types cross this
 * interface: setpoints are in the world frame (ENU) and modes are generic FlightModes, so planners
 * and the commander never see an autopilot protocol or frame.
 *
 * Lifecycle commands (set_mode, arm, takeoff, land) block until the autopilot acknowledges them or
 * the timeout expires; call them from one thread at a time. send() never blocks.
 */
class Vehicle {
public:
	virtual ~Vehicle() = default;
	Vehicle(const Vehicle&) = delete;
	Vehicle& operator=(const Vehicle&) = delete;

	/// True once the autopilot accepted the mode within `timeout`.
	virtual bool set_mode(FlightMode mode, std::chrono::milliseconds timeout) = 0;
	/// Arms the motors; true once accepted (false while pre-arm checks fail).
	virtual bool arm(std::chrono::milliseconds timeout) = 0;
	/// Takes off to `altitude_m` above the current position; true once accepted (needs armed motors;
	/// the implementation picks the takeoff mode itself). Afterwards call set_mode(External) to
	/// hand control to the CommanderStage.
	virtual bool takeoff(double altitude_m, std::chrono::milliseconds timeout) = 0;
	/// Lands where the vehicle is; true once accepted.
	virtual bool land(std::chrono::milliseconds timeout) = 0;
	/// Commands one setpoint (world frame). Non-blocking and thread-safe; meant to be called at a
	/// steady rate by the CommanderStage. Ignored while the vehicle is not ready for setpoints.
	virtual void send(const Setpoint& setpoint) = 0;

protected:
	/**
	 * @param state_out Topic the implementation publishes the vehicle status on.
	 */
	explicit Vehicle(Topic<VehicleState>& state_out) : m_state_out(state_out) {}

	/// Where the implementation publishes the vehicle's status.
	Topic<VehicleState>& m_state_out;
};

}
