#pragma once

#include "core/types.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <numbers>

/// Conversions between the world frame of the system (ENU: x east, y north, z up, as in Gazebo)
/// and the local NED frame of an autopilot (x north, y east, z down), plus the origin between them.
/// Pure math, no autopilot protocol.
namespace path3d {

/// ENU vector -> NED vector (and back: the swap is its own inverse).
inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d& v) { return {v.y(), v.x(), -v.z()}; }
inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d& v) { return {v.y(), v.x(), -v.z()}; }

/// ENU yaw (counter-clockwise from east) -> NED yaw (clockwise from north), and back.
inline double enu_to_ned_yaw(double yaw) { return wrap_angle(std::numbers::pi / 2.0 - yaw); }
inline double ned_to_enu_yaw(double yaw) { return wrap_angle(std::numbers::pi / 2.0 - yaw); }

/**
 * @brief World position of an autopilot's NED origin (its EKF origin), estimated from pairs of
 * positions of the same body in both frames.
 *
 * The axes are known (ENU vs NED), only the translation is not: it depends on where the vehicle
 * booted. Feed it pairs while the vehicle stands still (e.g. on the ground at startup); after
 * `samples` pairs the average is fixed and ready() becomes true. Not thread-safe.
 */
class NedOrigin {
public:
	/// @param samples Pairs to average before the origin is considered known (at least 1).
	explicit NedOrigin(std::size_t samples = 50) : m_samples(samples > 0 ? samples : 1) {}

	/// Adds one pair: the same position as world ENU and as autopilot NED. Ignored once ready().
	void add(const Eigen::Vector3d& world, const Eigen::Vector3d& ned) {
		if (ready()) return;
		m_sum += world - ned_to_enu(ned);
		++m_count;
	}

	bool ready() const { return m_count >= m_samples; }

	Eigen::Vector3d offset() const {
		return m_count > 0 ? Eigen::Vector3d(m_sum / static_cast<double>(m_count)) : Eigen::Vector3d::Zero();
	}

	Eigen::Vector3d to_ned(const Eigen::Vector3d& world) const { return enu_to_ned(world - offset()); }
	Eigen::Vector3d to_world(const Eigen::Vector3d& ned) const { return ned_to_enu(ned) + offset(); }

private:
	std::size_t m_samples;
	std::size_t m_count = 0;
	/// Sum of the origin estimates of all pairs added so far.
	Eigen::Vector3d m_sum = Eigen::Vector3d::Zero();
};

}
