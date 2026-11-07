#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

/// Project-wide data types. No ROS / Gazebo types leak past this header.
namespace path3d {

using TimeSec = double;  ///< seconds, simulation clock

/**
 * @brief Which camera an image or calibration belongs to.
 */
enum class CameraId { Color,
                      Depth };

/**
 * @brief Coordinate frames of the system. Readers convert their native frame names (ROS/TF strings,
 * URDF links, ...) to this enum, so no string is stored or copied per message.
 */
enum class FrameId : std::uint8_t {
	World,        ///< fixed world frame
	Base,         ///< drone base link
	ColorCamera,  ///< color camera optical frame
	DepthCamera,  ///< depth camera optical frame
};

/// Short readable name of a frame.
inline const char* to_string(FrameId frame) {
	switch (frame) {
		case FrameId::World:
			return "world";
		case FrameId::Base:
			return "base";
		case FrameId::ColorCamera:
			return "color_camera";
		case FrameId::DepthCamera:
			return "depth_camera";
	}
	return "unknown";
}

/// The frame a camera's images are expressed in.
constexpr FrameId camera_frame(CameraId camera) {
	return camera == CameraId::Color ? FrameId::ColorCamera : FrameId::DepthCamera;
}

/**
 * @brief Position and orientation of a body in some frame.
 */
struct Pose {
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

	/// The pose as a rigid transform.
	Eigen::Isometry3d to_isometry() const {
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
		T.linear() = orientation.normalized().toRotationMatrix();
		T.translation() = position;
		return T;
	}
};

/**
 * @brief Pose of `child_frame` expressed in `parent_frame` (i.e. T_parent_child).
 */
struct StampedPose {
	TimeSec stamp = 0.0;
	FrameId parent_frame = FrameId::World;
	FrameId child_frame = FrameId::Base;
	Pose pose;
};

/**
 * @brief Color: CV_8UC3, BGR. Depth: CV_32FC1, meters along the optical axis; non-finite (inf/NaN)
 * = no return. Read-only once published: copies of a cv::Mat share their pixels, and `const`
 * on the message does not stop writes through a copy. A consumer that needs to modify an
 * image must clone() it.
 */
struct ImageFrame {
	TimeSec stamp = 0.0;
	FrameId frame_id = FrameId::ColorCamera;
	cv::Mat image;
};

/**
 * @brief Pinhole camera model (sensor_msgs/CameraInfo layout). Camera frame follows the Gazebo
 * convention: x forward, y left, z up; K is expressed for the image (u right, v down).
 */
struct CameraCalibration {
	TimeSec stamp = 0.0;
	FrameId frame_id = FrameId::ColorCamera;
	int width = 0;
	int height = 0;
	cv::Matx33d K = cv::Matx33d::eye();    ///< intrinsics
	cv::Matx33d R = cv::Matx33d::eye();    ///< rectification
	cv::Matx34d P = cv::Matx34d::zeros();  ///< projection
	cv::Mat_<double> distortion;           ///< 1xN, empty if none
	std::string distortion_model;

	/// Focal length along x, in pixels.
	double fx() const { return K(0, 0); }
	/// Focal length along y, in pixels.
	double fy() const { return K(1, 1); }
	/// Principal point x, in pixels.
	double cx() const { return K(0, 2); }
	/// Principal point y, in pixels.
	double cy() const { return K(1, 2); }
};

/**
 * @brief Whether the global planner found a path for a request.
 */
enum class PlanStatus { Ok,
                        Failed };

/**
 * @brief Global -> local: the answer to the replan request `request_id`. Every request gets exactly
 * one answer; on `Failed` the waypoints are empty. Waypoints are in the world frame, first
 * point = start, last = goal.
 */
struct GlobalPath {
	TimeSec stamp = 0.0;  ///< time of the map/state the path was planned from
	std::uint64_t request_id = 0;
	PlanStatus status = PlanStatus::Ok;
	FrameId frame = FrameId::World;
	std::vector<Eigen::Vector3d> waypoints;
};

/**
 * @brief Why the local planner asks for a new global route.
 */
enum class ReplanReason { NoPath,
                          Blocked,
                          Deviated };

/**
 * @brief Local -> global: "plan again from `position`". `id` increases with every request and is
 * echoed in the answer; the LocalPlanningStage assigns it.
 */
struct ReplanRequest {
	TimeSec stamp = 0.0;
	std::uint64_t id = 0;
	ReplanReason reason = ReplanReason::NoPath;
	Eigen::Vector3d position = Eigen::Vector3d::Zero();  ///< world frame
};

/// Pose between `a` and `b` at time `t` (position: linear, orientation: slerp). `t` outside
/// [a.stamp, b.stamp] extrapolates; equal stamps return `a`. Frames are taken from `a`.
inline StampedPose interpolate(const StampedPose& a, const StampedPose& b, TimeSec t) {
	StampedPose out = a;
	out.stamp = t;
	const TimeSec span = b.stamp - a.stamp;
	if (span <= 0.0) return out;
	const double s = (t - a.stamp) / span;
	out.pose.position = a.pose.position + s * (b.pose.position - a.pose.position);
	out.pose.orientation = a.pose.orientation.slerp(s, b.pose.orientation).normalized();
	return out;
}

/**
 * @brief One depth observation with everything needed to project it into the world, assembled by
 * the FrameSynchronizer (sync.hpp). `stamp` is the depth image's stamp and `drone_pose` is
 * the pose interpolated at that stamp, so the image is projected from where it was taken.
 * The camera mount is static, so its newest transform is used.
 */
struct SensorFrame {
	TimeSec stamp = 0.0;
	StampedPose drone_pose;  ///< base link in the world frame
	ImageFrame depth;
	CameraCalibration depth_calibration;
	StampedPose base_to_depth;  ///< T_base_camera
};

/// Angle in (-pi, pi].
inline double wrap_angle(double angle) {
	angle = std::remainder(angle, 2.0 * std::numbers::pi);
	return angle <= -std::numbers::pi ? angle + 2.0 * std::numbers::pi : angle;
}

/// Heading of a rotation: the angle of its x axis in the x-y plane, counter-clockwise from x.
inline double yaw_of(const Eigen::Quaterniond& orientation) {
	const Eigen::Matrix3d r = orientation.normalized().toRotationMatrix();
	return std::atan2(r(1, 0), r(0, 0));
}

/**
 * @brief One commanded state of the drone, in the world frame (ENU, the frame of the map and
 *   planners).
 *
 * Velocity and acceleration are feed-forward terms for smooth tracking; set them to zero to just
 * fly to (or hold) `position`.
 */
struct Setpoint {
	TimeSec stamp = 0.0;                                     ///< sim time the state is meant for
	Eigen::Vector3d position = Eigen::Vector3d::Zero();      ///< m
	Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      ///< m/s
	Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  ///< m/s^2
	double yaw = 0.0;                                        ///< heading of the body x axis, rad, counter-clockwise from world x (ENU)
};

/**
 * @brief Local planner -> commander: the drone's planned states over a short horizon (~1-3 s).
 *
 * The commander samples it at its own rate, so planning may be slower or irregular.
 */
struct Trajectory {
	TimeSec stamp = 0.0;           ///< sim time it was planned at
	std::vector<Setpoint> points;  ///< increasing stamps

	/// State at time `t`, interpolated linearly between the surrounding points (yaw along the
	/// shorter way round); nullopt before the first point, after the last one, or if empty.
	std::optional<Setpoint> sample(TimeSec t) const {
		if (points.empty() || t < points.front().stamp || t > points.back().stamp) return std::nullopt;
		const auto after = std::upper_bound(points.begin(), points.end(), t,
		                                    [](TimeSec value, const Setpoint& p) { return value < p.stamp; });
		/// t is exactly the last stamp
		if (after == points.end()) return points.back();
		const Setpoint& b = *after;
		const Setpoint& a = *std::prev(after);
		const double span = b.stamp - a.stamp;
		const double s = span > 0.0 ? (t - a.stamp) / span : 0.0;
		Setpoint out;
		out.stamp = t;
		out.position = a.position + s * (b.position - a.position);
		out.velocity = a.velocity + s * (b.velocity - a.velocity);
		out.acceleration = a.acceleration + s * (b.acceleration - a.acceleration);
		out.yaw = wrap_angle(a.yaw + s * wrap_angle(b.yaw - a.yaw));
		return out;
	}
};

/**
 * @brief What a flight mode means, independent of the autopilot.
 *
 * Each Vehicle implementation maps its own modes to these (e.g. ArduCopter GUIDED -> External, see
 * mavsdk_vehicle.hpp).
 */
enum class FlightMode : std::uint8_t {
	Manual,      ///< a pilot flies it (e.g. STABILIZE, ALT_HOLD)
	Hold,        ///< the autopilot holds its position by itself (e.g. LOITER, POSHOLD, BRAKE)
	External,    ///< it follows our setpoints (ArduPilot GUIDED, PX4 OFFBOARD)
	ReturnHome,  ///< it flies home (RTL)
	Land,        ///< it lands where it is
	Unknown,     ///< any other mode
};

/// Short readable name of a flight mode.
inline const char* to_string(FlightMode mode) {
	switch (mode) {
		case FlightMode::Manual:
			return "manual";
		case FlightMode::Hold:
			return "hold";
		case FlightMode::External:
			return "external";
		case FlightMode::ReturnHome:
			return "return_home";
		case FlightMode::Land:
			return "land";
		case FlightMode::Unknown:
			return "unknown";
	}
	return "unknown";
}

/**
 * @brief Vehicle -> system: link and autopilot status, published by a Vehicle.
 */
struct VehicleState {
	TimeSec stamp = 0.0;        ///< wall time since the Vehicle was created, s
	bool connected = false;     ///< the autopilot is found and its heartbeat arrives
	bool armed = false;         ///< motors armed
	bool frames_ready = false;  ///< world <-> autopilot frame known, so setpoints can be sent
	FlightMode mode = FlightMode::Unknown;
};

}
