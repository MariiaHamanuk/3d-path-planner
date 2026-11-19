#pragma once

#include "core/types.hpp"
#include "mapping/map_view.hpp"
#include "pipeline/clients.hpp"
#include "planning/algorithms/map_adapters.hpp"
#include "planning/local_planner.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace path3d {

struct SmoothLocalPlannerConfig {
	double max_speed = 5.0;
	double max_accel = 2.5;
	double max_lateral_accel = 2.0;
	double max_yaw_rate = 1.0;
	double horizon = 3.0;
	double step = 0.1;
	double path_step = 0.25;
	double corner_radius = 2.0;
	double check_distance = 30.0;
	double stop_buffer = 0.5;
	double replan_lead = 0.5;
	double max_deviation = 1.5;
	double escape_distance = 2.0;
	SearchSpaceConfig space;
};

template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class SmoothLocalPlanner : public LocalPlanner<M, GlobalPath, ReplanRequest> {
public:
	SmoothLocalPlanner(MapClient<M>& map, UavClient& uav, SmoothLocalPlannerConfig config = {})
	    : LocalPlanner<M, GlobalPath, ReplanRequest>(map, uav), m_config(config) {}

	LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
		const StampedPose pose = *this->m_uav.pose();
		const OccupancySpace<M> space(this->m_map.map(), everywhere(), m_config.space);

		MotionState state = current_state(pose);

		if (new_route && new_route->status == PlanStatus::Ok && !new_route->waypoints.empty()) {
			const Eigen::Vector3d from = m_points.empty() ? pose.pose.position : position_at(state.s);
			set_path(space, from, new_route->waypoints);
			state.s = 0.0;
		}
		if (m_points.size() < 2) return {make_request(pose, pose.pose.position, ReplanReason::NoPath), std::nullopt};

		std::optional<ReplanReason> reason;
		if ((position_at(state.s) - pose.pose.position).norm() > m_config.max_deviation) {
			state.s = closest_s(pose.pose.position);
			if ((position_at(state.s) - pose.pose.position).norm() > m_config.max_deviation) reason = ReplanReason::Deviated;
		}

		double stop_s = m_s.back();
		bool escaping = true;
		for (std::size_t i = index_at(state.s); i < m_points.size() && m_s[i] <= state.s + m_config.check_distance; ++i) {
			if (space.is_free(m_points[i])) {
				escaping = false;
				continue;
			}
			if (escaping && m_s[i] <= state.s + m_config.escape_distance) continue;
			stop_s = std::max(state.s, m_s[i] - m_config.stop_buffer);
			reason = ReplanReason::Blocked;
			break;
		}

		const std::vector<double> limit = speed_limits(stop_s);
		const int count = static_cast<int>(std::ceil(m_config.horizon / m_config.step));
		m_plan.clear();
		for (int k = 0; k <= count; ++k) {
			state.t = pose.stamp + k * m_config.step;
			m_plan.push_back(state);
			const double reachable = std::min(state.v + m_config.max_accel * m_config.step, m_config.max_speed);
			const double v_next = std::max(0.0, std::min(reachable, limit_at(limit, state.s + state.v * m_config.step)));
			state.s = std::min(state.s + 0.5 * (state.v + v_next) * m_config.step, stop_s);
			state.v = state.s >= stop_s ? 0.0 : v_next;
			state.yaw = turn_towards(state.yaw, heading_at(state.s, state.yaw), m_config.max_yaw_rate * m_config.step);
		}

		Trajectory trajectory;
		trajectory.stamp = pose.stamp;
		for (const MotionState& sample : m_plan) {
			trajectory.points.push_back(Setpoint{
			    .stamp = sample.t,
			    .position = position_at(sample.s),
			    .velocity = tangent_at(sample.s) * sample.v,
			    .yaw = sample.yaw,
			});
		}
		for (std::size_t k = 0; k + 1 < trajectory.points.size(); ++k) {
			trajectory.points[k].acceleration = (trajectory.points[k + 1].velocity - trajectory.points[k].velocity) / m_config.step;
		}

		if (!reason) return {std::nullopt, std::move(trajectory)};
		const std::size_t lead = std::min(m_plan.size() - 1, static_cast<std::size_t>(std::lround(m_config.replan_lead / m_config.step)));
		return {make_request(pose, position_at(m_plan[lead].s), *reason), std::move(trajectory)};
	}

private:
	struct MotionState {
		TimeSec t = 0.0;
		double s = 0.0;
		double v = 0.0;
		double yaw = 0.0;
	};

	static Eigen::AlignedBox3d everywhere() {
		return {Eigen::Vector3d::Constant(-1e6), Eigen::Vector3d::Constant(1e6)};
	}

	static ReplanRequest make_request(const StampedPose& pose, const Eigen::Vector3d& position, ReplanReason reason) {
		ReplanRequest request;
		request.stamp = pose.stamp;
		request.reason = reason;
		request.position = position;
		return request;
	}

	static double turn_towards(double yaw, double target, double max_step) {
		return wrap_angle(yaw + std::clamp(wrap_angle(target - yaw), -max_step, max_step));
	}

	MotionState current_state(const StampedPose& pose) const {
		if (m_plan.empty()) return {pose.stamp, 0.0, 0.0, yaw_of(pose.pose.orientation)};
		if (pose.stamp <= m_plan.front().t) return m_plan.front();
		const auto after = std::upper_bound(m_plan.begin(), m_plan.end(), pose.stamp,
		                                    [](TimeSec t, const MotionState& sample) { return t < sample.t; });
		if (after == m_plan.end()) return m_plan.back();
		const MotionState& a = *std::prev(after);
		const MotionState& b = *after;
		const double f = (pose.stamp - a.t) / (b.t - a.t);
		return {pose.stamp, a.s + f * (b.s - a.s), a.v + f * (b.v - a.v), wrap_angle(a.yaw + f * wrap_angle(b.yaw - a.yaw))};
	}

	void set_path(const OccupancySpace<M>& space, const Eigen::Vector3d& from, const std::vector<Eigen::Vector3d>& waypoints) {
		std::vector<Eigen::Vector3d> raw{from};
		for (const Eigen::Vector3d& w : waypoints) {
			if ((w - raw.back()).norm() > 1e-3) raw.push_back(w);
		}
		resample(rounded(space, shortcut(space, raw)));
		m_plan.clear();
	}

	static std::vector<Eigen::Vector3d> shortcut(const OccupancySpace<M>& space, const std::vector<Eigen::Vector3d>& points) {
		std::vector<Eigen::Vector3d> result{points.front()};
		std::size_t i = 0;
		while (i + 1 < points.size()) {
			std::size_t j = points.size() - 1;
			while (j > i + 1 && !space.segment_free(points[i], points[j])) --j;
			result.push_back(points[j]);
			i = j;
		}
		return result;
	}

	std::vector<Eigen::Vector3d> rounded(const OccupancySpace<M>& space, const std::vector<Eigen::Vector3d>& corners) const {
		std::vector<Eigen::Vector3d> curve{corners.front()};
		for (std::size_t i = 1; i + 1 < corners.size(); ++i) {
			const Eigen::Vector3d in = corners[i] - corners[i - 1];
			const Eigen::Vector3d out = corners[i + 1] - corners[i];
			const double cut = std::min({m_config.corner_radius, 0.5 * in.norm(), 0.5 * out.norm()});
			const Eigen::Vector3d a = corners[i] - in.normalized() * cut;
			const Eigen::Vector3d b = corners[i] + out.normalized() * cut;
			const int n = std::max(2, static_cast<int>(std::ceil(2.0 * cut / m_config.path_step)));
			std::vector<Eigen::Vector3d> arc;
			for (int k = 0; k <= n; ++k) {
				const double t = static_cast<double>(k) / n;
				arc.push_back((1.0 - t) * (1.0 - t) * a + 2.0 * (1.0 - t) * t * corners[i] + t * t * b);
			}
			bool free = space.segment_free(curve.back(), arc.front());
			for (std::size_t k = 0; free && k + 1 < arc.size(); ++k) free = space.segment_free(arc[k], arc[k + 1]);
			if (free) {
				curve.insert(curve.end(), arc.begin(), arc.end());
			} else {
				curve.push_back(corners[i]);
			}
		}
		curve.push_back(corners.back());
		return curve;
	}

	void resample(const std::vector<Eigen::Vector3d>& polyline) {
		m_points.clear();
		m_s.clear();
		double total = 0.0;
		double next = 0.0;
		for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
			const Eigen::Vector3d d = polyline[i + 1] - polyline[i];
			const double length = d.norm();
			if (length < 1e-9) continue;
			while (next <= total + length) {
				m_points.push_back(polyline[i] + d * ((next - total) / length));
				m_s.push_back(next);
				next += m_config.path_step;
			}
			total += length;
		}
		if (m_s.empty() || total - m_s.back() > 1e-6) {
			m_points.push_back(polyline.back());
			m_s.push_back(total);
		}

		m_curve_limit.assign(m_points.size(), m_config.max_speed);
		for (std::size_t i = 1; i + 1 < m_points.size(); ++i) {
			const Eigen::Vector3d in = m_points[i] - m_points[i - 1];
			const Eigen::Vector3d out = m_points[i + 1] - m_points[i];
			const double cosine = std::clamp(in.normalized().dot(out.normalized()), -1.0, 1.0);
			const double curvature = std::acos(cosine) / (0.5 * (in.norm() + out.norm()));
			if (curvature > 1e-6) m_curve_limit[i] = std::min(m_config.max_speed, std::sqrt(m_config.max_lateral_accel / curvature));
		}
	}

	std::vector<double> speed_limits(double stop_s) const {
		std::vector<double> limit(m_points.size());
		double next = 0.0;
		for (std::size_t i = m_points.size(); i-- > 0;) {
			const double ds = i + 1 < m_points.size() ? m_s[i + 1] - m_s[i] : 0.0;
			const double braking = std::sqrt(next * next + 2.0 * m_config.max_accel * ds);
			limit[i] = m_s[i] >= stop_s ? 0.0 : std::min(m_curve_limit[i], braking);
			next = limit[i];
		}
		return limit;
	}

	std::size_t index_at(double s) const {
		const auto after = std::upper_bound(m_s.begin(), m_s.end(), s);
		const std::size_t i = after == m_s.begin() ? 0 : static_cast<std::size_t>(after - m_s.begin()) - 1;
		return std::min(i, m_s.size() - 2);
	}

	double fraction_at(std::size_t i, double s) const {
		return std::clamp((s - m_s[i]) / (m_s[i + 1] - m_s[i]), 0.0, 1.0);
	}

	Eigen::Vector3d position_at(double s) const {
		const std::size_t i = index_at(s);
		return m_points[i] + fraction_at(i, s) * (m_points[i + 1] - m_points[i]);
	}

	Eigen::Vector3d tangent_at(double s) const {
		const std::size_t i = index_at(s);
		return (m_points[i + 1] - m_points[i]).normalized();
	}

	double heading_at(double s, double fallback) const {
		const Eigen::Vector3d t = tangent_at(s);
		return t.head<2>().norm() < 0.3 ? fallback : std::atan2(t.y(), t.x());
	}

	double limit_at(const std::vector<double>& limit, double s) const {
		const std::size_t i = index_at(s);
		const double f = fraction_at(i, s);
		return limit[i] + f * (limit[i + 1] - limit[i]);
	}

	double closest_s(const Eigen::Vector3d& p) const {
		double best_s = 0.0;
		double best = std::numeric_limits<double>::infinity();
		for (std::size_t i = 0; i < m_points.size(); ++i) {
			const double d = (m_points[i] - p).squaredNorm();
			if (d < best) {
				best = d;
				best_s = m_s[i];
			}
		}
		return best_s;
	}

	SmoothLocalPlannerConfig m_config;
	std::vector<Eigen::Vector3d> m_points;
	std::vector<double> m_s;
	std::vector<double> m_curve_limit;
	std::vector<MotionState> m_plan;
};

}
