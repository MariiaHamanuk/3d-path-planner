#include "mapping/log_odds_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace path3d {

namespace {

/// Calls visit(index) for every voxel the segment a -> b passes through, from a's voxel to b's
/// voxel inclusive (Amanatides & Woo).
template <class Visit>
void trace(const LogOddsGridMap& map, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
           Visit&& visit) {
	const double res = map.resolution();
	Eigen::Vector3i current = map.index_of(a);
	const Eigen::Vector3i last = map.index_of(b);
	const Eigen::Vector3d dir = b - a;
	Eigen::Vector3i step = Eigen::Vector3i::Zero();
	Eigen::Vector3d t_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
	Eigen::Vector3d t_delta = t_max;
	for (int axis = 0; axis < 3; ++axis) {
		if (dir[axis] > 0.0) {
			step[axis] = 1;
			t_max[axis] = ((current[axis] + 1) * res - a[axis]) / dir[axis];
			t_delta[axis] = res / dir[axis];
		} else if (dir[axis] < 0.0) {
			step[axis] = -1;
			t_max[axis] = (current[axis] * res - a[axis]) / dir[axis];
			t_delta[axis] = -res / dir[axis];
		}
	}
	visit(current);
	/// Rounding can make the walk miss `last`; the step bound guarantees it ends anyway.
	const int max_steps = (current - last).lpNorm<1>() + 2;
	for (int i = 0; i < max_steps && current != last; ++i) {
		int axis = 0;
		if (t_max[1] < t_max[axis]) axis = 1;
		if (t_max[2] < t_max[axis]) axis = 2;
		current[axis] += step[axis];
		t_max[axis] += t_delta[axis];
		visit(current);
	}
}

}

LogOddsMapper::LogOddsMapper(const LogOddsMapperConfig& config)
    : m_config(config), m_map(config.resolution) {}

bool LogOddsMapper::update_voxel(Key key, float delta) {
	const float old_value = m_map.log_odds(key);
	const float value = std::clamp(old_value + delta, m_config.min_log_odds, m_config.max_log_odds);
	if (value == old_value) return false;
	m_map.set_log_odds(key, value);
	m_dirty.insert(key);
	return true;
}

void LogOddsMapper::integrate(const SensorFrame& frame) {
	const cv::Mat& depth = frame.depth.image;
	if (depth.empty() || depth.type() != CV_32FC1) return;
	const CameraCalibration& cal = frame.depth_calibration;
	if (cal.fx() <= 0.0 || cal.fy() <= 0.0) return;

	const Eigen::Isometry3d world_from_camera =
	    frame.drone_pose.pose.to_isometry() * frame.base_to_depth.pose.to_isometry();
	const Eigen::Vector3d origin = world_from_camera.translation();

	m_free.clear();
	m_hit.clear();
	const int stride = std::max(1, m_config.pixel_stride);
	for (int v = 0; v < depth.rows; v += stride) {
		const float* row = depth.ptr<float>(v);
		for (int u = 0; u < depth.cols; u += stride) {
			double range = row[u];
			bool hit = true;
			if (!std::isfinite(range)) {
				range = std::min(m_config.max_range, m_config.no_return_range);
				hit = false;
			} else if (range > m_config.max_range) {
				range = m_config.max_range;
				hit = false;
			} else if (range < m_config.min_range) {
				continue;
			}
			/// Camera frame: x forward (the depth), y left, z up; pixels: u right, v down.
			const Eigen::Vector3d point_camera(range, -(u - cal.cx()) / cal.fx() * range,
			                                   -(v - cal.cy()) / cal.fy() * range);
			const Eigen::Vector3d end = world_from_camera * point_camera;
			const Eigen::Vector3i end_index = m_map.index_of(end);
			trace(m_map, origin, end, [&](const Eigen::Vector3i& index) {
				if (hit && index == end_index) {
					m_hit.insert(LogOddsGridMap::key_of(index));
				} else {
					m_free.insert(LogOddsGridMap::key_of(index));
				}
			});
		}
	}

	bool changed = false;
	for (const Key key : m_hit) changed |= update_voxel(key, m_config.hit_log_odds);
	for (const Key key : m_free) {
		if (!m_hit.contains(key)) changed |= update_voxel(key, m_config.miss_log_odds);
	}
	if (changed) {
		++m_version;
		m_map.version = m_version;
		m_map.stamp = frame.stamp;
	}
}

std::optional<LogOddsMapper::Update> LogOddsMapper::make_update(bool keyframe) {
	if (!keyframe && m_dirty.empty()) return std::nullopt;
	Update update;
	update.stamp = m_map.stamp;
	update.from_version = m_sent_version;
	update.to_version = m_version;
	update.keyframe = keyframe;
	update.resolution = m_map.resolution();
	const auto add_cell = [&](Key key, float value) {
		const Eigen::Vector3i index = LogOddsGridMap::index_of_key(key);
		update.cells.push_back({index.x(), index.y(), index.z(), value});
	};
	if (keyframe) {
		update.cells.reserve(m_map.voxels().size());
		for (const auto& [key, value] : m_map.voxels()) add_cell(key, value);
	} else {
		update.cells.reserve(m_dirty.size());
		for (const Key key : m_dirty) add_cell(key, m_map.log_odds(key));
	}
	m_dirty.clear();
	m_sent_version = m_version;
	return update;
}

}
