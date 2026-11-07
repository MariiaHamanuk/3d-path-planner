#pragma once

#include "mapping/map_view.hpp"
#include "mapping/mapper.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace path3d::test {

/**
 * @brief A map with no queries, just a list of integers.
 *
 * Its updates are incremental: a diff carries the new cells, a keyframe carries all of them.
 */
class FakeView : public MapView {
public:
	struct Update {
		TimeSec stamp = 0.0;
		std::uint64_t from_version = 0;
		std::uint64_t to_version = 0;
		bool keyframe = false;
		std::vector<int> cells;
	};

	FakeView() = default;

	void apply(const Update& update) {
		if (update.keyframe) m_cells.clear();
		m_cells.insert(m_cells.end(), update.cells.begin(), update.cells.end());
		stamp = update.stamp;
		version = update.to_version;
	}
	const std::vector<int>& cells() const { return m_cells; }

private:
	std::vector<int> m_cells;
};

inline FakeView::Update keyframe_update(std::uint64_t to_version, std::vector<int> cells = {},
                                        TimeSec stamp = 0.0) {
	return {.stamp = stamp, .from_version = 0, .to_version = to_version, .keyframe = true, .cells = std::move(cells)};
}

inline FakeView::Update diff_update(std::uint64_t from_version, std::uint64_t to_version,
                                    std::vector<int> cells, TimeSec stamp = 0.0) {
	return {.stamp = stamp, .from_version = from_version, .to_version = to_version, .keyframe = false, .cells = std::move(cells)};
}

/**
 * @brief Mapper for FakeView: the n-th integrated frame adds the cell n, so the version equals the
 *   number of cells.
 *
 * Keeps track of what it handed out, as the Mapper contract asks.
 */
class FakeMapper : public Mapper<FakeView> {
public:
	void integrate(const SensorFrame& frame) override {
		m_cells.push_back(static_cast<int>(m_cells.size()) + 1);
		m_stamp = frame.stamp;
	}
	std::uint64_t version() const override { return m_cells.size(); }

	std::optional<FakeView::Update> make_update(bool keyframe) override {
		if (!keyframe && version() == m_sent) return std::nullopt;
		FakeView::Update update;
		update.stamp = m_stamp;
		update.from_version = m_sent;
		update.to_version = version();
		update.keyframe = keyframe;
		update.cells.assign(keyframe ? m_cells.begin() : m_cells.begin() + static_cast<long>(m_sent),
		                    m_cells.end());
		m_sent = version();
		return update;
	}

	const std::vector<int>& cells() const { return m_cells; }

private:
	std::vector<int> m_cells;
	TimeSec m_stamp = 0.0;
	/// version already handed out
	std::uint64_t m_sent = 0;
};

}
