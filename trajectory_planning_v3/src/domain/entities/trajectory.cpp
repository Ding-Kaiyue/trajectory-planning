#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::domain::entities {

void Trajectory::add_point(const TrajectoryPoint& point) {
	points_.push_back(point);
}

std::optional<TrajectoryPoint> Trajectory::point_at(size_t index) const {
	if (index < points_.size()) {
		return points_[index];
	}
	return std::nullopt;
}

const std::vector<TrajectoryPoint>& Trajectory::points() const {
	return points_;
}

bool Trajectory::empty() const { return points_.empty(); }

size_t Trajectory::size() const { return points_.size(); }

value_objects::Duration Trajectory::total_duration() const {
	if (points_.empty()) return value_objects::Duration(0.0);
	return points_.back().time_from_start;
}

void Trajectory::compute_progress_ratios() {
	if (points_.empty()) return;
	double total = total_duration().seconds();
	if (total <= 0.0) return;

	for (auto& p : points_) {
		p.progress_ratio = p.time_from_start.seconds() / total;
	}
}

void Trajectory::set_state(State s) { state_ = s; }

Trajectory::State Trajectory::state() const { return state_; }

}  // namespace trajectory_planning::domain::entities
