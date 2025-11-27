#include "trajectory_planning_v3/domain/entities/constraint.hpp"

#include <algorithm>

namespace trajectory_planning::domain::entities {

Constraint::Constraint(const std::vector<double>& pos_limits_min,
                       const std::vector<double>& pos_limits_max,
                       const std::vector<double>& vel_limits,
                       const std::vector<double>& acc_limits)
    : pos_limits_min_(pos_limits_min),
      pos_limits_max_(pos_limits_max),
      vel_limits_(vel_limits),
      acc_limits_(acc_limits) {}

const std::vector<double>& Constraint::position_limits_min() const {
	return pos_limits_min_;
}

const std::vector<double>& Constraint::position_limits_max() const {
	return pos_limits_max_;
}

const std::vector<double>& Constraint::velocity_limits() const {
	return vel_limits_;
}

const std::vector<double>& Constraint::acceleration_limits() const {
	return acc_limits_;
}

bool Constraint::is_within_limits(
    const std::vector<double>& positions, const std::vector<double>& velocities,
    const std::vector<double>& accelerations) const {
	size_t n =
	    std::min({positions.size(), velocities.size(), accelerations.size(),
	              pos_limits_min_.size(), pos_limits_max_.size(),
	              vel_limits_.size(), acc_limits_.size()});

	for (size_t i = 0; i < n; ++i) {
		if (positions[i] < pos_limits_min_[i] ||
		    positions[i] > pos_limits_max_[i]) {
			return false;
		}
		if (std::abs(velocities[i]) > vel_limits_[i]) {
			return false;
		}
		if (std::abs(accelerations[i]) > acc_limits_[i]) {
			return false;
		}
	}
	return true;
}

}  // namespace trajectory_planning::domain::entities
