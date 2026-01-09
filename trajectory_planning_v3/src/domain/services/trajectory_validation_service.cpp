#include "trajectory_planning_v3/domain/services/trajectory_validation_service.hpp"

#include <sstream>

namespace trajectory_planning::domain::services {

using namespace trajectory_planning::domain::entities;

bool TrajectoryValidationService::validate(const Trajectory& trajectory,
                                           const Constraint& constraint) const {
	errors_.clear();

	const auto& points = trajectory.points();
	for (size_t i = 0; i < points.size(); ++i) {
		const auto& pt = points[i];
		// Convert Eigen::VectorXd to std::vector<double>
		std::vector<double> pos_vec(pt.position.values().data(), pt.position.values().data() + pt.position.size());
		std::vector<double> vel_vec(pt.velocity.values().data(), pt.velocity.values().data() + pt.velocity.size());
		std::vector<double> acc_vec(pt.acceleration.values().data(), pt.acceleration.values().data() + pt.acceleration.size());

		if (!constraint.is_within_limits(pos_vec, vel_vec, acc_vec)) {
			std::ostringstream oss;
			oss << "Trajectory point " << i << " violates constraints.";
			errors_.push_back(oss.str());
		}
	}

	return errors_.empty();
}

const std::vector<std::string>& TrajectoryValidationService::errors() const {
	return errors_;
}

}  // namespace trajectory_planning::domain::services
