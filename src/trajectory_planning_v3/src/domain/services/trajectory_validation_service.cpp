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
        if (!constraint.is_within_limits(pt.position.values(),
                                         pt.velocity.values(),
                                         pt.acceleration.values())) {
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

}  // namespace trajectory_planning_v3::domain::services
