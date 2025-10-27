#include "trajectory_planning_v3/infrastructure/planning/strategies/cartesian_velocity_planning_strategy.hpp"
#include <rclcpp/rclcpp.hpp>

namespace trajectory_planning::infrastructure::planning {

std::optional<domain::value_objects::JointVelocity> CartesianVelocityPlanningStrategy::plan(
    const geometry_msgs::msg::TwistStamped& twist_cmd) {
    auto qdot_opt = servo_adapter_.computeJointVelocities(twist_cmd);
    if (!qdot_opt.has_value()) {
        RCLCPP_ERROR(rclcpp::get_logger("CartesianVelocityPlanningStrategy"),
                     "Failed to compute joint velocities from twist command!");
        return std::nullopt;
    }
    return domain::value_objects::JointVelocity(qdot_opt.value());
}

}  // namespace trajectory_planning::infrastructure::planning
