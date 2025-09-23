#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

namespace trajectory_planning::application::services {

MotionPlanningService::MotionPlanningService(
    std::shared_ptr<MoveJPlanningStrategy> movej_strategy,
    std::shared_ptr<MoveLPlanningStrategy> movel_strategy,
    std::shared_ptr<MoveCPlanningStrategy> movec_strategy,
    std::shared_ptr<JointConstrainedPlanningStrategy> joint_constrained_strategy,
    std::shared_ptr<MoveItAdapter> moveit_adapter,
    rclcpp::Logger logger)
    : movej_strategy_(movej_strategy)
    , movel_strategy_(movel_strategy)
    , movec_strategy_(movec_strategy)
    , joint_constrained_strategy_(joint_constrained_strategy)
    , moveit_adapter_(moveit_adapter)
    , logger_(logger) {
}

PlanningResult MotionPlanningService::planJointMotion(const sensor_msgs::msg::JointState& goal) {
    if (!movej_strategy_) {
        return createFailureResult("MoveJ", "Strategy not initialized");
    }

    try {
        JointPosition goal_position(goal.position);
        Trajectory trajectory = movej_strategy_->plan(goal_position);
        if (trajectory.empty()) {
            return createFailureResult("MoveJ", "Planning failed - empty trajectory");
        }

        return createSuccessResult(trajectory, "MoveJ");
    } catch (const std::exception& e) {
        return createFailureResult("MoveJ", std::string("Exception: ") + e.what());
    }
}

PlanningResult MotionPlanningService::planLinearMotion(const geometry_msgs::msg::Pose& goal) {
    if (!movel_strategy_) {
        return createFailureResult("MoveL", "Strategy not initialized");
    }

    try {
        Trajectory trajectory = movel_strategy_->plan(goal, MoveLPlanningStrategy::PlanningType::INTELLIGENT);
        if (trajectory.empty()) {
            return createFailureResult("MoveL", "Planning failed - empty trajectory");
        }

        return createSuccessResult(trajectory, "MoveL");
    } catch (const std::exception& e) {
        return createFailureResult("MoveL", std::string("Exception: ") + e.what());
    }
}

PlanningResult MotionPlanningService::planArcMotion(const robot_interfaces::msg::MoveCRequest& request) {
    if (!movec_strategy_) {
        return createFailureResult("MoveC", "Strategy not initialized");
    }

    try {
        Trajectory trajectory;
        switch (request.route_type) {
            case 0: // ARC
                trajectory = planArcTrajectory(request);
                break;
            case 1: // BEZIER
                trajectory = planBezierTrajectory(request);     // TODO: Test
                break;
            case 2: // CIRCLE
                trajectory = planCircleTrajectory(request);     // TODO: Test
                break;
            case 3: // CIRCLE3PT
                trajectory = planCircle3PtTrajectory(request);  // TODO: Test
                break;
            default:
                return createFailureResult("MoveC", "Unsupported route_type: " + std::to_string(request.route_type));
        }

        if (trajectory.empty()) {
            return createFailureResult("MoveC", "Planning failed - empty trajectory");
        }

        return createSuccessResult(trajectory, "MoveC");
    } catch (const std::exception& e) {
        return createFailureResult("MoveC", std::string("Exception: ") + e.what());
    }
}

PlanningResult MotionPlanningService::planConstrainedMotion(const robot_interfaces::msg::JointConstrainedRequest& request) {
    if (!joint_constrained_strategy_) {
        return createFailureResult("JointConstrained", "Strategy not initialized");
    }

    try {
        // 清除之前的约束
        joint_constrained_strategy_->clearConstraints();

        // 添加新约束
        for (const auto& jc : request.joint_constraints) {
            if (jc.type == robot_interfaces::msg::JointConstraint::FIXED) {
                auto constraint = std::make_shared<JointConstrainedPlanningStrategy::FixedJointConstraint>(
                    jc.joint_index, jc.fixed_value, jc.weight, jc.is_hard);
                joint_constrained_strategy_->addConstraint(constraint, true);
            } else if (jc.type == robot_interfaces::msg::JointConstraint::RANGE) {
                auto constraint = std::make_shared<JointConstrainedPlanningStrategy::RangeJointConstraint>(
                    jc.joint_index, jc.min_value, jc.max_value, jc.weight, jc.is_hard);
                joint_constrained_strategy_->addConstraint(constraint, true);
            }
        }

        Trajectory trajectory = joint_constrained_strategy_->plan(
            request.goal_pose,
            JointConstrainedPlanningStrategy::PlanningType::INTELLIGENT,
            request.max_attempts);

        if (trajectory.empty()) {
            return createFailureResult("JointConstrained", "Planning failed - empty trajectory");
        }

        return createSuccessResult(trajectory, "JointConstrained");
    } catch (const std::exception& e) {
        return createFailureResult("JointConstrained", std::string("Exception: ") + e.what());
    }
}

Trajectory MotionPlanningService::planArcTrajectory(const robot_interfaces::msg::MoveCRequest& request) {
    if (request.waypoints.size() != 2) {
        RCLCPP_ERROR(logger_, "ARC route requires exactly 2 waypoints: [goal, via_point]");
        return Trajectory{};
    }

    if (!moveit_adapter_) {
        RCLCPP_ERROR(logger_, "MoveIt adapter not initialized");
        return Trajectory{};
    }

    // 获取当前位姿
    geometry_msgs::msg::Pose current_pose = moveit_adapter_->getCurrentPoseFromTF();
    if (current_pose.position.x == 0 && current_pose.position.y == 0 && current_pose.position.z == 0 &&
        current_pose.orientation.x == 0 && current_pose.orientation.y == 0 &&
        current_pose.orientation.z == 0 && current_pose.orientation.w == 0) {
        RCLCPP_ERROR(logger_, "Failed to get current pose from TF");
        return Trajectory{};
    }

    return movec_strategy_->planArc(current_pose, request.waypoints[0], request.waypoints[1]);
}
Trajectory MotionPlanningService::planBezierTrajectory(const robot_interfaces::msg::MoveCRequest& request) {
    if (request.waypoints.size() != 4) {
        RCLCPP_ERROR(logger_, "BEZIER route requires exactly 4 waypoints: [start, control1, control2, end]");
        return Trajectory{};
    }

    return movec_strategy_->planBezier(request.waypoints[0], request.waypoints[1],
                                       request.waypoints[2], request.waypoints[3]);
}

Trajectory MotionPlanningService::planCircleTrajectory(const robot_interfaces::msg::MoveCRequest& request) {
    if (request.waypoints.size() != 2) {
        RCLCPP_ERROR(logger_, "CIRCLE route requires exactly 2 waypoints: [start, center]");
        return Trajectory{};
    }
    return movec_strategy_->planCircle(request.waypoints[0], request.waypoints[1]);
}

Trajectory MotionPlanningService::planCircle3PtTrajectory(const robot_interfaces::msg::MoveCRequest& request) {
    if (request.waypoints.size() != 3) {
        RCLCPP_ERROR(logger_, "CIRCLE3PT route requires exactly 3 waypoints: [p1, p2, p3]");
        return Trajectory{};
    }

    return movec_strategy_->planCircleThrough3Points(request.waypoints[0], request.waypoints[1], request.waypoints[2]);
}

PlanningResult MotionPlanningService::createFailureResult(const std::string& operation, const std::string& reason) {
    std::string error_msg = "[" + operation + "] " + reason;
    RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
    return PlanningResult(Trajectory{}, false, error_msg);
}

PlanningResult MotionPlanningService::createSuccessResult(const Trajectory& trajectory, const std::string& operation) {
    RCLCPP_INFO(logger_, "[%s] Planned %zu pts, duration=%.2f s",
                operation.c_str(), trajectory.size(), trajectory.total_duration().seconds());
    return PlanningResult(trajectory, true, "");
}

} // namespace trajectory_planning::application::services