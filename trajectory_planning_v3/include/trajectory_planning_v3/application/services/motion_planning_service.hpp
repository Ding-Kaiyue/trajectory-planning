#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_interfaces/msg/joint_constrained_request.hpp"
#include "robot_interfaces/msg/move_c_request.hpp"
#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/joint_constrained_planning_strategy.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movec_planning_strategy.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movej_planning_strategy.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movel_planning_strategy.hpp"

namespace trajectory_planning::application::services {

using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::domain::value_objects;
using namespace trajectory_planning::infrastructure::planning;
using namespace trajectory_planning::infrastructure::integration;

struct PlanningResult {
	Trajectory trajectory;
	bool success;
	std::string error_message;

	PlanningResult(const Trajectory& traj, bool ok, const std::string& msg = "")
	    : trajectory(traj), success(ok), error_message(msg) {}
};

class MotionPlanningService {
public:
	// 简化的构造函数
	MotionPlanningService(std::shared_ptr<MoveItAdapter> moveit_adapter,
	                      rclcpp::Node::SharedPtr node);

	// 策略注册方法
	void registerMoveJStrategy(std::shared_ptr<MoveJPlanningStrategy> strategy);
	void registerMoveLStrategy(std::shared_ptr<MoveLPlanningStrategy> strategy);
	void registerMoveCStrategy(std::shared_ptr<MoveCPlanningStrategy> strategy);
	void registerJointConstrainedStrategy(
	    std::shared_ptr<JointConstrainedPlanningStrategy> strategy);

	// 关节空间规划
	PlanningResult planJointMotion(const sensor_msgs::msg::JointState& goal);

	// 直线运动规划
	PlanningResult planLinearMotion(const geometry_msgs::msg::Pose& goal);

	// 圆弧运动规划
	PlanningResult planArcMotion(
	    const robot_interfaces::msg::MoveCRequest& request);

	// 约束规划
	PlanningResult planConstrainedMotion(
	    const robot_interfaces::msg::JointConstrainedRequest& request);

private:
	std::shared_ptr<MoveJPlanningStrategy> movej_strategy_;
	std::shared_ptr<MoveLPlanningStrategy> movel_strategy_;
	std::shared_ptr<MoveCPlanningStrategy> movec_strategy_;
	std::shared_ptr<JointConstrainedPlanningStrategy>
	    joint_constrained_strategy_;
	std::shared_ptr<MoveItAdapter> moveit_adapter_;

	rclcpp::Node::SharedPtr node_;
	rclcpp::Logger logger_;

	// 辅助方法
	Trajectory planArcTrajectory(
	    const robot_interfaces::msg::MoveCRequest& request);
	Trajectory planBezierTrajectory(
	    const robot_interfaces::msg::MoveCRequest& request);
	Trajectory planCircleTrajectory(
	    const robot_interfaces::msg::MoveCRequest& request);
	Trajectory planCircle3PtTrajectory(
	    const robot_interfaces::msg::MoveCRequest& request);
	PlanningResult createFailureResult(const std::string& operation,
	                                   const std::string& reason);
	PlanningResult createSuccessResult(const Trajectory& trajectory,
	                                   const std::string& operation);
};

}  // namespace trajectory_planning::application::services