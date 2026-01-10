#include "trajectory_planning_v3/infrastructure/planning/strategies/movej_planning_strategy.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace trajectory_planning::infrastructure::planning {

domain::entities::Trajectory MoveJPlanningStrategy::plan(
    const domain::value_objects::JointPosition& goal) {
	domain::entities::Trajectory trajectory;
	moveit_msgs::msg::RobotTrajectory moveit_trajectory;
	if (!moveit_adapter_.planJointMotion(goal.values(), moveit_trajectory)) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveJPlanningStrategy"),
		             "Failed to plan joint trajectory!");
		return {};  // 返回空轨迹
	}

	return convertTrajectoryType(moveit_trajectory);
}

domain::entities::Trajectory MoveJPlanningStrategy::convertTrajectoryType(
    const moveit_msgs::msg::RobotTrajectory& moveit_traj) const {
	domain::entities::Trajectory trajectory;

	// 将 MoveIt trajectory 转换为我们的 Trajectory 对象
	for (const auto& point : moveit_traj.joint_trajectory.points) {
		double time_sec = static_cast<double>(point.time_from_start.sec) +
		                  static_cast<double>(point.time_from_start.nanosec) * 1e-9;

		std::vector<double> velocities = point.velocities;
		if (velocities.empty()) {
			velocities = std::vector<double>(point.positions.size(), 0.0);
		}

		std::vector<double> accelerations = point.accelerations;
		if (accelerations.empty()) {
			accelerations = std::vector<double>(point.positions.size(), 0.0);
		}

		trajectory.add_point({
		    .position = domain::value_objects::JointPosition(point.positions),
		    .velocity = domain::value_objects::JointVelocity(velocities),
		    .acceleration = domain::value_objects::JointAcceleration(accelerations),
		    .time_from_start = domain::value_objects::Duration(time_sec),
		});
	}

	// 计算 progress_ratio
	trajectory.compute_progress_ratios();

	return trajectory;
}

}  // namespace trajectory_planning::infrastructure::planning
