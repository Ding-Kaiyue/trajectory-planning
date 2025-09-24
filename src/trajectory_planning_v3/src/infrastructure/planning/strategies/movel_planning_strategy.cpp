#include "trajectory_planning_v3/infrastructure/planning/strategies/movel_planning_strategy.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "trajectory_planning_v3/domain/value_objects/duration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"

namespace trajectory_planning::infrastructure::planning {

domain::entities::Trajectory MoveLPlanningStrategy::plan(
    const geometry_msgs::msg::Pose& goal, PlanningType planning_type) {
	moveit_msgs::msg::RobotTrajectory moveit_trajectory;
	bool success = false;

	switch (planning_type) {
		case PlanningType::JOINT: {
			success = moveit_adapter_.planPoseGoal(goal, moveit_trajectory);
			if (!success) {
				RCLCPP_ERROR(rclcpp::get_logger("MoveLPlanningStrategy"),
				             "Failed to plan pose goal with MoveIt KDL!");
			}
			break;
		}
		case PlanningType::CARTESIAN: {
			std::vector<geometry_msgs::msg::Pose> waypoints = {goal};
			success =
			    moveit_adapter_.planCartesianPath(waypoints, moveit_trajectory);
			if (!success) {
				RCLCPP_ERROR(rclcpp::get_logger("MoveLPlanningStrategy"),
				             "Failed to plan cartesian path with MoveIt KDL!");
			}
			break;
		}
		case PlanningType::INTELLIGENT: {
			std::vector<geometry_msgs::msg::Pose> waypoints = {goal};
			success =
			    moveit_adapter_.planCartesianPath(waypoints, moveit_trajectory);

			if (!success) {
				RCLCPP_WARN(
				    rclcpp::get_logger("MoveLPlanningStrategy"),
				    "MoveIt cartesian planning failed, trying pose goal...");
				success = moveit_adapter_.planPoseGoal(goal, moveit_trajectory);
				if (!success) {
					RCLCPP_ERROR(
					    rclcpp::get_logger("MoveLPlanningStrategy"),
					    "Both MoveIt cartesian and pose planning failed!");
				}
			}
			break;
		}
		default: {
			success = moveit_adapter_.planPoseGoal(goal, moveit_trajectory);
			RCLCPP_INFO(rclcpp::get_logger("MoveLPlanningStrategy"),
			            "Unknown planning type, defaulting to JOINT.");
			if (!success) {
				RCLCPP_ERROR(rclcpp::get_logger("MoveLPlanningStrategy"),
				             "Failed to plan pose goal with MoveIt KDL!");
			}
			break;
		}
	}

	if (!success) {
		return {};  // 返回空轨迹
	}

	return convertTrajectoryType(moveit_trajectory);
}

domain::entities::Trajectory MoveLPlanningStrategy::convertTrajectoryType(
    const moveit_msgs::msg::RobotTrajectory& moveit_traj) const {
	domain::entities::Trajectory trajectory;

	// 将 MoveIt trajectory 转换为我们的 Trajectory 对象
	for (const auto& point : moveit_traj.joint_trajectory.points) {
		domain::entities::TrajectoryPoint traj_point{
		    .position = domain::value_objects::JointPosition(point.positions),
		    .velocity = domain::value_objects::JointVelocity(
		        point.velocities.empty()
		            ? std::vector<double>(point.positions.size(), 0.0)
		            : point.velocities),
		    .acceleration = domain::value_objects::JointAcceleration(
		        point.accelerations.empty()
		            ? std::vector<double>(point.positions.size(), 0.0)
		            : point.accelerations),
		    .time_from_start = domain::value_objects::Duration(
		        static_cast<double>(point.time_from_start.sec) +
		        static_cast<double>(point.time_from_start.nanosec) * 1e-9),
		    .progress_ratio = 0.0};

		trajectory.add_point(traj_point);
	}

	// 计算 progress_ratio
	trajectory.compute_progress_ratios();

	return trajectory;
}

}  // namespace trajectory_planning::infrastructure::planning
