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

	// DEBUG: 检查MoveIt轨迹的速度和加速度数据
	RCLCPP_INFO(rclcpp::get_logger("MoveJPlanningStrategy"),
	            "=== MoveIt Generated Trajectory (MoveJ) ===");
	RCLCPP_INFO(rclcpp::get_logger("MoveJPlanningStrategy"),
	            "Total points: %zu", trajectory.size());

	// 检查原始MoveIt轨迹的速度/加速度大小
	const auto& first_point = moveit_traj.joint_trajectory.points.front();
	RCLCPP_INFO(rclcpp::get_logger("MoveJPlanningStrategy"),
	            "Raw MoveIt trajectory: positions.size()=%zu, velocities.size()=%zu, accelerations.size()=%zu",
	            first_point.positions.size(), first_point.velocities.size(), first_point.accelerations.size());

	if (first_point.velocities.empty()) {
		RCLCPP_WARN(rclcpp::get_logger("MoveJPlanningStrategy"),
		            "⚠️ MoveIt trajectory has NO velocity data! RRTConnect planner doesn't generate velocities.");
		RCLCPP_WARN(rclcpp::get_logger("MoveJPlanningStrategy"),
		            "   MoveJ is using zero velocities (filled by fallback).");
	} else if (first_point.velocities.size() != first_point.positions.size()) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveJPlanningStrategy"),
		            "❌ MoveIt velocity array size mismatch! pos=%zu, vel=%zu - these are garbage data!",
		            first_point.positions.size(), first_point.velocities.size());
	}

	// DEBUG: 打印所有轨迹点的详细信息
	RCLCPP_INFO(rclcpp::get_logger("MoveJPlanningStrategy"),
	            "=== Raw MoveIt Trajectory Details ===");
	for (size_t i = 0; i < moveit_traj.joint_trajectory.points.size(); ++i) {
		const auto& pt = moveit_traj.joint_trajectory.points[i];

		std::string pos_str = "[";
		std::string vel_str = "[";
		std::string acc_str = "[";

		for (size_t j = 0; j < pt.positions.size(); ++j) {
			if (j > 0) {
				pos_str += ", ";
				vel_str += ", ";
				acc_str += ", ";
			}
			char buf[32];
			snprintf(buf, sizeof(buf), "%.4f", pt.positions[j]);
			pos_str += buf;

			if (j < pt.velocities.size()) {
				snprintf(buf, sizeof(buf), "%.4f", pt.velocities[j]);
				vel_str += buf;
			} else {
				vel_str += "N/A";
			}

			if (j < pt.accelerations.size()) {
				snprintf(buf, sizeof(buf), "%.4f", pt.accelerations[j]);
				acc_str += buf;
			} else {
				acc_str += "N/A";
			}
		}

		pos_str += "]";
		vel_str += "]";
		acc_str += "]";

		double t = static_cast<double>(pt.time_from_start.sec) +
		           static_cast<double>(pt.time_from_start.nanosec) * 1e-9;

		RCLCPP_INFO(rclcpp::get_logger("MoveJPlanningStrategy"),
		            "MoveJ Point %zu: t=%.4fs, pos=%s, vel=%s, acc=%s",
		            i, t, pos_str.c_str(), vel_str.c_str(), acc_str.c_str());
	}


	return trajectory;
}

}  // namespace trajectory_planning::infrastructure::planning
