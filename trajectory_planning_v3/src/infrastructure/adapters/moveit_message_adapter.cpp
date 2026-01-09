#include "trajectory_planning_v3/infrastructure/adapters/moveit_message_adapter.hpp"

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace trajectory_planning::infrastructure::adapters {

trajectory_planning::domain::entities::Trajectory
MoveitMessageAdapter::fromMoveitMessage(
    const moveit_msgs::msg::RobotTrajectory& msg) {
	trajectory_planning::domain::entities::Trajectory trajectory;

	const auto& joint_traj = msg.joint_trajectory;

	for (const auto& point : joint_traj.points) {
		trajectory_planning::domain::entities::TrajectoryPoint traj_point;

		// Convert std::vector<double> to value objects
		traj_point.position = trajectory_planning::domain::value_objects::JointPosition(
		    point.positions);

		if (!point.velocities.empty()) {
			traj_point.velocity = trajectory_planning::domain::value_objects::JointVelocity(
			    point.velocities);
		} else {
			traj_point.velocity = trajectory_planning::domain::value_objects::JointVelocity(
			    std::vector<double>(point.positions.size(), 0.0));
		}

		if (!point.accelerations.empty()) {
			traj_point.acceleration = trajectory_planning::domain::value_objects::JointAcceleration(
			    point.accelerations);
		} else {
			traj_point.acceleration = trajectory_planning::domain::value_objects::JointAcceleration(
			    std::vector<double>(point.positions.size(), 0.0));
		}

		traj_point.time_from_start = trajectory_planning::domain::value_objects::Duration(
		    static_cast<double>(point.time_from_start.sec) +
		    static_cast<double>(point.time_from_start.nanosec) * 1e-9);

		trajectory.add_point(traj_point);
	}

	trajectory.compute_progress_ratios();
	return trajectory;
}

moveit_msgs::msg::RobotTrajectory MoveitMessageAdapter::toMoveitMessage(
    const trajectory_planning::domain::entities::Trajectory& trajectory,
    const std::vector<std::string>& joint_names) {
	moveit_msgs::msg::RobotTrajectory msg;

	// 设置关节名称
	msg.joint_trajectory.joint_names = joint_names;

	// 转换轨迹点
	for (const auto& point : trajectory.points()) {
		trajectory_msgs::msg::JointTrajectoryPoint jt_point;

		// Convert value objects to std::vector<double>
		jt_point.positions = point.position.values();
		jt_point.velocities = point.velocity.values();
		jt_point.accelerations = point.acceleration.values();

		double time_seconds = point.time_from_start.seconds();
		jt_point.time_from_start.sec = static_cast<int32_t>(time_seconds);
		jt_point.time_from_start.nanosec =
		    static_cast<uint32_t>((time_seconds - jt_point.time_from_start.sec) * 1e9);

		msg.joint_trajectory.points.push_back(jt_point);
	}

	return msg;
}

}  // namespace trajectory_planning::infrastructure::adapters
