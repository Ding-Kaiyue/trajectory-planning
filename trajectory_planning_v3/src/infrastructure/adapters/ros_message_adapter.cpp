#include "trajectory_planning_v3/infrastructure/adapters/ros_message_adapter.hpp"

namespace trajectory_planning::infrastructure::adapters {

trajectory_planning::domain::entities::Trajectory
RosMessageAdapter::fromRosMessage(
    const trajectory_msgs::msg::JointTrajectory &msg) {
	trajectory_planning::domain::entities::Trajectory trajectory;

	// 转换每个轨迹点
	for (const auto &point : msg.points) {
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

	// 计算进度比例
	trajectory.compute_progress_ratios();
	return trajectory;
}

trajectory_msgs::msg::JointTrajectory RosMessageAdapter::toRosMessage(
    const trajectory_planning::domain::entities::Trajectory &trajectory,
    const std::vector<std::string> &joint_names) {
	trajectory_msgs::msg::JointTrajectory msg;

	msg.joint_names = joint_names;

	// 转换轨迹点
	for (const auto &point : trajectory.points()) {
		trajectory_msgs::msg::JointTrajectoryPoint jt_point;

		// Convert value objects to std::vector<double>
		jt_point.positions = point.position.values();
		jt_point.velocities = point.velocity.values();
		jt_point.accelerations = point.acceleration.values();

		double time_seconds = point.time_from_start.seconds();
		jt_point.time_from_start.sec = static_cast<int32_t>(time_seconds);
		jt_point.time_from_start.nanosec =
		    static_cast<uint32_t>((time_seconds - jt_point.time_from_start.sec) * 1e9);

		msg.points.push_back(jt_point);
	}

	return msg;
}

}  // namespace trajectory_planning::infrastructure::adapters