#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::infrastructure::adapters {

class RosMessageAdapter {
public:
	// ROS2 JointTrajectory → 内部 Trajectory
	static domain::entities::Trajectory fromRosMessage(
	    const trajectory_msgs::msg::JointTrajectory &msg);

	// 内部 Trajectory → ROS2 JointTrajectory
	static trajectory_msgs::msg::JointTrajectory toRosMessage(
	    const trajectory_planning::domain::entities::Trajectory &trajectory,
	    const std::vector<std::string> &joint_names);
};

}  // namespace trajectory_planning::infrastructure::adapters
