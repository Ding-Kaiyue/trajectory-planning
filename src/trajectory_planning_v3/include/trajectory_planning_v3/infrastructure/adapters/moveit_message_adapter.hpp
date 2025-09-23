#pragma once

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::infrastructure::adapters {

class MoveitMessageAdapter {
public:
    // 将 MoveIt 的 RobotTrajectory 转为内部 Trajectory
    static domain::entities::Trajectory fromMoveitMessage(const moveit_msgs::msg::RobotTrajectory& msg);
    // 将内部 Trajectory 转为 MoveIt 的 RobotTrajectory  
    static moveit_msgs::msg::RobotTrajectory toMoveitMessage(
        const domain::entities::Trajectory& trajectory,
        const std::vector<std::string>& joint_names
    );
};

} // namespace trajectory_planning::infrastructure::adapters
