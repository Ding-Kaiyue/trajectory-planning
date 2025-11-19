#pragma once

#include <memory>
#include <vector>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

namespace trajectory_planning::infrastructure::planning {

/**
 * @brief MoveJ 规划策略
 *
 * 将目标关节位置生成连续的轨迹 (Trajectory)。
 * 可以通过设置最大速度和加速度生成平滑轨迹。
 */

class MoveJPlanningStrategy {
public:
	explicit MoveJPlanningStrategy(integration::MoveItAdapter& moveit_adapter)
	    : moveit_adapter_(moveit_adapter) {}

	/**
	 * @brief 规划关节空间轨迹
	 * @param goal 目标关节位置
	 * @return 生成的Trajectory对象
	 */
	domain::entities::Trajectory plan(
	    const domain::value_objects::JointPosition& goal);

private:
	integration::MoveItAdapter& moveit_adapter_;

	domain::entities::Trajectory convertTrajectoryType(
	    const moveit_msgs::msg::RobotTrajectory& moveit_traj) const;
};

}  // namespace trajectory_planning::infrastructure::planning
