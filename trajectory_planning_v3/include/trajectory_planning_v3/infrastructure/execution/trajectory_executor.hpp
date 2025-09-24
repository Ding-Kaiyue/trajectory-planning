#pragma once
#include <rclcpp/rclcpp.hpp>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

namespace trajectory_planning::infrastructure::execution {

class TrajectoryExecutor {
public:
	explicit TrajectoryExecutor(integration::MoveItAdapter& moveit_adapter)
	    : moveit_adapter_(moveit_adapter) {}

	/**
	 * @brief 执行 Trajectory（只规划，不执行）
	 * @param trajectory 待执行的轨迹
	 * @return 是否成功
	 */
	bool execute(const domain::entities::Trajectory& trajectory);

private:
	integration::MoveItAdapter& moveit_adapter_;

	/**
	 * @brief 执行轨迹并显示进度
	 * @param trajectory 原始轨迹（包含progress_ratio）
	 * @param moveit_trajectory MoveIt轨迹
	 */
	void executeWithProgress(
	    const domain::entities::Trajectory& trajectory,
	    const moveit_msgs::msg::RobotTrajectory& moveit_trajectory);
};

}  // namespace trajectory_planning::infrastructure::execution
