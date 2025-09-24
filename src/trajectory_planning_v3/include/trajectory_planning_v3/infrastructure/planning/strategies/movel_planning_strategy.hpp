#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

namespace trajectory_planning::infrastructure::planning {

/**
 * @brief MoveL 规划策略
 *
 * 将目标末端位姿生成连续的轨迹 (Trajectory)。
 * 可以通过设置最大速度和加速度生成平滑轨迹。
 */
class MoveLPlanningStrategy {
public:
	/**
	 * @brief MoveL 的规划模式
	 */
	enum class PlanningType {
		JOINT,        // 使用关节空间进行规划
		CARTESIAN,    // 使用笛卡尔空间进行规划
		INTELLIGENT,  // 智能选择（优先笛卡尔，失败则关节）
	};

	explicit MoveLPlanningStrategy(integration::MoveItAdapter& moveit_adapter)
	    : moveit_adapter_(moveit_adapter) {}

	/**
	 * @brief 规划笛卡尔空间轨迹
	 * @param goal 目标末端位姿
	 * @return 生成的Trajectory对象
	 */
	domain::entities::Trajectory plan(
	    const geometry_msgs::msg::Pose& goal,
	    PlanningType planning_type = PlanningType::INTELLIGENT);

private:
	infrastructure::integration::MoveItAdapter& moveit_adapter_;

	domain::entities::Trajectory convertTrajectoryType(
	    const moveit_msgs::msg::RobotTrajectory& moveit_traj) const;
};

}  // namespace trajectory_planning::infrastructure::planning
