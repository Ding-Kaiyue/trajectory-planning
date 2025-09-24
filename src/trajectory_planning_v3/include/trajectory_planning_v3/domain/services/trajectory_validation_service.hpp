#pragma once

#include <string>
#include <vector>

#include "trajectory_planning_v3/domain/entities/constraint.hpp"
#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::domain::services {

/**
 * @brief 轨迹验证服务
 *
 * 根据给定的约束条件验证一条轨迹是否合法。
 */
class TrajectoryValidationService {
public:
	TrajectoryValidationService() = default;

	/**
	 * @brief 验证整条轨迹
	 * @param trajectory 待验证的轨迹
	 * @param constraint 约束条件
	 * @return 如果轨迹满足约束，返回 true；否则 false
	 */
	bool validate(const entities::Trajectory& trajectory,
	              const entities::Constraint& constraint) const;

	/**
	 * @brief 获取验证失败时的错误信息
	 * @return 错误信息列表
	 */
	const std::vector<std::string>& errors() const;

private:
	mutable std::vector<std::string> errors_;
};

}  // namespace trajectory_planning::domain::services
