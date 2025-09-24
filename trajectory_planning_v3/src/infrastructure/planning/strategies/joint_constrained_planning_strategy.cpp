#include "trajectory_planning_v3/infrastructure/planning/strategies/joint_constrained_planning_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "trajectory_planning_v3/domain/value_objects/duration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"

namespace trajectory_planning::infrastructure::planning {

void JointConstrainedPlanningStrategy::addConstraint(
    std::shared_ptr<JointConstraint> constraint, bool validate) {
	if (validate && constraint) {
		auto result =
		    validateSingleConstraint(constraint, 1);  // INFO/WARN level logging

		// 对于冲突的约束，额外显示警告
		if (result == ConstraintValidationResult::RANGE_NO_INTERSECTION ||
		    result == ConstraintValidationResult::FIXED_OUT_OF_LIMITS) {
			RCLCPP_WARN(rclcpp::get_logger("JointConstrainedPlanningStrategy"),
			            "Adding conflicting constraint anyway, but this will "
			            "cause planning failures!");
		}
	}

	constraints_.push_back(constraint);
}

void JointConstrainedPlanningStrategy::clearConstraints() {
	constraints_.clear();
}

bool JointConstrainedPlanningStrategy::validateConstraints(
    const std::vector<double>& joint_values) const {
	for (const auto& constraint : constraints_) {
		if (constraint->joint_index >= static_cast<int>(joint_values.size())) {
			continue;  // 跳过超出范围的约束
		}

		if (auto fixed =
		        std::dynamic_pointer_cast<FixedJointConstraint>(constraint)) {
			double error =
			    std::abs(joint_values[fixed->joint_index] - fixed->fixed_value);
			if (error > DEFAULT_POSITION_TOLERANCE) {
				return false;
			}
		} else if (auto range = std::dynamic_pointer_cast<RangeJointConstraint>(
		               constraint)) {
			double val = joint_values[range->joint_index];
			// 添加小的容差以处理数值精度问题
			if (val < (range->min_value - DEFAULT_POSITION_TOLERANCE) ||
			    val > (range->max_value + DEFAULT_POSITION_TOLERANCE)) {
				return false;
			}
		}
	}
	return true;
}

bool JointConstrainedPlanningStrategy::validateAllConstraints(
    const std::string& arm_type, int log_level) const {
	if (constraints_.empty()) {
		return true;  // 没有约束，总是可行
	}

	auto joint_limits = moveit_adapter_.getJointLimits(arm_type);
	if (joint_limits.empty()) {
		if (log_level >= 1) {
			RCLCPP_WARN(
			    rclcpp::get_logger("ConstrainedPlanningStrategy"),
			    "Cannot validate constraints: joint limits unavailable");
		}
		return true;  // 无法验证时假设可行
	}

	bool all_feasible = true;

	// 1. 验证每个约束与关节限制的兼容性
	for (const auto& constraint : constraints_) {
		auto result = validateSingleConstraint(constraint, log_level);

		if (result != ConstraintValidationResult::VALID &&
		    result != ConstraintValidationResult::LIMITS_UNAVAILABLE) {
			all_feasible = false;
		}
	}

	// 2. 验证同一关节的多个约束之间的兼容性
	std::map<int, std::vector<std::shared_ptr<JointConstraint>>>
	    constraints_by_joint;

	// 按关节分组约束
	for (const auto& constraint : constraints_) {
		if (constraint->joint_index >= 0 &&
		    constraint->joint_index < static_cast<int>(joint_limits.size())) {
			constraints_by_joint[constraint->joint_index].push_back(constraint);
		}
	}

	// 检查每个关节的约束兼容性
	for (const auto& [joint_index, joint_constraints] : constraints_by_joint) {
		if (joint_constraints.size() <= 1) {
			continue;  // 单个约束或无约束，无需检查交集
		}

		// 计算所有约束的交集
		double effective_min = joint_limits[joint_index].first;
		double effective_max = joint_limits[joint_index].second;

		for (const auto& constraint : joint_constraints) {
			auto range = getEffectiveConstraintRange(constraint, joint_limits);
			if (!range.has_value()) {
				// 某个约束无效，整个关节的约束不可行
				if (log_level >= 2) {
					RCLCPP_ERROR(
					    rclcpp::get_logger("ConstrainedPlanningStrategy"),
					    "CONSTRAINT CONFLICT: Joint %d has conflicting "
					    "constraints",
					    joint_index);
				}
				all_feasible = false;
				break;
			}

			// 求交集
			effective_min = std::max(effective_min, range->first);
			effective_max = std::min(effective_max, range->second);

			if (effective_min > effective_max) {
				// 约束之间无交集
				if (log_level >= 2) {
					RCLCPP_ERROR(
					    rclcpp::get_logger("ConstrainedPlanningStrategy"),
					    "CONSTRAINT CONFLICT: Joint %d multiple constraints "
					    "have no intersection! "
					    "Effective range becomes [%.3f, %.3f]",
					    joint_index, effective_min, effective_max);
				}
				all_feasible = false;
				break;
			}
		}

		// 如果有交集，输出有效范围信息
		if (all_feasible && log_level >= 1 && joint_constraints.size() > 1) {
			RCLCPP_INFO(
			    rclcpp::get_logger("ConstrainedPlanningStrategy"),
			    "Joint %d has %zu constraints, effective range: [%.3f, %.3f]",
			    joint_index, joint_constraints.size(), effective_min,
			    effective_max);
		}
	}

	return all_feasible;
}

JointConstrainedPlanningStrategy::ConstraintValidationResult
JointConstrainedPlanningStrategy::validateSingleConstraint(
    std::shared_ptr<JointConstraint> constraint, int log_level) const {
	if (!constraint) {
		return ConstraintValidationResult::VALID;
	}

	auto joint_limits = moveit_adapter_.getJointLimits();
	if (joint_limits.empty()) {
		if (log_level >= 1) {
			RCLCPP_WARN(rclcpp::get_logger("ConstrainedPlanningStrategy"),
			            "Cannot validate constraint: joint limits unavailable");
		}
		return ConstraintValidationResult::LIMITS_UNAVAILABLE;
	}

	// 检查关节索引
	if (constraint->joint_index < 0 ||
	    constraint->joint_index >= static_cast<int>(joint_limits.size())) {
		if (log_level >= 2) {
			RCLCPP_ERROR(
			    rclcpp::get_logger("ConstrainedPlanningStrategy"),
			    "CONSTRAINT ERROR: Joint index %d out of range [0, %zu)",
			    constraint->joint_index, joint_limits.size());
		}
		return ConstraintValidationResult::JOINT_OUT_OF_RANGE;
	}

	const auto& limit = joint_limits[constraint->joint_index];

	// 检查不同类型的约束
	if (auto range =
	        std::dynamic_pointer_cast<RangeJointConstraint>(constraint)) {
		double min_val = std::max(range->min_value, limit.first);
		double max_val = std::min(range->max_value, limit.second);

		if (min_val > max_val) {
			if (log_level >= 2) {
				RCLCPP_ERROR(
				    rclcpp::get_logger("JointConstrainedPlanningStrategy"),
				    "CONSTRAINT CONFLICT: Joint %d range [%.3f, %.3f] does not "
				    "intersect with joint limits [%.3f, %.3f]",
				    constraint->joint_index, range->min_value, range->max_value,
				    limit.first, limit.second);
			}
			return ConstraintValidationResult::RANGE_NO_INTERSECTION;
		} else {
			if (log_level >= 1) {
				RCLCPP_INFO(
				    rclcpp::get_logger("JointConstrainedPlanningStrategy"),
				    "Added range constraint: joint%d ∈ [%.3f, %.3f] "
				    "(effective: [%.3f, %.3f])",
				    constraint->joint_index, range->min_value, range->max_value,
				    min_val, max_val);
			}
		}
	} else if (auto fixed = std::dynamic_pointer_cast<FixedJointConstraint>(
	               constraint)) {
		if (fixed->fixed_value < limit.first ||
		    fixed->fixed_value > limit.second) {
			if (log_level >= 2) {
				RCLCPP_ERROR(
				    rclcpp::get_logger("JointConstrainedPlanningStrategy"),
				    "CONSTRAINT CONFLICT: Joint %d fixed value %.3f outside "
				    "joint limits [%.3f, %.3f]",
				    constraint->joint_index, fixed->fixed_value, limit.first,
				    limit.second);
			}
			return ConstraintValidationResult::FIXED_OUT_OF_LIMITS;
		} else {
			if (log_level >= 1) {
				RCLCPP_INFO(
				    rclcpp::get_logger("JointConstrainedPlanningStrategy"),
				    "Added fixed constraint: joint%d = %.3f",
				    constraint->joint_index, fixed->fixed_value);
			}
		}
	}

	return ConstraintValidationResult::VALID;
}

std::optional<std::pair<double, double>>
JointConstrainedPlanningStrategy::getEffectiveConstraintRange(
    std::shared_ptr<JointConstraint> constraint,
    const std::vector<std::pair<double, double>>& joint_limits) const {
	if (!constraint || joint_limits.empty() || constraint->joint_index < 0 ||
	    constraint->joint_index >= static_cast<int>(joint_limits.size())) {
		return std::nullopt;
	}

	const auto& limit = joint_limits[constraint->joint_index];

	if (auto range =
	        std::dynamic_pointer_cast<RangeJointConstraint>(constraint)) {
		double min_val = std::max(range->min_value, limit.first);
		double max_val = std::min(range->max_value, limit.second);

		if (min_val <= max_val) {
			return std::make_pair(min_val, max_val);
		} else {
			return std::nullopt;  // 无交集
		}
	} else if (auto fixed = std::dynamic_pointer_cast<FixedJointConstraint>(
	               constraint)) {
		if (fixed->fixed_value >= limit.first &&
		    fixed->fixed_value <= limit.second) {
			return std::make_pair(fixed->fixed_value, fixed->fixed_value);
		} else {
			return std::nullopt;  // 超出范围
		}
	}

	return std::nullopt;
}

domain::entities::Trajectory JointConstrainedPlanningStrategy::plan(
    const geometry_msgs::msg::Pose& goal_pose, const std::string& arm_type,
    PlanningType planning_type, int max_attempts) {
	// 预检查约束的有效性
	if (!validateAllConstraints(arm_type, 2)) {  // ERROR level logging
		RCLCPP_ERROR(rclcpp::get_logger("ConstrainedPlanningStrategy"),
		             "Planning aborted: Constraints conflict with joint limits "
		             "or each other. Check error messages above.");
		return {};  // 直接返回空轨迹
	}

	moveit_msgs::msg::RobotTrajectory best_trajectory;
	double best_score = -1.0;
	bool found_valid = false;

	RCLCPP_INFO(
	    rclcpp::get_logger("ConstrainedPlanningStrategy"),
	    "Attempting to find trajectory satisfying constraints with %d attempts",
	    max_attempts);

	// 尝试多次规划，寻找最佳解
	for (int attempt = 0; attempt < max_attempts; ++attempt) {
		auto seed_config = generateSeedConfiguration(arm_type, attempt);

		// 只在第一次尝试时使用种子配置，避免重复调用setStartState导致时间戳问题
		auto actual_seed_config = (attempt == 0) ? seed_config : std::nullopt;
		auto [trajectory, score] =
		    attemptPlanWithSeed(goal_pose, planning_type, actual_seed_config);

		// 如果轨迹为空，跳过此次尝试
		if (trajectory.joint_trajectory.points.empty()) {
			continue;
		}

		RCLCPP_DEBUG(rclcpp::get_logger("ConstrainedPlanningStrategy"),
		             "Attempt %d: score = %.3f", attempt + 1, score);

		// 检查是否满足约束要求
		bool has_soft_constraints = false;
		for (const auto& constraint : constraints_) {
			if (!constraint->is_hard) {
				has_soft_constraints = true;
				break;
			}
		}

		// 动态设置接受阈值
		double acceptance_threshold =
		    has_soft_constraints ? 0.9 : 1.0;  // 有软约束时降低要求

		if (score >= acceptance_threshold) {
			RCLCPP_INFO(
			    rclcpp::get_logger("ConstrainedPlanningStrategy"),
			    "Found acceptable trajectory (score: %.3f) on attempt %d",
			    score, attempt + 1);
			return convertTrajectoryType(trajectory);
		}

		// 记录最佳解（即使不完全满足约束）
		if (score > best_score) {
			best_score = score;
			best_trajectory = trajectory;
			found_valid = true;
		}
	}

	// 如果没有找到完全满足约束的解，但有部分满足的解
	bool has_soft_constraints = false;
	for (const auto& constraint : constraints_) {
		if (!constraint->is_hard) {
			has_soft_constraints = true;
			break;
		}
	}

	double final_threshold =
	    has_soft_constraints ? 0.7 : 0.8;  // 软约束时进一步降低要求

	if (found_valid && best_score > final_threshold) {
		RCLCPP_WARN(rclcpp::get_logger("ConstrainedPlanningStrategy"),
		            "No trajectory fully satisfies constraints. Using best "
		            "solution with score %.3f (threshold: %.1f)",
		            best_score, final_threshold);
		return convertTrajectoryType(best_trajectory);
	}

	RCLCPP_ERROR(rclcpp::get_logger("ConstrainedPlanningStrategy"),
	             "Failed to find acceptable trajectory after %d attempts. Best "
	             "score: %.3f",
	             max_attempts, best_score);
	return {};  // 返回空轨迹
}

bool JointConstrainedPlanningStrategy::validateTrajectoryConstraints(
    const moveit_msgs::msg::RobotTrajectory& trajectory) const {
	// 验证轨迹中每个点是否满足约束
	for (const auto& point : trajectory.joint_trajectory.points) {
		if (!validateConstraints(point.positions)) {
			return false;
		}
	}
	return true;
}

domain::entities::Trajectory
JointConstrainedPlanningStrategy::convertTrajectoryType(
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

double JointConstrainedPlanningStrategy::computeConstraintScore(
    const moveit_msgs::msg::RobotTrajectory& trajectory) const {
	if (trajectory.joint_trajectory.points.empty()) {
		return 0.0;
	}

	if (constraints_.empty()) {
		return 1.0;  // 没有约束时，认为完全满足
	}

	double total_weighted_violations = 0.0;
	double total_weight = 0.0;

	// 检查轨迹中每个点的约束满足情况
	for (const auto& point : trajectory.joint_trajectory.points) {
		for (const auto& constraint : constraints_) {
			if (constraint->joint_index >=
			    static_cast<int>(point.positions.size())) {
				continue;  // 跳过超出范围的约束
			}

			double violation = 0.0;
			double weight = constraint->weight;

			if (auto fixed = std::dynamic_pointer_cast<FixedJointConstraint>(
			        constraint)) {
				double error = std::abs(point.positions[fixed->joint_index] -
				                        fixed->fixed_value);

				if (constraint->is_hard) {
					// 硬约束：超出容差就算违反
					if (error > DEFAULT_POSITION_TOLERANCE) {
						violation = std::min(error / DEFAULT_POSITION_TOLERANCE,
						                     10.0);  // 最大10倍惩罚
					}
				} else {
					// 软约束：平方罚函数，距离越远惩罚越重
					violation = (error * error) / (DEFAULT_POSITION_TOLERANCE *
					                               DEFAULT_POSITION_TOLERANCE);
					violation = std::min(violation, 5.0);  // 软约束最大5倍惩罚
				}
			} else if (auto range =
			               std::dynamic_pointer_cast<RangeJointConstraint>(
			                   constraint)) {
				double val = point.positions[range->joint_index];
				double range_violation = 0.0;

				if (val < range->min_value) {
					range_violation = range->min_value - val;
				} else if (val > range->max_value) {
					range_violation = val - range->max_value;
				}

				if (constraint->is_hard) {
					// 硬约束：超出容差就算违反
					if (range_violation > DEFAULT_POSITION_TOLERANCE) {
						violation = std::min(
						    range_violation / DEFAULT_POSITION_TOLERANCE, 10.0);
					}
				} else {
					// 软约束：平方罚函数
					if (range_violation > 0) {
						violation = (range_violation * range_violation) /
						            (DEFAULT_POSITION_TOLERANCE *
						             DEFAULT_POSITION_TOLERANCE);
						violation = std::min(violation, 5.0);
					}
				}
			}

			total_weighted_violations += violation * weight;
			total_weight += weight;
		}
	}

	if (total_weight == 0.0) {
		return 1.0;
	}

	// 计算加权满足度评分 (0-1)
	double weighted_violation_ratio = total_weighted_violations / total_weight;
	return std::max(
	    0.0,
	    1.0 / (1.0 + weighted_violation_ratio));  // 使用倒数函数，避免线性下降
}

std::optional<std::vector<double>>
JointConstrainedPlanningStrategy::generateSeedConfiguration(
    const std::string& arm_type, int attempt_index) const {
	if (attempt_index == 0) {
		return std::nullopt;  // 第一次尝试使用默认配置
	}

	// 从MoveItAdapter获取关节限制
	auto joint_limits = moveit_adapter_.getJointLimits(arm_type);
	if (joint_limits.empty()) {
		RCLCPP_WARN(rclcpp::get_logger("ConstrainedPlanningStrategy"),
		            "No joint limits available, using default range");
		return std::nullopt;
	}

	// 生成随机种子点配置
	static std::random_device rd;
	static std::mt19937 gen(rd());

	std::vector<double> seed_config;

	for (size_t i = 0; i < joint_limits.size(); ++i) {
		// 查找该关节的约束
		std::shared_ptr<JointConstraint> joint_constraint = nullptr;
		for (const auto& constraint : constraints_) {
			if (constraint->joint_index == static_cast<int>(i)) {
				joint_constraint = constraint;
				break;
			}
		}

		if (joint_constraint) {
			// 获取约束的有效范围
			auto effective_range =
			    getEffectiveConstraintRange(joint_constraint, joint_limits);
			if (effective_range.has_value()) {
				double min_val = effective_range->first;
				double max_val = effective_range->second;

				if (min_val == max_val) {
					// 固定约束
					seed_config.push_back(min_val);
				} else {
					// 范围约束
					std::uniform_real_distribution<double> constrained_dis(
					    min_val, max_val);
					seed_config.push_back(constrained_dis(gen));
				}
			} else {
				// 约束无效，报错并返回
				RCLCPP_ERROR(
				    rclcpp::get_logger("ConstrainedPlanningStrategy"),
				    "CONSTRAINT CONFLICT: Joint %zu constraint conflicts with "
				    "joint limits. This will result in NO SOLUTION!",
				    i);
				return std::nullopt;
			}
		} else {
			// 没有约束，使用关节限制范围
			std::uniform_real_distribution<double> joint_dis(
			    joint_limits[i].first, joint_limits[i].second);
			seed_config.push_back(joint_dis(gen));
		}
	}

	return seed_config;
}

std::pair<moveit_msgs::msg::RobotTrajectory, double>
JointConstrainedPlanningStrategy::attemptPlanWithSeed(
    const geometry_msgs::msg::Pose& goal_pose, PlanningType planning_type,
    const std::optional<std::vector<double>>& seed_config) const {
	moveit_msgs::msg::RobotTrajectory trajectory;
	bool success = false;

	// 如果有种子配置，设置起始状态
	if (seed_config.has_value()) {
		if (!moveit_adapter_.setStartState(seed_config.value())) {
			RCLCPP_WARN(rclcpp::get_logger("ConstrainedPlanningStrategy"),
			            "Failed to set start state with seed configuration");
			// 继续规划，但使用默认起始状态
		}
	}

	switch (planning_type) {
		case PlanningType::JOINT: {
			success = moveit_adapter_.planPoseGoal(goal_pose, trajectory);
			break;
		}
		case PlanningType::CARTESIAN: {
			std::vector<geometry_msgs::msg::Pose> waypoints = {goal_pose};
			success = moveit_adapter_.planCartesianPath(waypoints, trajectory);
			break;
		}
		case PlanningType::INTELLIGENT: {
			std::vector<geometry_msgs::msg::Pose> waypoints = {goal_pose};
			success = moveit_adapter_.planCartesianPath(waypoints, trajectory);

			if (!success) {
				success = moveit_adapter_.planPoseGoal(goal_pose, trajectory);
			}
			break;
		}
		default: {
			success = moveit_adapter_.planPoseGoal(goal_pose, trajectory);
			break;
		}
	}

	// 重置起始状态，避免影响后续规划
	moveit_adapter_.resetStartStateToDefault();

	if (!success) {
		return {moveit_msgs::msg::RobotTrajectory{}, 0.0};
	}

	double score = computeConstraintScore(trajectory);
	return {trajectory, score};
}

}  // namespace trajectory_planning::infrastructure::planning