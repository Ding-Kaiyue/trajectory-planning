#pragma once

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vector>
#include <memory>
#include <string>
#include <optional>

namespace trajectory_planning::infrastructure::planning {

/**
 * @brief 约束运动规划策略
 *
 * 使用MoveIt在关节约束条件下进行运动规划，自动使用TRAC_IK求解器
 */
class JointConstrainedPlanningStrategy {
public:
    /**
     * @brief 规划模式
     */
    enum class PlanningType {
        JOINT,      // 使用关节空间进行规划
        CARTESIAN,  // 使用笛卡尔空间进行规划
        INTELLIGENT, // 智能选择（优先笛卡尔，失败则关节）
    };
    /**
     * @brief 关节约束基类
     */
    struct JointConstraint {
        int joint_index;
        double weight = 1.0;        // 约束权重 (1.0 = 标准权重)
        bool is_hard = true;        // true = 硬约束, false = 软约束
        virtual ~JointConstraint() = default;
    };

    /**
     * @brief 关节固定约束
     */
    struct FixedJointConstraint : public JointConstraint {
        double fixed_value;

        FixedJointConstraint(int index, double value, double weight = 1.0, bool is_hard = true) {
            joint_index = index;
            fixed_value = value;
            this->weight = weight;
            this->is_hard = is_hard;
        }
    };

    /**
     * @brief 关节范围约束
     */
    struct RangeJointConstraint : public JointConstraint {
        double min_value;
        double max_value;

        RangeJointConstraint(int index, double min_val, double max_val, double weight = 1.0, bool is_hard = true) {
            joint_index = index;
            min_value = min_val;
            max_value = max_val;
            this->weight = weight;
            this->is_hard = is_hard;
        }
    };

    /**
     * @brief 构造函数
     * @param moveit_adapter MoveIt适配器
     */
    explicit JointConstrainedPlanningStrategy(integration::MoveItAdapter& moveit_adapter)
        : moveit_adapter_(moveit_adapter) {}

    /**
     * @brief 添加关节约束
     * @param constraint 约束（智能指针）
     * @param validate 是否验证约束与关节限制的兼容性（默认true）
     */
    void addConstraint(std::shared_ptr<JointConstraint> constraint, bool validate = true);

    /**
     * @brief 移除所有约束
     */
    void clearConstraints();

    /**
     * @brief 规划到目标位姿
     * @param goal_pose 目标位姿
     * @param planning_type 规划类型
     * @param max_attempts 最大尝试次数（用于寻找满足约束的解）
     * @return 轨迹
     */
    domain::entities::Trajectory plan(
        const geometry_msgs::msg::Pose& goal_pose,
        PlanningType planning_type = PlanningType::INTELLIGENT,
        int max_attempts = 5);

    /**
     * @brief 验证关节值是否满足约束
     * @param joint_values 关节值
     * @return 是否满足约束
     */
    bool validateConstraints(const std::vector<double>& joint_values) const;

    /**
     * @brief 综合验证所有约束的可行性
     * @param log_level 日志级别 (0=无日志, 1=INFO/WARN, 2=ERROR)
     * @return 约束是否可行
     */
    bool validateAllConstraints(int log_level = 2) const;

private:
    infrastructure::integration::MoveItAdapter& moveit_adapter_;
    std::vector<std::shared_ptr<JointConstraint>> constraints_;

    // 验证参数
    static constexpr double DEFAULT_POSITION_TOLERANCE = 1e-3; // 1mm for positions (more reasonable than 1e-6)
    static constexpr double DEFAULT_VELOCITY_TOLERANCE = 1e-2;  // 0.01 rad/s for velocities
    static constexpr double DEFAULT_ACCELERATION_TOLERANCE = 1e-1; // 0.1 rad/s² for accelerations

    /**
     * @brief 约束验证结果
     */
    enum class ConstraintValidationResult {
        VALID,              // 约束有效
        JOINT_OUT_OF_RANGE, // 关节索引超出范围
        FIXED_OUT_OF_LIMITS, // 固定值超出关节限制
        RANGE_NO_INTERSECTION, // 范围约束与关节限制不相交
        LIMITS_UNAVAILABLE  // 关节限制不可用
    };

    /**
     * @brief 验证单个约束与关节限制的兼容性
     * @param constraint 要验证的约束
     * @param log_level 日志级别 (0=无日志, 1=INFO/WARN, 2=ERROR)
     * @return 验证结果
     */
    ConstraintValidationResult validateSingleConstraint(
        std::shared_ptr<JointConstraint> constraint,
        int log_level = 1) const;

    /**
     * @brief 获取约束的有效范围（与关节限制的交集）
     * @param constraint 约束
     * @param joint_limits 关节限制
     * @return 有效范围 {min, max}，如果无效则返回 nullopt
     */
    std::optional<std::pair<double, double>> getEffectiveConstraintRange(
        std::shared_ptr<JointConstraint> constraint,
        const std::vector<std::pair<double, double>>& joint_limits) const;

    /**
     * @brief 验证轨迹是否满足约束
     * @param trajectory MoveIt轨迹
     * @return 是否满足约束
     */
    bool validateTrajectoryConstraints(const moveit_msgs::msg::RobotTrajectory& trajectory) const;

    /**
     * @brief 计算轨迹的约束满足度评分
     * @param trajectory MoveIt轨迹
     * @return 评分（0-1，1表示完全满足）
     */
    double computeConstraintScore(const moveit_msgs::msg::RobotTrajectory& trajectory) const;

    /**
     * @brief 生成不同的种子点配置
     * @param attempt_index 尝试索引
     * @return 种子点关节角度，nullopt表示使用默认
     */
    std::optional<std::vector<double>> generateSeedConfiguration(int attempt_index) const;

    /**
     * @brief 尝试使用指定种子点进行规划
     * @param goal_pose 目标位姿
     * @param planning_type 规划类型
     * @param seed_config 种子点配置（可选）
     * @return 轨迹和评分的配对
     */
    std::pair<moveit_msgs::msg::RobotTrajectory, double> attemptPlanWithSeed(
        const geometry_msgs::msg::Pose& goal_pose,
        PlanningType planning_type,
        const std::optional<std::vector<double>>& seed_config) const;

    domain::entities::Trajectory convertTrajectoryType(const moveit_msgs::msg::RobotTrajectory& moveit_traj) const;
};

} // namespace trajectory_planning::infrastructure::planning