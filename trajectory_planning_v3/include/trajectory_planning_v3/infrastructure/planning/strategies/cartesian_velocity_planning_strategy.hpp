#pragma once

#include <memory>
#include <optional>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_servo_adapter.hpp"


namespace trajectory_planning::infrastructure::planning {

/**
 * @brief 笛卡尔空间末端速度控制规划策略
 *
 * 将目标末端执行器速度（Twist）转化为关节速度。
 * 适用于 MIT 模式控制电机，只需要返回速度信息。
 */
class CartesianVelocityPlanningStrategy {
public:
    /**
     * @brief 构造函数
     * @param servo_adapter MoveIt Servo 适配器引用
     */
    explicit CartesianVelocityPlanningStrategy(integration::MoveItServoAdapter& servo_adapter)
        : servo_adapter_(servo_adapter) {}

    /**
     * @brief 规划末端执行器速度运动
     *
     * 基于目标末端执行器速度，计算所需的关节速度。
     *
     * @param twist_cmd 目标末端执行器速度 (Twist)
     * @return 包含计算出的关节速度的值对象，失败时返回空值
     */
    std::optional<domain::value_objects::JointVelocity> plan(
        const geometry_msgs::msg::TwistStamped& twist_cmd);

private:
    // MoveIt Servo 适配器引用
    integration::MoveItServoAdapter& servo_adapter_;
};

}  // namespace trajectory_planning::infrastructure::planning
