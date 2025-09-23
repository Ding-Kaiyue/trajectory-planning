#pragma once

#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/end_effector_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/pose.hpp"

namespace trajectory_planning::domain::services {

/**
 * @brief KinematicsService
 * 
 * 提供正/逆运动学及微分运动学接口的默认实现。
 * 当前仅定义方法签名，具体逻辑由基础设施层适配器负责填充。
 */
class KinematicsService {
public:
    KinematicsService() = default;
    ~KinematicsService() = default;

    /**
     * @brief 前向运动学 (FK)
     * @param joints 关节角度
     * @return Pose 末端位姿
     */
    value_objects::Pose forward(const value_objects::JointPosition& joints) const;

    /**
     * @brief 逆向运动学 (IK)
     * @param pose 目标末端位姿
     * @return JointPosition 解算出的关节角度
     */
    value_objects::JointPosition inverse(const value_objects::Pose& pose) const;

    /**
     * @brief 微分运动学 (Jacobian)
     * @param joints 当前关节角度
     * @return JointVelocity 末端速度对应的关节速度
     */
    value_objects::JointVelocity inverse_velocity(
        const value_objects::JointPosition& joints,
        const value_objects::EndEffectorVelocity& ee_velocity) const;
};

} // namespace trajectory_planning::domain::services
