#include "trajectory_planning_v3/domain/services/kinematics_service.hpp"

namespace trajectory_planning::domain::services {

value_objects::Pose KinematicsService::forward(
    const value_objects::JointPosition& joints) const 
{
    // ⚠️ 占位实现
    // 在 infrastructure 层重载/扩展时，用 MoveIt 或 TRAC-IK 来计算
    (void)joints;
    return value_objects::Pose{};
}

value_objects::JointPosition KinematicsService::inverse(
    const value_objects::Pose& pose) const 
{
    // ⚠️ 占位实现
    // 在 infrastructure 层重载/扩展时，用 MoveIt 或 TRAC-IK 来计算
    (void)pose;
    return value_objects::JointPosition{};
}

value_objects::JointVelocity KinematicsService::inverse_velocity(
    const value_objects::JointPosition& joints,
    const value_objects::EndEffectorVelocity& ee_velocity) const 
{
    // ⚠️ 占位实现
    // 在 infrastructure 层重载/扩展时，用雅可比矩阵来计算
    (void)joints;
    (void)ee_velocity;
    return value_objects::JointVelocity{};
}

} // namespace trajectory_planning::domain::services
