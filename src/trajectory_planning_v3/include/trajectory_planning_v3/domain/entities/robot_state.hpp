#pragma once

#include <vector>
#include <string>
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"
#include "trajectory_planning_v3/domain/value_objects/pose.hpp"

namespace trajectory_planning::domain::entities {

/**
 * @brief 表示机器人在某一时刻的状态
 */
class RobotState {
public:
    RobotState() = default;
    RobotState(const value_objects::JointPosition& pos,
               const value_objects::JointVelocity& vel,
               const value_objects::JointAcceleration& acc,
               const value_objects::Pose& ee_pose);

    const value_objects::JointPosition& position() const;
    const value_objects::JointVelocity& velocity() const;
    const value_objects::JointAcceleration& acceleration() const;
    const value_objects::Pose& end_effector_pose() const;

    void set_position(const value_objects::JointPosition& pos);
    void set_velocity(const value_objects::JointVelocity& vel);
    void set_acceleration(const value_objects::JointAcceleration& acc);
    void set_end_effector_pose(const value_objects::Pose& pose);

private:
    value_objects::JointPosition position_;
    value_objects::JointVelocity velocity_;
    value_objects::JointAcceleration acceleration_;
    value_objects::Pose end_effector_pose_;
};

}  // namespace trajectory_planning_v3::domain::entities
