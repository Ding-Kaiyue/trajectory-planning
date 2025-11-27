#include "trajectory_planning_v3/domain/entities/robot_state.hpp"

namespace trajectory_planning::domain::entities {

RobotState::RobotState(const value_objects::JointPosition& pos,
                       const value_objects::JointVelocity& vel,
                       const value_objects::JointAcceleration& acc,
                       const value_objects::Pose& ee_pose)
    : position_(pos),
      velocity_(vel),
      acceleration_(acc),
      end_effector_pose_(ee_pose) {}

const value_objects::JointPosition& RobotState::position() const {
	return position_;
}

const value_objects::JointVelocity& RobotState::velocity() const {
	return velocity_;
}

const value_objects::JointAcceleration& RobotState::acceleration() const {
	return acceleration_;
}

const value_objects::Pose& RobotState::end_effector_pose() const {
	return end_effector_pose_;
}

void RobotState::set_position(const value_objects::JointPosition& pos) {
	position_ = pos;
}

void RobotState::set_velocity(const value_objects::JointVelocity& vel) {
	velocity_ = vel;
}

void RobotState::set_acceleration(const value_objects::JointAcceleration& acc) {
	acceleration_ = acc;
}

void RobotState::set_end_effector_pose(const value_objects::Pose& pose) {
	end_effector_pose_ = pose;
}

}  // namespace trajectory_planning::domain::entities
