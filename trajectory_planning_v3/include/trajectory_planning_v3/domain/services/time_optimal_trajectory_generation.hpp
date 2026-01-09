#pragma once

#include <Eigen/Core>
#include <vector>
#include <string>

#include <moveit/robot_model/robot_model.h>

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::domain::services
{

class TimeOptimalTrajectoryParameterization
{
public:
    TimeOptimalTrajectoryParameterization(
        moveit::core::RobotModelPtr robot_model,
        const std::string& group_name,
        double velocity_scaling,
        double acceleration_scaling);

    domain::entities::Trajectory compute(
        const std::vector<Eigen::VectorXd>& q_path) const;

private:
    moveit::core::RobotModelPtr robot_model_;
    std::string group_name_;
    double velocity_scaling_;
    double acceleration_scaling_;
};

}  // namespace trajectory_planning::domain::services
