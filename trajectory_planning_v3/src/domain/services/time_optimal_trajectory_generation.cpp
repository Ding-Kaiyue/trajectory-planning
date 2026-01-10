#include "trajectory_planning_v3/domain/services/time_optimal_trajectory_generation.hpp"

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdio>

namespace trajectory_planning::domain::services
{

TimeOptimalTrajectoryParameterization::TimeOptimalTrajectoryParameterization(
    moveit::core::RobotModelPtr robot_model,
    const std::string& group_name,
    double velocity_scaling,
    double acceleration_scaling)
  : robot_model_(std::move(robot_model)),
    group_name_(group_name),
    velocity_scaling_(velocity_scaling),
    acceleration_scaling_(acceleration_scaling)
{
}

domain::entities::Trajectory
TimeOptimalTrajectoryParameterization::compute(
    const std::vector<Eigen::VectorXd>& q_path) const
{
    domain::entities::Trajectory traj;

    if (!robot_model_ || q_path.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("TOTG"),
                    "Invalid robot model or path too short");
        return traj;
    }

    const auto* jmg = robot_model_->getJointModelGroup(group_name_);
    if (!jmg) {
        RCLCPP_ERROR(rclcpp::get_logger("TOTG"),
                    "JointModelGroup '%s' not found", group_name_.c_str());
        return traj;
    }

    const size_t dof = jmg->getVariableCount();

    robot_trajectory::RobotTrajectory rt(robot_model_, group_name_);
    moveit::core::RobotState state(robot_model_);
    
    // -----------------------------
    // 1. 添加关键点
    // -----------------------------
    for (size_t i = 0; i < q_path.size(); ++i) {
        if (static_cast<size_t>(q_path[i].size()) != dof) {
            RCLCPP_ERROR(rclcpp::get_logger("TOTG"),
                         "Waypoint %zu DOF mismatch", i);
            return traj;
        }

        std::vector<double> q(dof);
        for (size_t j = 0; j < dof; ++j)
            q[j] = q_path[i][j];

        state.setJointGroupPositions(jmg, q);
        rt.addSuffixWayPoint(state, 0.0);  // 时间由 TOTG 填充
    }

    // -----------------------------
    // 2. TOTG 时间优化
    // 直接使用 velocity_scaling 和 acceleration_scaling 作为缩放因子
    // 缩放因子范围 0-1，表示使用多少百分比的最大速度/加速度
    // -----------------------------
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;

    if (!totg.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_)) {
        RCLCPP_ERROR(rclcpp::get_logger("TOTG"),
                     "TimeOptimalTrajectoryGeneration failed");
        return traj;
    }

    // -----------------------------
    // 3. 返回TOTG的关键点
    // getStateAtDurationFromStart()返回的速度在关键点之间不可靠
    // 所以直接返回TOTG计算的关键点及其精确的速度/加速度
    // 上层通过 Cubic Hermite 插补在这些点之间插值并生成密集采样点
    // 由于边界条件已设置为端点速度，样条插值会保留约束
    // -----------------------------
    size_t N_waypoints = rt.getWayPointCount();
    for (size_t i = 0; i < N_waypoints; ++i) {
        const auto& wp = rt.getWayPoint(i);

        std::vector<double> q(dof), qdot(dof), qddot(dof);
        wp.copyJointGroupPositions(jmg, q);
        wp.copyJointGroupVelocities(jmg, qdot);
        wp.copyJointGroupAccelerations(jmg, qddot);

        double time_from_start = rt.getWayPointDurationFromStart(i);

        traj.add_point({
            .position = domain::value_objects::JointPosition(q),
            .velocity = domain::value_objects::JointVelocity(qdot),
            .acceleration = domain::value_objects::JointAcceleration(qddot),
            .time_from_start = domain::value_objects::Duration(time_from_start),
            .progress_ratio = (N_waypoints > 1) ? static_cast<double>(i) / (N_waypoints - 1) : 1.0
        });
    }

    RCLCPP_INFO(rclcpp::get_logger("TOTG"),
                "Generated trajectory with %zu keypoints, duration=%.3f s",
                traj.size(), rt.getDuration());

    return traj;
}


}  // namespace trajectory_planning::domain::services
