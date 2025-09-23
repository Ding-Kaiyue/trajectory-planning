#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <future>

namespace trajectory_planning::infrastructure::execution {

bool TrajectoryExecutor::execute(const domain::entities::Trajectory& trajectory) {
    if (trajectory.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("TrajectoryExecutor"), "Trajectory is empty!");
        return false;
    }

    // 将 Trajectory 转换为 MoveIt RobotTrajectory
    moveit_msgs::msg::RobotTrajectory moveit_trajectory;
    
    // 从 MoveItAdapter 获取关节名称
    moveit_trajectory.joint_trajectory.joint_names = moveit_adapter_.getJointNames();

    for (const auto& point : trajectory.points()) {
        trajectory_msgs::msg::JointTrajectoryPoint jt_point;
        jt_point.positions = point.position.values();

        // 速度和加速度可以先填空或者全零
        jt_point.velocities.resize(point.position.size(), 0.0);
        jt_point.accelerations.resize(point.position.size(), 0.0);

        jt_point.time_from_start.sec = static_cast<int>(point.time_from_start.seconds());
        jt_point.time_from_start.nanosec = static_cast<uint32_t>((point.time_from_start.seconds() - jt_point.time_from_start.sec) * 1e9);

        moveit_trajectory.joint_trajectory.points.push_back(jt_point);
    }

    // 在后台执行轨迹并显示进度
    executeWithProgress(trajectory, moveit_trajectory);
    
    return true;
}

void TrajectoryExecutor::executeWithProgress(const domain::entities::Trajectory& trajectory, 
                                           const moveit_msgs::msg::RobotTrajectory& moveit_trajectory) {
    auto logger = rclcpp::get_logger("TrajectoryExecutor");
    
    // 获取轨迹总时长
    double total_duration = 0.0;
    const auto& points = trajectory.points();
    if (!points.empty()) {
        total_duration = points.back().time_from_start.seconds();
    }
    
    if (total_duration <= 0.0) {
        RCLCPP_WARN(logger, "Trajectory duration is zero or negative, cannot show progress");
        bool success = moveit_adapter_.executeTrajectory(moveit_trajectory);
        if (!success) {
            RCLCPP_ERROR(logger, "Failed to execute trajectory!");
        }
        return;
    }
    
    RCLCPP_INFO(logger, "Executing trajectory (%.2f seconds)...", total_duration);

    // 启动轨迹执行（进度条现在在硬件层显示）
    bool success = moveit_adapter_.executeTrajectory(moveit_trajectory);
    if (!success) {
        RCLCPP_ERROR(logger, "Failed to execute trajectory!");
    } else {
        RCLCPP_INFO(logger, "Trajectory execution completed successfully!");
    }
}

} // namespace trajectory_planning::infrastructure::execution
