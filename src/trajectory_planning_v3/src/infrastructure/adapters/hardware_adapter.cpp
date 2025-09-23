#include "trajectory_planning_v3/infrastructure/adapters/hardware_adapter.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>

namespace trajectory_planning::infrastructure::adapters {

HardwareAdapter::HardwareAdapter(
    std::shared_ptr<RobotHardware> robot_hw,
    const std::string& interface,
    size_t num_joints)
    : robot_hw_(std::move(robot_hw))
    , interface_(interface)
    , num_joints_(num_joints) {
    
    // 注意：robot_hw_可以为nullptr，稍后通过setRobotHardware设置
}

HardwareAdapter::~HardwareAdapter() = default;

void HardwareAdapter::setRobotHardware(std::shared_ptr<RobotHardware> robot_hw) {
    robot_hw_ = std::move(robot_hw);
}

// === 批量控制接口实现 ===

bool HardwareAdapter::sendPositionCommand(const std::vector<double>& positions) {
    if (!robot_hw_ || positions.size() != num_joints_) {
        return false;
    }
    
    return robot_hw_->send_realtime_position_command(interface_, positions);
}

bool HardwareAdapter::sendVelocityCommand(const std::vector<double>& velocities) {
    if (!robot_hw_ || velocities.size() != num_joints_) {
        return false;
    }
    
    return robot_hw_->send_realtime_velocity_command(interface_, velocities);
}

bool HardwareAdapter::sendEffortCommand(const std::vector<double>& efforts) {
    if (!robot_hw_ || efforts.size() != num_joints_) {
        return false;
    }
    
    return robot_hw_->send_realtime_effort_command(interface_, efforts);
}

bool HardwareAdapter::sendMitCommand(const std::vector<double>& positions,
                                    const std::vector<double>& velocities,
                                    const std::vector<double>& efforts) {
    if (!robot_hw_ || 
        positions.size() != num_joints_ || 
        velocities.size() != num_joints_ || 
        efforts.size() != num_joints_) {
        return false;
    }
    
    return robot_hw_->send_realtime_mit_command(interface_, positions, velocities, efforts);
}

// === 轨迹执行接口实现 ===

bool HardwareAdapter::executeTrajectory(const domain::entities::Trajectory& traj) {
    if (!robot_hw_) {
        return false;
    }
    
    // 转换轨迹格式
    auto hw_trajectory = convertTrajectory(traj);
    
    // 调用RobotHardware的轨迹执行功能
    return robot_hw_->execute_trajectory(interface_, hw_trajectory);
}

// === 电机控制接口实现 ===

bool HardwareAdapter::disableAllJoints() {
    if (!robot_hw_) {
        std::cout << "❌ HardwareAdapter: robot_hw_ is null, cannot disable motors" << std::endl;
        return false;
    }
    
    std::cout << "🔌 HardwareAdapter: Disabling all 6 motors..." << std::endl;
    
    // 失能电机
    robot_hw_->disable_motor(interface_, 1);
    robot_hw_->disable_motor(interface_, 2);
    robot_hw_->disable_motor(interface_, 3);
    robot_hw_->disable_motor(interface_, 4);
    robot_hw_->disable_motor(interface_, 5);
    robot_hw_->disable_motor(interface_, 6);
    
    // 等待一下让命令发送完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "✅ HardwareAdapter: All motors disabled" << std::endl;
    return true;
}

// === 私有方法实现 ===

::Trajectory HardwareAdapter::convertTrajectory(const domain::entities::Trajectory& traj) const {
    ::Trajectory hw_traj;
    
    // 生成关节名称
    hw_traj.joint_names.reserve(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
        hw_traj.joint_names.push_back("joint" + std::to_string(i));
    }
    
    // 转换轨迹点
    const auto& domain_points = traj.points();
    hw_traj.points.reserve(domain_points.size());
    
    for (const auto& point : domain_points) {
        ::TrajectoryPoint hw_point;
        hw_point.time_from_start = point.time_from_start.seconds();
        hw_point.positions = point.position.values();
        hw_point.velocities = point.velocity.values();
        hw_point.accelerations = point.acceleration.values();
        
        hw_traj.points.push_back(std::move(hw_point));
    }
    
    return hw_traj;
}

}   // namespace trajectory_planning::infrastructure::adapters
