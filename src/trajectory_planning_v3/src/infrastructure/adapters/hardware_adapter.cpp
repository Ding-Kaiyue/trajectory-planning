#include "trajectory_planning_v3/infrastructure/adapters/hardware_adapter.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace trajectory_planning::infrastructure::adapters {

// 硬件适配器实现

HardwareAdapter::HardwareAdapter(std::shared_ptr<RobotHardware> robot_hw,
                                 const std::string& interface,
                                 size_t num_joints)
    : robot_hw_(std::move(robot_hw)),
      interface_(interface),
      num_joints_(num_joints) {
	// 注意：robot_hw_可以为nullptr，稍后通过setRobotHardware设置
}

HardwareAdapter::~HardwareAdapter() = default;

void HardwareAdapter::setRobotHardware(
    std::shared_ptr<RobotHardware> robot_hw) {
	robot_hw_ = std::move(robot_hw);
}

// === 批量控制接口实现 ===

bool HardwareAdapter::sendPositionCommand(
    const std::vector<double>& positions,
    const std::vector<double>& kps,
    const std::vector<double>& kds) {
	if (!robot_hw_ || positions.size() != num_joints_ || num_joints_ > 6) {
		return false;
	}

	// 转换为std::array<double, 6>
	std::array<double, 6> pos_array{};
	std::array<double, 6> kp_array{};
	std::array<double, 6> kd_array{};

	for (size_t i = 0; i < num_joints_; ++i) {
		pos_array[i] = positions[i];
		kp_array[i] = (i < kps.size()) ? kps[i] : 0.05;  // 默认kp=0.05
		kd_array[i] = (i < kds.size()) ? kds[i] : 0.005; // 默认kd=0.005
	}

	return robot_hw_->send_realtime_position_command(interface_, pos_array, kp_array, kd_array);
}

bool HardwareAdapter::sendVelocityCommand(
    const std::vector<double>& velocities,
    const std::vector<double>& kps,
    const std::vector<double>& kds) {
	if (!robot_hw_ || velocities.size() != num_joints_ || num_joints_ > 6) {
		return false;
	}

	// 转换为std::array<double, 6>
	std::array<double, 6> vel_array{};
	std::array<double, 6> kp_array{};
	std::array<double, 6> kd_array{};

	for (size_t i = 0; i < num_joints_; ++i) {
		vel_array[i] = velocities[i];
		kp_array[i] = (i < kps.size()) ? kps[i] : 0.0;   // 默认kp=0.0
		kd_array[i] = (i < kds.size()) ? kds[i] : 0.005; // 默认kd=0.005
	}

	return robot_hw_->send_realtime_velocity_command(interface_, vel_array, kp_array, kd_array);
}

bool HardwareAdapter::sendEffortCommand(const std::vector<double>& efforts,
                                       const std::vector<double>& kps,
                                       const std::vector<double>& kds) {
	if (!robot_hw_ || efforts.size() != num_joints_ || num_joints_ > 6) {
		return false;
	}

	// 转换为std::array<double, 6>
	std::array<double, 6> effort_array{};
	std::array<double, 6> kp_array{};
	std::array<double, 6> kd_array{};

	for (size_t i = 0; i < num_joints_; ++i) {
		effort_array[i] = efforts[i];
		kp_array[i] = (i < kps.size()) ? kps[i] : 0.05;  // 默认kp=0.05
		kd_array[i] = (i < kds.size()) ? kds[i] : 0.005; // 默认kd=0.005
	}

	return robot_hw_->send_realtime_effort_command(interface_, effort_array, kp_array, kd_array);
}

bool HardwareAdapter::sendMitCommand(const std::vector<double>& positions,
                                     const std::vector<double>& velocities,
                                     const std::vector<double>& efforts,
                                     const std::vector<double>& kps,
                                     const std::vector<double>& kds) {
	if (!robot_hw_ || positions.size() != num_joints_ ||
	    velocities.size() != num_joints_ || efforts.size() != num_joints_ ||
	    num_joints_ > 6) {
		return false;
	}

	// 转换为std::array<double, 6>
	std::array<double, 6> pos_array{};
	std::array<double, 6> vel_array{};
	std::array<double, 6> effort_array{};
	std::array<double, 6> kp_array{};
	std::array<double, 6> kd_array{};

	for (size_t i = 0; i < num_joints_; ++i) {
		pos_array[i] = positions[i];
		vel_array[i] = velocities[i];
		effort_array[i] = efforts[i];
		kp_array[i] = (i < kps.size()) ? kps[i] : 0.05;  // 默认kp=0.05
		kd_array[i] = (i < kds.size()) ? kds[i] : 0.005; // 默认kd=0.005
	}

	return robot_hw_->send_realtime_mit_command(interface_, pos_array,
	                                            vel_array, effort_array, kp_array, kd_array);
}

// === 轨迹执行接口实现 ===

bool HardwareAdapter::executeTrajectory(
    const domain::entities::Trajectory& traj) {
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
		std::cout
		    << "❌ HardwareAdapter: robot_hw_ is null, cannot disable motors"
		    << std::endl;
		return false;
	}

	std::cout << "🔌 HardwareAdapter: Disabling all " << num_joints_ << " motors..." << std::endl;

	// 失能电机 - 依次执行所有可能的模式 (3=MIT_MODE, 4=SPEED_MODE, 5=POSITION_ABS_MODE)
	const std::array<uint8_t, 3> modes = {3, 4, 5};

	for (uint8_t mode : modes) {
		for (size_t i = 1; i <= num_joints_; ++i) {
			robot_hw_->disable_motor(interface_, i, mode);
		}
		// 每个模式间隔5ms
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	// 等待一下让命令发送完成
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	std::cout << "✅ HardwareAdapter: All motors disabled" << std::endl;
	return true;
}

// === 私有方法实现 ===

::Trajectory HardwareAdapter::convertTrajectory(
    const domain::entities::Trajectory& traj) const {
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


}  // namespace trajectory_planning::infrastructure::adapters
