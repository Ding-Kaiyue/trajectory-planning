#pragma once
#include <any>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#ifdef USE_HARDWARE_DRIVER
#include "hardware_driver/interface/robot_hardware.hpp"
#endif

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::infrastructure::adapters {

#ifdef USE_HARDWARE_DRIVER
// 前向声明
using RobotHardware = hardware_driver::RobotHardware;
#else
// CI环境下的模拟类型
class RobotHardware {};
// 模拟硬件轨迹类型
struct Trajectory {
    struct Point {
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        double time_from_start;
    };
    std::vector<Point> points;
};
#endif

/**
 * @brief 硬件适配器 - 桥接轨迹规划与硬件驱动
 *
 * 利用RobotHardware的批量控制接口实现高效的关节控制
 * 职责：纯粹的硬件控制抽象，只负责发送命令，不接收状态
 */
class HardwareAdapter {
public:
	/**
	 * @brief 构造函数
	 * @param robot_hw
	 * RobotHardware实例（可以为nullptr，稍后通过setRobotHardware设置）
	 * @param interface 硬件接口名称（如"can0"）
	 * @param num_joints 关节数量
	 */
	explicit HardwareAdapter(std::shared_ptr<RobotHardware> robot_hw,
	                         const std::string& interface, size_t num_joints);

	/**
	 * @brief 设置RobotHardware实例（用于解决循环依赖）
	 * @param robot_hw RobotHardware实例
	 */
	void setRobotHardware(std::shared_ptr<RobotHardware> robot_hw);

	~HardwareAdapter();

	// === 批量控制接口（利用RobotHardware的实时批量接口） ===

	/**
	 * @brief 批量发送位置命令
	 * @param positions 所有关节的目标位置
	 * @return 是否成功
	 */
	bool sendPositionCommand(const std::vector<double>& positions);

	/**
	 * @brief 批量发送速度命令
	 * @param velocities 所有关节的目标速度
	 * @return 是否成功
	 */
	bool sendVelocityCommand(const std::vector<double>& velocities);

	/**
	 * @brief 批量发送力矩命令
	 * @param efforts 所有关节的目标力矩
	 * @return 是否成功
	 */
	bool sendEffortCommand(const std::vector<double>& efforts);

	/**
	 * @brief 批量发送MIT模式命令
	 * @param positions 目标位置
	 * @param velocities 目标速度
	 * @param efforts 目标力矩
	 * @return 是否成功
	 */
	bool sendMitCommand(const std::vector<double>& positions,
	                    const std::vector<double>& velocities,
	                    const std::vector<double>& efforts);

	// === 轨迹执行接口 ===

	/**
	 * @brief 执行轨迹（使用RobotHardware的轨迹执行功能）
	 * @param traj 轨迹数据
	 * @return 是否成功执行
	 */
	bool executeTrajectory(const domain::entities::Trajectory& traj);

	// === 电机控制接口 ===

	/**
	 * @brief 失能所有关节
	 * @return 是否成功
	 */
	bool disableAllJoints();

private:
	/**
	 * @brief 转换轨迹格式（从domain格式到RobotHardware格式）
	 * @param traj 输入轨迹
	 * @return 转换后的轨迹
	 */
#ifdef USE_HARDWARE_DRIVER
	::Trajectory convertTrajectory(
	    const domain::entities::Trajectory& traj) const;
#else
	Trajectory convertTrajectory(
	    const domain::entities::Trajectory& traj) const;
#endif

	// === 成员变量 ===

	std::shared_ptr<RobotHardware> robot_hw_;
	std::string interface_;  // 硬件接口名称
	size_t num_joints_;      // 关节数量
};

}  // namespace trajectory_planning::infrastructure::adapters
