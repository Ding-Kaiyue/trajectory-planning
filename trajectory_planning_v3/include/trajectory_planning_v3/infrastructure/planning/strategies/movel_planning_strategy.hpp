#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/domain/services/time_optimal_trajectory_generation.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"

namespace trajectory_planning::infrastructure::planning {

class MoveLPlanningStrategy {
public:
    MoveLPlanningStrategy(
        std::shared_ptr<integration::MoveItAdapter> moveit,
        std::shared_ptr<integration::TracIKAdapter> tracik);

    /**
     * @brief 笛卡尔直线规划 - Joint Space 约束版本（使用 TimeOptimalTrajectoryGeneration）
     * @param goal 目标位姿
     * @param sample_alpha 笛卡尔路径采样间隔 (0.0-1.0)，默认 0.02
     * @return 轨迹（满足关节空间约束，末端速度严格为零）
     *
     * 使用 MoveIt 的 TimeOptimalTrajectoryGeneration 算法，保证：
     * - 末端速度为零（硬约束）
     * - 所有关节速度/加速度限制被尊重
     * - 路径曲率项被正确处理
     * - 执行时间在约束下最优
     */
    domain::entities::Trajectory planWithJointConstraints(
        const geometry_msgs::msg::Pose& goal,
        double eef_step = 0.02);

private:
    std::shared_ptr<integration::MoveItAdapter> moveit_;
    std::shared_ptr<integration::TracIKAdapter> tracik_;

    /**
     * @brief 内部辅助：沿笛卡尔直线进行 IK 采样，得到关节空间路径
     * @param start_pose 起始位姿
     * @param goal_pose 目标位姿
     * @param sample_alpha 采样间隔
     * @return 采样得到的关节位置序列
     */
    std::vector<Eigen::VectorXd> sampleCartesianPath(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& goal_pose,
        double cartesian_step = 0.02) const;
	
	/**
	 * @brief 调试用：打印轨迹详细位置信息
	 * @param traj 轨迹对象
	 */
	static void printTrajectoryPositions(const domain::entities::Trajectory& traj) {
		std::cout << "\n========== Position (rad) ==========\n";
		std::cout << std::setw(6) << "Pt" << std::setw(10) << "Time(s)" << std::setw(12) << "Progress(%)";
		if (!traj.points().empty()) {
			for (size_t j = 0; j < traj.points()[0].position.values().size(); ++j) {
				std::cout << std::setw(12) << ("J" + std::to_string(j));
			}
		}
		std::cout << "\n" << std::string(120, '-') << "\n";

		for (size_t i = 0; i < traj.points().size(); ++i) {
			const auto& pt = traj.points()[i];
			const auto& pos = pt.position.values();
			std::cout << std::setw(6) << i
					<< std::setw(10) << std::fixed << std::setprecision(3) << pt.time_from_start.seconds()
					<< std::setw(12) << std::fixed << std::setprecision(1) << (pt.progress_ratio * 100.0);
			for (double p : pos) {
				std::cout << std::setw(12) << std::fixed << std::setprecision(5) << p;
			}
			std::cout << "\n";
		}
	}

	/**
	 * @brief 调试用：打印轨迹详细速度信息
	 * @param traj 轨迹对象
	 */
	static void printTrajectoryVelocities(const domain::entities::Trajectory& traj) {
		std::cout << "\n========== Velocity (rad/s) ==========\n";
		std::cout << std::setw(6) << "Pt" << std::setw(10) << "Time(s)" << std::setw(12) << "Progress(%)";
		if (!traj.points().empty()) {
			for (size_t j = 0; j < traj.points()[0].velocity.values().size(); ++j) {
				std::cout << std::setw(12) << ("J" + std::to_string(j));
			}
		}
		std::cout << "\n" << std::string(120, '-') << "\n";

		for (size_t i = 0; i < traj.points().size(); ++i) {
			const auto& pt = traj.points()[i];
			const auto& vel = pt.velocity.values();
			std::cout << std::setw(6) << i
					<< std::setw(10) << std::fixed << std::setprecision(3) << pt.time_from_start.seconds()
					<< std::setw(12) << std::fixed << std::setprecision(1) << (pt.progress_ratio * 100.0);
			for (double v : vel) {
				std::cout << std::setw(12) << std::fixed << std::setprecision(5) << v;
			}
			std::cout << "\n";
		}
	}

	/**
	 * @brief 调试用：打印轨迹详细加速度信息
	 * @param traj 轨迹对象
	 */
	static void printTrajectoryAccelerations(const domain::entities::Trajectory& traj) {
		std::cout << "\n========== Acceleration (rad/s²) ==========\n";
		std::cout << std::setw(6) << "Pt" << std::setw(10) << "Time(s)" << std::setw(12) << "Progress(%)";
		if (!traj.points().empty()) {
			for (size_t j = 0; j < traj.points()[0].acceleration.values().size(); ++j) {
				std::cout << std::setw(12) << ("J" + std::to_string(j));
			}
		}
		std::cout << "\n" << std::string(120, '-') << "\n";

		for (size_t i = 0; i < traj.points().size(); ++i) {
			const auto& pt = traj.points()[i];
			const auto& acc = pt.acceleration.values();
			std::cout << std::setw(6) << i
					<< std::setw(10) << std::fixed << std::setprecision(3) << pt.time_from_start.seconds()
					<< std::setw(12) << std::fixed << std::setprecision(1) << (pt.progress_ratio * 100.0);
			for (double a : acc) {
				std::cout << std::setw(12) << std::fixed << std::setprecision(5) << a;
			}
			std::cout << "\n";
		}
	}
	
	/**
	 * @brief 调试用：打印轨迹完整摘要信息
	 * @param traj 轨迹对象
	 */
	static void printTrajectory(const domain::entities::Trajectory& traj) {
		std::cout << "\n========== MoveL Trajectory Summary ==========\n";
		std::cout << "Total points: " << traj.points().size() << "\n";

		if (!traj.points().empty()) {
			std::cout << "Duration: " << std::fixed << std::setprecision(3)
					<< traj.points().back().time_from_start.seconds() << " s\n";
		}

		printTrajectoryPositions(traj);
		printTrajectoryVelocities(traj);
		printTrajectoryAccelerations(traj);

		std::cout << "========== End of Trajectory ==========\n\n";
		std::cout.flush();
	}

};

}  // namespace trajectory_planning::infrastructure::planning
