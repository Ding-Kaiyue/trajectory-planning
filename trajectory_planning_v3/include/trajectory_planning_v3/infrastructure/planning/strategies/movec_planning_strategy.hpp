#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"

namespace trajectory_planning::infrastructure::planning {

/**
 * @brief MoveC 规划策略
 *
 * 将目标末端位姿生成连续的圆弧轨迹 (Trajectroy)。
 * 可以通过设置最大速度和加速度生成平滑轨迹。
 */
class MoveCPlanningStrategy {
public:
	/**
	 * @brief MoveC 的圆弧类路线
	 */
	enum class MoveCRoute {
		// Bezier,     // 贝塞尔曲线
		ARC,     // 圆弧
		CIRCLE,  // 整圆轨迹
		// Helix,      // 螺旋轨迹
		// Spline,     // 样条曲线
		// LineBlend,  // 直线 + 圆弧段混合
		CIRCLETHROUGH3POINTS,  // 通过三个点的圆轨迹
	};

	MoveCPlanningStrategy(
	    std::shared_ptr<integration::MoveItAdapter> moveit,
	    std::shared_ptr<integration::TracIKAdapter> tracik)
	    : moveit_(moveit),
	      tracik_(tracik) {}

	/**
	 * @brief 规划经过中间点的圆弧轨迹
	 * @param via_point  中间经过点位姿
	 * @param goal_pose  目标位姿
	 * @param arm_type 机械臂类型（用于加载对应的关节限制）
	 * @return 生成的Trajectory对象
	 */
	domain::entities::Trajectory planArc(
	    const geometry_msgs::msg::Pose& via_point,
	    const geometry_msgs::msg::Pose& goal_pose,
	    const std::string& arm_type = "arm620");

	/**
	 * @brief 规划整圆轨迹 (CIRCLE模式: center为圆心, goal定义半径)
	 * @param center       圆心位姿
	 * @param radius_point 定义半径的点位姿
	 * @param arm_type 机械臂类型（用于加载对应的关节限制）
	 * @return 生成的Trajectory对象
	 */
	domain::entities::Trajectory planCircle(
	    const geometry_msgs::msg::Pose& center,
	    const geometry_msgs::msg::Pose& radius_point,
	    const std::string& arm_type = "arm620");

	/**
	 * @brief 规划通过三点的圆弧轨迹
	 * @param point1 第一个点位姿
	 * @param point2 第二个点位姿
	 * @param point3 第三个点位姿
	 * @return 生成的Trajectory对象
	 */
	domain::entities::Trajectory planCircleThrough3Points(
	    const geometry_msgs::msg::Pose& point1,
	    const geometry_msgs::msg::Pose& point2,
	    const geometry_msgs::msg::Pose& point3);

	/**
	 * @brief 规划笛卡尔空间贝塞尔曲线
	 * @param start   起点位姿
	 * @param ctrl1   中间点1
	 * @param ctrl2   中间点2
	 * @param goal    目标位姿
	 * @param arm_type 机械臂类型（用于加载对应的关节限制）
	 * @return 生成的Trajectory对象
	 */
	domain::entities::Trajectory planBezier(
	    const geometry_msgs::msg::Pose& start,
	    const geometry_msgs::msg::Pose& ctrl1,
	    const geometry_msgs::msg::Pose& ctrl2,
	    const geometry_msgs::msg::Pose& goal,
	    const std::string& arm_type = "arm620");

private:
	std::shared_ptr<integration::MoveItAdapter> moveit_;
	std::shared_ptr<integration::TracIKAdapter> tracik_;

	/**
	 * @brief 圆弧路径笛卡尔采样（包含关节跳跃检测）
	 * @param start_pose 起点位姿
	 * @param via_point 中间点位姿
	 * @param goal_pose 目标位姿
	 * @param arm_type 机械臂类型（用于加载对应的关节限制）
	 * @param cartesian_step 笛卡尔采样步长
	 * @return 关节空间路径
	 */
	std::vector<Eigen::VectorXd> sampleArcCartesianPath(
	    const geometry_msgs::msg::Pose& start_pose,
	    const geometry_msgs::msg::Pose& via_point,
	    const geometry_msgs::msg::Pose& goal_pose,
	    const std::string& arm_type = "arm620",
	    double cartesian_step = 0.02) const;

	/**
	 * @brief 贝塞尔曲线路径笛卡尔采样（包含关节跳跃检测）
	 * @param start 起点位姿
	 * @param ctrl1 控制点1位姿
	 * @param ctrl2 控制点2位姿
	 * @param goal 目标位姿
	 * @param arm_type 机械臂类型（用于加载对应的关节限制）
	 * @param cartesian_step 笛卡尔采样步长
	 * @return 关节空间路径
	 */
	std::vector<Eigen::VectorXd> sampleBezierCartesianPath(
	    const geometry_msgs::msg::Pose& start,
	    const geometry_msgs::msg::Pose& ctrl1,
	    const geometry_msgs::msg::Pose& ctrl2,
	    const geometry_msgs::msg::Pose& goal,
	    const std::string& arm_type = "arm620",
	    double cartesian_step = 0.02) const;

	/**
	 * @brief 圆形路径笛卡尔采样（包含关节跳跃检测）
	 * @param waypoints 圆形路径上的笛卡尔路径点
	 * @param arm_type 机械臂类型（用于加载对应的关节限制）
	 * @param cartesian_step 笛卡尔采样步长
	 * @return 关节空间路径
	 */
	std::vector<Eigen::VectorXd> sampleCircleCartesianPath(
	    const std::vector<geometry_msgs::msg::Pose>& waypoints,
	    const std::string& arm_type = "arm620",
	    double cartesian_step = 0.02) const;

	/**
	 * @brief 根据路径长度动态计算采样点数
	 * @param path_length 路径总长度（单位：米）
	 * @param min_points 最小采样点数（默认10）
	 * @param sampling_interval 采样间距（默认0.01m，即10mm）
	 * @return 计算得到的采样点数
	 */
	int calculateNumPoints(double path_length, int min_points = 10,
	                        double sampling_interval = 0.01) const;
						
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