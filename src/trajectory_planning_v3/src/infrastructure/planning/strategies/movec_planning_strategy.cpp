#include "trajectory_planning_v3/infrastructure/planning/strategies/movec_planning_strategy.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <iostream>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "trajectory_planning_v3/domain/value_objects/duration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"

namespace trajectory_planning::infrastructure::planning {

domain::entities::Trajectory MoveCPlanningStrategy::planArc(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& via_point,
    const geometry_msgs::msg::Pose& goal_pose) {
	// 四元数预处理（与贝塞尔曲线相同）
	tf2::Quaternion start_q, via_q, goal_q;
	tf2::fromMsg(start_pose.orientation, start_q);
	tf2::fromMsg(via_point.orientation, via_q);
	tf2::fromMsg(goal_pose.orientation, goal_q);

	if (start_q.dot(via_q) < 0) via_q *= -1;
	if (start_q.dot(goal_q) < 0) goal_q *= -1;

	// 三点坐标
	const double x1 = start_pose.position.x, y1 = start_pose.position.y;
	const double x2 = via_point.position.x, y2 = via_point.position.y;
	const double x3 = goal_pose.position.x, y3 = goal_pose.position.y;

	// 计算圆心坐标
	const double d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

	// 检查是否共线，如果共线则生成直线轨迹
	if (std::fabs(d) < 1e-6) {
		RCLCPP_INFO(
		    rclcpp::get_logger("MoveCPlanningStrategy"),
		    "Points are nearly collinear, generating linear trajectory");

		const int num_points = 50;
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.reserve(num_points + 1);

		for (int i = 0; i <= num_points; ++i) {
			const double t = static_cast<double>(i) / num_points;

			geometry_msgs::msg::Pose p;
			// 线性插值位置
			p.position.x = start_pose.position.x +
			               t * (goal_pose.position.x - start_pose.position.x);
			p.position.y = start_pose.position.y +
			               t * (goal_pose.position.y - start_pose.position.y);
			p.position.z = start_pose.position.z +
			               t * (goal_pose.position.z - start_pose.position.z);

			// 姿态球面插值（使用预处理的四元数）
			tf2::Quaternion q_i = start_q.slerp(goal_q, t);
			p.orientation = tf2::toMsg(q_i);

			waypoints.push_back(p);
		}

		// 调用 MoveIt 笛卡尔路径规划
		moveit_msgs::msg::RobotTrajectory moveit_traj;
		bool success =
		    moveit_adapter_.planCartesianPath(waypoints, moveit_traj);

		if (!success) {
			RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
			             "Failed to plan linear trajectory");
			return {};
		}
		return convertTrajectoryType(moveit_traj);
	}

	const double cx =
	    ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
	     (x3 * x3 + y3 * y3) * (y1 - y2)) /
	    d;
	const double cy =
	    ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
	     (x3 * x3 + y3 * y3) * (x2 - x1)) /
	    d;

	const double r = std::hypot(x1 - cx, y1 - cy);

	// 角度
	auto ang = [&](double x, double y) { return std::atan2(y - cy, x - cx); };
	double a_start = ang(x1, y1);
	double a_via = ang(x2, y2);
	double a_goal = ang(x3, y3);

	// 规范化到 [0, 2π)
	auto norm = [](double a) {
		while (a < 0) a += 2 * M_PI;
		while (a >= 2 * M_PI) a -= 2 * M_PI;
		return a;
	};
	a_start = norm(a_start);
	a_via = norm(a_via);
	a_goal = norm(a_goal);

	// 确保经过 via：判断 goal 是否在 start->goal 弧段上
	auto is_between = [](double s, double m, double e) {
		if (e < s) e += 2 * M_PI;
		if (m < s) m += 2 * M_PI;
		return (m >= s && m <= e);
	};

	if (!is_between(a_start, a_via, a_goal)) {
		// 如果 via 不在 start->goal 上，反向 goal
		if (a_goal > a_start)
			a_goal -= 2 * M_PI;
		else
			a_goal += 2 * M_PI;
	}

	const double delta = a_goal - a_start;

	// 采样
	const int num_points = 80;  // 更密的弧线
	std::vector<geometry_msgs::msg::Pose> waypoints;
	waypoints.reserve(num_points + 1);

	// 使用已经预处理的四元数

	for (int i = 0; i <= num_points; ++i) {
		const double t = static_cast<double>(i) / num_points;
		const double ang_i = a_start + t * delta;

		geometry_msgs::msg::Pose p;
		p.position.x = cx + r * std::cos(ang_i);
		p.position.y = cy + r * std::sin(ang_i);
		// z 线性插值
		p.position.z = start_pose.position.z +
		               t * (goal_pose.position.z - start_pose.position.z);

		// 姿态球面插值（使用预处理的四元数）
		tf2::Quaternion q_i = start_q.slerp(goal_q, t);
		p.orientation = tf2::toMsg(q_i);

		waypoints.push_back(p);
	}

	// 调用 MoveIt 笛卡尔路径规划
	moveit_msgs::msg::RobotTrajectory moveit_traj;
	// const double eef_step = 0.002;   // 2mm
	// const double jump_thr = 0.0;     // 不使用跳跃阈值

	bool success = moveit_adapter_.planCartesianPath(waypoints, moveit_traj);

	if (!success) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to plan arc trajectory");
		return {};
	}
	return convertTrajectoryType(moveit_traj);
}

domain::entities::Trajectory MoveCPlanningStrategy::planBezier(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& ctrl1,
    const geometry_msgs::msg::Pose& ctrl2,
    const geometry_msgs::msg::Pose& goal) {
	std::vector<geometry_msgs::msg::Pose> bezier_waypoints;
	const int num_points = 20;  // 可配置的采样点数量

	for (int i = 0; i <= num_points; ++i) {
		double t = static_cast<double>(i) / num_points;
		double t2 = t * t;
		double t3 = t2 * t;

		// 三次贝塞尔曲线公式: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
		double b0 = (1 - t) * (1 - t) * (1 - t);
		double b1 = 3 * (1 - t) * (1 - t) * t;
		double b2 = 3 * (1 - t) * t * t;
		double b3 = t3;

		geometry_msgs::msg::Pose interpolated_pose;

		// 位置插值
		interpolated_pose.position.x =
		    b0 * start.position.x + b1 * ctrl1.position.x +
		    b2 * ctrl2.position.x + b3 * goal.position.x;
		interpolated_pose.position.y =
		    b0 * start.position.y + b1 * ctrl1.position.y +
		    b2 * ctrl2.position.y + b3 * goal.position.y;
		interpolated_pose.position.z =
		    b0 * start.position.z + b1 * ctrl1.position.z +
		    b2 * ctrl2.position.z + b3 * goal.position.z;

		// 姿态使用球面线性插值 (简化为线性插值)
		interpolated_pose.orientation.x =
		    b0 * start.orientation.x + b1 * ctrl1.orientation.x +
		    b2 * ctrl2.orientation.x + b3 * goal.orientation.x;
		interpolated_pose.orientation.y =
		    b0 * start.orientation.y + b1 * ctrl1.orientation.y +
		    b2 * ctrl2.orientation.y + b3 * goal.orientation.y;
		interpolated_pose.orientation.z =
		    b0 * start.orientation.z + b1 * ctrl1.orientation.z +
		    b2 * ctrl2.orientation.z + b3 * goal.orientation.z;
		interpolated_pose.orientation.w =
		    b0 * start.orientation.w + b1 * ctrl1.orientation.w +
		    b2 * ctrl2.orientation.w + b3 * goal.orientation.w;

		bezier_waypoints.push_back(interpolated_pose);
	}

	// 使用MoveIt进行笛卡尔路径规划
	moveit_msgs::msg::RobotTrajectory moveit_traj;
	bool success =
	    moveit_adapter_.planCartesianPath(bezier_waypoints, moveit_traj);

	if (!success) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to plan Bezier curve trajectory");
		return domain::entities::Trajectory{};
	}

	return convertTrajectoryType(moveit_traj);
}

domain::entities::Trajectory MoveCPlanningStrategy::planCircle(
    const geometry_msgs::msg::Pose& center,
    const geometry_msgs::msg::Pose& radius_point) {
	std::vector<geometry_msgs::msg::Pose> circle_waypoints;
	const int num_points = 36;  // 36个点形成完整圆

	double radius =
	    std::sqrt(std::pow(radius_point.position.x - center.position.x, 2) +
	              std::pow(radius_point.position.y - center.position.y, 2));

	for (int i = 0; i <= num_points; ++i) {
		double angle = 2 * M_PI * i / num_points;

		geometry_msgs::msg::Pose waypoint;
		waypoint.position.x = center.position.x + radius * std::cos(angle);
		waypoint.position.y = center.position.y + radius * std::sin(angle);
		waypoint.position.z = center.position.z;  // 保持Z轴不变

		// 保持姿态不变
		waypoint.orientation = center.orientation;

		circle_waypoints.push_back(waypoint);
	}

	moveit_msgs::msg::RobotTrajectory moveit_traj;
	bool success =
	    moveit_adapter_.planCartesianPath(circle_waypoints, moveit_traj);

	if (!success) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to plan circle trajectory");
		return domain::entities::Trajectory{};
	}

	return convertTrajectoryType(moveit_traj);
}

domain::entities::Trajectory MoveCPlanningStrategy::planCircleThrough3Points(
    const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2,
    const geometry_msgs::msg::Pose& p3) {
	// 通过三点计算圆心
	double x1 = p1.position.x, y1 = p1.position.y;
	double x2 = p2.position.x, y2 = p2.position.y;
	double x3 = p3.position.x, y3 = p3.position.y;

	// 计算圆心坐标
	double d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

	if (std::abs(d) < 1e-6) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Three points are collinear, cannot form a circle");
		return domain::entities::Trajectory{};
	}

	double ux =
	    ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
	     (x3 * x3 + y3 * y3) * (y1 - y2)) /
	    d;
	double uy =
	    ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
	     (x3 * x3 + y3 * y3) * (x2 - x1)) /
	    d;

	geometry_msgs::msg::Pose center;
	center.position.x = ux;
	center.position.y = uy;
	center.position.z =
	    (p1.position.z + p2.position.z + p3.position.z) / 3.0;  // 取平均Z值
	center.orientation = p1.orientation;  // 使用第一个点的姿态

	// 计算起始角度和结束角度
	double start_angle = std::atan2(y1 - uy, x1 - ux);
	double end_angle = std::atan2(y3 - uy, x3 - ux);

	// 确保通过中间点
	double mid_angle = std::atan2(y2 - uy, x2 - ux);

	// 调整角度顺序
	if (end_angle < start_angle) {
		end_angle += 2 * M_PI;
	}
	if (mid_angle < start_angle) {
		mid_angle += 2 * M_PI;
	}

	std::vector<geometry_msgs::msg::Pose> circle_waypoints;
	const int num_points = 20;
	double radius = std::sqrt((x1 - ux) * (x1 - ux) + (y1 - uy) * (y1 - uy));

	for (int i = 0; i <= num_points; ++i) {
		double t = static_cast<double>(i) / num_points;
		double angle = start_angle + t * (end_angle - start_angle);

		geometry_msgs::msg::Pose waypoint;
		waypoint.position.x = ux + radius * std::cos(angle);
		waypoint.position.y = uy + radius * std::sin(angle);
		waypoint.position.z =
		    center.position.z + t * (p3.position.z - p1.position.z);

		// 线性插值姿态
		waypoint.orientation.x =
		    (1 - t) * p1.orientation.x + t * p3.orientation.x;
		waypoint.orientation.y =
		    (1 - t) * p1.orientation.y + t * p3.orientation.y;
		waypoint.orientation.z =
		    (1 - t) * p1.orientation.z + t * p3.orientation.z;
		waypoint.orientation.w =
		    (1 - t) * p1.orientation.w + t * p3.orientation.w;

		circle_waypoints.push_back(waypoint);
	}

	moveit_msgs::msg::RobotTrajectory moveit_traj;
	bool success =
	    moveit_adapter_.planCartesianPath(circle_waypoints, moveit_traj);

	if (!success) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to plan circle through 3 points trajectory");
		return domain::entities::Trajectory{};
	}

	return convertTrajectoryType(moveit_traj);
}

domain::entities::Trajectory MoveCPlanningStrategy::convertTrajectoryType(
    const moveit_msgs::msg::RobotTrajectory& moveit_traj) const {
	domain::entities::Trajectory trajectory;

	// 将 MoveIt trajectory 转换为我们的 Trajectory 对象
	for (const auto& point : moveit_traj.joint_trajectory.points) {
		domain::entities::TrajectoryPoint traj_point{
		    .position = domain::value_objects::JointPosition(point.positions),
		    .velocity = domain::value_objects::JointVelocity(
		        point.velocities.empty()
		            ? std::vector<double>(point.positions.size(), 0.0)
		            : point.velocities),
		    .acceleration = domain::value_objects::JointAcceleration(
		        point.accelerations.empty()
		            ? std::vector<double>(point.positions.size(), 0.0)
		            : point.accelerations),
		    .time_from_start = domain::value_objects::Duration(
		        static_cast<double>(point.time_from_start.sec) +
		        static_cast<double>(point.time_from_start.nanosec) * 1e-9),
		    .progress_ratio = 0.0};

		trajectory.add_point(traj_point);
	}

	// 计算 progress_ratio
	trajectory.compute_progress_ratios();

	return trajectory;
}

}  // namespace trajectory_planning::infrastructure::planning