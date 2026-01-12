#include "trajectory_planning_v3/infrastructure/planning/strategies/movec_planning_strategy.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <iostream>
#include <iomanip>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "trajectory_planning_v3/domain/services/time_optimal_trajectory_generation.hpp"

namespace trajectory_planning::infrastructure::planning {

// 辅助函数：四元数球面线性插值
static geometry_msgs::msg::Quaternion slerp(
    const geometry_msgs::msg::Quaternion& q0,
    const geometry_msgs::msg::Quaternion& q1,
    double s)
{
    tf2::Quaternion a(q0.x, q0.y, q0.z, q0.w);
    tf2::Quaternion b(q1.x, q1.y, q1.z, q1.w);

    if (a.dot(b) < 0.0) {
        b = tf2::Quaternion(-b.x(), -b.y(), -b.z(), -b.w());
    }

    tf2::Quaternion q = a.slerp(b, s);
    q.normalize();

    geometry_msgs::msg::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
}

// 辅助函数：关节角度包装到最近值
inline double wrapToNearest(double q, double q_ref)
{
    double dq = q - q_ref;
    while (dq > M_PI)  dq -= 2.0 * M_PI;
    while (dq < -M_PI) dq += 2.0 * M_PI;
    return q_ref + dq;
}

domain::entities::Trajectory MoveCPlanningStrategy::planArc(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    const geometry_msgs::msg::Pose& via_point) {
	
	domain::entities::Trajectory traj;
	
	// 重新加载缩放参数（支持动态参数更新）
	moveit_->loadScalingParameters();

	/* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
	std::vector<Eigen::VectorXd> q_path =
		sampleArcCartesianPath(start_pose, via_point, goal_pose);

	if (q_path.size() < 3) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to sample arc cartesian path");
		return traj;
	}

	/* -----------------------------
    * 2. Get scaling factors (NOT limits)
    * ----------------------------- */	
	double velocity_scaling = moveit_->getVelocityScalingFactor();
	double acceleration_scaling = moveit_->getAccelerationScalingFactor();

	auto robot_model = moveit_->getRobotModel();
	if (!robot_model) 
		return traj;

	/* -----------------------------
    * 3. Time-optimal parameterization
    * ----------------------------- */
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		"arm",   // TODO: make configurable
		velocity_scaling,
		acceleration_scaling);

	traj = totg.compute(q_path);

	if (traj.points().empty()) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveC"),
		             "TOTG failed for arc planning");
		return traj;
	}

	return traj;
}

domain::entities::Trajectory MoveCPlanningStrategy::planBezier(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& ctrl1,
    const geometry_msgs::msg::Pose& ctrl2,
    const geometry_msgs::msg::Pose& goal) {
	
	domain::entities::Trajectory traj;

    // 重新加载缩放参数（支持动态参数更新）
	moveit_->loadScalingParameters();

	/* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
	std::vector<Eigen::VectorXd> q_path = 
		sampleBezierCartesianPath(start, ctrl1, ctrl2, goal);

	if (q_path.size() < 3) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to sample bezier cartesian path");
		return traj;
	}

	/* -----------------------------
    * 2. Get scaling factors (NOT limits)
    * ----------------------------- */
	double velocity_scaling = moveit_->getVelocityScalingFactor();
	double acceleration_scaling = moveit_->getAccelerationScalingFactor();

	auto robot_model = moveit_->getRobotModel();
	if (!robot_model) 
		return traj;

	/* -----------------------------
    * 3. Time-optimal parameterization
    * ----------------------------- */
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		"arm",
		velocity_scaling,
		acceleration_scaling);

	traj = totg.compute(q_path);

	if (traj.points().empty()) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "TOTG failed for bezier planning");
		return traj;
	}

	printTrajectory(traj);
	return traj;
}

domain::entities::Trajectory MoveCPlanningStrategy::planCircle(
    const geometry_msgs::msg::Pose& center,
    const geometry_msgs::msg::Pose& radius_point) {
	
	domain::entities::Trajectory traj;

	// 重新加载缩放参数（支持动态参数更新）
	moveit_->loadScalingParameters();

	/* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
	double radius = std::sqrt(
	    std::pow(radius_point.position.x - center.position.x, 2) +
	    std::pow(radius_point.position.y - center.position.y, 2));

	double circle_length = 2 * M_PI * radius;
	int num_points = calculateNumPoints(circle_length);

	std::vector<geometry_msgs::msg::Pose> waypoints;
	waypoints.reserve(num_points + 1);

	for (int i = 0; i <= num_points; ++i) {
		double angle = 2 * M_PI * i / num_points;
		geometry_msgs::msg::Pose p;
		p.position.x = center.position.x + radius * std::cos(angle);
		p.position.y = center.position.y + radius * std::sin(angle);
		p.position.z = center.position.z;
		p.orientation = center.orientation;
		waypoints.push_back(p);
	}

	// 使用高级采样方法获得关节空间路径（包含关节跳跃检测）
	std::vector<Eigen::VectorXd> q_path = sampleCircleCartesianPath(waypoints, 0.01);

	if (q_path.size() < 3) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to sample circle cartesian path");
		return traj;
	}

	/* -----------------------------
    * 2. Get scaling factors (NOT limits)
    * ----------------------------- */
	double velocity_scaling = moveit_->getVelocityScalingFactor();
	double acceleration_scaling = moveit_->getAccelerationScalingFactor();

	auto robot_model = moveit_->getRobotModel();
	if (!robot_model)
		return traj;

	/* -----------------------------
    * 3. Time-optimal parameterization
    * ----------------------------- */
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		"arm",
		velocity_scaling,
		acceleration_scaling);

	traj = totg.compute(q_path);

	if (traj.points().empty()) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "TOTG failed for circle planning");
		return traj;
	}

	printTrajectory(traj);
	return traj;
}

domain::entities::Trajectory MoveCPlanningStrategy::planCircleThrough3Points(
    const geometry_msgs::msg::Pose& p1,
    const geometry_msgs::msg::Pose& p2,
    const geometry_msgs::msg::Pose& p3) {
	
	domain::entities::Trajectory traj;

	// 重新加载缩放参数（支持动态参数更新）
    moveit_->loadScalingParameters();

	/* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
	std::vector<Eigen::VectorXd> q_path = 
		sampleArcCartesianPath(p1, p2, p3);

	if (q_path.size() < 3) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "Failed to sample circle through 3 points cartesian path");
		return traj;
	}

	/* -----------------------------
    * 2. Get scaling factors (NOT limits)
    * ----------------------------- */
	double velocity_scaling = moveit_->getVelocityScalingFactor();
	double acceleration_scaling = moveit_->getAccelerationScalingFactor();

	auto robot_model = moveit_->getRobotModel();
	if (!robot_model) 
		return traj;

	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		"arm",
		velocity_scaling,
		acceleration_scaling);

	traj = totg.compute(q_path);

	if (traj.points().empty()) {
		RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
		             "TOTG failed for circle through 3 points planning");
		return traj;
	}

	printTrajectory(traj);
	return traj;
}

int MoveCPlanningStrategy::calculateNumPoints(double path_length, int min_points,
                                              double sampling_interval) const {
	if (path_length < 1e-6) return min_points;

	int calculated_points = static_cast<int>(std::ceil(path_length / sampling_interval));
	int num_points = std::max(min_points, calculated_points);
	const int max_points = 100;
	return std::min(num_points, max_points);
}

std::vector<Eigen::VectorXd>
MoveCPlanningStrategy::sampleArcCartesianPath(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& via_point,
    const geometry_msgs::msg::Pose& goal_pose,
    double cartesian_step) const
{
    std::vector<Eigen::VectorXd> q_path;

    /* ============================================================
     * 1. 计算圆弧几何参数
     * ============================================================ */
    // 三点坐标
    const double x1 = start_pose.position.x, y1 = start_pose.position.y;
    const double x2 = via_point.position.x, y2 = via_point.position.y;
    const double x3 = goal_pose.position.x, y3 = goal_pose.position.y;

    // 计算圆心坐标
    const double d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    // 检查是否共线，如果共线则退化为直线
    // 使用更严格的阈值，避免L形轨迹被误判为共线
    if (std::fabs(d) < 1e-9) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Arc points are collinear, falling back to linear path");

        Eigen::Vector3d p0(start_pose.position.x, start_pose.position.y, start_pose.position.z);
        Eigen::Vector3d p1(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
        Eigen::Vector3d dp = p1 - p0;
        const double length = dp.norm();

        if (length < 1e-6) return q_path;

        size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(length / cartesian_step)));

        // 初始种子（拓扑锁定）
        Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
            moveit_->getCurrentJointState().data(),
            moveit_->getCurrentJointState().size());

        q_path.reserve(steps + 1);
        q_path.push_back(q_prev);

        // ABB / KUKA 阈值
        const double SOFT_JUMP = 0.35;   // rad  → SingArea
        const double HARD_JUMP = 1.20;   // rad  → topology break
        bool in_sing_area = false;
        bool ik_failure = false;

        // 使用直线插值（包含关节跳跃检测）
        for (size_t i = 1; i <= steps; ++i) {
            double s = std::min(1.0, double(i) / steps);
            geometry_msgs::msg::Pose pose;
            pose.position.x = p0.x() + s * dp.x();
            pose.position.y = p0.y() + s * dp.y();
            pose.position.z = p0.z() + s * dp.z();
            pose.orientation = slerp(start_pose.orientation, goal_pose.orientation, s);

            std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
            std::vector<double> q_raw;

            if (!tracik_->computeIKClosest(pose, seed, q_raw, 10)) {  // 增加到10次重试
                RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                            "❎ IK failed at linear fallback s=%.3f - aborting", s);
                ik_failure = true;
                break;
            }

            Eigen::VectorXd q(q_raw.size());

            // 关节包装（ABB风格）
            for (int j = 0; j < q.size(); ++j)
                q[j] = wrapToNearest(q_raw[j], q_prev[j]);

            // 软/硬关节跳跃评估
            for (int j = 0; j < q.size(); ++j) {
                double dq = std::abs(q[j] - q_prev[j]);

                if (dq > HARD_JUMP) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Hard joint jump %.3f rad at joint %d (linear fallback s=%.3f), abort",
                        dq, j, s);
                    goto EXIT_LINEAR_FALLBACK;
                }

                if (dq > SOFT_JUMP && !in_sing_area) {
                    in_sing_area = true;
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Entering SingArea at linear fallback s=%.3f (joint %d, dq=%.3f)",
                        s, j, dq);
                }
            }

            q_path.push_back(q);
            q_prev = q;
        }

EXIT_LINEAR_FALLBACK:

        // 如果IK失败，立即返回空轨迹
        if (ik_failure) {
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "❎ Linear fallback aborted due to IK failure - returning empty trajectory");
            return std::vector<Eigen::VectorXd>{};
        }

        // 强制精确目标位姿IK（仅在采样成功收集足够点数时执行）
        if (q_path.size() >= 3) {
            std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
            std::vector<double> q_goal;

            if (tracik_->computeIKClosest(goal_pose, seed, q_goal, 10)) {  // 增加到10次重试
                Eigen::VectorXd qg(q_goal.size());
                for (size_t j = 0; j < q_goal.size(); ++j)
                    qg[j] = wrapToNearest(q_goal[j], q_prev[j]);

                q_path.back() = qg;
            }
        }

        // 最终检查：采样必须成功收集至少3个点
        if (q_path.size() < 3) {
            RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Linear fallback produced insufficient points (%zu < 3)", q_path.size());
            q_path.clear();
        }
        return q_path;
    }

    // 计算圆心
    const double cx = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                        (x3 * x3 + y3 * y3) * (y1 - y2)) / d;
    const double cy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                        (x3 * x3 + y3 * y3) * (x2 - x1)) / d;
    const double r = std::hypot(x1 - cx, y1 - cy);

    // 计算角度
    auto ang = [&](double x, double y) { return std::atan2(y - cy, x - cx); };
    double a_start = ang(x1, y1);
    double a_via = ang(x2, y2);
    double a_goal = ang(x3, y3);

    // 规范化角度并确保通过via点
    auto norm = [](double a) {
        while (a < 0) a += 2 * M_PI;
        while (a >= 2 * M_PI) a -= 2 * M_PI;
        return a;
    };
    a_start = norm(a_start);
    a_via = norm(a_via);
    a_goal = norm(a_goal);

    // 确保通过via点的路径选择
    auto is_between = [](double s, double m, double e) {
        if (e < s) e += 2 * M_PI;
        if (m < s) m += 2 * M_PI;
        return (m >= s && m <= e);
    };

    if (!is_between(a_start, a_via, a_goal)) {
        if (a_goal > a_start) a_goal -= 2 * M_PI;
        else a_goal += 2 * M_PI;
    }

    const double delta = a_goal - a_start;
    double arc_length = std::abs(delta) * r;
    double total_length = std::sqrt(
        arc_length * arc_length +
        std::pow(goal_pose.position.z - start_pose.position.z, 2));

    size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(total_length / cartesian_step)));

    /* ============================================================
     * 2. 初始种子（拓扑锁定）
     * ============================================================ */
    Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
        moveit_->getCurrentJointState().data(),
        moveit_->getCurrentJointState().size());

    q_path.reserve(steps + 1);

    // 将起始关节配置加入路径
    q_path.push_back(q_prev);

    /* ============================================================
     * 3. ABB / KUKA 阈值
     * ============================================================ */
    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;

    /* ============================================================
     * 4. 圆弧采样 + IK
     * ============================================================ */
    // 第一阶段：生成所有笛卡尔采样点（纯几何，不涉及IK）
    std::vector<geometry_msgs::msg::Pose> cartesian_poses;
    cartesian_poses.reserve(steps + 1);

    // 加入起始点
    cartesian_poses.push_back(start_pose);

    for (size_t i = 1; i <= steps; ++i) {
        double t = std::min(1.0, double(i) / steps);
        double ang_i = a_start + t * delta;

        geometry_msgs::msg::Pose pose;
        pose.position.x = cx + r * std::cos(ang_i);
        pose.position.y = cy + r * std::sin(ang_i);
        pose.position.z = start_pose.position.z + t * (goal_pose.position.z - start_pose.position.z);
        pose.orientation = slerp(start_pose.orientation, goal_pose.orientation, t);

        cartesian_poses.push_back(pose);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("MoveCPlanningStrategy"),
                "Phase 1 complete: Generated %zu cartesian poses", cartesian_poses.size());

    /* ============================================================
     * 5. 第二阶段：对笛卡尔点进行IK求解
     * ============================================================ */
    bool ik_failure = false;
    int failed_count = 0;
    for (size_t i = 0; i < cartesian_poses.size(); ++i) {
        const auto& pose = cartesian_poses[i];
        double t = (i == 0) ? 0.0 : static_cast<double>(i - 1) / steps;

        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_raw;

        if (!tracik_->computeIKClosest(pose, seed, q_raw)) {
            failed_count++;
            RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "⚠️ IK failed for cartesian pose %zu/%zu (progress: %.1f%%)",
                        i, cartesian_poses.size(), (100.0 * i / cartesian_poses.size()));
            // IK失败，中止规划
            ik_failure = true;
            break;
        }

        Eigen::VectorXd q(q_raw.size());

        /* ========================================================
         * 5. 关节包装（ABB风格）
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j)
            q[j] = wrapToNearest(q_raw[j], q_prev[j]);

        /* ========================================================
         * 6. 软/硬关节跳跃评估
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j) {
            double dq = std::abs(q[j] - q_prev[j]);

            if (dq > HARD_JUMP) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Hard joint jump %.3f rad at joint %d (t=%.3f), abort",
                    dq, j, t);
                goto EXIT_ARC_SAMPLING;
            }

            if (dq > SOFT_JUMP && !in_sing_area) {
                in_sing_area = true;
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Entering SingArea at t=%.3f (joint %d, dq=%.3f)",
                    t, j, dq);
            }
        }

        q_path.push_back(q);
        q_prev = q;
    }

EXIT_ARC_SAMPLING:

    // 如果IK失败，立即返回空轨迹
    if (ik_failure) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "❎ Arc planning aborted due to IK failure - returning empty trajectory");
        return std::vector<Eigen::VectorXd>{};
    }

    /* ============================================================
     * 7. 强制精确目标位姿IK（ABB行为）
     * 仅在采样成功收集足够点数时执行
     * ============================================================ */
    // 只有采样成功得到3个或更多点时，才执行精确目标IK覆盖
    if (q_path.size() >= 3) {
        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_goal;

        if (tracik_->computeIKClosest(goal_pose, seed, q_goal)) {
            Eigen::VectorXd qg(q_goal.size());
            for (size_t j = 0; j < q_goal.size(); ++j)
                qg[j] = wrapToNearest(q_goal[j], q_prev[j]);

            q_path.back() = qg;
        }
    }

    /* ============================================================
     * 8. 最终检查：采样必须成功收集至少3个点
     * ============================================================ */
    if (q_path.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Arc sampling produced insufficient points (%zu < 3)", q_path.size());
        q_path.clear();
    }

    return q_path;
}

std::vector<Eigen::VectorXd>
MoveCPlanningStrategy::sampleBezierCartesianPath(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& ctrl1,
    const geometry_msgs::msg::Pose& ctrl2,
    const geometry_msgs::msg::Pose& goal,
    double cartesian_step) const
{
    std::vector<Eigen::VectorXd> q_path;

    if (cartesian_step <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "cartesian_step must be > 0");
        return q_path;
    }

    /* ============================================================
     * 1. 估算贝塞尔曲线长度
     * ============================================================ */
    double length_approx = 0.0;
    std::vector<const geometry_msgs::msg::Pose*> points = {&start, &ctrl1, &ctrl2, &goal};
    for (size_t i = 1; i < points.size(); ++i) {
        double dx = points[i]->position.x - points[i-1]->position.x;
        double dy = points[i]->position.y - points[i-1]->position.y;
        double dz = points[i]->position.z - points[i-1]->position.z;
        length_approx += std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    if (length_approx < 1e-6) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Bezier curve has zero length");
        return q_path;
    }

    size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(length_approx / cartesian_step)));

    /* ============================================================
     * 2. 初始种子（拓扑锁定）
     * ============================================================ */
    Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
        moveit_->getCurrentJointState().data(),
        moveit_->getCurrentJointState().size());

    q_path.reserve(steps + 1);

    // 将起始关节配置加入路径
    q_path.push_back(q_prev);

    /* ============================================================
     * 3. ABB / KUKA 阈值
     * ============================================================ */
    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;

    /* ============================================================
     * 4. 贝塞尔曲线采样 + IK
     * ============================================================ */
    bool ik_failure = false;
    for (size_t i = 1; i <= steps; ++i) {
        double t = std::min(1.0, double(i) / steps);
        double t2 = t * t, t3 = t2 * t;

        // 三次贝塞尔曲线基函数
        double b0 = (1 - t) * (1 - t) * (1 - t);
        double b1 = 3 * (1 - t) * (1 - t) * t;
        double b2 = 3 * (1 - t) * t * t;
        double b3 = t3;

        geometry_msgs::msg::Pose pose;

        // 位置插值
        pose.position.x = b0 * start.position.x + b1 * ctrl1.position.x +
                         b2 * ctrl2.position.x + b3 * goal.position.x;
        pose.position.y = b0 * start.position.y + b1 * ctrl1.position.y +
                         b2 * ctrl2.position.y + b3 * goal.position.y;
        pose.position.z = b0 * start.position.z + b1 * ctrl1.position.z +
                         b2 * ctrl2.position.z + b3 * goal.position.z;

        // 姿态插值（使用slerp而不是线性插值以避免四元数问题）
        pose.orientation = slerp(start.orientation, goal.orientation, t);

        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_raw;

        if (!tracik_->computeIKClosest(pose, seed, q_raw, 10)) {  // 增加到10次重试
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "❎ IK failed at t=%.3f - aborting bezier planning", t);
            ik_failure = true;
            break;
        }

        Eigen::VectorXd q(q_raw.size());

        /* ========================================================
         * 5. 关节包装（ABB风格）
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j)
            q[j] = wrapToNearest(q_raw[j], q_prev[j]);

        /* ========================================================
         * 6. 软/硬关节跳跃评估
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j) {
            double dq = std::abs(q[j] - q_prev[j]);

            if (dq > HARD_JUMP) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Hard joint jump %.3f rad at joint %d (t=%.3f), abort",
                    dq, j, t);
                goto EXIT_BEZIER_SAMPLING;
            }

            if (dq > SOFT_JUMP && !in_sing_area) {
                in_sing_area = true;
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Entering SingArea at t=%.3f (joint %d, dq=%.3f)",
                    t, j, dq);
            }
        }

        q_path.push_back(q);
        q_prev = q;
    }

EXIT_BEZIER_SAMPLING:

    // 如果IK失败，立即返回空轨迹
    if (ik_failure) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "❎ Bezier planning aborted due to IK failure - returning empty trajectory");
        return std::vector<Eigen::VectorXd>{};
    }

    /* ============================================================
     * 7. 强制精确目标位姿IK（ABB行为）
     * 仅在采样成功收集足够点数时执行
     * ============================================================ */
    // 只有采样成功得到3个或更多点时，才执行精确目标IK覆盖
    if (q_path.size() >= 3) {
        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_goal;

        if (tracik_->computeIKClosest(goal, seed, q_goal)) {
            Eigen::VectorXd qg(q_goal.size());
            for (size_t j = 0; j < q_goal.size(); ++j)
                qg[j] = wrapToNearest(q_goal[j], q_prev[j]);

            q_path.back() = qg;
        }
    }

    /* ============================================================
     * 8. 最终检查：采样必须成功收集至少3个点
     * ============================================================ */
    if (q_path.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Bezier sampling produced insufficient points (%zu < 3)", q_path.size());
        q_path.clear();
    }

    return q_path;
}

std::vector<Eigen::VectorXd>
MoveCPlanningStrategy::sampleCircleCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double cartesian_step) const
{
    std::vector<Eigen::VectorXd> q_path;

    if (cartesian_step <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "cartesian_step must be > 0");
        return q_path;
    }

    if (waypoints.size() < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "Need at least 2 waypoints for circle path");
        return q_path;
    }

    /* ============================================================
     * 1. 估算圆形路径长度
     * ============================================================ */
    double total_length = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i].position.x - waypoints[i-1].position.x;
        double dy = waypoints[i].position.y - waypoints[i-1].position.y;
        double dz = waypoints[i].position.z - waypoints[i-1].position.z;
        total_length += std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    if (total_length < 1e-6) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Circle path has zero length");
        return q_path;
    }

    size_t total_steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(total_length / cartesian_step)));

    /* ============================================================
     * 2. 初始种子（拓扑锁定）
     * ============================================================ */
    Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
        moveit_->getCurrentJointState().data(),
        moveit_->getCurrentJointState().size());

    q_path.reserve(total_steps + 1);

    /* ============================================================
     * 3. ABB / KUKA 阈值
     * ============================================================ */
    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;

    // 将起始关节配置加入路径
    q_path.push_back(q_prev);

    /* ============================================================
     * 4. 圆形路径采样 + IK
     * ============================================================ */
    bool ik_failure = false;
    size_t current_step = 0;
    for (size_t seg = 0; seg < waypoints.size() - 1; ++seg) {
        const auto& start_pose = waypoints[seg];
        const auto& end_pose = waypoints[seg + 1];

        // 计算当前段的长度
        double dx = end_pose.position.x - start_pose.position.x;
        double dy = end_pose.position.y - start_pose.position.y;
        double dz = end_pose.position.z - start_pose.position.z;
        double seg_length = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (seg_length < 1e-6) continue;

        size_t seg_steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(seg_length / cartesian_step)));

        for (size_t i = (seg == 0 ? 1 : 0); i <= seg_steps; ++i) {
            // 对于第一段，从 i=1 开始（跳过起始点，因为已经加入了）
            // 对于之后的段，从 i=0 开始但会跳过重复点

            double t = std::min(1.0, double(i) / seg_steps);

            geometry_msgs::msg::Pose pose;
            pose.position.x = start_pose.position.x + t * dx;
            pose.position.y = start_pose.position.y + t * dy;
            pose.position.z = start_pose.position.z + t * dz;
            pose.orientation = slerp(start_pose.orientation, end_pose.orientation, t);

            std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
            std::vector<double> q_raw;

            if (!tracik_->computeIKClosest(pose, seed, q_raw, 10)) {  // 增加到10次重试
                RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                            "❎ IK failed at segment %zu, t=%.3f - aborting circle planning", seg, t);
                ik_failure = true;
                goto EXIT_CIRCLE_SAMPLING;
            }

            Eigen::VectorXd q(q_raw.size());

            /* ========================================================
             * 5. 关节包装（ABB风格）
             * ======================================================== */
            for (int j = 0; j < q.size(); ++j)
                q[j] = wrapToNearest(q_raw[j], q_prev[j]);

            /* ========================================================
             * 6. 软/硬关节跳跃评估
             * ======================================================== */
            for (int j = 0; j < q.size(); ++j) {
                double dq = std::abs(q[j] - q_prev[j]);

                if (dq > HARD_JUMP) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Hard joint jump %.3f rad at joint %d (seg %zu, t=%.3f), abort",
                        dq, j, seg, t);
                    goto EXIT_CIRCLE_SAMPLING;
                }

                if (dq > SOFT_JUMP && !in_sing_area) {
                    in_sing_area = true;
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Entering SingArea at seg %zu, t=%.3f (joint %d, dq=%.3f)",
                        seg, t, j, dq);
                }
            }

            q_path.push_back(q);
            q_prev = q;
            current_step++;
        }
    }

EXIT_CIRCLE_SAMPLING:

    // 如果IK失败，立即返回空轨迹
    if (ik_failure) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "❎ Circle planning aborted due to IK failure - returning empty trajectory");
        return std::vector<Eigen::VectorXd>{};
    }

    /* ============================================================
     * 7. 强制精确目标位姿IK（ABB行为）
     * 仅在采样成功收集足够点数时执行
     * ============================================================ */
    // 只有采样成功得到3个或更多点时，才执行精确目标IK覆盖
    if (q_path.size() >= 3 && !waypoints.empty()) {
        const auto& final_pose = waypoints.back();
        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_goal;

        if (tracik_->computeIKClosest(final_pose, seed, q_goal, 10)) {  // 增加到10次重试
            Eigen::VectorXd qg(q_goal.size());
            for (size_t j = 0; j < q_goal.size(); ++j)
                qg[j] = wrapToNearest(q_goal[j], q_prev[j]);

            q_path.back() = qg;
        }
    }

    /* ============================================================
     * 8. 最终检查：采样必须成功收集至少3个点
     * ============================================================ */
    if (q_path.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Circle sampling produced insufficient points (%zu < 3)", q_path.size());
        q_path.clear();
    }

    return q_path;
}

}  // namespace trajectory_planning::infrastructure::planning
