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
inline double wrapToNearest(double q, double q_ref, double q_min, double q_max)
{
    double dq = q - q_ref;
    while (dq > M_PI)  dq -= 2.0 * M_PI;
    while (dq < -M_PI) dq += 2.0 * M_PI;
    double wrapped = q_ref + dq;

    // 检查是否在限位范围内
    if (wrapped >= q_min && wrapped <= q_max) {
        return wrapped;
    }

    // 尝试其他 2π 倍数偏移
    for (int k = 1; k <= 3; ++k) {
        double offset = 2.0 * M_PI * k;
        double q_plus = wrapped + offset;
        double q_minus = wrapped - offset;

        if (q_plus >= q_min && q_plus <= q_max) {
            return q_plus;
        }
        if (q_minus >= q_min && q_minus <= q_max) {
            return q_minus;
        }
    }

    // 无法找到有效解，返回 NaN
    return std::numeric_limits<double>::quiet_NaN();
}

domain::entities::Trajectory MoveCPlanningStrategy::planArc(
    const geometry_msgs::msg::Pose& via_point,
    const geometry_msgs::msg::Pose& goal_pose,
    const std::string& arm_type) {

	domain::entities::Trajectory traj;

	// 重新加载缩放参数（支持动态参数更新）
	moveit_->loadScalingParameters();

	// 在策略层内部获取当前位姿，确保和关节状态同步
	geometry_msgs::msg::Pose start_pose = moveit_->getCurrentPoseFromTF();

	/* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
	std::vector<Eigen::VectorXd> q_path =
		sampleArcCartesianPath(start_pose, via_point, goal_pose, arm_type);

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
	std::string planning_group = moveit_->getPlanningGroupName();
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		planning_group,   // Use the correct planning group name (left_arm or right_arm)
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
    const geometry_msgs::msg::Pose& goal,
    const std::string& arm_type) {

	domain::entities::Trajectory traj;

    // 重新加载缩放参数（支持动态参数更新）
	moveit_->loadScalingParameters();

	/* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
	std::vector<Eigen::VectorXd> q_path =
		sampleBezierCartesianPath(start, ctrl1, ctrl2, goal, arm_type);

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
	std::string planning_group = moveit_->getPlanningGroupName();
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		planning_group,   // Use the correct planning group name
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
    const geometry_msgs::msg::Pose& radius_point,
    const std::string& arm_type) {

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
	std::vector<Eigen::VectorXd> q_path = sampleCircleCartesianPath(waypoints, arm_type, 0.01);

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
	std::string planning_group = moveit_->getPlanningGroupName();
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		planning_group,   // Use the correct planning group name
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

	std::string planning_group = moveit_->getPlanningGroupName();
	domain::services::TimeOptimalTrajectoryParameterization totg(
	    robot_model,
		planning_group,   // Use the correct planning group name
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
    const std::string& arm_type,
    double cartesian_step) const
{
    std::vector<Eigen::VectorXd> q_path;

    if (cartesian_step <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "cartesian_step must be > 0");
        return q_path;
    }

    /* ============================================================
     * 0. 坐标系转换：将world坐标转换到base_link坐标
     * ============================================================ */
    geometry_msgs::msg::Pose start_pose_bl = moveit_->worldPoseToBaseLinkPose(start_pose);
    geometry_msgs::msg::Pose via_point_bl = moveit_->worldPoseToBaseLinkPose(via_point);
    geometry_msgs::msg::Pose goal_pose_bl = moveit_->worldPoseToBaseLinkPose(goal_pose);

    /* ============================================================
     * 1. 计算圆弧几何参数（在base_link坐标系中）
     * ============================================================ */
    // 三点坐标 (使用base_link坐标系)
    const double x1 = start_pose_bl.position.x, y1 = start_pose_bl.position.y;
    const double x2 = via_point_bl.position.x, y2 = via_point_bl.position.y;
    const double x3 = goal_pose_bl.position.x, y3 = goal_pose_bl.position.y;

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

        size_t steps = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(length / cartesian_step)));

        // 初始种子（拓扑锁定）
        Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
            moveit_->getCurrentJointState().data(),
            moveit_->getCurrentJointState().size());

        q_path.reserve(steps + 1);

        auto joint_limits = moveit_->getJointLimits(arm_type);
        if (joint_limits.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Failed to get joint limits");
            return q_path;
        }

        // ABB / KUKA 阈值
        const double SOFT_JUMP = 0.35;   // rad  → SingArea
        const double HARD_JUMP = 1.20;   // rad  → topology break
        bool in_sing_area = false;
        bool sampling_failed = false;


        // 使用直线插值（包含关节跳跃检测）
        for (size_t i = 0; i <= steps && !sampling_failed; ++i) {
            double s = std::min(1.0, double(i) / steps);

            geometry_msgs::msg::Pose pose_world;
            pose_world.position.x = p0.x() + s * dp.x();
            pose_world.position.y = p0.y() + s * dp.y();
            pose_world.position.z = p0.z() + s * dp.z();
            pose_world.orientation = slerp(start_pose.orientation, goal_pose.orientation, s);

            // 转换从 world 坐标到 base_link 坐标（IK solver 期望 base_link 坐标）
            geometry_msgs::msg::Pose pose = moveit_->worldPoseToBaseLinkPose(pose_world);

            std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
            std::vector<double> q_raw;

            if (!tracik_->computeIKClosest(pose, seed, q_raw)) {
                RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                            "❌ IK failed at linear fallback s=%.3f (already retried 5 times internally), aborting MoveC planning", s);
                sampling_failed = true;
                break;
            }

            Eigen::VectorXd q(q_raw.size());

            bool valid_solution = true;
            for (int j = 0; j < q.size(); ++j) {
                double q_wrapped = wrapToNearest(q_raw[j], q_prev[j],
                                                joint_limits[j].first,
                                                joint_limits[j].second);
                if (std::isnan(q_wrapped)) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Joint %d IK solution %.3f rad exceeds limits [%.3f, %.3f] at s=%.3f, abort",
                        static_cast<int>(j), q_raw[j], joint_limits[j].first, joint_limits[j].second, s);
                    valid_solution = false;
                    break;
                }
                q[j] = q_wrapped;
            }

            if (!valid_solution) {
                sampling_failed = true;
                break;
            }

            // 软/硬关节跳跃评估
            for (int j = 0; j < q.size(); ++j) {
                double dq = std::abs(q[j] - q_prev[j]);

                if (dq > HARD_JUMP) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Hard joint jump %.3f rad at joint %d (linear fallback s=%.3f), abort",
                        dq, j, s);
                    sampling_failed = true;
                    break;
                }

                if (dq > SOFT_JUMP && !in_sing_area) {
                    in_sing_area = true;
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Entering SingArea at linear fallback s=%.3f (joint %d, dq=%.3f)",
                        s, j, dq);
                }
            }

            if (sampling_failed)
                break;

            q_path.push_back(q);
            q_prev = q;
        }

        /* ============================================================
         * Linear fallback error check
         * ============================================================ */
        if (sampling_failed) {
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "❎ Linear fallback aborted - returning empty trajectory");
            return std::vector<Eigen::VectorXd>{};
        }

        // 强制精确目标位姿IK（仅在采样成功收集足够点数时执行）
        if (q_path.size() >= 3) {
            std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
            std::vector<double> q_goal;

            // goal_pose_bl已经是base_link坐标，直接使用
            if (tracik_->computeIKClosest(goal_pose_bl, seed, q_goal, 10)) {  // 增加到10次重试
                Eigen::VectorXd qg(q_goal.size());
                for (size_t j = 0; j < q_goal.size(); ++j)
                    qg[j] = wrapToNearest(q_goal[j], q_prev[j],
                                         joint_limits[j].first,
                                         joint_limits[j].second);

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

    // 验证三个点是否都在圆上
    double r_via = std::hypot(x2 - cx, y2 - cy);
    double r_goal = std::hypot(x3 - cx, y3 - cy);

    if (std::fabs(r - r_via) > 1e-6 || std::fabs(r - r_goal) > 1e-6) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Via or goal point not on circle! Points may not be collinear or have numerical error");
    }

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
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Via point not between start and goal, adjusting goal angle");
        if (a_goal > a_start) a_goal -= 2 * M_PI;
        else a_goal += 2 * M_PI;
        RCLCPP_INFO(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Adjusted a_goal=%.4f rad", a_goal);
    }

    const double delta = a_goal - a_start;
    double arc_length = std::abs(delta) * r;
    double total_length = std::sqrt(
        arc_length * arc_length +
        std::pow(goal_pose_bl.position.z - start_pose_bl.position.z, 2));

    size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(total_length / cartesian_step)));

    /* ============================================================
     * 2. 初始种子（拓扑锁定）
     * ============================================================ */
    Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
        moveit_->getCurrentJointState().data(),
        moveit_->getCurrentJointState().size());
    
    q_path.reserve(steps + 1);

    /* ============================================================
     * 2.5. Get joint limits
     * ============================================================ */
    auto joint_limits = moveit_->getJointLimits(arm_type);
    if (joint_limits.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "Failed to get joint limits");
        return q_path;
    }

    /* ============================================================
     * 3. ABB / KUKA 阈值
     * ============================================================ */
    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;
    bool sampling_failed = false;

    /* ============================================================
     * 4. 圆弧采样 + IK（在单个循环中，base_link坐标采样）
     * ============================================================ */
    for (size_t i = 0; i <= steps && !sampling_failed; ++i) {
        double s = std::min(1.0, double(i) / steps);
        double ang_i = a_start + s * delta;

        // 在 base_link 坐标中生成圆弧点
        geometry_msgs::msg::Pose pose;
        pose.position.x = cx + r * std::cos(ang_i);
        pose.position.y = cy + r * std::sin(ang_i);
        pose.position.z = start_pose_bl.position.z + s * (goal_pose_bl.position.z - start_pose_bl.position.z);
        pose.orientation = slerp(start_pose_bl.orientation, goal_pose_bl.orientation, s);

        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_raw;

        if (!tracik_->computeIKClosest(pose, seed, q_raw)) {
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "❌ IK failed at s=%.3f (already retried 5 times internally), aborting MoveC arc planning", s);
            sampling_failed = true;
            break;
        }

        Eigen::VectorXd q(q_raw.size());

        /* ========================================================
         * 5. 关节包装 with limit checking
         * ======================================================== */
        bool valid_solution = true;
        for (int j = 0; j < q.size(); ++j) {
            double q_wrapped = wrapToNearest(q_raw[j], q_prev[j],
                                            joint_limits[j].first,
                                            joint_limits[j].second);
            if (std::isnan(q_wrapped)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Joint %d IK solution %.3f rad exceeds limits [%.3f, %.3f] at s=%.3f, abort",
                    static_cast<int>(j), q_raw[j], joint_limits[j].first, joint_limits[j].second, s);
                valid_solution = false;
                break;
            }
            q[j] = q_wrapped;
        }

        if (!valid_solution) {
            sampling_failed = true;
            break;
        }

        /* ========================================================
         * 6. 软/硬关节跳跃评估
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j) {
            double dq = std::abs(q[j] - q_prev[j]);

            if (dq > HARD_JUMP) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Hard joint jump %.3f rad at joint %d (s=%.3f), abort",
                    dq, j + 1, s);
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "  q_prev[%d]=%.4f, q[%d]=%.4f",
                    j, q_prev[j], j, q[j]);
                sampling_failed = true;
                break;
            }

            if (dq > SOFT_JUMP && !in_sing_area) {
                in_sing_area = true;
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Entering SingArea at s=%.3f (joint %d, dq=%.3f)",
                    s, j + 1, dq);
            }
        }

        if (sampling_failed)
            break;

        q_path.push_back(q);
        q_prev = q;
    }

    /* ============================================================
     * 7. 强制精确目标位姿IK
     * ============================================================ */
    if (!q_path.empty()) {
        std::vector<double> seed(q_prev.data(),
                                 q_prev.data() + q_prev.size());
        std::vector<double> q_goal;

        if (!tracik_->computeIKClosest(goal_pose_bl, seed, q_goal)) {
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "❌ Goal pose IK failed (already retried 5 times internally), aborting MoveC arc planning");
            sampling_failed = true;
        } else {
            Eigen::VectorXd qg(q_goal.size());
            bool valid_goal = true;
            for (size_t j = 0; j < q_goal.size(); ++j) {
                double q_wrapped = wrapToNearest(q_goal[j], q_prev[j],
                                                joint_limits[j].first,
                                                joint_limits[j].second);
                if (std::isnan(q_wrapped)) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Goal IK solution joint %zu %.3f rad exceeds limits [%.3f, %.3f]",
                        j, q_goal[j], joint_limits[j].first, joint_limits[j].second);
                    valid_goal = false;
                    break;
                }
                qg[j] = q_wrapped;
            }

            if (valid_goal) {
                q_path.back() = qg;
            } else {
                sampling_failed = true;
            }
        }
    }

    /* ============================================================
     * 8. Final unified error check
     * ============================================================ */
    if (sampling_failed) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "❌ MoveC arc planning failed, returning empty trajectory");
        q_path.clear();
        return q_path;
    }

    /* ============================================================
     * 9. Final sanity
     * ============================================================ */
    if (q_path.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Arc sampling produced insufficient points");
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
    const std::string& arm_type,
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

    /* ============================================================
     * 2.5. Get joint limits
     * ============================================================ */
    auto joint_limits = moveit_->getJointLimits(arm_type);
    if (joint_limits.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "Failed to get joint limits");
        return q_path;
    }

    /* ============================================================
     * 3. ABB / KUKA 阈值
     * ============================================================ */
    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;

    /* ============================================================
     * 4. 贝塞尔曲线采样 + IK
     * ============================================================ */
    bool sampling_failed = false;

    for (size_t i = 0; i <= steps && !sampling_failed; ++i) {
        double t = std::min(1.0, double(i) / steps);
        double t2 = t * t, t3 = t2 * t;

        // 三次贝塞尔曲线基函数
        double b0 = (1 - t) * (1 - t) * (1 - t);
        double b1 = 3 * (1 - t) * (1 - t) * t;
        double b2 = 3 * (1 - t) * t * t;
        double b3 = t3;

        geometry_msgs::msg::Pose pose_world;

        // 位置插值
        pose_world.position.x = b0 * start.position.x + b1 * ctrl1.position.x +
                         b2 * ctrl2.position.x + b3 * goal.position.x;
        pose_world.position.y = b0 * start.position.y + b1 * ctrl1.position.y +
                         b2 * ctrl2.position.y + b3 * goal.position.y;
        pose_world.position.z = b0 * start.position.z + b1 * ctrl1.position.z +
                         b2 * ctrl2.position.z + b3 * goal.position.z;

        // 姿态插值（使用slerp而不是线性插值以避免四元数问题）
        pose_world.orientation = slerp(start.orientation, goal.orientation, t);

        // Transform from world coordinate to base_link coordinate
        geometry_msgs::msg::Pose pose = moveit_->worldPoseToBaseLinkPose(pose_world);

        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_raw;

        if (!tracik_->computeIKClosest(pose, seed, q_raw, 10)) {  // 增加到10次重试
            RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                        "❌ IK failed at t=%.3f (already retried 10 times internally) - aborting bezier planning", t);
            return std::vector<Eigen::VectorXd>{};
        }

        Eigen::VectorXd q(q_raw.size());

        /* ========================================================
         * 5. 关节包装（ABB风格）with limit checking
         * ======================================================== */
        bool valid_solution = true;
        for (int j = 0; j < q.size(); ++j) {
            double q_wrapped = wrapToNearest(q_raw[j], q_prev[j],
                                            joint_limits[j].first,
                                            joint_limits[j].second);
            if (std::isnan(q_wrapped)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveCPlanningStrategy"),
                    "Joint %d IK solution %.3f rad exceeds limits [%.3f, %.3f] at t=%.3f, abort",
                    j, q_raw[j], joint_limits[j].first, joint_limits[j].second, t);
                valid_solution = false;
                break;
            }
            q[j] = q_wrapped;
        }

        if (!valid_solution) {
            sampling_failed = true;
            break;
        }

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
                sampling_failed = true;
                break;
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

    /* ============================================================
     * Bezier sampling error check
     * ============================================================ */
    if (sampling_failed) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "❎ Bezier planning aborted due to sampling failure - returning empty trajectory");
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

        // Transform goal_pose from world to base_link coordinate
        geometry_msgs::msg::Pose goal_pose_baselink = moveit_->worldPoseToBaseLinkPose(goal);
        if (tracik_->computeIKClosest(goal_pose_baselink, seed, q_goal)) {
            Eigen::VectorXd qg(q_goal.size());
            bool valid_goal = true;
            for (size_t j = 0; j < q_goal.size(); ++j) {
                double q_wrapped = wrapToNearest(q_goal[j], q_prev[j],
                                                joint_limits[j].first,
                                                joint_limits[j].second);
                if (std::isnan(q_wrapped)) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Goal IK solution joint %zu %.3f rad exceeds limits [%.3f, %.3f]",
                        j, q_goal[j], joint_limits[j].first, joint_limits[j].second);
                    valid_goal = false;
                    break;
                }
                qg[j] = q_wrapped;
            }

            if (valid_goal) {
                q_path.back() = qg;
            }
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
    const std::string& arm_type,
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
     * 2.5. Get joint limits
     * ============================================================ */
    auto joint_limits = moveit_->getJointLimits(arm_type);
    if (joint_limits.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                     "Failed to get joint limits");
        return q_path;
    }

    /* ============================================================
     * 3. ABB / KUKA 阈值
     * ============================================================ */
    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;

    /* ============================================================
     * 4. 圆形路径采样 + IK
     * ============================================================ */
    bool sampling_failed = false;
    size_t current_step = 0;
    for (size_t seg = 0; seg < waypoints.size() - 1 && !sampling_failed; ++seg) {
        const auto& start_pose = waypoints[seg];
        const auto& end_pose = waypoints[seg + 1];

        // 计算当前段的长度
        double dx = end_pose.position.x - start_pose.position.x;
        double dy = end_pose.position.y - start_pose.position.y;
        double dz = end_pose.position.z - start_pose.position.z;
        double seg_length = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (seg_length < 1e-6) continue;

        size_t seg_steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(seg_length / cartesian_step)));

        for (size_t i = 0; i <= seg_steps && !sampling_failed; ++i) {
            // 注意：对于第一段的i=0，这会给出segment起始点
            // 而对于之后的段，起始点会重复（这没关系，因为连续的IK会给出一致的解）

            double t = std::min(1.0, double(i) / seg_steps);

            geometry_msgs::msg::Pose pose_world;
            pose_world.position.x = start_pose.position.x + t * dx;
            pose_world.position.y = start_pose.position.y + t * dy;
            pose_world.position.z = start_pose.position.z + t * dz;
            pose_world.orientation = slerp(start_pose.orientation, end_pose.orientation, t);

            // Transform from world coordinate to base_link coordinate
            geometry_msgs::msg::Pose pose = moveit_->worldPoseToBaseLinkPose(pose_world);

            std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
            std::vector<double> q_raw;

            if (!tracik_->computeIKClosest(pose, seed, q_raw, 10)) {  // 增加到10次重试
                RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                            "❌ IK failed at segment %zu, t=%.3f (already retried 10 times internally) - aborting circle planning", seg, t);
                return std::vector<Eigen::VectorXd>{};
            }

            Eigen::VectorXd q(q_raw.size());

            /* ========================================================
             * 5. 关节包装（ABB风格）with limit checking
             * ======================================================== */
            bool valid_solution = true;
            for (int j = 0; j < q.size(); ++j) {
                double q_wrapped = wrapToNearest(q_raw[j], q_prev[j],
                                                joint_limits[j].first,
                                                joint_limits[j].second);
                if (std::isnan(q_wrapped)) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Joint %d IK solution %.3f rad exceeds limits [%.3f, %.3f] at seg %zu t=%.3f, abort",
                        j, q_raw[j], joint_limits[j].first, joint_limits[j].second, seg, t);
                    valid_solution = false;
                    break;
                }
                q[j] = q_wrapped;
            }

            if (!valid_solution) {
                sampling_failed = true;
                break;
            }

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
                    sampling_failed = true;
                    break;
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

    /* ============================================================
     * Circle sampling error check
     * ============================================================ */
    if (sampling_failed) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveCPlanningStrategy"),
                    "❎ Circle planning aborted due to sampling failure - returning empty trajectory");
        return std::vector<Eigen::VectorXd>{};
    }

    /* ============================================================
     * 7. 强制精确目标位姿IK（ABB行为）
     * 仅在采样成功收集足够点数时执行
     * ============================================================ */
    // 只有采样成功得到3个或更多点时，才执行精确目标IK覆盖
    if (q_path.size() >= 3 && !waypoints.empty()) {
        const auto& final_pose_world = waypoints.back();
        // Transform from world coordinate to base_link coordinate
        geometry_msgs::msg::Pose final_pose = moveit_->worldPoseToBaseLinkPose(final_pose_world);
        std::vector<double> seed(q_prev.data(), q_prev.data() + q_prev.size());
        std::vector<double> q_goal;

        if (tracik_->computeIKClosest(final_pose, seed, q_goal, 10)) {  // 增加到10次重试
            Eigen::VectorXd qg(q_goal.size());
            bool valid_goal = true;
            for (size_t j = 0; j < q_goal.size(); ++j) {
                double q_wrapped = wrapToNearest(q_goal[j], q_prev[j],
                                                joint_limits[j].first,
                                                joint_limits[j].second);
                if (std::isnan(q_wrapped)) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MoveCPlanningStrategy"),
                        "Goal IK solution joint %zu %.3f rad exceeds limits [%.3f, %.3f]",
                        j, q_goal[j], joint_limits[j].first, joint_limits[j].second);
                    valid_goal = false;
                    break;
                }
                qg[j] = q_wrapped;
            }

            if (valid_goal) {
                q_path.back() = qg;
            }
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
