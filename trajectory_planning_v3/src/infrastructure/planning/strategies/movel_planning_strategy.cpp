#include "trajectory_planning_v3/infrastructure/planning/strategies/movel_planning_strategy.hpp"

#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stack>

namespace trajectory_planning::infrastructure::planning
{

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

inline double wrapToNearest(double q, double q_ref)
{
    double dq = q - q_ref;
    while (dq > M_PI)  dq -= 2.0 * M_PI;
    while (dq < -M_PI) dq += 2.0 * M_PI;
    return q_ref + dq;
}


std::vector<Eigen::VectorXd>
MoveLPlanningStrategy::sampleCartesianPath(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    double cartesian_step) const
{
    std::vector<Eigen::VectorXd> q_path;

    if (cartesian_step <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveLPlanningStrategy"),
                     "cartesian_step must be > 0");
        return q_path;
    }

    /* ============================================================
     * 1. Cartesian straight-line geometry
     * ============================================================ */
    Eigen::Vector3d p0(start_pose.position.x,
                       start_pose.position.y,
                       start_pose.position.z);
    Eigen::Vector3d p1(goal_pose.position.x,
                       goal_pose.position.y,
                       goal_pose.position.z);

    Eigen::Vector3d dp = p1 - p0;
    const double length = dp.norm();

    if (length < 1e-6)
        return q_path;

    size_t steps = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(length / cartesian_step)));

    /* ============================================================
     * 2. Initial seed (topology lock)
     * ============================================================ */
    Eigen::VectorXd q_prev = Eigen::Map<const Eigen::VectorXd>(
        moveit_->getCurrentJointState().data(),
        moveit_->getCurrentJointState().size());

    q_path.reserve(steps + 1);

    /* ============================================================
     * 3. ABB / KUKA thresholds
     * ============================================================ */

    const double SOFT_JUMP = 0.35;   // rad  → SingArea
    const double HARD_JUMP = 1.20;   // rad  → topology break
    bool in_sing_area = false;

    /* ============================================================
     * 4. Cartesian sampling + IK
     * ============================================================ */
    for (size_t i = 0; i <= steps; ++i) {
        double s = std::min(1.0, double(i) / steps);

        geometry_msgs::msg::Pose pose;
        pose.position.x = p0.x() + s * dp.x();
        pose.position.y = p0.y() + s * dp.y();
        pose.position.z = p0.z() + s * dp.z();
        pose.orientation =
            slerp(start_pose.orientation, goal_pose.orientation, s);

        std::vector<double> seed(q_prev.data(),
                                 q_prev.data() + q_prev.size());
        std::vector<double> q_raw;

        if (!tracik_->computeIKClosest(pose, seed, q_raw)) {
            RCLCPP_WARN(rclcpp::get_logger("MoveLPlanningStrategy"),
                        "IK failed at s=%.3f", s);
            break;
        }

        Eigen::VectorXd q(q_raw.size());

        /* ========================================================
         * 5. Joint wrapping (ABB-style)
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j)
            q[j] = wrapToNearest(q_raw[j], q_prev[j]);

        /* ========================================================
         * 6. Soft / Hard joint jump evaluation
         * ======================================================== */
        for (int j = 0; j < q.size(); ++j) {
            double dq = std::abs(q[j] - q_prev[j]);

            if (dq > HARD_JUMP) {
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveLPlanningStrategy"),
                    "Hard joint jump %.3f rad at joint %d (s=%.3f), abort",
                    dq, j, s);
                goto EXIT_SAMPLING;
            }

            if (dq > SOFT_JUMP && !in_sing_area) {
                in_sing_area = true;
                RCLCPP_WARN(
                    rclcpp::get_logger("MoveLPlanningStrategy"),
                    "Entering SingArea at s=%.3f (joint %d, dq=%.3f)",
                    s, j, dq);
            }
        }

        q_path.push_back(q);
        q_prev = q;
    }

EXIT_SAMPLING:

    /* ============================================================
     * 7. Enforce exact goal pose IK (ABB behavior)
     * ============================================================ */
    if (!q_path.empty()) {
        std::vector<double> seed(q_prev.data(),
                                 q_prev.data() + q_prev.size());
        std::vector<double> q_goal;

        if (tracik_->computeIKClosest(goal_pose, seed, q_goal)) {
            Eigen::VectorXd qg(q_goal.size());
            for (size_t j = 0; j < q_goal.size(); ++j)
                qg[j] = wrapToNearest(q_goal[j], q_prev[j]);

            q_path.back() = qg;
        }
    }

    /* ============================================================
     * 8. Final sanity
     * ============================================================ */
    if (q_path.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("MoveLPlanningStrategy"),
                    "Cartesian sampling produced insufficient points");
        q_path.clear();
    }

    return q_path;
}



domain::entities::Trajectory
MoveLPlanningStrategy::planWithJointConstraints(
    const geometry_msgs::msg::Pose& goal,
    double eef_step)
{

    domain::entities::Trajectory traj;

    // 重新加载缩放参数（支持动态参数更新）
    moveit_->loadScalingParameters();

    const auto start_pose = moveit_->getCurrentPoseFromTF();
    const auto q0 = moveit_->getCurrentJointState();
    if (q0.empty())
        return traj;

    /* -----------------------------
    * 1. Cartesian sampling → joint path
    * ----------------------------- */
    std::vector<Eigen::VectorXd> q_path =
        sampleCartesianPath(start_pose, goal, eef_step);

    if (q_path.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("MoveLPlanningStrategy"),
                    "Insufficient joint samples");
        return traj;
    }

    /* -----------------------------
    * 2. Get scaling factors (NOT limits)
    * ----------------------------- */
    double velocity_scaling =
        moveit_->getVelocityScalingFactor();
    double acceleration_scaling =
        moveit_->getAccelerationScalingFactor();

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
        RCLCPP_ERROR(rclcpp::get_logger("MoveLPlanningStrategy"),
                    "TOTG failed");
        return traj;
    }

    // printTrajectory(traj);

    return traj;
}


}  // namespace trajectory_planning::infrastructure::planning

