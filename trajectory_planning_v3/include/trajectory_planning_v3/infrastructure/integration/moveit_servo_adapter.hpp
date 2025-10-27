#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>

namespace trajectory_planning::infrastructure::integration {

class MoveItServoAdapter {
public:
    MoveItServoAdapter(
        rclcpp::Node::SharedPtr node,
        const std::string& planning_group,
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_ptr = nullptr);

    ~MoveItServoAdapter() = default;

    // ===== 实时速度 → 关节速度 =====
    std::optional<std::vector<double>> computeJointVelocities(
        const geometry_msgs::msg::TwistStamped& twist_cmd);

    // ===== 可选执行关节速度 =====
    bool executeJointVelocities(const std::vector<double>& joint_velocity, double dt_sec = 0.01);

    // ===== 查询接口 =====
    std::vector<std::string> getJointNames() const;
    geometry_msgs::msg::PoseStamped getCurrentPose() const;
    bool isServoRunning() const;

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    std::string planning_group_;

    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    std::unique_ptr<moveit_servo::Servo> servo_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    mutable std::mutex joint_state_mutex_;

    rclcpp::Logger logger_;
};

}  // namespace trajectory_planning::infrastructure::integration
