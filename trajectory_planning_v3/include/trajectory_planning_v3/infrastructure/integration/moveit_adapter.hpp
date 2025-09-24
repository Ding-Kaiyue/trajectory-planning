#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trajectory_planning::infrastructure::integration {

class MoveItAdapter {
public:
	MoveItAdapter(rclcpp::Node::SharedPtr node,
	              const std::string& move_group_name);

	// ===== 关节空间 =====
	bool planJointMotion(const std::vector<double>& target_joints,
	                     moveit_msgs::msg::RobotTrajectory& trajectory);

	// ===== 笛卡尔空间 =====
	// 使用MoveIt内置位姿规划
	bool planPoseGoal(const geometry_msgs::msg::Pose& target_pose,
	                  moveit_msgs::msg::RobotTrajectory& trajectory);

	bool planCartesianPath(
	    const std::vector<geometry_msgs::msg::Pose>& waypoints,
	    moveit_msgs::msg::RobotTrajectory& trajectory, double eef_step = 0.01,
	    double jump_threshold = 0.0);

	// ===== 执行统一接口 =====
	bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);

	// ===== 获取机器人信息 =====
	std::vector<std::string> getJointNames() const;
	geometry_msgs::msg::PoseStamped getCurrentPose() const;
	geometry_msgs::msg::Pose getCurrentPoseFromTF() const;
	std::vector<std::pair<double, double>> getJointLimits(
	    const std::string& arm_type = "arm620") const;

	// ===== 状态设置 =====
	bool setStartState(const std::vector<double>& joint_values);
	void resetStartStateToDefault();

private:
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

	// TF支持
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	// 直接订阅joint_states以避免MoveIt监视器问题
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
	    joint_state_sub_;
	sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
	std::mutex joint_state_mutex_;
};

}  // namespace trajectory_planning::infrastructure::integration