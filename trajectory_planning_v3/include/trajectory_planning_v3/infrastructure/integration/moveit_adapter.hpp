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
#include <Eigen/Dense>

namespace trajectory_planning::infrastructure::integration {

class MoveItAdapter {
public:
	MoveItAdapter(rclcpp::Node::SharedPtr node,
	              const std::string& move_group_name,
	              const std::string& controller_type = "");

	// ===== 关节空间 =====
	bool planJointMotion(const std::vector<double>& target_joints,
	                     moveit_msgs::msg::RobotTrajectory& trajectory);

	// ===== 笛卡尔空间 =====
	// 使用MoveIt内置位姿规划
	bool planPoseGoal(const geometry_msgs::msg::Pose& target_pose,
	                  moveit_msgs::msg::RobotTrajectory& trajectory);

	/**
	 * @brief 位姿规划 - 多次尝试找最短轨迹
	 * 当笛卡尔规划失败时回退使用，多次规划以避免选择"绕远路"的解
	 * @param target_pose 目标位姿
	 * @param trajectory 输出轨迹
	 * @param max_attempts 最多尝试次数（默认5次）
	 * @return 规划是否成功
	 */
	bool planPoseGoalMultiAttempt(const geometry_msgs::msg::Pose& target_pose,
	                               moveit_msgs::msg::RobotTrajectory& trajectory,
	                               int max_attempts = 5);

	bool planCartesianPath(
	    const std::vector<geometry_msgs::msg::Pose>& waypoints,
	    moveit_msgs::msg::RobotTrajectory& trajectory, double eef_step = 0.01,
	    double jump_threshold = 0.0);
	
	bool planCartesianPathMultiAttempt(
		const std::vector<geometry_msgs::msg::Pose>& waypoints,
		moveit_msgs::msg::RobotTrajectory& trajectory,
		int max_attempts = 3); 
		
	// ===== 执行统一接口 =====
	bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);

	// ===== 获取机器人信息 =====
	std::vector<std::string> getJointNames() const;
	geometry_msgs::msg::PoseStamped getCurrentPose() const;
	geometry_msgs::msg::Pose getCurrentPoseFromTF() const;
	std::vector<std::pair<double, double>> getJointLimits(
	    const std::string& arm_type = "arm620") const;

	std::string getEndEffectorLink() const;
	std::vector<double> getCurrentJointState() const;

	// ===== 获取缩放参数 =====
	/**
	 * @brief 获取速度缩放因子
	 * @return 当前速度缩放因子（0.0~1.0）
	 */
	double getVelocityScalingFactor() const { return velocity_scaling_factor_; }

	/**
	 * @brief 获取加速度缩放因子
	 * @return 当前加速度缩放因子（0.0~1.0）
	 */
	double getAccelerationScalingFactor() const { return acceleration_scaling_factor_; }

	/**
	 * @brief 获取 URDF 字符串
	 * @param arm_type 机械臂类型（如 "arm620", "arm380"），空字符串时尝试自动检测
	 * @return URDF 字符串，如果失败返回空字符串
	 */
	std::string getURDFString(const std::string& arm_type = "") const;

	/**
	 * @brief 获取 MoveIt 的机器人模型
	 * @return 指向 RobotModel 的 shared_ptr，如果失败返回 nullptr
	 */
	moveit::core::RobotModelPtr getRobotModel() const;

	// ===== 运动学信息 =====
	/**
	 * @brief 计算指定关节位置下的 Jacobian 矩阵
	 * @param joint_positions 关节位置向量 (rad)
	 * @return Jacobian 矩阵 (6 x DOF)，失败返回空矩阵
	 */
	Eigen::MatrixXd computeJacobian(const std::vector<double>& joint_positions) const;

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
	mutable std::mutex joint_state_mutex_;

	// 速度缩放参数
	double velocity_scaling_factor_;
	double acceleration_scaling_factor_;
	std::string controller_type_;  // "movej", "movel", "movec" 或空字符串

	// 参数管理方法
	void loadScalingParameters();
	void applyScalingFactors();

	// 关节限位缓存（避免重复加载YAML文件）
	mutable std::map<std::string, std::vector<std::pair<double, double>>> joint_limits_cache_;
	mutable std::mutex joint_limits_cache_mutex_;
};

}  // namespace trajectory_planning::infrastructure::integration