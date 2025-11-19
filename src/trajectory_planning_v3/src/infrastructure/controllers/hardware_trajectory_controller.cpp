#include "trajectory_planning_v3/infrastructure/controllers/hardware_trajectory_controller.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace trajectory_planning::infrastructure::controllers {

HardwareTrajectoryController::HardwareTrajectoryController(
    std::shared_ptr<adapters::HardwareAdapter> hardware_adapter,
    std::shared_ptr<RobotHardware> robot_hw, const std::string& interface_name,
    const std::string& controller_name)
    : Node("hardware_trajectory_controller"),
      hardware_adapter_(hardware_adapter),
      robot_hw_(robot_hw),
      controller_name_(controller_name),
      interface_name_(interface_name),
      trajectory_executing_(false) {
	// 配置关节名称和状态存储
	joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
	joint_states_.resize(joint_names_.size());

	// 创建FollowJointTrajectory动作服务器
	std::string action_name =
	    "/" + controller_name_ + "/follow_joint_trajectory";
	action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
	    this, action_name,
	    std::bind(&HardwareTrajectoryController::handle_goal, this, _1, _2),
	    std::bind(&HardwareTrajectoryController::handle_cancel, this, _1),
	    std::bind(&HardwareTrajectoryController::handle_accepted, this, _1));

	// 创建关节状态发布器，使用可靠的 QoS
	auto qos = rclcpp::QoS(10);
	qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
	qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
	joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
	    "joint_states", qos);

	// 创建TF broadcaster
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	// 注意：观察者在RobotHardware构造时已经注册
	// robot_hw_已经在构造时接收了this作为观察者

	RCLCPP_INFO(this->get_logger(),
	            "Hardware trajectory controller initialized");
	RCLCPP_INFO(this->get_logger(), "Action server: %s", action_name.c_str());
}

HardwareTrajectoryController::HardwareTrajectoryController(
    const std::string& interface_name, const std::string& controller_name)
    : Node("hardware_trajectory_controller"),
      hardware_adapter_(nullptr),
      robot_hw_(nullptr),
      controller_name_(controller_name),
      interface_name_(interface_name),
      trajectory_executing_(false) {
	// 配置关节名称和状态存储
	joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
	joint_states_.resize(joint_names_.size());

	// 创建FollowJointTrajectory动作服务器
	std::string action_name =
	    "/" + controller_name_ + "/follow_joint_trajectory";
	action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
	    this, action_name,
	    std::bind(&HardwareTrajectoryController::handle_goal, this, _1, _2),
	    std::bind(&HardwareTrajectoryController::handle_cancel, this, _1),
	    std::bind(&HardwareTrajectoryController::handle_accepted, this, _1));

	// 创建关节状态发布器，使用可靠的 QoS
	auto qos = rclcpp::QoS(10);
	qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
	qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
	joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
	    "joint_states", qos);

	// 创建TF broadcaster
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	RCLCPP_INFO(this->get_logger(),
	            "Hardware trajectory controller initialized (observer mode)");
	RCLCPP_INFO(this->get_logger(), "Action server: %s", action_name.c_str());
}

HardwareTrajectoryController::~HardwareTrajectoryController() {
	trajectory_executing_ = false;
}

void HardwareTrajectoryController::setHardwareAdapter(
    std::shared_ptr<adapters::HardwareAdapter> hardware_adapter) {
	hardware_adapter_ = hardware_adapter;
}

rclcpp_action::GoalResponse HardwareTrajectoryController::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
	RCLCPP_INFO(this->get_logger(), "Received goal request");

	// 检查轨迹是否有效
	if (goal->trajectory.points.empty()) {
		RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
		return rclcpp_action::GoalResponse::REJECT;
	}

	// 检查关节名称是否匹配
	if (goal->trajectory.joint_names.size() != joint_names_.size()) {
		RCLCPP_WARN(this->get_logger(),
		            "Joint names mismatch. Expected %zu, got %zu",
		            joint_names_.size(), goal->trajectory.joint_names.size());
		return rclcpp_action::GoalResponse::REJECT;
	}

	// 如果正在执行轨迹，拒绝新目标
	if (trajectory_executing_) {
		RCLCPP_WARN(this->get_logger(),
		            "Already executing trajectory, rejecting new goal");
		return rclcpp_action::GoalResponse::REJECT;
	}

	RCLCPP_INFO(this->get_logger(), "Accepting goal");
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HardwareTrajectoryController::handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> /*goal_handle*/) {
	RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
	trajectory_executing_ = false;
	return rclcpp_action::CancelResponse::ACCEPT;
}

void HardwareTrajectoryController::handle_accepted(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
	using namespace std::placeholders;

	// 保存当前目标句柄
	{
		std::lock_guard<std::mutex> lock(goal_handle_mutex_);
		current_goal_handle_ = goal_handle;
	}

	// 在新线程中执行轨迹
	std::thread{
	    std::bind(&HardwareTrajectoryController::execute_trajectory, this, _1),
	    goal_handle}
	    .detach();
}

void HardwareTrajectoryController::execute_trajectory(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
	const auto goal = goal_handle->get_goal();
	auto result = std::make_shared<FollowJointTrajectory::Result>();
	int point_num = goal->trajectory.points.size();

	RCLCPP_INFO(this->get_logger(), "MoveIt give us %d trajectory points",
	            point_num);
	trajectory_executing_ = true;

	// 判断轨迹点数是否有效
	if (point_num > 0) {
		// RCLCPP_INFO(this->get_logger(), "=== 开始打印MoveIt原始轨迹数据
		// ===");

		trajectory_planning::domain::entities::Trajectory domain_trajectory;

		double max_time =
		    goal->trajectory.points[point_num - 1].time_from_start.sec +
		    goal->trajectory.points[point_num - 1].time_from_start.nanosec /
		        1e9;
		RCLCPP_INFO(this->get_logger(), "Trajectory max_time is: %.3f seconds",
		            max_time);

		// 打印ROS轨迹原始数据
		for (int i = 0; i < point_num; i++) {
			const auto& ros_point = goal->trajectory.points[i];
			double time_sec = ros_point.time_from_start.sec +
			                  ros_point.time_from_start.nanosec / 1e9;

			std::vector<double> positions_deg(ros_point.positions.size());
			for (size_t j = 0; j < ros_point.positions.size(); j++) {
				positions_deg[j] = ros_point.positions[j] * 180.0 / M_PI;
			}

			// RCLCPP_INFO(this->get_logger(), "ROS Point[%d] time=%.3fs:", i,
			// time_sec); RCLCPP_INFO(this->get_logger(), "  positions: [%.4f,
			// %.4f, %.4f, %.4f, %.4f, %.4f]",
			//            positions_deg[0], positions_deg[1], positions_deg[2],
			//            positions_deg[3], positions_deg[4], positions_deg[5]);
			// RCLCPP_INFO(this->get_logger(), "  velocities: [%.4f, %.4f, %.4f,
			// %.4f, %.4f, %.4f]",
			//            ros_point.velocities[0], ros_point.velocities[1],
			//            ros_point.velocities[2], ros_point.velocities[3],
			//            ros_point.velocities[4], ros_point.velocities[5]);
			// RCLCPP_INFO(this->get_logger(), "  accelerations: [%.4f, %.4f,
			// %.4f, %.4f, %.4f, %.4f]",
			//            ros_point.accelerations[0],
			//            ros_point.accelerations[1],
			//            ros_point.accelerations[2],
			//            ros_point.accelerations[3],
			//            ros_point.accelerations[4],
			//            ros_point.accelerations[5]);

			// 转换ROS轨迹到domain格式
			trajectory_planning::domain::entities::TrajectoryPoint domain_point{
			    .position =
			        trajectory_planning::domain::value_objects::JointPosition(
			            positions_deg),
			    .velocity =
			        trajectory_planning::domain::value_objects::JointVelocity(
			            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
			    .acceleration = trajectory_planning::domain::value_objects::
			        JointAcceleration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
			    .time_from_start =
			        trajectory_planning::domain::value_objects::Duration(
			            time_sec),
			    .progress_ratio = static_cast<double>(i) / (point_num - 1)};

			domain_trajectory.add_point(domain_point);
		}

		if (hardware_adapter_->executeTrajectory(domain_trajectory)) {
			RCLCPP_INFO(this->get_logger(),
			            "Trajectory execution completed successfully");
			result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
			result->error_string = "success";
			goal_handle->succeed(result);
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to execute trajectory");
			result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
			result->error_string = "Hardware trajectory execution failed";
			goal_handle->abort(result);
		}
	} else {
		RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
		result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
		result->error_string = "Empty trajectory";
		goal_handle->abort(result);
	}

	trajectory_executing_ = false;
}

void HardwareTrajectoryController::on_motor_status_update(
    const std::string& interface, uint32_t motor_id,
    const hardware_driver::motor_driver::Motor_Status& status) {
	if (interface != interface_name_) {
		return;  // 不是我们关心的接口
	}

	if (motor_id < 1 || motor_id > joint_names_.size()) {
		return;  // 不是我们关心的电机
	}

	// 更新关节状态
	updateJointState(motor_id - 1, status);

	// 发布关节状态到ROS话题
	publishJointStates();
}

void HardwareTrajectoryController::updateJointState(
    size_t joint_index,
    const hardware_driver::motor_driver::Motor_Status& status) {
	std::lock_guard<std::mutex> lock(joint_states_mutex_);
	joint_states_[joint_index].position = status.position;
	joint_states_[joint_index].velocity = status.velocity;
	joint_states_[joint_index].effort = status.effort;
}

void HardwareTrajectoryController::publishJointStates() {
	auto joint_state_msg = sensor_msgs::msg::JointState();
	joint_state_msg.header.stamp = this->get_clock()->now();
	joint_state_msg.name = joint_names_;
	joint_state_msg.position.resize(joint_states_.size());
	joint_state_msg.velocity.resize(joint_states_.size());
	joint_state_msg.effort.resize(joint_states_.size());

	{
		std::lock_guard<std::mutex> lock(joint_states_mutex_);

		for (size_t i = 0; i < joint_states_.size(); ++i) {
			joint_state_msg.position[i] =
			    joint_states_[i].position * M_PI / 180.0;
			joint_state_msg.velocity[i] = joint_states_[i].velocity;
			joint_state_msg.effort[i] = joint_states_[i].effort;
		}
	}

	joint_state_pub_->publish(joint_state_msg);
}

}  // namespace trajectory_planning::infrastructure::controllers
