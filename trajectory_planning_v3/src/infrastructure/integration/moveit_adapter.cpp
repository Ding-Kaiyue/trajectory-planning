#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

#include <rcutils/logging.h>
#include <tf2/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trajectory_planning::infrastructure::integration {

MoveItAdapter::MoveItAdapter(rclcpp::Node::SharedPtr node,
                             const std::string& move_group_name,
                             const std::string& controller_type)
    : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_),
      controller_type_(controller_type) {
	move_group_ =
	    std::make_shared<moveit::planning_interface::MoveGroupInterface>(
	        node, move_group_name);

	// 订阅joint_states话题以避免MoveIt监视器问题
	joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
	    "/joint_states", 10,
	    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
		    std::lock_guard<std::mutex> lock(joint_state_mutex_);
		    latest_joint_state_ = msg;
	    });

	// 加载速度和加速度缩放参数
	loadScalingParameters();

	// 设置 MoveIt 相关日志器的日志级别为 WARN
	auto ret =
	    rcutils_logging_set_logger_level("moveit", RCUTILS_LOG_SEVERITY_WARN);
	ret = rcutils_logging_set_logger_level(
	    "moveit_move_group_default_capabilities", RCUTILS_LOG_SEVERITY_WARN);
	ret = rcutils_logging_set_logger_level("moveit.simple_controller_manager",
	                                       RCUTILS_LOG_SEVERITY_WARN);
	ret = rcutils_logging_set_logger_level(
	    "moveit_ros.trajectory_execution_manager", RCUTILS_LOG_SEVERITY_WARN);
	ret = rcutils_logging_set_logger_level(
	    "moveit.plugins.moveit_simple_controller_manager",
	    RCUTILS_LOG_SEVERITY_WARN);
	(void)ret;  // 避免未使用变量警告
}

// ===== 关节规划 =====
bool MoveItAdapter::planJointMotion(
    const std::vector<double>& target_joints,
    moveit_msgs::msg::RobotTrajectory& trajectory) {
	if (!move_group_) return false;

	move_group_->setJointValueTarget(target_joints);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success =
	    (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (!success) {
		RCLCPP_ERROR(node_->get_logger(), "Joint motion planning failed.");
		return false;
	}
	trajectory = plan.trajectory_;
	return true;
}

// ===== 位姿规划（使用MoveIt内置） =====
bool MoveItAdapter::planPoseGoal(
    const geometry_msgs::msg::Pose& target_pose,
    moveit_msgs::msg::RobotTrajectory& trajectory) {
	if (!move_group_) return false;

	move_group_->setPoseTarget(target_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success =
	    (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (!success) {
		RCLCPP_ERROR(node_->get_logger(), "Pose planning failed.");
		return false;
	}
	trajectory = plan.trajectory_;
	return true;
}

// ===== 笛卡尔路径规划 =====
bool MoveItAdapter::planCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory, double eef_step,
    double jump_threshold) {
	if (!move_group_) return false;

	double fraction = move_group_->computeCartesianPath(
	    waypoints, eef_step, jump_threshold, trajectory);
	if (fraction < 0.98) {
		RCLCPP_WARN(node_->get_logger(), "Cartesian path planned fraction: %f",
		            fraction);
		return false;
	}
	return true;
}

// ===== 统一执行接口 =====
bool MoveItAdapter::executeTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory) {
	if (!move_group_) return false;

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory;

	bool success =
	    (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (!success) {
		RCLCPP_ERROR(node_->get_logger(), "Execute trajectory failed!");
	}
	return success;
}

// ===== 获取机器人信息 =====
std::vector<std::string> MoveItAdapter::getJointNames() const {
	if (!move_group_) {
		return {};
	}

	return move_group_->getJointNames();
}

geometry_msgs::msg::PoseStamped MoveItAdapter::getCurrentPose() const {
	if (!move_group_) {
		return geometry_msgs::msg::PoseStamped{};
	}

	return move_group_->getCurrentPose();
}

geometry_msgs::msg::Pose MoveItAdapter::getCurrentPoseFromTF() const {
	geometry_msgs::msg::Pose current_pose;

	try {
		// 使用TF获取当前位姿（避免MoveIt的时钟同步问题）
		auto transform =
		    tf_buffer_.lookupTransform("world", "Link6", tf2::TimePointZero);
		current_pose.position.x = transform.transform.translation.x;
		current_pose.position.y = transform.transform.translation.y;
		current_pose.position.z = transform.transform.translation.z;
		current_pose.orientation = transform.transform.rotation;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_ERROR(node_->get_logger(),
		             "Failed to get current pose from TF: %s", ex.what());
		// 返回空的位姿
		current_pose = geometry_msgs::msg::Pose{};
	}

	return current_pose;
}

std::vector<std::pair<double, double>> MoveItAdapter::getJointLimits(
    const std::string& arm_type) const {
	std::vector<std::pair<double, double>> limits;

	// 检查缓存
	{
		std::lock_guard<std::mutex> lock(joint_limits_cache_mutex_);
		auto it = joint_limits_cache_.find(arm_type);
		if (it != joint_limits_cache_.end()) {
			RCLCPP_DEBUG(node_->get_logger(), "Using cached joint limits for %s",
			            arm_type.c_str());
			return it->second;
		}
	}

	// 缓存未命中，从YAML文件加载
	try {
		std::string package_path = ament_index_cpp::get_package_share_directory(
		    "trajectory_planning_v3");
		std::string yaml_path =
		    package_path + "/config/" + arm_type + "_joint_limits.yaml";

		RCLCPP_INFO(node_->get_logger(), "Loading joint limits for %s from: %s",
		            arm_type.c_str(), yaml_path.c_str());

		YAML::Node config = YAML::LoadFile(yaml_path);

		if (config["joint_limits"]) {
			// 按顺序读取关节限制 (joint1, joint2, joint3, joint4, joint5,
			// joint6)
			std::vector<std::string> joint_names = {
			    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

			for (const auto& joint_name : joint_names) {
				if (config["joint_limits"][joint_name]) {
					auto joint_config = config["joint_limits"][joint_name];

					if (joint_config["has_position_limits"] &&
					    joint_config["has_position_limits"].as<bool>()) {
						double min_pos =
						    joint_config["min_position"].as<double>();
						double max_pos =
						    joint_config["max_position"].as<double>();
						limits.emplace_back(min_pos, max_pos);
					} else {
						// 如果没有位置限制，使用默认值
						limits.emplace_back(-M_PI, M_PI);
					}
				} else {
					// 如果关节配置不存在，使用默认值
					limits.emplace_back(-M_PI, M_PI);
				}
			}

			RCLCPP_INFO(node_->get_logger(),
			            "Successfully loaded joint limits for %s from YAML "
			            "file: %zu joints",
			            arm_type.c_str(), limits.size());

			// 将结果存入缓存
			{
				std::lock_guard<std::mutex> lock(joint_limits_cache_mutex_);
				joint_limits_cache_[arm_type] = limits;
			}

			return limits;
		}
	} catch (const std::exception& e) {
		RCLCPP_WARN(node_->get_logger(),
		            "Failed to load joint limits from YAML: %s.", e.what());
	}
	return limits;
}

bool MoveItAdapter::setStartState(const std::vector<double>& joint_values) {
	if (!move_group_) {
		RCLCPP_ERROR(node_->get_logger(), "MoveGroup not initialized");
		return false;
	}

	// 直接创建机器人状态，避免依赖MoveIt的状态监视器
	auto robot_model = move_group_->getRobotModel();
	if (!robot_model) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get robot model");
		return false;
	}

	auto start_state = std::make_shared<moveit::core::RobotState>(robot_model);

	// 首先从我们自己的joint_states订阅中获取当前状态作为基础
	{
		std::lock_guard<std::mutex> lock(joint_state_mutex_);
		if (latest_joint_state_) {
			// 使用最新的joint_states数据设置基础状态
			for (size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
				const std::string& joint_name = latest_joint_state_->name[i];
				if (i < latest_joint_state_->position.size()) {
					double position = latest_joint_state_->position[i];
					start_state->setJointPositions(joint_name, &position);
				}
			}
		} else {
			// 如果还没有收到joint_states，使用默认值
			start_state->setToDefaultValues();
			RCLCPP_WARN(
			    node_->get_logger(),
			    "No joint_states received yet, using default values as base");
		}
	}

	// 然后应用种子配置中的关节值
	auto joint_names = getJointNames();
	if (joint_values.size() != joint_names.size()) {
		RCLCPP_ERROR(
		    node_->get_logger(),
		    "Joint values size (%zu) doesn't match joint names size (%zu)",
		    joint_values.size(), joint_names.size());
		return false;
	}

	// 覆盖指定的关节值
	for (size_t i = 0; i < joint_names.size(); ++i) {
		start_state->setJointPositions(joint_names[i], &joint_values[i]);
	}

	// 确保状态有效
	start_state->enforceBounds();

	// 设置为MoveIt的起始状态
	move_group_->setStartState(*start_state);

	return true;
}

void MoveItAdapter::resetStartStateToDefault() {
	if (!move_group_) {
		return;
	}

	// 重置起始状态为当前状态
	move_group_->setStartStateToCurrentState();
}

// ===== 运动学信息 =====
Eigen::MatrixXd MoveItAdapter::computeJacobian(
    const std::vector<double>& joint_positions) const {
	if (!move_group_) {
		RCLCPP_ERROR(node_->get_logger(), "MoveGroup not initialized");
		return Eigen::MatrixXd();
	}

	auto robot_model = move_group_->getRobotModel();
	if (!robot_model) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get robot model");
		return Eigen::MatrixXd();
	}

	// 创建机器人状态
	auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
	const auto* joint_model_group = robot_state->getJointModelGroup(move_group_->getName());
	if (!joint_model_group) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get joint model group");
		return Eigen::MatrixXd();
	}

	// 设置关节位置
	if (joint_positions.size() != joint_model_group->getActiveJointModels().size()) {
		RCLCPP_ERROR(node_->get_logger(),
		             "Joint positions size (%zu) doesn't match active joints size (%zu)",
		             joint_positions.size(),
		             joint_model_group->getActiveJointModels().size());
		return Eigen::MatrixXd();
	}

	robot_state->setJointGroupPositions(joint_model_group, joint_positions);

	// 获取末端执行器链接
	const auto& ee_link_names = joint_model_group->getLinkModelNames();
	if (ee_link_names.empty()) {
		RCLCPP_ERROR(node_->get_logger(), "No link models found in planning group");
		return Eigen::MatrixXd();
	}
	const std::string& ee_link_name = ee_link_names.back();

	// 计算 Jacobian
	Eigen::MatrixXd jacobian;
	Eigen::Vector3d reference_point = Eigen::Vector3d::Zero();

	const auto* ee_link_model = robot_state->getLinkModel(ee_link_name);
	if (!ee_link_model) {
		RCLCPP_ERROR(node_->get_logger(), "End effector link '%s' not found",
		             ee_link_name.c_str());
		return Eigen::MatrixXd();
	}

	if (!robot_state->getJacobian(joint_model_group, ee_link_model, reference_point,
	                              jacobian)) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to compute Jacobian");
		return Eigen::MatrixXd();
	}

	return jacobian;
}

void MoveItAdapter::loadScalingParameters() {
	// 从参数服务器获取缩放因子
	velocity_scaling_factor_ = 1.0;
	acceleration_scaling_factor_ = 1.0;

	// 根据 controller_type 从参数服务器读取对应的参数
	if (!controller_type_.empty()) {
		std::string velocity_param_name = controller_type_ + ".velocity_scaling_factor";
		std::string acceleration_param_name = controller_type_ + ".acceleration_scaling_factor";

		RCLCPP_INFO(node_->get_logger(),
		           "MoveItAdapter: Looking for parameters: '%s' and '%s'",
		           velocity_param_name.c_str(), acceleration_param_name.c_str());

		// 尝试从参数服务器获取参数
		if (node_->has_parameter(velocity_param_name)) {
			velocity_scaling_factor_ = node_->get_parameter(velocity_param_name).as_double();
			RCLCPP_INFO(node_->get_logger(),
			           "MoveItAdapter: Found '%s' = %.2f",
			           velocity_param_name.c_str(), velocity_scaling_factor_);
		} else {
			RCLCPP_WARN(node_->get_logger(),
			           "MoveItAdapter: Parameter '%s' not found, using default 1.0",
			           velocity_param_name.c_str());
		}

		if (node_->has_parameter(acceleration_param_name)) {
			acceleration_scaling_factor_ = node_->get_parameter(acceleration_param_name).as_double();
			RCLCPP_INFO(node_->get_logger(),
			           "MoveItAdapter: Found '%s' = %.2f",
			           acceleration_param_name.c_str(), acceleration_scaling_factor_);
		} else {
			RCLCPP_WARN(node_->get_logger(),
			           "MoveItAdapter: Parameter '%s' not found, using default 1.0",
			           acceleration_param_name.c_str());
		}
	} else {
		// 如果没有指定 controller_type，使用通用参数
		if (node_->has_parameter("velocity_scaling_factor")) {
			velocity_scaling_factor_ = node_->get_parameter("velocity_scaling_factor").as_double();
		}
		if (node_->has_parameter("acceleration_scaling_factor")) {
			acceleration_scaling_factor_ = node_->get_parameter("acceleration_scaling_factor").as_double();
		}
	}

	RCLCPP_INFO(node_->get_logger(),
	           "MoveItAdapter: Final scaling factors - velocity=%.2f, acceleration=%.2f",
	           velocity_scaling_factor_, acceleration_scaling_factor_);

	applyScalingFactors();
}

void MoveItAdapter::applyScalingFactors() {
	// 应用缩放因子到 MoveIt 规划器
	if (!move_group_) {
		return;
	}

	// 钳制缩放因子在合理范围内
	velocity_scaling_factor_ = std::max(0.01, std::min(1.0, velocity_scaling_factor_));
	acceleration_scaling_factor_ = std::max(0.01, std::min(1.0, acceleration_scaling_factor_));

	move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
	move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);

	RCLCPP_INFO(node_->get_logger(),
	           "MoveIt scaling factors applied: velocity=%.2f, acceleration=%.2f (type: %s)",
	           velocity_scaling_factor_, acceleration_scaling_factor_,
	           controller_type_.empty() ? "default" : controller_type_.c_str());
}

bool MoveItAdapter::planPoseGoalMultiAttempt(
    const geometry_msgs::msg::Pose& target_pose,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    int max_attempts) {
	for (int attempt = 0; attempt < max_attempts; ++attempt) {
		if (planPoseGoal(target_pose, trajectory)) {
			return true;
		}
		RCLCPP_WARN(node_->get_logger(),
		           "Pose planning attempt %d failed, retrying...", attempt + 1);
	}
	RCLCPP_ERROR(node_->get_logger(),
	            "Pose planning failed after %d attempts", max_attempts);
	return false;
}

bool MoveItAdapter::planCartesianPathMultiAttempt(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    int max_attempts) {
	for (int attempt = 0; attempt < max_attempts; ++attempt) {
		if (planCartesianPath(waypoints, trajectory)) {
			return true;
		}
		RCLCPP_WARN(node_->get_logger(),
		           "Cartesian path planning attempt %d failed, retrying...", attempt + 1);
	}
	RCLCPP_ERROR(node_->get_logger(),
	            "Cartesian path planning failed after %d attempts", max_attempts);
	return false;
}

std::string MoveItAdapter::getEndEffectorLink() const {
	if (!move_group_) {
		return "";
	}

	return move_group_->getEndEffectorLink();
}

std::vector<double> MoveItAdapter::getCurrentJointState() const {
	if (!move_group_) {
		return {};
	}

	std::lock_guard<std::mutex> lock(joint_state_mutex_);
	if (!latest_joint_state_) {
		return {};
	}

	return latest_joint_state_->position;
}

std::string MoveItAdapter::getBaseLink() const {
	if (!move_group_) {
		RCLCPP_ERROR(node_->get_logger(), "MoveGroup not initialized");
		return "";
	}

	auto robot_model = move_group_->getRobotModel();
	if (!robot_model) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get robot model");
		return "";
	}

	const auto* joint_model_group = robot_model->getJointModelGroup(move_group_->getName());
	if (!joint_model_group) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get joint model group for '%s'", move_group_->getName().c_str());
		return "";
	}

	// 获取planning group中的第一个活跃关节，其parent link就是base_link
	const auto& active_joints = joint_model_group->getActiveJointModels();
	if (active_joints.empty()) {
		RCLCPP_ERROR(node_->get_logger(), "No active joints found in planning group '%s'", move_group_->getName().c_str());
		return "";
	}

	// 第一个active joint的parent link就是base_link
	const auto* first_joint = active_joints.front();
	if (!first_joint) {
		RCLCPP_ERROR(node_->get_logger(), "First active joint is null in planning group '%s'", move_group_->getName().c_str());
		return "";
	}

	const auto* parent_link = first_joint->getParentLinkModel();
	if (!parent_link) {
		RCLCPP_ERROR(node_->get_logger(), "Parent link of first active joint is null in planning group '%s'", move_group_->getName().c_str());
		return "";
	}

	std::string base_link = parent_link->getName();
	RCLCPP_INFO(node_->get_logger(), "Base link for planning group '%s': %s",
	            move_group_->getName().c_str(), base_link.c_str());
	return base_link;
}

std::string MoveItAdapter::getURDFString(const std::string& arm_type) const {
	if (!node_) {
		return "";
	}

	try {
		// 优先尝试从参数服务器获取 URDF
		std::string urdf_string;
		if (node_->get_parameter("robot_description", urdf_string)) {
			RCLCPP_DEBUG(node_->get_logger(), "Got URDF from robot_description parameter");
			return urdf_string;
		}

		// 如果提供了机械臂类型，从指定的文件加载
		if (arm_type.empty()) {
			RCLCPP_ERROR(node_->get_logger(), "arm_type is empty and robot_description not in parameter server");
			return "";
		}

		std::string package_path = ament_index_cpp::get_package_share_directory(
		    "robot_description");
		std::string urdf_path = package_path + "/urdf/" + arm_type + ".urdf";

		std::ifstream file(urdf_path);
		if (!file.is_open()) {
			RCLCPP_ERROR(node_->get_logger(), "Failed to open URDF file: %s",
			            urdf_path.c_str());
			return "";
		}

		std::stringstream buffer;
		buffer << file.rdbuf();
		RCLCPP_INFO(node_->get_logger(),
		           "Successfully loaded URDF from: %s", urdf_path.c_str());
		return buffer.str();
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get URDF string: %s", e.what());
		return "";
	}
}

moveit::core::RobotModelPtr MoveItAdapter::getRobotModel() const
{
	if (!move_group_) {
		return nullptr;
	}
	// getRobotModel() returns RobotModelConstPtr, we need to cast it
	// Since we're using it for reading only in TOTG, this is safe
	auto const_model = move_group_->getRobotModel();
	return std::const_pointer_cast<moveit::core::RobotModel>(const_model);
}

}  // namespace trajectory_planning::infrastructure::integration
