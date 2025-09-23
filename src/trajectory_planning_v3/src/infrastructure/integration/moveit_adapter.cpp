#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trajectory_planning::infrastructure::integration {

MoveItAdapter::MoveItAdapter(rclcpp::Node::SharedPtr node, const std::string& move_group_name)
    : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, move_group_name);

    // 订阅joint_states话题以避免MoveIt监视器问题
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            latest_joint_state_ = msg;
        });

    // 设置 MoveIt 相关日志器的日志级别为 WARN
    auto ret = rcutils_logging_set_logger_level("moveit", RCUTILS_LOG_SEVERITY_WARN);
    ret = rcutils_logging_set_logger_level("moveit_move_group_default_capabilities", RCUTILS_LOG_SEVERITY_WARN);
    ret = rcutils_logging_set_logger_level("moveit.simple_controller_manager", RCUTILS_LOG_SEVERITY_WARN);
    ret = rcutils_logging_set_logger_level("moveit_ros.trajectory_execution_manager", RCUTILS_LOG_SEVERITY_WARN);
    ret = rcutils_logging_set_logger_level("moveit.plugins.moveit_simple_controller_manager", RCUTILS_LOG_SEVERITY_WARN);
    (void)ret;  // 避免未使用变量警告
}

// ===== 关节规划 =====
bool MoveItAdapter::planJointMotion(const std::vector<double>& target_joints,
                                    moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (!move_group_) return false;

    move_group_->setJointValueTarget(target_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Joint motion planning failed.");
        return false;
    }
    trajectory = plan.trajectory_;
    return true;
}

// ===== 位姿规划（使用MoveIt内置） =====
bool MoveItAdapter::planPoseGoal(const geometry_msgs::msg::Pose& target_pose,
                                 moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (!move_group_) return false;

    move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Pose planning failed.");
        return false;
    }
    trajectory = plan.trajectory_;
    return true;
}

// ===== 笛卡尔路径规划 =====
bool MoveItAdapter::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                      moveit_msgs::msg::RobotTrajectory& trajectory,
                                      double eef_step,
                                      double jump_threshold)
{
    if (!move_group_) return false;

    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction < 0.98) {
        RCLCPP_WARN(node_->get_logger(), "Cartesian path planned fraction: %f", fraction);
        return false;
    }
    return true;
}

// ===== 统一执行接口 =====
bool MoveItAdapter::executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    if (!move_group_) return false;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    bool success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
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
        auto transform = tf_buffer_.lookupTransform("world", "Link6", tf2::TimePointZero);
        current_pose.position.x = transform.transform.translation.x;
        current_pose.position.y = transform.transform.translation.y;
        current_pose.position.z = transform.transform.translation.z;
        current_pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get current pose from TF: %s", ex.what());
        // 返回空的位姿
        current_pose = geometry_msgs::msg::Pose{};
    }

    return current_pose;
}

std::vector<std::pair<double, double>> MoveItAdapter::getJointLimits() const {
    std::vector<std::pair<double, double>> limits;

    // 首先尝试从YAML文件直接读取关节限制
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory("trajectory_planning_v3");
        std::string yaml_path = package_path + "/config/joint_limits.yaml";

        YAML::Node config = YAML::LoadFile(yaml_path);

        if (config["joint_limits"]) {
            // 按顺序读取关节限制 (joint1, joint2, joint3, joint4, joint5, joint6)
            std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

            for (const auto& joint_name : joint_names) {
                if (config["joint_limits"][joint_name]) {
                    auto joint_config = config["joint_limits"][joint_name];

                    if (joint_config["has_position_limits"] &&
                        joint_config["has_position_limits"].as<bool>()) {
                        double min_pos = joint_config["min_position"].as<double>();
                        double max_pos = joint_config["max_position"].as<double>();
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

            RCLCPP_INFO(node_->get_logger(), "Successfully loaded joint limits from YAML file: %zu joints", limits.size());
            return limits;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load joint limits from YAML: %s. Falling back to MoveIt.", e.what());
    }

    // 如果YAML读取失败，回退到MoveIt方式（但添加安全检查）
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "MoveGroup not initialized and YAML fallback failed");
        return limits;
    }

    try {
        // 从MoveIt获取关节模型（添加安全检查）
        auto current_state = move_group_->getCurrentState();
        if (!current_state) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot state");
            return limits;
        }

        const auto& joint_model_group = current_state->getJointModelGroup(move_group_->getName());
        if (!joint_model_group) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get joint model group");
            return limits;
        }

        // 获取活动关节
        const auto& joint_models = joint_model_group->getActiveJointModels();

        for (const auto& joint_model : joint_models) {
            if (joint_model->getType() == moveit::core::JointModel::REVOLUTE ||
                joint_model->getType() == moveit::core::JointModel::PRISMATIC) {

                const auto& bounds = joint_model->getVariableBounds();
                if (!bounds.empty()) {
                    double lower = bounds[0].min_position_;
                    double upper = bounds[0].max_position_;
                    limits.emplace_back(lower, upper);
                } else {
                    // 如果没有限制信息，使用默认值
                    limits.emplace_back(-M_PI, M_PI);
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while getting joint limits from MoveIt: %s", e.what());
        return limits;
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
            RCLCPP_WARN(node_->get_logger(), "No joint_states received yet, using default values as base");
        }
    }

    // 然后应用种子配置中的关节值
    auto joint_names = getJointNames();
    if (joint_values.size() != joint_names.size()) {
        RCLCPP_ERROR(node_->get_logger(),
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

} // namespace trajectory_planning::infrastructure::integration
