#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/application/services/trajectory_execution_service.hpp"
#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/joint_constrained_planning_strategy.hpp"
#include "robot_interfaces/msg/joint_constrained_request.hpp"
#include "robot_interfaces/msg/joint_constrained_test_request.hpp"

using namespace trajectory_planning::application::services;
using namespace trajectory_planning::infrastructure::execution;
using namespace trajectory_planning::infrastructure::integration;
using namespace trajectory_planning::infrastructure::planning;
using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::domain::value_objects;

/*
    ros2 topic pub --once /joint_constrained_goals_test robot_interfaces/msg/JointConstrainedTestRequest \
    "{
    goal_position: {x: 0.4, y: 0.0, z: 0.4},
    roll: -90.0,
    pitch: 0.0,
    yaw: -90.0,
    planning_type: 2,
    max_attempts: 5,
    joint_constraints: [
        {type: 0, joint_index: 1, weight: 1.0, is_hard: true, fixed_value: 0.0},
        {type: 1, joint_index: 3, weight: 0.8, is_hard: false, min_value: -0.5, max_value: 0.5}
    ]
    }"

    ros2 topic pub --once /joint_constrained_goals_test robot_interfaces/msg/JointConstrainedTestRequest "{
        goal_position: {x: 0.4, y: 0.0, z: 0.4},
        roll: -90.0,
        pitch: 0.0, 
        yaw: -90.0,
        planning_type: 2,
        max_attempts: 5,
        joint_constraints: [
            {type: 0, joint_index: 0, weight: 1.0, is_hard: true, fixed_value: 0.0}, 
            {type: 1, joint_index: 3, weight: 0.8, is_hard: false, min_value: -0.2, max_value: 0.2}
        ]
    }"
 */
/**
 * @brief 关节约束运动节点，订阅JointConstrainedMotion消息，规划并执行关节约束运动
 * @note 该节点依赖MoveIt进行规划，需确保MoveIt配置正确
 * @author Ding Kaiyue
 * @date 2025-09-18
 * @version 1.0
 */
class JointConstrainedNode : public rclcpp::Node {
public:
    JointConstrainedNode() : Node("joint_constrained_node") {
        // ---- 1. 声明参数 ----
        this->declare_parameter<std::string>("move_group", "arm");
        this->declare_parameter<std::string>("arm_type", "arm620");
        this->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
        this->declare_parameter<double>("planning_time", 5.0);
        this->declare_parameter<double>("velocity_scaling", 0.2);
        this->declare_parameter<double>("acceleration_scaling", 0.2);
        this->declare_parameter<double>("goal_tolerance", 0.001);

        // ---- 2. 订阅 /joint_constrained_goals 和 /joint_constrained_goals_test ----
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        goal_sub_ = this->create_subscription<robot_interfaces::msg::JointConstrainedRequest>(
            "/joint_constrained_goals", qos,
            std::bind(&JointConstrainedNode::goalCallback, this, std::placeholders::_1)
        );
        goal_test_sub_ = this->create_subscription<robot_interfaces::msg::JointConstrainedTestRequest>(
            "/joint_constrained_goals_test", qos,
            std::bind(&JointConstrainedNode::goalTestCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Joint Constrained Node ready. Publish to: \n"
            "  /joint_constrained_goals(JointConstrainedRequest) - Full message\n"
            "  /joint_constrained_goals_test(JointConstrainedTestRequest) - RPY test message\n");
    }

    void initializeComponents() {
        if (!motion_planning_service_) {
            // 创建 MoveItAdapter, JointConstrainedPlanningStrategy 和 TrajectoryExecutor
            auto group_name = this->get_parameter("move_group").as_string();

            // 初始化基础设施层
            moveit_adapter_ = std::make_shared<MoveItAdapter>(shared_from_this(), group_name);
            trajectory_executor_ = std::make_shared<TrajectoryExecutor>(*moveit_adapter_);

            auto joint_constrained_strategy_ = std::make_shared<JointConstrainedPlanningStrategy>(*moveit_adapter_);

            // 初始化应用服务
            motion_planning_service_ = std::make_shared<MotionPlanningService>(
                moveit_adapter_, shared_from_this()
            );

            // 注册需要的策略
            motion_planning_service_->registerJointConstrainedStrategy(joint_constrained_strategy_);

            trajectory_execution_service_ = std::make_shared<TrajectoryExecutionService>(
                trajectory_executor_, this->get_logger()
            );
            
            RCLCPP_INFO(this->get_logger(), "MoveItAdapter, JointConstrainedPlanningStrategy and TrajectoryExecutor initialized.");
        }
    }
private:
    void goalCallback(const robot_interfaces::msg::JointConstrainedRequest::SharedPtr msg) {
        if (!checkReady("JointConstrained")) return;

        auto result = motion_planning_service_->planConstrainedMotion(*msg);

        if (result.success) {
            executeTrajectory(result.trajectory, "JointConstrained");
        } else {
            RCLCPP_ERROR(this->get_logger(), "约束规划失败：%s", result.error_message.c_str());
        }
    }

    void goalTestCallback(const robot_interfaces::msg::JointConstrainedTestRequest::SharedPtr msg) {
        // 转换 TestRequest 到 Request 消息格式
        auto request_msg = std::make_shared<robot_interfaces::msg::JointConstrainedRequest>();

        // 设置目标位姿
        request_msg->goal_pose.position.x = msg->goal_position.x;
        request_msg->goal_pose.position.y = msg->goal_position.y;
        request_msg->goal_pose.position.z = msg->goal_position.z;

        tf2::Quaternion quat;
        quat.setRPY(msg->roll * M_PI / 180.0, msg->pitch * M_PI / 180.0, msg->yaw * M_PI / 180.0);
        request_msg->goal_pose.orientation = tf2::toMsg(quat);

        // 复制其他字段
        request_msg->joint_constraints = msg->joint_constraints;
        request_msg->max_attempts = msg->max_attempts;

        // 直接调用 goalCallback
        goalCallback(request_msg);
    }

    // 工具方法
    bool checkReady(const std::string &tag) {
        if (!motion_planning_service_ || !trajectory_execution_service_) {
            RCLCPP_WARN(get_logger(), "[%s] 应用服务未初始化", tag.c_str());
            return false;
        }
        return true;
    }

    void executeTrajectory(const Trajectory &trajectory, const std::string &tag) {
        auto result = trajectory_execution_service_->execute(trajectory);
        if (!result.success) {
            RCLCPP_ERROR(get_logger(), "[%s] 执行失败：%s", tag.c_str(), result.error_message.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "[%s] 执行成功", tag.c_str());
        }
    }

    // 基础设施层
    std::shared_ptr<MoveItAdapter> moveit_adapter_;
    std::shared_ptr<TrajectoryExecutor> trajectory_executor_;

    // 应用服务层
    std::shared_ptr<MotionPlanningService> motion_planning_service_;
    std::shared_ptr<TrajectoryExecutionService> trajectory_execution_service_;

    // ROS订阅者
    rclcpp::Subscription<robot_interfaces::msg::JointConstrainedRequest>::SharedPtr goal_sub_;
    rclcpp::Subscription<robot_interfaces::msg::JointConstrainedTestRequest>::SharedPtr goal_test_sub_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointConstrainedNode>();
    // 初始化组件（延迟初始化）
    node->initializeComponents();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}