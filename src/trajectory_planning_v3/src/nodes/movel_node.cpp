#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/application/services/trajectory_execution_service.hpp"
#include "trajectory_planning_v3/infrastructure/execution/trajectory_executor.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/planning/strategies/movel_planning_strategy.hpp"

using namespace trajectory_planning::application::services;
using namespace trajectory_planning::infrastructure::execution;
using namespace trajectory_planning::infrastructure::integration;
using namespace trajectory_planning::infrastructure::planning;
using namespace trajectory_planning::domain::entities;
using namespace trajectory_planning::domain::value_objects;

// ros2 topic pub --once /movel_goals_rpy geometry_msgs/msg/Vector3 "{x: -80.0, y: 0.0, z: -90.0}"

/**
 * @brief MoveL节点，订阅目标末端位姿，规划并执行轨迹
 * @note 该节点依赖MoveIt进行规划，需确保MoveIt配置正确
 * @author Ding Kaiyue
 * @date 2025-09-13
 * @version 1.0
 */
class MoveLNode : public rclcpp::Node {
public:
    MoveLNode() : Node("movel_node") {
        // ---- 1. 声明参数 ----
        this->declare_parameter<std::string>("move_group", "arm");
        this->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
        this->declare_parameter<double>("planning_time", 5.0);
        this->declare_parameter<double>("velocity_scaling", 0.2);
        this->declare_parameter<double>("acceleration_scaling", 0.2);
        this->declare_parameter<double>("goal_tolerance", 0.001);

        // ---- 2. 订阅 /movel_goals 和 /movel_goals_rpy ----
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/movel_goals", qos,
            std::bind(&MoveLNode::goalCallback, this, std::placeholders::_1));

        // 订阅RPY格式的目标 (x, y, z, roll, pitch, yaw)
        goal_rpy_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/movel_goals_rpy", qos,
            std::bind(&MoveLNode::goalRPYCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MoveL Node ready. Publish to:\n"
            "  /movel_goals(Pose) - Full pose message\n"
            "  /movel_goals_rpy(Vector3) - RPY test message\n");
    }
    
    void initializeComponents() {
        if (!motion_planning_service_) {
            // 创建 MoveItAdapter, MoveLPlanningStrategy 和 TrajectoryExecutor
            auto group_name = this->get_parameter("move_group").as_string();

            // 初始化基础设施层
            moveit_adapter_ = std::make_shared<MoveItAdapter>(shared_from_this(), group_name);
            trajectory_executor_ = std::make_shared<TrajectoryExecutor>(*moveit_adapter_);

            auto movel_strategy = std::make_shared<MoveLPlanningStrategy>(*moveit_adapter_);

            // 初始化应用服务
            motion_planning_service_ = std::make_shared<MotionPlanningService>(
                moveit_adapter_, shared_from_this()
            );

            // 注册需要的策略
            motion_planning_service_->registerMoveLStrategy(movel_strategy);

            trajectory_execution_service_ = std::make_shared<TrajectoryExecutionService>(
                trajectory_executor_, this->get_logger()
            );

            RCLCPP_INFO(this->get_logger(), "MoveItAdapter, MoveLPlanningStrategy and TrajectoryExecutor initialized.");
        }
    }

private:
    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!checkReady("MoveL")) return;

        auto result = motion_planning_service_->planLinearMotion(*msg);

        if (result.success) {
            executeTrajectory(result.trajectory, "MoveL");
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveL 规划失败：%s", result.error_message.c_str());
        }
    }

    void goalRPYCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Vector3: x=x位置, y=y位置, z=z位置 (需要另外输入位置)
        // 此处假设msg为RPY角度(roll, pitch, yaw)，位置需要另外设定

        // 获取当前位置作为目标位置
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_x_;
        target_pose.position.y = current_y_;
        target_pose.position.z = current_z_;

        // 将RPY转换为四元数
        tf2::Quaternion q;
        q.setRPY(msg->x * M_PI / 180.0, msg->y * M_PI / 180.0, msg->z * M_PI / 180.0);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(), "Received RPY goal: roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                    msg->x, msg->y, msg->z);

        // 调用原有的位姿回调函数
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(target_pose);
        goalCallback(pose_msg);
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
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_rpy_sub_;

    // 测试
    double current_x_ = 0.2;
    double current_y_ = 0.0;
    double current_z_ = 0.6;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveLNode>();
    // 初始化组件（延迟初始化）
    node->initializeComponents();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}