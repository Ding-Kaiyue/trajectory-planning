#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "trajectory_planning_v3/infrastructure/adapters/hardware_adapter.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include <memory>
#include <thread>
#include <mutex>

namespace trajectory_planning::infrastructure::controllers {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

/**
 * @brief 硬件轨迹控制器 - FollowJointTrajectory动作服务器
 * 
 * 接收MoveIt发送的轨迹，使用HardwareAdapter执行到真实硬件
 * 同时作为MotorStatusObserver接收硬件状态并发布到/joint_states话题
 */
class HardwareTrajectoryController : public rclcpp::Node, 
                                   public hardware_driver::motor_driver::MotorStatusObserver {
public:
    // 完整构造函数
    explicit HardwareTrajectoryController(
        std::shared_ptr<adapters::HardwareAdapter> hardware_adapter,
        std::shared_ptr<RobotHardware> robot_hw,
        const std::string& interface_name,
        const std::string& controller_name = "arm_controller");
    
    // 简化构造函数（用于观察者模式）
    explicit HardwareTrajectoryController(
        const std::string& interface_name,
        const std::string& controller_name = "arm_controller");

    ~HardwareTrajectoryController();
    
    // 设置硬件适配器（用于解决循环依赖）
    void setHardwareAdapter(std::shared_ptr<adapters::HardwareAdapter> hardware_adapter);

private:
    // 动作服务器回调
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    // 轨迹执行线程
    void execute_trajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    // MotorStatusObserver接口实现
    void on_motor_status_update(const std::string& interface, uint32_t motor_id,
                               const hardware_driver::motor_driver::Motor_Status& status) override;

    // 状态更新和发布的私有辅助函数
    void updateJointState(size_t joint_index,
                         const hardware_driver::motor_driver::Motor_Status& status);
    void publishJointStates();
    
    void on_motor_function_result(const std::string& interface,
                                 uint32_t motor_id,
                                 uint8_t op_code,
                                 bool success) override {
        (void)interface; (void)motor_id; (void)op_code; (void)success;
    }
    
    void on_motor_parameter_result(const std::string& interface,
                                  uint32_t motor_id,
                                  uint16_t address,
                                  uint8_t data_type,
                                  const std::any& data) override {
        (void)interface; (void)motor_id; (void)address; (void)data_type; (void)data;
    }

    // 成员变量
    std::shared_ptr<adapters::HardwareAdapter> hardware_adapter_;
    std::shared_ptr<RobotHardware> robot_hw_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string controller_name_;
    std::string interface_name_;
    std::vector<std::string> joint_names_;
    
    std::atomic<bool> trajectory_executing_;
    std::shared_ptr<GoalHandleFollowJointTrajectory> current_goal_handle_;
    std::mutex goal_handle_mutex_;
    
    // 关节状态存储
    struct JointState {
        double position = 0.0;
        double velocity = 0.0; 
        double effort = 0.0;
    };
    std::vector<JointState> joint_states_;
    std::mutex joint_states_mutex_;
};

} // namespace trajectory_planning::infrastructure::controllers