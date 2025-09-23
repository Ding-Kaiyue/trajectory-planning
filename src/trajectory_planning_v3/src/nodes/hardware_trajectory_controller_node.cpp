#include <rclcpp/rclcpp.hpp>
#include "trajectory_planning_v3/infrastructure/controllers/hardware_trajectory_controller.hpp"
#include "trajectory_planning_v3/infrastructure/adapters/hardware_adapter.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include <map>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("hardware_trajectory_controller_node"), 
                "Starting hardware trajectory controller node");
    
    try {
        // 配置电机接口和ID映射
        std::map<std::string, std::vector<uint32_t>> motor_config = {
            {"can0", {1, 2, 3, 4, 5, 6}}    // can0接口上的电机
        };
        
        // 创建CANFD电机驱动
        auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
        
        // 先创建硬件轨迹控制器（作为观察者）
        auto controller = std::make_shared<trajectory_planning::infrastructure::controllers::HardwareTrajectoryController>(
            "can0", "arm_controller");
        
        // 创建RobotHardware，将controller作为观察者传入
        auto robot_hw = std::make_shared<RobotHardware>(motor_driver, motor_config, controller);
        
        // 创建硬件适配器，直接传入RobotHardware
        auto hardware_adapter = std::make_shared<trajectory_planning::infrastructure::adapters::HardwareAdapter>(
            robot_hw, "can0", 6);
        
        // 设置controller的hardware_adapter
        controller->setHardwareAdapter(hardware_adapter);
        
        RCLCPP_INFO(rclcpp::get_logger("hardware_trajectory_controller_node"), 
                    "Hardware trajectory controller initialized successfully");
        
        // 禁用所有电机，避免异响
        // RCLCPP_INFO(rclcpp::get_logger("hardware_trajectory_controller_node"), 
        //             "Disabling all motors to prevent noise");
        // hardware_adapter->disableAllJoints();
        
        // 启动节点
        rclcpp::spin(controller);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("hardware_trajectory_controller_node"), 
                     "Failed to initialize: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}