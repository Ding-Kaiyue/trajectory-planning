#include "trajectory_planning_v3/infrastructure/adapters/hardware_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/controllers/hardware_trajectory_controller.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cmath>

using namespace trajectory_planning::infrastructure::adapters;
using namespace trajectory_planning::infrastructure::controllers;

int main(int argc, char* argv[]) {
    std::cout << "=== HardwareAdapter 测试程序 ===" << std::endl;
    
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    try {
        // 1. 创建电机驱动
        auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
        
        // 2. 配置电机映射
        std::map<std::string, std::vector<uint32_t>> motor_config = {
            {"can0", {1, 2, 3, 4, 5, 6}}  // can0接口上的6个电机
        };
        
        // 3. 创建硬件轨迹控制器作为观察者
        auto controller = std::make_shared<HardwareTrajectoryController>("can0", "arm_controller");
        
        // 4. 创建RobotHardware，将controller作为观察者
        auto robot_hw = std::make_shared<RobotHardware>(
            motor_driver, 
            motor_config, 
            controller
        );
        
        // 5. 创建HardwareAdapter，直接传入RobotHardware
        auto hardware_adapter = std::make_shared<HardwareAdapter>(
            robot_hw,  // 直接传入RobotHardware
            "can0",    // 接口名称
            6          // 关节数量
        );
        
        // 6. 设置controller的hardware_adapter
        controller->setHardwareAdapter(hardware_adapter);
        
        std::cout << "硬件适配器初始化完成！" << std::endl;
        
        // 等待一下，让状态更新开始工作
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 测试1: 移动到起始位置
        std::cout << "\n=== 测试1: 移动到起始位置 ===" << std::endl;
        std::cout << "按Enter开始移动到起始位置...";
        std::cin.get();
        
        std::vector<double> start_positions = {0.1730 * 57.2957, 0.0000, -0.0006 * 57.2957, 0.0000, 0.0003 * 57.2957, -0.0002 * 57.2957};
        bool result = hardware_adapter->sendPositionCommand(start_positions);
        std::cout << "移动到初始位置命令发送" << (result ? "✅ 成功" : "❌ 失败") << std::endl;
        
        std::cout << "等待移动完成..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 测试2: MoveIt轨迹执行
        std::cout << "\n=== 测试2: MoveIt轨迹执行 ===" << std::endl;
        std::cout << "按Enter开始MoveIt轨迹执行测试...";
        std::cin.get();
        
        // 创建MoveIt轨迹（完美复现你的轨迹数据 - 18个点，6个关节）
        trajectory_planning::domain::entities::Trajectory trajectory;
        
        // MoveIt轨迹位置数据（时间，关节1-6位置）
        std::vector<std::vector<double>> moveit_positions = {
            {0.000, 0.1730, 0.0000, -0.0006, 0.0000, 0.0003, -0.0002},  // time, j1-j6
            {0.100, 0.1680, 0.0000, -0.0006, -0.0000, 0.0003, -0.0002},
            {0.200, 0.1530, 0.0000, -0.0006, -0.0000, 0.0003, -0.0002},
            {0.300, 0.1280, 0.0000, -0.0006, -0.0000, 0.0003, -0.0002},
            {0.400, 0.0967, 0.0000, -0.0006, -0.0000, 0.0003, -0.0002},
            {0.500, 0.0653, 0.0000, -0.0006, -0.0000, 0.0004, -0.0002},
            {0.600, 0.0339, 0.0000, -0.0006, -0.0000, 0.0004, -0.0002},
            {0.700, 0.0025, 0.0000, -0.0006, -0.0000, 0.0004, -0.0002},
            {0.800, -0.0289, 0.0000, -0.0006, -0.0000, 0.0004, -0.0002},
            {0.900, -0.0603, 0.0000, -0.0006, -0.0000, 0.0004, -0.0002},
            {1.000, -0.0917, 0.0001, -0.0006, -0.0000, 0.0004, -0.0003},
            {1.100, -0.1231, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003},
            {1.200, -0.1545, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003},
            {1.300, -0.1859, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003},
            {1.400, -0.2129, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003},
            {1.500, -0.2300, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003},
            {1.600, -0.2371, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003},
            {1.621, -0.2373, 0.0001, -0.0006, -0.0001, 0.0004, -0.0003}
        };
        
        std::cout << "创建包含 " << moveit_positions.size() << " 个轨迹点的MoveIt轨迹..." << std::endl;
        
        for (size_t i = 0; i < moveit_positions.size(); i++) {
            const auto& data = moveit_positions[i];
            double time = data[0];
            
            // 将弧度转换为度数 (rad * 180 / π)
            std::vector<double> positions_rad = {data[1], data[2], data[3], data[4], data[5], data[6]};
            std::vector<double> positions_deg(6);
            for (size_t j = 0; j < 6; j++) {
                positions_deg[j] = positions_rad[j] * 180.0 / M_PI;
            }
            
            trajectory_planning::domain::entities::TrajectoryPoint point {
                .position = trajectory_planning::domain::value_objects::JointPosition(positions_deg),
                .velocity = trajectory_planning::domain::value_objects::JointVelocity({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
                .acceleration = trajectory_planning::domain::value_objects::JointAcceleration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
                .time_from_start = trajectory_planning::domain::value_objects::Duration(time),
                .progress_ratio = static_cast<double>(i) / (moveit_positions.size() - 1)
            };
            trajectory.add_point(point);
            
            std::cout << "  Point[" << i << "] time=" << std::fixed << std::setprecision(3) << time 
                     << "s pos_rad=[" << std::setprecision(4) 
                     << positions_rad[0] << ", " << positions_rad[1] << ", " << positions_rad[2] << ", "
                     << positions_rad[3] << ", " << positions_rad[4] << ", " << positions_rad[5] 
                     << "] pos_deg=[" << std::setprecision(2)
                     << positions_deg[0] << ", " << positions_deg[1] << ", " << positions_deg[2] << ", "
                     << positions_deg[3] << ", " << positions_deg[4] << ", " << positions_deg[5] << "]" << std::endl;
        }
        
        result = hardware_adapter->executeTrajectory(trajectory);
        std::cout << "轨迹执行" << (result ? "✅ 开始成功" : "❌ 失败") << std::endl;
        
        if (result) {
            std::cout << "MoveIt轨迹正在执行，预计时长1.621秒..." << std::endl;
            // 等待轨迹执行完成，加上一些余量
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            std::cout << "MoveIt轨迹执行完成！" << std::endl;
        }
        
        // 失能电机
        std::cout << "失能所有关节..." << std::endl;
        hardware_adapter->disableAllJoints();
        std::cout << "✅ 所有关节已失能" << std::endl; 
        
        std::cout << "\n=== 所有测试完成 ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    // 清理ROS2
    rclcpp::shutdown();
    
    return 0;
}