cmake_minimum_required(VERSION 3.8)
project(trajectory_planning_v3 VERSION 1.0.0 LANGUAGES CXX)

# 默认禁用测试
if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set custom paths for local installations
list(APPEND CMAKE_PREFIX_PATH
  "$ENV{HOME}/Bags/moveit2_ws/install"
  "$ENV{HOME}/Bags/trac_ik_ws/install"
)

# Optional TRAC-IK support
option(USE_TRAC_IK "Enable TRAC-IK support" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_planning_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(hardware_driver REQUIRED)

# Conditional TRAC-IK dependency
if(USE_TRAC_IK)
    find_package(trac_ik_lib QUIET)
    if(trac_ik_lib_FOUND)
        message(STATUS "TRAC-IK found, enabling TRAC-IK support")
        add_compile_definitions(USE_TRAC_IK)
        set(TRAC_IK_AVAILABLE TRUE)
    else()
        message(WARNING "TRAC-IK requested but not found, disabling TRAC-IK support")
        set(TRAC_IK_AVAILABLE FALSE)
    endif()
else()
    message(STATUS "TRAC-IK support disabled")
    set(TRAC_IK_AVAILABLE FALSE)
endif()

# Include directories
include_directories(include)

# Domain entities sources
set(DOMAIN_ENTITY_SOURCES
  src/domain/entities/robot_state.cpp
  src/domain/entities/trajectory.cpp
  src/domain/entities/constraint.cpp
)

# Domain services sources
set(DOMAIN_SERVICE_SOURCES
  src/domain/services/trajectory_validation_service.cpp
  src/domain/services/collision_checker.cpp
  src/domain/services/trajectory_control_service.cpp
  src/domain/services/kinematics_service.cpp
)

# Application orchestrators sources
set(APPLICATION_ORCHESTRATOR_SOURCES
  src/application/orchestrators/safety_orchestrator.cpp
  src/application/orchestrators/single_arm_orchestrator.cpp
  src/application/orchestrators/dual_arm_orchestrator.cpp
)

# Application use cases sources
set(APPLICATION_USE_CASE_SOURCES
  src/application/use_cases/safety_motion_planning.cpp
  src/application/use_cases/plan_linear_motion.cpp
  src/application/use_cases/plan_joint_motion.cpp
  src/application/use_cases/plan_arc_motion.cpp
)

# Application services sources
set(APPLICATION_SERVICE_SOURCES
  src/application/services/robot_control_service.cpp
  src/application/services/motion_planning_service.cpp
)

# Infrastructure sources
set(INFRASTRUCTURE_SOURCES
  src/infrastructure/integration/moveit_adapter.cpp
  src/infrastructure/integration/tracik_adapter.cpp
  src/infrastructure/integration/ros2_bridge.cpp
  src/infrastructure/planning/strategies/position_planning_strategy.cpp
  src/infrastructure/planning/strategies/orientation_planning_strategy.cpp
  src/infrastructure/planning/strategies/movel_planning_strategy.cpp
  src/infrastructure/planning/strategies/differential_planning_strategy.cpp
  src/infrastructure/planning/strategies/movej_planning_strategy.cpp
  src/infrastructure/planning/strategies/movec_planning_strategy.cpp
  src/infrastructure/planning/strategies/joint_constrained_planning_strategy.cpp
  src/infrastructure/execution/trajectory_executor.cpp
  src/infrastructure/adapters/ros_message_adapter.cpp
  src/infrastructure/adapters/moveit_message_adapter.cpp
  src/infrastructure/adapters/hardware_adapter.cpp
  src/infrastructure/adapters/perception_adapter.cpp
  src/infrastructure/controllers/hardware_trajectory_controller.cpp
)

# Create main library
add_library(${PROJECT_NAME}
  ${DOMAIN_ENTITY_SOURCES}
  ${DOMAIN_SERVICE_SOURCES}
  ${APPLICATION_ORCHESTRATOR_SOURCES}
  ${APPLICATION_USE_CASE_SOURCES}
  ${APPLICATION_SERVICE_SOURCES}
  ${INFRASTRUCTURE_SOURCES}
)

# Link directories and libraries for the main library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link external libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  rclcpp_action
  control_msgs
  trajectory_planning_interfaces
  ament_index_cpp
  hardware_driver
)

# Add TRAC-IK dependency if available
if(TRAC_IK_AVAILABLE)
  target_link_libraries(${PROJECT_NAME} trac_ik_lib::trac_ik_lib)
endif()

# 显式链接hardware_driver库到主库
target_link_libraries(${PROJECT_NAME} yaml-cpp)


# Node executables (只保留硬件相关的节点)


# Hardware adapter test executable (仅在有实际硬件驱动时构建)
if(TARGET hardware_driver::hardware_driver_canfd)
    add_executable(hardware_adapter_test test/hardware_adapter_test.cpp)
    target_link_libraries(hardware_adapter_test ${PROJECT_NAME} hardware_driver::hardware_driver_canfd)
    ament_target_dependencies(hardware_adapter_test
      rclcpp
      rclcpp_action
      control_msgs
      sensor_msgs
    )

    # Hardware trajectory controller node
    add_executable(hardware_trajectory_controller_node src/nodes/hardware_trajectory_controller_node.cpp)
    target_link_libraries(hardware_trajectory_controller_node ${PROJECT_NAME} hardware_driver::hardware_driver_canfd)
    ament_target_dependencies(hardware_trajectory_controller_node
      rclcpp
      rclcpp_action
      control_msgs
    )

    set(HAS_HARDWARE_TARGETS TRUE)
else()
    message(STATUS "Real hardware driver not found, skipping hardware-dependent targets")
    set(HAS_HARDWARE_TARGETS FALSE)
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装基础节点 (目前没有基础节点)
set(INSTALL_TARGETS)

# 如果构建了硬件相关目标，添加到安装列表
if(HAS_HARDWARE_TARGETS)
    list(APPEND INSTALL_TARGETS hardware_trajectory_controller_node)
endif()

install(TARGETS ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config files
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  tf2_eigen
  tf2_geometry_msgs
  rclcpp_action
  control_msgs
  trajectory_planning_interfaces
  hardware_driver
)

# 只在TRAC-IK可用时导出trac_ik_lib依赖
if(TRAC_IK_AVAILABLE)
  ament_export_dependencies(trac_ik_lib)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
