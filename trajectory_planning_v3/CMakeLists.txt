cmake_minimum_required(VERSION 3.8)
project(trajectory_planning_v3 VERSION 1.0.0 LANGUAGES CXX)

# Ensure consistent behavior for option() vs normal variables (suppress CMP0077 developer warnings)
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

# Build options
option(CMAKE_INTERPROCEDURAL_OPTIMIZATION "Enable LTO for optimization" OFF)

# Memory-efficient linker settings for low-memory systems
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-keep-memory")
set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-keep-memory")

# Use lld linker if available (reduces link-time memory by 30-50%)
find_program(LLD_LINKER ld.lld)
if(LLD_LINKER)
  message(STATUS "Found lld linker: ${LLD_LINKER}")
  set(CMAKE_LINKER ${LLD_LINKER})
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=lld")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=lld")
else()
  message(STATUS "lld linker not found, using system default")
  message(STATUS "To install lld: sudo apt install lld")
endif()

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
# Set custom paths for local installations (if they exist)
if(EXISTS "$ENV{HOME}/Bags/moveit2_ws/install")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Bags/moveit2_ws/install")
endif()
if(EXISTS "$ENV{HOME}/Bags/trac_ik_ws/install")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Bags/trac_ik_ws/install")
endif()

# Optional TRAC-IK support
option(USE_TRAC_IK "Enable TRAC-IK support" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
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

# ===============================
# Main library with all sources
# ===============================

# Collect all source files
file(GLOB_RECURSE DOMAIN_SOURCES
  src/domain/entities/*.cpp
  src/domain/services/*.cpp
)

file(GLOB_RECURSE APPLICATION_SOURCES
  src/application/orchestrators/*.cpp
  src/application/use_cases/*.cpp
  src/application/services/*.cpp
)

file(GLOB_RECURSE INFRASTRUCTURE_SOURCES
  src/infrastructure/integration/*.cpp
  src/infrastructure/planning/strategies/*.cpp
  src/infrastructure/execution/*.cpp
  src/infrastructure/adapters/*.cpp
  src/infrastructure/controllers/*.cpp
)

# Create main shared library from all sources
add_library(${PROJECT_NAME} SHARED
  ${DOMAIN_SOURCES}
  ${APPLICATION_SOURCES}
  ${INFRASTRUCTURE_SOURCES}
)

# Link directories and libraries for the main library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link external libraries to final target
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
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

# Explicit linking for final library
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  hardware_driver::hardware_driver_canfd
)

# Add TRAC-IK dependency if available
if(TRAC_IK_AVAILABLE)
  target_link_libraries(${PROJECT_NAME} trac_ik_lib::trac_ik_lib)
endif()


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
