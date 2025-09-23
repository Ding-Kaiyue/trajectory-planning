cmake_minimum_required(VERSION 3.8)
project(trac_ik_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# include directories
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
)

# library
add_library(trac_ik_plugin SHARED src/trac_ik_kinematics_plugin.cpp)

target_include_directories(trac_ik_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(trac_ik_plugin PUBLIC cxx_std_17)

ament_target_dependencies(trac_ik_plugin
  moveit_core
  pluginlib
  rclcpp
  tf2
  tf2_kdl
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
  trac_ik_lib
  orocos_kdl
  kdl_parser
)

# link libraries
target_link_libraries(trac_ik_plugin ${orocos_kdl_LIBRARIES})

# install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS trac_ik_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES trac_ik_kinematics_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description
pluginlib_export_plugin_description_file(moveit_core trac_ik_kinematics_description.xml)

ament_export_include_directories(
  include
)
ament_export_libraries(
  trac_ik_kinematics_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()