#include "trajectory_planning_v3/infrastructure/adapters/ros_message_adapter.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_position.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_velocity.hpp"
#include "trajectory_planning_v3/domain/value_objects/joint_acceleration.hpp"
#include "trajectory_planning_v3/domain/value_objects/duration.hpp"

namespace trajectory_planning::infrastructure::adapters {

trajectory_planning::domain::entities::Trajectory
RosMessageAdapter::fromRosMessage(const trajectory_msgs::msg::JointTrajectory &msg) {
    trajectory_planning::domain::entities::Trajectory trajectory;
    
    // 转换每个轨迹点
    for (const auto& point : msg.points) {
        trajectory_planning::domain::entities::TrajectoryPoint traj_point{
            .position = trajectory_planning::domain::value_objects::JointPosition(point.positions),
            .velocity = trajectory_planning::domain::value_objects::JointVelocity(
                point.velocities.empty() ? std::vector<double>(point.positions.size(), 0.0) : point.velocities
            ),
            .acceleration = trajectory_planning::domain::value_objects::JointAcceleration(
                point.accelerations.empty() ? std::vector<double>(point.positions.size(), 0.0) : point.accelerations
            ),
            .time_from_start = trajectory_planning::domain::value_objects::Duration(
                static_cast<double>(point.time_from_start.sec) + static_cast<double>(point.time_from_start.nanosec) * 1e-9
            ),
            .progress_ratio = 0.0
        };
        trajectory.add_point(traj_point);
    }
    
    // 计算进度比例
    trajectory.compute_progress_ratios();
    return trajectory;
}

trajectory_msgs::msg::JointTrajectory 
RosMessageAdapter::toRosMessage(
    const trajectory_planning::domain::entities::Trajectory &trajectory,
    const std::vector<std::string> &joint_names) {

    trajectory_msgs::msg::JointTrajectory msg;

    msg.joint_names = joint_names;

    // 转换轨迹点
    for (const auto& point : trajectory.points()) {
        trajectory_msgs::msg::JointTrajectoryPoint jt_point;
        jt_point.positions = point.position.values();
        jt_point.velocities = point.velocity.values();
        jt_point.accelerations = point.acceleration.values();
        
        jt_point.time_from_start.sec = static_cast<int32_t>(point.time_from_start.seconds());
        jt_point.time_from_start.nanosec = static_cast<uint32_t>(
            (point.time_from_start.seconds() - jt_point.time_from_start.sec) * 1e9
        );
        
        msg.points.push_back(jt_point);
    }
    
    return msg;
}

}   // namespace trajectory_planning::infrastructure::adapters