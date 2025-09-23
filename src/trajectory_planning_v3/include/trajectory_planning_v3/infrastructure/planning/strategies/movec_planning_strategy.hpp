#pragma once

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <memory>
#include <string>
#include <optional>

namespace trajectory_planning::infrastructure::planning {

/**
 * @brief MoveC 规划策略
 * 
 * 将目标末端位姿生成连续的圆弧轨迹 (Trajectroy)。
 * 可以通过设置最大速度和加速度生成平滑轨迹。
 */
class MoveCPlanningStrategy {
public:
    /**
     * @brief MoveC 的圆弧类路线
     */
    enum class MoveCRoute {
        // Bezier,     // 贝塞尔曲线
        ARC,        // 圆弧
        CIRCLE,     // 整圆轨迹
        // Helix,      // 螺旋轨迹
        // Spline,     // 样条曲线
        // LineBlend,  // 直线 + 圆弧段混合
        CIRCLETHROUGH3POINTS, // 通过三个点的圆轨迹
    };

    explicit MoveCPlanningStrategy(integration::MoveItAdapter& moveit_adapter)
        : moveit_adapter_(moveit_adapter) {}

    /**
     * @brief 规划经过中间点的圆弧轨迹
     * @param start_pose 起点位姿
     * @param via_point  中间经过点位姿
     * @param goal_pose  目标位姿
     * @return 生成的Trajectory对象
     */
    domain::entities::Trajectory planArc(const geometry_msgs::msg::Pose& start_pose,
                                        const geometry_msgs::msg::Pose& via_point,
                                        const geometry_msgs::msg::Pose& goal_pose);

    /**
     * @brief 规划整圆轨迹 (CIRCLE模式: center为圆心, goal定义半径)
     * @param center       圆心位姿
     * @param radius_point 定义半径的点位姿
     * @return 生成的Trajectory对象
     */
    domain::entities::Trajectory planCircle(const geometry_msgs::msg::Pose& center,
                                           const geometry_msgs::msg::Pose& radius_point);

    /**
     * @brief 规划通过三点的圆弧轨迹
     * @param point1 第一个点位姿
     * @param point2 第二个点位姿
     * @param point3 第三个点位姿
     * @return 生成的Trajectory对象
     */
    domain::entities::Trajectory planCircleThrough3Points(const geometry_msgs::msg::Pose& point1,
                                                          const geometry_msgs::msg::Pose& point2,
                                                          const geometry_msgs::msg::Pose& point3);

    /**
     * @brief 规划笛卡尔空间贝塞尔曲线
     * @param start   起点位姿
     * @param ctrl1   中间点1
     * @param ctrl2   中间点2
     * @param goal    目标位姿
     * @return 生成的Trajectory对象
     */
    domain::entities::Trajectory planBezier(const geometry_msgs::msg::Pose& start,
                                             const geometry_msgs::msg::Pose& ctrl1,
                                             const geometry_msgs::msg::Pose& ctrl2,
                                             const geometry_msgs::msg::Pose& goal
                                            );

private:
    infrastructure::integration::MoveItAdapter& moveit_adapter_;

    domain::entities::Trajectory convertTrajectoryType(const moveit_msgs::msg::RobotTrajectory& moveit_traj) const;
};

}   // namespace trajectory_planning::infrastructure::planning