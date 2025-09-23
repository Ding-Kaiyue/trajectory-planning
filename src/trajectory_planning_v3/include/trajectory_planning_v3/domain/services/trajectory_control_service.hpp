#pragma once

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"

namespace trajectory_planning::domain::services {

using trajectory_planning::domain::entities::Trajectory;

/**
 * @brief 轨迹控制服务，负责状态切换逻辑
 */
class TrajectoryControlService {
public:
    explicit TrajectoryControlService(Trajectory& trajectory);

    void start();
    void pause();
    void resume();
    void cancel();
    void complete();

private:
    Trajectory& trajectory_;
};

} // namespace trajectory_planning_v3::domain::services
