#include "trajectory_planning_v3/domain/services/trajectory_control_service.hpp"

namespace trajectory_planning::domain::services {

TrajectoryControlService::TrajectoryControlService(Trajectory& trajectory)
    : trajectory_(trajectory) {}

void TrajectoryControlService::start() {
	trajectory_.set_state(Trajectory::State::Running);
}

void TrajectoryControlService::pause() {
	if (trajectory_.state() == Trajectory::State::Running ||
	    trajectory_.state() == Trajectory::State::Resumed) {
		trajectory_.set_state(Trajectory::State::Paused);
	}
}

void TrajectoryControlService::resume() {
	if (trajectory_.state() == Trajectory::State::Paused) {
		trajectory_.set_state(Trajectory::State::Resumed);
	}
}

void TrajectoryControlService::cancel() {
	trajectory_.set_state(Trajectory::State::Cancelled);
}

void TrajectoryControlService::complete() {
	trajectory_.set_state(Trajectory::State::Completed);
}

}  // namespace trajectory_planning::domain::services
