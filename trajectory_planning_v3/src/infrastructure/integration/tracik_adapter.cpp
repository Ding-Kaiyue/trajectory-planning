#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

namespace trajectory_planning::infrastructure::integration {

TracIKAdapter::TracIKAdapter(rclcpp::Node::SharedPtr node,
                             const std::string& move_group_name)
    : node_(node), move_group_name_(move_group_name), moveit_adapter_(nullptr),
      chain_initialized_(false), ik_solver_(nullptr) {}

void TracIKAdapter::setMoveItAdapter(MoveItAdapter* moveit_adapter) {
	moveit_adapter_ = moveit_adapter;
}

bool TracIKAdapter::initializeKDLChain(const std::string& urdf_string,
                                       const std::string& base_link,
                                       const std::string& tip_link) {
	if (chain_initialized_) {
		return true;  // Already initialized
	}

	try {
		if (urdf_string.empty()) {
			RCLCPP_ERROR(node_->get_logger(),
						 "URDF string is empty");
			return false;
		}

		// Parse URDF to KDL tree
		KDL::Tree tree;
		if (!kdl_parser::treeFromString(urdf_string, tree)) {
			RCLCPP_ERROR(node_->get_logger(),
						 "Failed to parse URDF to KDL tree");
			return false;
		}

		// Get the chain from base_link to end_effector_link
		if (!tree.getChain(base_link, tip_link, kdl_chain_)) {
			RCLCPP_ERROR(node_->get_logger(),
						 "Failed to get KDL chain from %s to %s",
						 base_link.c_str(), tip_link.c_str());
			return false;
		}

		chain_initialized_ = true;
		RCLCPP_INFO(node_->get_logger(),
					"KDL chain initialized: %s -> %s with %u joints",
					base_link.c_str(), tip_link.c_str(),
					kdl_chain_.getNrOfJoints());
		return true;

	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(),
					 "Exception in initializeKDLChain: %s", e.what());
		return false;
	}
}

bool TracIKAdapter::initializeSolver(const std::string& arm_type) {
	if (!chain_initialized_) {
		RCLCPP_ERROR(node_->get_logger(),
					 "KDL chain not initialized. Call initializeKDLChain() first.");
		return false;
	}

	try {
		// Load joint limits from MoveItAdapter if available
		std::vector<std::pair<double, double>> limits;
		if (moveit_adapter_) {
			limits = moveit_adapter_->getJointLimits(arm_type);
		}

		if (limits.empty()) {
			RCLCPP_WARN(node_->get_logger(),
						"Joint limits not loaded from MoveItAdapter, using default limits");
			// Use default limits if not available
			for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
				limits.emplace_back(-M_PI, M_PI);
			}
		}

		if (limits.size() != kdl_chain_.getNrOfJoints()) {
			RCLCPP_ERROR(node_->get_logger(),
						 "Joint limits size (%zu) doesn't match chain joints (%u)",
						 limits.size(), kdl_chain_.getNrOfJoints());
			return false;
		}

		// Initialize joint limit arrays for persistent solver
		q_min_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
		q_max_ = KDL::JntArray(kdl_chain_.getNrOfJoints());

		for (size_t i = 0; i < limits.size(); ++i) {
			q_min_(i) = limits[i].first;
			q_max_(i) = limits[i].second;
		}

		// Create persistent TRAC_IK solver (only once per MoveL planning)
		ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
			kdl_chain_, q_min_, q_max_,
			0.005,   // max time per attempt (5ms)
			1e-5);  // epsilon

		RCLCPP_INFO(node_->get_logger(),
					"TRAC_IK persistent solver initialized for %s (%u joints)",
					arm_type.c_str(), kdl_chain_.getNrOfJoints());
		return true;

	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(),
					 "Exception in initializeSolver: %s", e.what());
		return false;
	}
}

bool TracIKAdapter::computeIK(const geometry_msgs::msg::Pose& target_pose,
                              const std::vector<double>& seed_state,
                              std::vector<double>& solution) {
	try {
		// Check if chain is initialized
		if (!chain_initialized_) {
			RCLCPP_ERROR(node_->get_logger(),
						 "KDL chain not initialized. Call initializeKDLChain() first.");
			return false;
		}

		// Check if solver is initialized
		if (!ik_solver_) {
			RCLCPP_ERROR(node_->get_logger(),
						 "TRAC_IK solver not initialized. Call initializeSolver() first.");
			return false;
		}

		// Validate seed state
		if (seed_state.size() != kdl_chain_.getNrOfJoints()) {
			RCLCPP_ERROR(node_->get_logger(),
						 "Seed state size (%zu) doesn't match chain joints (%u)",
						 seed_state.size(), kdl_chain_.getNrOfJoints());
			return false;
		}

		// Convert target pose to KDL::Frame
		KDL::Frame target_frame;
		target_frame.p.x(target_pose.position.x);
		target_frame.p.y(target_pose.position.y);
		target_frame.p.z(target_pose.position.z);

		KDL::Rotation rot = KDL::Rotation::Quaternion(
			target_pose.orientation.x, target_pose.orientation.y,
			target_pose.orientation.z, target_pose.orientation.w);
		target_frame.M = rot;

		// Convert seed state to KDL::JntArray
		KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
		KDL::JntArray q_out(kdl_chain_.getNrOfJoints());
		for (size_t i = 0; i < seed_state.size(); ++i) {
			q_init(i) = seed_state[i];
		}

		// Solve IK using persistent solver
		int ret = ik_solver_->CartToJnt(q_init, target_frame, q_out);
		if (ret < 0) {
			RCLCPP_WARN(node_->get_logger(),
						 "IK solving failed (error code: %d): target_pos=[%.4f, %.4f, %.4f], "
						 "target_quat=[%.4f, %.4f, %.4f, %.4f], seed=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
						 ret, target_pose.position.x, target_pose.position.y, target_pose.position.z,
						 target_pose.orientation.x, target_pose.orientation.y,
						 target_pose.orientation.z, target_pose.orientation.w,
						 seed_state[0], seed_state[1], seed_state[2],
						 seed_state[3], seed_state[4], seed_state[5]);
			return false;
		}

		// Convert solution back to std::vector
		solution.clear();
		solution.resize(kdl_chain_.getNrOfJoints());
		for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
			solution[i] = q_out(i);
		}

		RCLCPP_DEBUG(node_->get_logger(),
					 "IK success: solution=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
					 solution[0], solution[1], solution[2],
					 solution[3], solution[4], solution[5]);

		return true;
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(),
					 "Exception in TracIKAdapter::computeIK: %s", e.what());
		return false;
	}
}

bool TracIKAdapter::computeIKClosest(const geometry_msgs::msg::Pose& target_pose,
                                      const std::vector<double>& seed_state,
                                      std::vector<double>& solution,
                                      int num_attempts)
{
	try {
		if (!chain_initialized_) {
			RCLCPP_ERROR(node_->get_logger(),
						 "KDL chain not initialized");
			return false;
		}

		if (!ik_solver_) {
			RCLCPP_ERROR(node_->get_logger(),
						 "TRAC_IK solver not initialized");
			return false;
		}

		if (seed_state.size() != kdl_chain_.getNrOfJoints()) {
			RCLCPP_ERROR(node_->get_logger(),
						 "Seed state size mismatch");
			return false;
		}

		// Convert target pose to KDL::Frame
		KDL::Frame target_frame;
		target_frame.p.x(target_pose.position.x);
		target_frame.p.y(target_pose.position.y);
		target_frame.p.z(target_pose.position.z);

		KDL::Rotation rot = KDL::Rotation::Quaternion(
			target_pose.orientation.x, target_pose.orientation.y,
			target_pose.orientation.z, target_pose.orientation.w);
		target_frame.M = rot;

		// Convert seed state to KDL::JntArray
		KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
		KDL::JntArray q_out(kdl_chain_.getNrOfJoints());
		for (size_t i = 0; i < seed_state.size(); ++i) {
			q_init(i) = seed_state[i];
		}

		// Try multiple times and keep track of best solution
		double best_distance = std::numeric_limits<double>::max();
		std::vector<double> best_solution;
		bool found_any = false;

		for (int attempt = 0; attempt < num_attempts; ++attempt) {
			int ret = ik_solver_->CartToJnt(q_init, target_frame, q_out);
			if (ret < 0) {
				// This attempt failed, try again
				continue;
			}

			found_any = true;

			// Normalize solution to closest representation of seed
			std::vector<double> q_normalized(kdl_chain_.getNrOfJoints());
			for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
				q_normalized[i] = q_out(i);
				// Map to equivalent angle closest to seed
				double candidate = q_normalized[i];
				// Try wrapping by ±2π to find closest representation
				while (candidate - seed_state[i] > M_PI)
					candidate -= 2.0 * M_PI;
				while (candidate - seed_state[i] < -M_PI)
					candidate += 2.0 * M_PI;
				q_normalized[i] = candidate;
			}

			// Calculate L2 distance from seed (after normalization)
			double distance = 0.0;
			for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
				double delta = q_normalized[i] - seed_state[i];
				distance += delta * delta;
			}
			distance = std::sqrt(distance);

			RCLCPP_DEBUG(node_->get_logger(),
						 "IK attempt %d: distance=%.6f, q3_orig=%.4f, q3_norm=%.4f, seed=%.4f",
						 attempt + 1, distance, q_out(3), q_normalized[3], seed_state[3]);

			// Keep the solution closest to seed
			if (distance < best_distance) {
				best_distance = distance;
				best_solution = q_normalized;
			}
		}

		if (!found_any) {
			RCLCPP_WARN(node_->get_logger(),
						 "IK solving failed after %d attempts", num_attempts);
			return false;
		}

		solution = best_solution;

		RCLCPP_DEBUG(node_->get_logger(),
					 "IK success (closest): distance=%.6f, solution=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
					 best_distance,
					 solution[0], solution[1], solution[2],
					 solution[3], solution[4], solution[5]);

		return true;

	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(),
					 "Exception in TracIKAdapter::computeIKClosest: %s", e.what());
		return false;
	}
}

}  // namespace trajectory_planning::infrastructure::integration
