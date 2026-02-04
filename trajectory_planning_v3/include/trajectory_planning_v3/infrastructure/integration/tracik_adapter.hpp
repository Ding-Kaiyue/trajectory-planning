#pragma once

#include <memory>
#include <vector>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>

namespace trajectory_planning::infrastructure::integration {

/**
 * @brief TRAC_IK 适配器 - 用于MoveL的真笛卡尔规划IK求解
 *
 * 使用TRAC_IK求解器进行逆运动学计算，支持连续的IK求解链
 */
class TracIKAdapter {
public:
	explicit TracIKAdapter(rclcpp::Node::SharedPtr node,
	                       const std::string& move_group_name);

	/**
	 * @brief 初始化 KDL 链（使用提供的 URDF）
	 * @param urdf_string URDF 字符串
	 * @param base_link 基础链接名称
	 * @param tip_link 末端链接名称
	 * @return 是否初始化成功
	 */
	bool initializeKDLChain(const std::string& urdf_string,
	                        const std::string& base_link,
	                        const std::string& tip_link);

	/**
	 * @brief 初始化 TRAC_IK Solver（必须在 initializeKDLChain 之后调用）
	 * @param arm_type 机械臂类型（例如 "arm620"，用于加载对应的关节限制）
	 * @return 是否初始化成功
	 */
	bool initializeSolver(const std::string& arm_type = "arm620");

	/**
	 * @brief 逆运动学求解
	 * @param target_pose 目标末端位姿
	 * @param seed_state 初始关节状态（用作IK种子）
	 * @param solution 输出的关节解
	 * @return 是否成功求解
	 */
	bool computeIK(const geometry_msgs::msg::Pose& target_pose,
	               const std::vector<double>& seed_state,
	               std::vector<double>& solution);

	/**
	 * @brief 多次求解IK并选择与seed距离最近的解（避免IK分支跳变）
	 *
	 * 对于同一目标pose，多次调用IK求解器可能得到不同分支的解。
	 * 此方法多次求解并选择与seed点L2距离最小的解，避免IK分支跳变。
	 *
	 * @param target_pose 目标末端位姿
	 * @param seed_state 初始关节状态（参考点）
	 * @param solution 输出的关节解（选择最接近seed的）
	 * @param num_attempts 尝试次数（默认5次）
	 * @return 是否成功求解至少一次
	 */
	bool computeIKClosest(const geometry_msgs::msg::Pose& target_pose,
	                       const std::vector<double>& seed_state,
	                       std::vector<double>& solution,
	                       int num_attempts = 5);

	/**
	 * @brief 设置 MoveItAdapter 引用（用于获取关节限制等信息）
	 * @param moveit_adapter MoveItAdapter 指针
	 */
	void setMoveItAdapter(class MoveItAdapter* moveit_adapter);

private:
	rclcpp::Node::SharedPtr node_;
	std::string move_group_name_;
	std::string base_link_;  // 保存 base_link 名称，用于坐标转换
	class MoveItAdapter* moveit_adapter_;

	// KDL 缓存
	KDL::Chain kdl_chain_;
	bool chain_initialized_;

	// 持久化的 TRAC_IK solver - 避免每次 IK 调用都创建新实例
	// 这对于 MoveL 连续 IK 求解链至关重要
	std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
	KDL::JntArray q_min_, q_max_;
};

}  // namespace trajectory_planning::infrastructure::integration
