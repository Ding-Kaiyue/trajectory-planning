# JointConstraintRequest.msg
#
# 目标末端位姿
geometry_msgs/Pose goal_pose

# 规划类型
uint8 JOINT = 0
uint8 CARTESIAN = 1
uint8 INTELLIGENT = 2
uint8 planning_type

# 关节约束列表
trajectory_planning_interfaces/JointConstraint[] joint_constraints

# 规划参数
int32 max_attempts  # 最大尝试次数，默认5
