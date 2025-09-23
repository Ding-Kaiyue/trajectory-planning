# JointConstraint.msg
#
# 约束的通用描述
uint8 FIXED = 0
uint8 RANGE = 1

uint8 type  # FIXED or RANGE
int32 joint_index  # 被约束的关节索引
float64 weight  # 约束的权重, 默认1.0
bool is_hard  # 硬约束 => true, 软约束 => false

# FIXED
float64 fixed_value  # 约束的固定值

# RANGE
float64 min_value  # 约束的下界
float64 max_value  # 约束的上界