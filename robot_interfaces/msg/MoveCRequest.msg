# MoveCRequest.msg
#
# 请求生成圆弧 / 贝塞尔等 MoveC 轨迹
# 仅提供几何信息，时间学参数由服务器端决定

# 轨迹类型
uint8 route_type    # 0: ARC 1: BEZIER 2: CIRCLE 3: CIRCLE3PT
string route_name  # 轨迹名称（可选）

# 几何关键点
geometry_msgs/Pose[] waypoints
#   - 对ARC: [goal, center]
#   - 对BEZIER: [start, control1, control2, end]
#   - 对CIRCLE: [center, radius_point]
#   - 对CIRCLE3PT: [p1, p2, p3]

# 可选：轨迹分辨率（如果需要用户可选精度，可保留；否则去掉）
# float64 sampling_resolution
