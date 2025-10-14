import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def launch_moveit_config(context, *args, **kwargs):
    robot_model_name = LaunchConfiguration('robot_model_name').perform(context)

    if robot_model_name == 'arm620':
        config_pkg = 'arm620_config'
    elif robot_model_name == 'arm380':
        config_pkg = 'arm380_config'
    else:
        raise ValueError(f'Unsupported robot model name: {robot_model_name}')
    
    moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(config_pkg), 'launch', 'real_moveit_demo.launch.py')
        )
    )
    return [moveit_config_launch]

def generate_launch_description():  # 修复：添加了括号
    log_level_arg = DeclareLaunchArgument(
    	'log_level',
    	default_value='WARN',
    	description='Log level for all nodes'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value='arm620',
        description='Robot model name (e.g., arm620 or arm380)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # 修改：真实硬件应该用false
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 添加轨迹规划节点类型参数
    planning_node_type_arg = DeclareLaunchArgument(
        'planning_node_type',
        default_value='none',   # 默认不启动任何节点
        description='Type of planning node to launch (movej/movel/movec/joint_constrained/trajectory_planning/none)',
        choices=['movej', 'movel', 'movec', 'joint_constrained', 'trajectory_planning', 'none']
    )
    
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_description'), 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'robot_model_name': LaunchConfiguration('robot_model_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    trajectory_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('trajectory_planning_v3'), 'launch', 'planning_demo.launch.py')),
        launch_arguments={
            'planning_node_type': LaunchConfiguration('planning_node_type')
        }.items()  # 修复：添加了.items()和参数传递
    )
    
    moveit_config = OpaqueFunction(function=launch_moveit_config)
    
    return LaunchDescription([
	log_level_arg,
        robot_model_arg,
        use_sim_time_arg,
        planning_node_type_arg,  # 添加：规划节点类型参数
        robot_description,
        trajectory_planning,
        moveit_config,  # 添加：MoveIt配置
    ])
