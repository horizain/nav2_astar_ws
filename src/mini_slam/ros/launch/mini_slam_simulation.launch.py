#!/usr/bin/env python3
# Copyright (c) 2026
# 控制器实验室仿真启动文件
#
# 复用官方 tb3_simulation_launch.py，只替换 params_file。
# 全局规划器仍是 nav2_astar_planner::AStarPlanner，控制器由 params 文件决定。
#
# 用法：
#   ros2 launch nav2_controller_lab controller_lab_launch.py
#   ros2 launch nav2_controller_lab controller_lab_launch.py headless:=True
#   ros2 launch nav2_controller_lab controller_lab_launch.py use_rviz:=False

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lab_dir = get_package_share_directory('nav2_controller_lab')
    bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    default_params = os.path.join(
        lab_dir, 'params', 'nav2_params_controller_lab.yaml')
    default_rviz = os.path.join(
        lab_dir, 'rviz', 'controller_lab_view.rviz')
    default_world = os.path.join(
        sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')

    params_file = LaunchConfiguration('params_file')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    use_visualizer = LaunchConfiguration('use_visualizer')
    world = LaunchConfiguration('world')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 参数文件（默认使用 Pure Pursuit 控制器）'))
    ld.add_action(DeclareLaunchArgument(
        'headless', default_value='False',
        description='是否无头运行（True 则不显示 Gazebo 窗口）'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='是否启动 RViz'))
    ld.add_action(DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='是否使用 composed bringup（本机建议 False）'))
    ld.add_action(DeclareLaunchArgument(
        'use_visualizer', default_value='True',
        description='是否启动 Controller Lab 调试可视化节点'))
    ld.add_action(DeclareLaunchArgument(
        'world', default_value=default_world,
        description='仿真世界文件'))
    ld.add_action(DeclareLaunchArgument(
        'rviz_config_file', default_value=default_rviz,
        description='RViz 配置文件（已移除会段错误的 Docking 面板）'))
    # SLAM 模式（docs/20）：slam:=True 时 bringup 用 slam_toolbox 替代
    # map_server+AMCL —— 机器人边跑边建图（map_server/amcl 不启动）
    slam = LaunchConfiguration('slam')
    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='False',
        description='True=slam_toolbox 在线建图（代替静态地图+AMCL）'))
    # 自建地图（docs/20）：SLAM 存图后传给定位会话，如
    #   map:=docs/tools/maps/lab_built.yaml
    map_yaml = LaunchConfiguration('map')
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=os.path.join(
            bringup_dir, 'maps', 'tb3_sandbox.yaml'),
        description='定位/导航用的地图 yaml（默认官方沙盒图）'))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'mini_slam_tb3_simulation.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'headless': headless,
            'use_rviz': use_rviz,
            'use_composition': use_composition,
            'world': world,
            'rviz_config_file': rviz_config_file,
            'slam': slam,
            'map': map_yaml,
        }.items(),
    ))

    ld.add_action(Node(
        package='nav2_controller_lab_rviz',
        executable='controller_lab_visualizer',
        name='controller_lab_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_visualizer),
    ))

    # CBF 安全过滤器（docs/17）：独立安全层节点，插在速度链路里
    #   velocity_smoother → cmd_vel_smoothed → cbf_safety_filter → cmd_vel_cbf
    #   → collision_monitor → cmd_vel → 底盘
    # 节点始终启动（collision_monitor 的输入依赖它），enabled 参数控制
    # 过滤/旁路——运行时切换：ros2 param set /cbf_safety_filter enabled false
    use_cbf = LaunchConfiguration('use_cbf')
    ld.add_action(DeclareLaunchArgument(
        'use_cbf', default_value='True',
        description='CBF 安全过滤器默认启用（False 则纯透传，用于对比实验）'))
    ld.add_action(Node(
        package='nav2_controller_lab',
        executable='cbf_safety_filter',
        name='cbf_safety_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enabled': use_cbf,
            'd_safe': 0.22,  # 屏障=机器人半径(接触边界)，与DWB 碰撞阈值253/内切语义严格对齐：docs/17 §6
            'alpha': 2.0,
            'scan_influence_range': 1.2,
            # TB3 waffle 雷达外参：中心后方 6.4cm（gz_waffle.sdf.xacro lidar_joint）
            # 不补偿 → 侧向束 h 被错误判负 → CBF 误拦合法运动（docs/17 §6）
            'lidar_offset_x': -0.064,
            'lidar_offset_y': 0.0,
        }],
    ))

    return ld
