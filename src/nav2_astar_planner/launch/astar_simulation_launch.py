#!/usr/bin/env python3
# Copyright (c) 2026
# A* 规划器仿真启动文件
#
# 用法：
#   ros2 launch nav2_astar_planner astar_simulation_launch.py
#   ros2 launch nav2_astar_planner astar_simulation_launch.py headless:=True
#   ros2 launch nav2_astar_planner astar_simulation_launch.py world:=/path/to/world.sdf
#
# 说明：
#   本 launch 复用官方 tb3_simulation_launch.py 的全部能力（Gazebo、RViz、机器人模型、
#   TF、传感器桥接），只把 params_file 换成本包的 A* 参数文件。
#   这样既能保证仿真环境完整，又能让 planner_server 加载我们的 A* 插件。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    astar_dir = get_package_share_directory('nav2_astar_planner')
    bringup_dir = get_package_share_directory('nav2_bringup')

    # 默认使用本包的 A* 参数文件
    default_params = os.path.join(astar_dir, 'params', 'nav2_params_astar.yaml')

    params_file = LaunchConfiguration('params_file')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Nav2 参数文件（默认使用 A* 插件配置）')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='是否无头运行（True 则不显示 Gazebo 窗口）')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='是否启动 RViz')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('nav2_minimal_tb3_sim'),
            'worlds', 'tb3_sandbox.sdf.xacro'),
        description='仿真世界文件')

    # 复用官方 tb3 仿真 launch
    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'headless': headless,
            'use_rviz': use_rviz,
            'world': world,
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
        }.items(),
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(astar_dir, 'rviz', 'nav2_astar_view.rviz'),
        description='RViz 配置文件（默认已禁用会崩溃的 Docking 面板）')

    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(declare_headless)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_world)
    ld.add_action(declare_rviz_config)
    ld.add_action(tb3_launch)
    return ld
