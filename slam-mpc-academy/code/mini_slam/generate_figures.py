# -*- coding: utf-8 -*-
"""
generate_figures.py —— 一键重新生成网站所需的全部结果图与指标

运行：python generate_figures.py
输出：../assets/img/fig_*.png  与 results/*.json
"""
import run_slam
import run_mpc
import run_astar

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print(">>> mini-SLAM 流水线")
    print("=" * 60)
    run_slam.main()
    print("\n" + "=" * 60)
    print(">>> MPC 控制器")
    print("=" * 60)
    run_mpc.main()
    print("\n" + "=" * 60)
    print(">>> A* 路径规划")
    print("=" * 60)
    run_astar.main()
    print("\n全部图表生成完毕 ✅")
