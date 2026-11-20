# AGENTS.md

面向在本仓库工作的 AI agent 的项目须知。人类读者请从 [README.md](README.md) 开始。

## 项目是什么

ROS 2 **Jazzy** 教学工作空间：`mini_slam`（自研 C++ 图优化 SLAM）→ `nav2_astar_planner`（A* 全局规划插件）→ `nav2_controller_lab`（PID/Pure Pursuit/Stanley/DWB/MPPI/MPC 六种控制器插件 + CBF 安全过滤）。重点不是算法本身，而是"从算法到 Nav2 工程插件"的过程。文档以中文编号课程（`docs/01~28`）为主干。

## 目录结构

```
src/
├── mini_slam/              # C++ SLAM 包；core/（纯算法）→ ros/（节点封装）→ app/（离线工具）
│                            注意：slam-mpc-academy/code/mini_slam/ 是纯 Python 教学仿真，同名但无代码关系
├── nav2_astar_planner/     # A* GlobalPlanner 插件
├── nav2_controller_lab/    # 六控制器 + CBF；rviz/controller_lab_view.rviz 在这里
│                            （不在 nav2_controller_lab_rviz 包里！后者只有 Qt 面板与 visualizer）
└── nav2_controller_lab_rviz/
docs/
├── 01~28_*.md              # 编号课程文档（改路径引用后必须跑链接检查器，见下）
├── tools/                  # 教学脚本（只放工具，不放运行产物）
├── results/                # curated 实验证据（logs/ 与 lesson_runs/；新产物被 .gitignore 挡住）
└── website/                # 静态教学网站
slam-mpc-academy/           # 独立静态教学站（chapters/*.html + js/sim/*.js）
build/ install/ log/       # colcon 产物，已 gitignore，勿提交
```

## 构建与测试

```bash
# 环境加载顺序：ROS 发行版 → 上层工作空间 underlay → 本工作空间
source /opt/ros/jazzy/setup.bash
source ../../install/setup.bash   # 即 ~/ros2_ws/install（依赖 nav2 等的上层工作空间）
colcon build --packages-select mini_slam nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select mini_slam nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz
colcon test-result --verbose
```

- 单测不需要 Gazebo，纯算法测试不链接 ROS（`test/test_controller_core.cpp` 等）。
- `ros2 launch` 报 package not found 但 `install/<pkg>/` 存在 → 先跑 `./check_workspace.sh`（环境自检），确认前缀损坏用 `--fix`，不要手删 `build/`。
- 根目录 `run_*.sh` 是启动入口，路径一律基于 `$SCRIPT_DIR` 解析，勿写绝对路径。

## 文档与引用验证

- 改动 `docs/` 下任何 md 的本地链接后，必须跑：

  ```bash
  python3 docs/tools/check_tutorial_links.py   # 期望 264 链接 0 缺失
  ```

- **该检查器只查 md 文件链接，不查 Python import**。判断一个 `.py` 是否可删，必须另行 `grep -rn 'import <模块名>'` 并实际 import 一次（教训：`docs/website/pages_b.py` 曾被当遗留文件误删，实际是 `slides/build.py` 的运行时数据源）。

## 本仓库的约定与陷阱

1. **勿以文件名判断"遗留"**。`docs/website/` 的 `pages_b.py`、`slides/framework.py` 都是构建期依赖；删任何 `.py` 前先验证 import。
2. **katex 等 vendor 资产只保留 `.min` 版**，两处（`slam-mpc-academy/assets/vendor/katex/` 与 `docs/website/vendor/katex/`）内容保持一致；HTML 只引用 `contrib/auto-render.min.js`（自包含）。
3. **实验运行产物不入库**：`.gitignore` 已挡 `docs/results/**/*.log` 与 `docs/results/lesson_runs/**/events.jsonl.gz`。历史证据已 curated 在 `docs/results/`，别把它们挪走，10+ 篇文档在引用。
4. **`controller_lab_view.rviz` 属于 `nav2_controller_lab`**（CMakeLists `install(DIRECTORY params launch rviz)` 安装它）；`nav2_controller_lab_rviz` 只提供 RViz 面板插件和 `controller_lab_visualizer`，没有 rviz 配置目录。目录排序时 `nav2_controller_lab/rviz` 与 `nav2_controller_lab_rviz` 相邻易看串。
5. **同名 `mini_slam` 有两处**：`src/mini_slam`（C++ ROS 包）与 `slam-mpc-academy/code/mini_slam`（Python 仿真），无代码关系，两边 README 顶部有澄清说明。
6. git 身份是 **repo-local** 配置（`git config user.name/email`，勿动全局）；提交信息用 `chore/fix/refactor/docs:` 前缀，正文中文说明动机与验证方式。
7. 工作树必须保持干净收工；`git ls-files` 里不应出现 `build/ install/ log/` 下任何文件。
