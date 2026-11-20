# 真实课堂数据样本

这些数据来自本次开发环境的实际Gazebo实验，不是合成成功日志。原始报告保留原版本信息、工作区有未提交修改的状态、世界与参数哈希。它们用于学习重算和故障分析，不是发行版精度认证，也不是控制器性能排名。

| 目录 | 内容 |
|---|---|
| fixed_straight_before_goal_anchor | 默认容差的固定直线MPC基线，连续目标保留修正之前 |
| tight_before_1 / 2 / 3 | 10cm/约5°配置的三次独立实验，修正之前 |
| tight_after_1 / 2 / 3 | 保留连续目标后，10cm/约5°三次实验，原生进程正常回收 |
| medium_after_1 / 2 / 3 | 保留连续目标后，20cm/约5°三次实验 |

events.jsonl.gz是完整采样，不含凭空补出的真值。report.json是当时的报告；world.sdf和params.yaml保留实验条件。真值来自独立Gazebo世界位姿，变换由已知出生配置给定。传感器噪声种子未固定，开发机外部负载未严格隔离；不据此宣称统计显著优于。

## 无需ROS离线重算

在项目根目录执行，输出目录必须不存在：

```bash
python3 docs/tools/navigation_metrics.py docs/results/lesson_runs/fixed_straight_before_goal_anchor/events.jsonl.gz --configuration docs/results/lesson_runs/fixed_straight_before_goal_anchor/report.json --output /tmp/teaching_recomputed_example
python3 docs/tools/plot_navigation_report.py docs/results/lesson_runs/fixed_straight_before_goal_anchor/events.jsonl.gz --report docs/results/lesson_runs/fixed_straight_before_goal_anchor/report.json --output /tmp/teaching_example.png
python3 -m unittest discover -s docs/tools/tests -v
```

重算工具可复用原报告的匹配窗口和固定坐标变换，避免把world直接与map比较。新版本可能添加解释字段，但原有数值可用回归测试核对。图表依赖matplotlib；纯指标计算和测试只依赖标准库。

配套[指标课](../../25_导航评价工具与公平实验.md)、[精度课](../../27_独立真值与精度进阶实验.md)解释为什么Action成功、容差、估计误差和真实误差不能互相替代。
