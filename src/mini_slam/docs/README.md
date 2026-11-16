# mini_slam 学习文档索引

本目录是 mini_slam 各特性的学习文档：每篇对应一个从 slam_toolbox 复刻的特性，
按"动机 → slam_toolbox 的做法 → 我们的设计 → 验证结果 → 踩坑"组织。

| 序号 | 文档 | 对应特性 | 状态 |
|---|---|---|---|
| 01 | [定位模式](01_定位模式.md) | Mapper localization 模式：图冻结位姿跟踪 | ✅ 已实现 |
| 02 | [位姿图序列化](02_位姿图序列化.md) | 位姿图 + 扫描持久化/恢复 | ✅ 已实现 |
| 03 | [扫描匹配协方差](03_扫描匹配协方差.md) | 响应面协方差 → 真实信息矩阵 | ✅ 已实现 |
| 04 | [图连通链闭环候选](04_图连通链闭环候选.md) | karto 式连通链候选搜索 | ✅ 已实现 |
| 05 | [异步处理队列](05_异步处理队列.md) | 节点层 worker 线程 + 限长队列 | ✅ 已实现 |
| 06 | [地图稀疏化](06_地图稀疏化.md) | 扫描点剪枝与内存控制 | ✅ 已实现 |
| 07 | [参数体系与服务接口](07_参数与服务接口.md) | 参数补全 + 保存/加载/模式服务 | ✅ 已实现 |
| 08 | [交互式建图](08_交互式建图.md) | RViz 拖拽位姿图节点 + 局部 SPA（InteractiveMarkers） | ✅ 已实现 |

回归基线（Phase 4C 实测）：`ctest --test-dir build` **21/21** 通过；
`colcon test-result --verbose` **76 tests, 0 errors, 0 failures, 0 skipped**；
`./scripts/closed_loop_test.sh --skip-mapping` 端到端通过（需已有
`out_nav/mini_map.yaml`，缺失时脚本会直接报错退出）。
