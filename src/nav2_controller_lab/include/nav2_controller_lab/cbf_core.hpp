// Copyright (c) 2026
// CBF（控制屏障函数）安全过滤器核心 —— 纯 C++，不依赖 ROS
//
// 数学模型（推导见 docs/17）：
//   安全集 C = { x : h(x) ≥ 0 }，h = range - d_safe（激光测距点到障碍表面的余量）
//   前向不变性条件（对每条扫描线 i）：
//       ḣ_i ≥ -α·h_i      即      -v·cos(ψ_i) ≥ -α·h_i
//   其中 ψ_i 是障碍方位角。差速机器人绕中心旋转不改变到障碍的距离，
//   所以一阶展开里 ḣ 只依赖 v——角速度 ω 在一阶下不受约束（已知局限）。
//   约束对 v 是线性的：v·cos(ψ_i) ≤ α·h_i，全部约束取交集后
//   v 的可行域是一个区间 → "QP" 有解析解（区间裁剪），无需求解器。

#ifndef NAV2_CONTROLLER_LAB__CBF_CORE_HPP_
#define NAV2_CONTROLLER_LAB__CBF_CORE_HPP_

#include <utility>
#include <vector>

namespace nav2_controller_lab
{

struct CbfParams
{
  /// 屏障偏移：h = 中心距障碍表面 - d_safe。
  /// d_safe = 机器人外接半径（接触边界）时的语义：永不接触。
  double d_safe{0.30};

  /// class-K 增益（线性）：约束 ḣ ≥ -alpha·h。
  /// 越大 → 离障碍越远就开始减速；太小 → 逼近障碍才急刹。
  double alpha{2.0};

  /// 输出限幅（与底盟能力一致）
  double max_linear_velocity{0.5};
  double min_linear_velocity{-0.5};

  /// 只考虑该距离内的扫描点（机器人中心系，更远的点约束自然不起作用）
  double scan_influence_range{1.2};

  /// |cos(θ)| 低于此值的切向点跳过：一阶下切向运动不改变距离，
  /// 它们对 v 不构成约束（数学上等价于除以近零数）。
  double cos_epsilon{0.05};

  /// 雷达安装外参（base 系，米）。TB3 waffle 雷达在中心后方：
  /// gz_waffle.sdf.xacro 的 lidar_joint pose = (-0.064, 0, 0.121)。
  /// 不补偿的后果：侧向束的原始 range ≠ 中心距离，h 被错误判负，
  /// CBF 会误拦合法运动（docs/17 §6 实车案例）。
  double lidar_offset_x{0.0};
  double lidar_offset_y{0.0};
};

struct CbfResult
{
  double linear{0.0};       ///< 过滤后的线速度
  double angular{0.0};      ///< 过滤后的角速度（一阶 CBF 不修改，原样透传）
  bool active{false};       ///< 是否真的修改了标称指令
  bool infeasible{false};   ///< 前后同时被逼到无解 → 强制停车
  double min_h{0.0};        ///< 影响范围内的最小屏障值（诊断：h<0 = 已侵入余量）
  int binding_index{-1};    ///< 起决定作用的扫描点索引（诊断：谁把 v 摁住了）
};

/**
 * @brief CBF 安全过滤器：把"任意控制器输出的标称指令"修正为满足
 *        屏障条件 ḣ ≥ -α·h 的最接近指令（欧氏范数意义下的投影）。
 *
 * 设计定位：它不是控制器，是**独立的安全层**——插在速度链路的
 * 任意标称源（DWB/MPPI/遥控/学习策略）之后、底盘之前。
 */
class CbfFilter
{
public:
  explicit CbfFilter(const CbfParams & params);

  /**
   * @param nominal_v 标称线速度（来自上游控制器）
   * @param nominal_w 标称角速度
   * @param scan 扫描点 (方位角ψ[弧度], 距离range[米])，机器人系；
   *             inf/nan/无效点由调用方过滤
   */
  CbfResult filter(
    double nominal_v, double nominal_w,
    const std::vector<std::pair<double, double>> & scan);

private:
  CbfParams params_;
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__CBF_CORE_HPP_
