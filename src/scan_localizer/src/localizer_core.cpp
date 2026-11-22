// ============================================================================
// LocalizerCore 实现
// ============================================================================
#include "scan_localizer/localizer_core.hpp"

#include <cmath>

namespace sloc {

LocalizerCore::LocalizerCore(CoreConfig cfg) : cfg_(cfg) {}

void LocalizerCore::set_map(std::shared_ptr<const MapField> f) {
    field_ = std::move(f);
}
void LocalizerCore::set_matcher(std::shared_ptr<const Matcher> m) {
    matcher_ = std::move(m);
}
void LocalizerCore::set_initial_pose(const Pose2& p) {
    last_pose_ = p;
    first_fix_ = true;
    reject_streak_ = 0;
    last_cov_ = Cov3::Identity() * 1e-4;
    // 基线同步：已知 odom 时立即锚定（调用方应先喂当前 odom 再设初值）；
    // 未知时等首个 odom 到来再锚定（近似，可能损失 Δt 的运动量）
    if (has_odom_) {
        odom_at_last_scan_ = last_odom_;
        state_ = LocState::TRACKING;
    } else {
        state_ = LocState::UNINIT;
    }
    imu_yaw_at_last_scan_ = imu_yaw_;  // IMU 锚点同步
    imu_consumed_ = true;
}
void LocalizerCore::set_global_fn(GlobalFn fn) { global_fn_ = std::move(fn); }

void LocalizerCore::on_odom(const Pose2& odom_pose) {
    // set_initial_pose 之后的首帧 odom：基线未锚定则近似锚定于此
    if (state_ == LocState::UNINIT && first_fix_) {
        if (!has_odom_) { odom_at_last_scan_ = odom_pose; }
        state_ = LocState::TRACKING;
    }
    last_odom_ = odom_pose;
    has_odom_ = true;
}

void LocalizerCore::on_imu_yaw(double integrated_yaw) {
    if (!cfg_.use_imu_yaw) { return; }
    if (imu_consumed_) {
        // 新积累期起点：把上一帧的锚点放在当前
        imu_yaw_at_last_scan_ = imu_yaw_;
        imu_consumed_ = false;
    }
    imu_yaw_ = integrated_yaw;
    has_imu_ = true;
}

Pose2 LocalizerCore::map_to_odom() const {
    // T_map_odom = T_map_base * T_base_odom = last_pose * inv(last_odom)
    return last_pose_ * last_odom_.inverse();
}

CoreOutput LocalizerCore::on_scan(const Scan& scan) {
    CoreOutput out;
    out.pose = last_pose_;
    out.state = state_;
    if (!field_ || !matcher_ || !has_odom_) { return out; }

    // ---------- UNINIT / LOST：尝试全局搜索 ----------
    if (state_ == LocState::UNINIT || state_ == LocState::LOST) {
        if (global_fn_) {
            Pose2 g{};
            double sc = 0.0;
            if (global_fn_(scan, g, sc)) {
                last_pose_ = g;
                last_cov_ = Cov3::Identity() * 0.04;
                odom_at_last_scan_ = last_odom_;  // 基线锚定于修复时刻
                imu_yaw_at_last_scan_ = imu_yaw_;  // IMU 锚点同步
                imu_consumed_ = true;
                state_ = LocState::TRACKING;
                reject_streak_ = 0;
                first_fix_ = true;
                out.pose = g;
                out.state = state_;
                out.accepted = true;
                out.score = sc;
            }
        }
        return out;
    }

    // ---------- 预测：odom 增量复合到上一 map 系位姿 ----------
    // Δ = inv(odom_at_last_scan_) * odom_now
    const Pose2 odom_now = last_odom_;
    Pose2 delta = Pose2{};
    if (first_fix_) {
        delta = odom_at_last_scan_.inverse() * odom_now;
    }
    odom_at_last_scan_ = odom_now;

    // IMU 航向融合：平移取轮式，旋转取陀螺帧间增量
    bool imu_used = false;
    if (cfg_.use_imu_yaw && has_imu_ && !imu_consumed_) {
        double dth = imu_yaw_ - imu_yaw_at_last_scan_;
        while (dth > M_PI) { dth -= 2 * M_PI; }
        while (dth < -M_PI) { dth += 2 * M_PI; }
        const double trans = std::hypot(delta.x, delta.y);
        const double rot1 = trans < 1e-9 ? 0.0 : std::atan2(delta.y, delta.x);
        delta = {trans * std::cos(rot1), trans * std::sin(rot1), dth};
        imu_yaw_at_last_scan_ = imu_yaw_;
        imu_consumed_ = true;
        imu_used = true;
    }

    const Pose2 pred = first_fix_ ? last_pose_ * delta : last_pose_;

    // 预测协方差（滑移随运动量增长）——覆盖 2% 尺度滑移与转向耦合
    // IMU 接管旋转时，旋转不确定度按陀螺精度给（小两个数量级）
    Cov3 pred_cov = last_cov_;
    const double dtrans = std::hypot(delta.x, delta.y);
    pred_cov(0, 0) += dtrans * cfg_.odom_trans_var_per_m;
    pred_cov(1, 1) += dtrans * cfg_.odom_trans_var_per_m;
    pred_cov(2, 2) +=
        dtrans * cfg_.odom_rot_var_per_m +
        std::abs(delta.theta) * (imu_used ? cfg_.imu_rot_var_per_rad
                                           : cfg_.odom_rot_var_per_rad);

    // ---------- 匹配 ----------
    const MatchResult mr = matcher_->match(*field_, pred, scan,
                                            cfg_.match_opt);
    out.score = mr.score;
    out.inlier_ratio = mr.inlier_ratio;
    out.pose = pred;  // 默认拒绝时输出预测

    // ---------- 三道门 ----------
    bool gate_inlier = mr.inlier_ratio >= cfg_.min_inlier_ratio;
    bool gate_score = mr.score >= cfg_.min_score;

    // 马氏门：match_pose 与 pred 的差，相对 (pred_cov + match_cov)
    Eigen::Vector3d innov(mr.pose.x - pred.x, mr.pose.y - pred.y,
                          mr.pose.theta - pred.theta);
    while (innov(2) > M_PI) { innov(2) -= 2 * M_PI; }
    while (innov(2) < -M_PI) { innov(2) += 2 * M_PI; }
    const Cov3 S = pred_cov + mr.covariance + Cov3::Identity() * 1e-9;
    const double mahal = innov.transpose() * S.inverse() * innov;
    const bool gate_mahal = mahal < cfg_.mahal_gate;

    const double trace =
        mr.covariance(0, 0) + mr.covariance(1, 1) + mr.covariance(2, 2);
    (void)trace;
    // 协方差门（最小特征值）：只要存在充分约束的方向就接受匹配。
    // 走廊等退化场景沿轴方差→∞，但横向/航向仍有效，不应整帧丢弃。
    Eigen::SelfAdjointEigenSolver<Cov3> es(mr.covariance);
    const double min_eig =
        es.eigenvalues().size() ? es.eigenvalues().minCoeff() : 1e9;
    const bool gate_cov = min_eig < cfg_.max_cov_eig;

    const bool accept = mr.valid && gate_inlier && gate_score &&
                        gate_mahal && gate_cov;

    if (accept) {
        // 新息吸收平滑（slew limit）
        Pose2 target = mr.pose;
        const Eigen::Vector2d dxy(target.x - pred.x, target.y - pred.y);
        const double dth = innov(2);
        Pose2 blended(pred.x + cfg_.slew_alpha * dxy.x(),
                      pred.y + cfg_.slew_alpha * dxy.y(),
                      pred.theta + cfg_.slew_alpha * dth);
        (void)target;
        last_pose_ = blended;
        last_cov_ = mr.covariance;
        reject_streak_ = 0;
        state_ = LocState::TRACKING;
        out.pose = blended;
        out.covariance = mr.covariance;
        out.accepted = true;
    } else {
        last_pose_ = pred;       // 纯 odom 外推
        last_cov_ = pred_cov;
        ++reject_streak_;
        out.reject_streak = reject_streak_;
        out.covariance = pred_cov;
        if (reject_streak_ >= cfg_.degraded_after) {
            state_ = LocState::DEGRADED;
        }
        if (reject_streak_ >= cfg_.lost_after) {
            state_ = LocState::LOST;
            reject_streak_ = 0;
        }
        out.state = state_;
    }
    first_fix_ = true;
    return out;
}

}  // namespace sloc
