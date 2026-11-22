// ============================================================================
// LocalizerSim —— 端到端仿真源（集成测试 / 演示）
// 发布 /map（latched）、/odom（带 2% 滑移）、/scan（仓库环路 + 人群段遮挡）
// 机器人在仓库自由区跑矩形环路 ×2，日志末尾打印定位误差统计。
// ============================================================================
#include <cmath>
#include <memory>
#include <random>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "scan_localizer/sim_world.hpp"

namespace sloc {

class LocalizerSim : public rclcpp::Node {
public:
    LocalizerSim() : Node("scan_localizer_sim") {
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local());
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 100);
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10).best_effort());

        grid_ = std::make_shared<OccupancyGrid>(sim::make_warehouse(0.05));
        field_pub_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { publishMapOnce(); });

        // 轨迹：与基准 E9 相同的环路
        struct Node2 { double x, y; };
        const Node2 base_wp[] = {{2, 13.5}, {13.5, 13.5},
                                 {13.5, 28}, {2, 28}, {2, 13.5}};
        std::vector<Node2> wp;
        for (int lap = 0; lap < 2; ++lap) {
            for (const auto& w : base_wp) { wp.push_back(w); }
        }
        Pose2 cur(2, 13.5, 0);
        for (std::size_t seg = 0; seg + 1 < wp.size(); ++seg) {
            const double dx = wp[seg + 1].x - wp[seg].x;
            const double dy = wp[seg + 1].y - wp[seg].y;
            const double n = std::hypot(dx, dy) / 0.25;
            for (int i = 0; i < static_cast<int>(n); ++i) {
                cur.x += dx / n;
                cur.y += dy / n;
                cur.theta = std::atan2(dy, dx);
                traj_.push_back(cur);
            }
        }
        odom_ = traj_.front();

        sim_timer_ = create_wall_timer(
            std::chrono::milliseconds(250),  // 0.25s/步
            [this]() { step(); });
        RCLCPP_INFO(get_logger(),
                    "sim started: %zu steps (%.0f m), publishing /map /odom "
                    "/scan",
                    traj_.size(), 0.25 * traj_.size());
    }

    // 最近真值（供外部对拍）
    Pose2 gt_now() const { return traj_[std::min(k_ + 1, traj_.size() - 1)]; }
    std::size_t steps() const { return k_; }

private:
    void publishMapOnce() {
        if (map_sent_) { return; }
        nav_msgs::msg::OccupancyGrid m;
        m.header.stamp = now();
        m.header.frame_id = "map";
        m.info.resolution = grid_->resolution;
        m.info.width = static_cast<std::uint32_t>(grid_->width);
        m.info.height = static_cast<std::uint32_t>(grid_->height);
        m.info.origin.position.x = grid_->origin_x;
        m.info.origin.position.y = grid_->origin_y;
        m.data = grid_->cells;
        map_pub_->publish(m);
        map_sent_ = true;
    }

    void step() {
        if (k_ + 1 >= traj_.size()) {
            RCLCPP_INFO(get_logger(), "sim done (%zu steps)", k_);
            sim_timer_->cancel();
            return;
        }
        ++k_;
        const Pose2 prev = traj_[k_ - 1];
        const Pose2 gt = traj_[k_];
        // 滑移 odom
        const double c = std::cos(prev.theta), s = std::sin(prev.theta);
        const double mx = gt.x - prev.x, my = gt.y - prev.y;
        const Pose2 d_local(c * mx + s * my, -s * mx + c * my,
                           gt.theta - prev.theta);
        std::normal_distribution<double> gn(0.0, 0.01);
        const Pose2 d_odom(d_local.x * 1.02 + gn(rng_) * 0.01,
                           d_local.y * 1.02 + gn(rng_) * 0.01,
                           d_local.theta * 1.01 + gn(rng_) * 0.002);
        odom_ = odom_ * d_odom;

        // /odom
        nav_msgs::msg::Odometry om;
        om.header.stamp = now();
        om.header.frame_id = "odom";
        om.child_frame_id = "base_link";
        om.pose.pose.position.x = odom_.x;
        om.pose.pose.position.y = odom_.y;
        om.pose.pose.orientation.z = std::sin(0.5 * odom_.theta);
        om.pose.pose.orientation.w = std::cos(0.5 * odom_.theta);
        odom_pub_->publish(om);

        // 真值日志（每 4 步 = 1s，供外部对拍）
        if (k_ % 4 == 0) {
            RCLCPP_INFO(get_logger(), "SIMGT %.3f %.3f %.3f", gt.x, gt.y,
                        gt.theta);
        }

        // IMU：真实角速度 + 消费级 MEMS 噪声（诚实仿真，不作弊）
        sensor_msgs::msg::Imu im;
        im.header.stamp = now();
        im.header.frame_id = "base_link";
        std::normal_distribution<double> mn(0.0, 0.0002);
        im.angular_velocity.z =
            d_local.theta / 0.25 + mn(rng_);  // 步长 0.25s
        imu_pub_->publish(im);

        // 人群段遮挡（每 4 步一帧扫描 → 对应 traj 段 150-190）
        std::vector<sim::DynBox> dyn;
        if (k_ > 150 && k_ < 190) {
            dyn.push_back({gt.x + 1.5 * std::cos(gt.theta),
                           gt.y + 1.5 * std::sin(gt.theta), 0.35, 0.35});
            dyn.push_back({gt.x + 2.8 * std::cos(gt.theta + 0.4),
                           gt.y + 2.8 * std::sin(gt.theta + 0.4), 0.35, 0.35});
        }
        if (k_ % 4 == 0) {
            sim::ScanSimConfig cfg;
            const auto scan = sim::make_scan(*grid_, gt, cfg, rng_, dyn);
            sensor_msgs::msg::LaserScan ls;
            ls.header.stamp = now();
            ls.header.frame_id = "base_link";
            ls.angle_min = -M_PI;
            ls.angle_max = M_PI;
            ls.angle_increment = 2.0 * M_PI / 720.0;
            ls.range_min = 0.10;
            ls.range_max = 12.0;
            ls.ranges.assign(720, std::numeric_limits<float>::infinity());
            // 重建 range 序列（角度对齐最近束）
            for (const auto& p : scan.points) {
                double a = std::atan2(p.y(), p.x());
                if (a < -M_PI) { a += 2 * M_PI; }
                int idx = static_cast<int>(
                    (a + M_PI) / ls.angle_increment);
                if (idx >= 0 && idx < 720) {
                    ls.ranges[static_cast<std::size_t>(idx)] =
                        static_cast<float>(std::hypot(p.x(), p.y()));
                }
            }
            scan_pub_->publish(ls);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr field_pub_timer_, sim_timer_;
    std::shared_ptr<OccupancyGrid> grid_;
    std::vector<Pose2> traj_;
    Pose2 odom_{};
    std::size_t k_{0};
    bool map_sent_{false};
    std::mt19937 rng_{7};
};

}  // namespace sloc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sloc::LocalizerSim>());
    rclcpp::shutdown();
    return 0;
}
