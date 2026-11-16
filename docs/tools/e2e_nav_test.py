#!/usr/bin/env python3
"""端到端导航自动测试：初始位姿 → 激活栈 → 发目标 → 等结果。

为什么需要这个脚本：
  1. headless（无 RViz）时没法点 "2D Pose Estimate"，必须用代码设初始位姿；
  2. nav2 bringup 激活顺序要求 map->base_link 先可用（见 docs/03 坑三），
     所以正确顺序是：先初始位姿 → 再激活 → 再发目标；
  3. `ros2 topic pub --once` / `ros2 service call` 在本机可能因 DDS 发现
     慢而挂起或丢失消息，改用 rclpy 单进程直连 service/action，
     和 activate_nodes.py 同一套思路。

用法（另开一个终端先启动仿真）：
  source install/setup.bash
  python3 docs/tools/e2e_nav_test.py                          # 默认目标 (1.5, 0.5)
  python3 docs/tools/e2e_nav_test.py --goal 1.5 2.0           # 自定义目标
  python3 docs/tools/e2e_nav_test.py --controller MPPI        # 运行时切控制器
  python3 docs/tools/e2e_nav_test.py --timeout 120            # 导航超时（秒）

退出码：0 = 到达目标；1 = 失败（TF 断链/规划失败/超时等）。
"""
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from std_msgs.msg import String

GOAL_STATUS = {
    1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED',
    5: 'CANCELED', 6: 'ABORTED',
}


class E2ENavTester(Node):

    def __init__(self):
        super().__init__('e2e_nav_test')
        # 仿真时间：让消息时间戳与 Gazebo /clock 对齐（AMCL/TF 都用 sim time）
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

    # ---------- 基础设施：service 调用 ----------
    def call_service(self, srv_type, name, req, timeout=8.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    # ---------- Step 1：检查 TF 下半段（odom -> base_footprint） ----------
    def check_odom_tf(self, timeout=15.0):
        from tf2_ros import Buffer
        from tf2_ros import TransformListener
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            try:
                t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                tx, ty = t.transform.translation.x, t.transform.translation.y
                self.get_logger().info(
                    f'[TF] odom->base_footprint OK: ({tx:.2f}, {ty:.2f})')
                return True
            except Exception:
                pass
        self.get_logger().error(
            '[TF] odom->base_footprint 不存在！DiffDrive/parameter_bridge 没把仿真 TF '
            '桥接过来。排查：gz topic -l 看 /model/turtlebot3_waffle/tf 是否有数据。')
        return False

    # ---------- Step 2：设初始位姿（必须与 Gazebo 出生点一致） ----------
    @staticmethod
    def yaw_to_quat(yaw_deg):
        """yaw -> 四元数，手写以避免 tf_transformations 依赖"""
        from geometry_msgs.msg import Quaternion
        half = math.radians(yaw_deg) / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    def set_initial_pose(self, x, y, yaw_deg, use_sim_time):
        """设初始位姿并**验证生效**（map→odom TF 出现才算数），失败自动重试。

        教训（docs/15 坑一）：发布"成功"不等于送达。必须以 AMCL 的反应
        （map→odom 变换出现）为准，没出现就换途径再来。
        """
        from geometry_msgs.msg import PoseWithCovarianceStamped

        q = self.yaw_to_quat(yaw_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = q

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

        def map_tf_ok():
            try:
                self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
                return True
            except Exception:
                return False

        for attempt in range(1, 4):
            r = self.call_service(SetInitialPose, '/reinitialize_localization',
                                  SetInitialPose.Request(pose=msg), timeout=8.0)
            if r is not None:
                self.get_logger().info(
                    f'[INIT] 第{attempt}次：已通过 /reinitialize_localization 服务设置')
            else:
                self.get_logger().warn(
                    f'[INIT] 第{attempt}次：服务不通，回退到 /initialpose 话题连发')
                for _ in range(5):
                    pub.publish(msg)
                    rclpy.spin_once(self, timeout_sec=0.5)
                    time.sleep(0.5)

            # 等待生效（最多 8 秒）：map→odom 出现才算成功
            deadline = time.time() + 8.0
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
                if map_tf_ok():
                    self.get_logger().info('[INIT] 已生效：map→odom TF 出现')
                    return True

        self.get_logger().error('[INIT] 三次尝试后 map→odom 仍不存在，放弃')
        return False

    # ---------- Step 3：激活生命周期节点（分两组，破解循环依赖） ----------
    # 循环依赖：planner_server 激活需要 map→base_link（全局代价地图），
    # 而 map→odom 依赖 AMCL 收到初始位姿。所以正确顺序是：
    #   定位组(map_server→amcl) → 初始位姿 → 导航组(其余节点)
    # 这正是 bringup 的"半激活"现象的来由（docs/03 §6.4）。
    LOCALIZATION_NODES = ['map_server', 'amcl']
    NAVIGATION_NODES = ['controller_server', 'planner_server', 'smoother_server',
                        'behavior_server', 'bt_navigator', 'velocity_smoother',
                        'collision_monitor', 'waypoint_follower']

    def activate_node(self, n):
        """激活单个节点（含重试；DDS 服务响应在本机会超时）"""
        for attempt in range(2):
            r = self.call_service(GetState, f'/{n}/get_state',
                                  GetState.Request(), timeout=5.0)
            cur = r.current_state.id if r else None
            if cur == State.PRIMARY_STATE_ACTIVE:
                return True
            if cur == State.PRIMARY_STATE_UNCONFIGURED:
                self.call_service(ChangeState, f'/{n}/change_state',
                                  ChangeState.Request(
                                      transition=Transition(id=Transition.TRANSITION_CONFIGURE)),
                                  timeout=10.0)
            self.call_service(ChangeState, f'/{n}/change_state',
                              ChangeState.Request(
                                  transition=Transition(id=Transition.TRANSITION_ACTIVATE)),
                              timeout=15.0)
            r2 = self.call_service(GetState, f'/{n}/get_state',
                                   GetState.Request(), timeout=5.0)
            if r2 and r2.current_state.id == State.PRIMARY_STATE_ACTIVE:
                return True
            self.get_logger().warn(f'  {n} 激活尝试 {attempt + 1} 失败，重试')
        r2 = self.call_service(GetState, f'/{n}/get_state', GetState.Request(), timeout=5.0)
        ok = r2 and r2.current_state.id == State.PRIMARY_STATE_ACTIVE
        self.get_logger().info(f'  {n:<20} -> {"active" if ok else "FAILED"}')
        return bool(ok)

    def activate_localization(self, slam_mode=False):
        if slam_mode:
            self.get_logger().info('[LIFECYCLE] SLAM 模式：定位组由 slam_toolbox 承担，'
                                   '跳过 map_server/amcl（它们不存在）')
            return True
        self.get_logger().info('[LIFECYCLE] 阶段1：激活定位组（map_server → amcl）')
        ok = True
        for n in self.LOCALIZATION_NODES:
            ok &= self.activate_node(n)
        return ok

    def activate_stack(self):
        self.get_logger().info('[LIFECYCLE] 阶段2：激活导航组（初始位姿已生效）')
        ok = True
        for n in self.NAVIGATION_NODES:
            ok &= self.activate_node(n)
        return ok

    # ---------- Step 4：等 map -> odom（AMCL 发出） ----------
    def wait_map_tf(self, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            try:
                self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
                self.get_logger().info('[TF] map->odom OK（AMCL 已开始定位）')
                return True
            except Exception:
                pass
        self.get_logger().error('[TF] map->odom 不存在！检查初始位姿是否被 AMCL 接受'
                                '（日志应有 initialPoseReceived）。')
        return False

    # ---------- Step 4.5：可选，运行时切换控制器 ----------
    def select_controller(self, name):
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(String, '/controller_selector', qos)
        msg = String(data=name)
        for _ in range(3):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.3)
        self.get_logger().info(f'[CTRL] 已请求切换控制器到 {name}')

    # ---------- Step 5：发导航目标并等待结果 ----------
    def navigate_to(self, x, y, yaw_deg, timeout):
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.yaw_to_quat(yaw_deg)

        client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info(f'[GOAL] 等待 bt_navigator action server...')
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('[GOAL] /navigate_to_pose action server 不可用')
            return False

        goal = NavigateToPose.Goal(pose=goal_pose)
        self.get_logger().info(
            f'[GOAL] 发送目标 ({x:.2f}, {y:.2f}, yaw={yaw_deg:.0f}°)，超时 {timeout}s')
        future = client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('[GOAL] 目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        last_log = 0.0
        last_fb_log = 0.0
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                self.get_logger().error('[GOAL] 导航超时，取消目标...')
                # 必须用异步取消：服务器卡死时同步 cancel_goal 会永久阻塞
                cancel_fut = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=3.0)
                return False
            # 兜底心跳，防止 feedback 不到时看不到进展
            if time.time() - last_log > 10.0:
                last_log = time.time()
                self.get_logger().info(f'[WAIT] 已等待 {time.time()-start:.0f}s ...')

        res = result_future.result()
        status = GOAL_STATUS.get(res.status, str(res.status))
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[RESULT] ✅ 到达目标（SUCCEEDED）')
            return True
        self.get_logger().error(f'[RESULT] ❌ 导航失败：{status}')
        return False

    def _on_feedback(self, fb):
        # bt_navigator 反馈频率很高（可达 100Hz），限频 1 条/秒防刷屏
        now = time.time()
        if now - getattr(self, '_last_fb', 0.0) < 1.0:
            return
        self._last_fb = now
        f = fb.feedback
        pose = getattr(f, 'current_pose', None)
        dist = getattr(f, 'distance_remaining', None)
        if pose is not None and dist is not None:
            p = pose.pose.position
            self.get_logger().info(
                f'[FEEDBACK] 当前 ({p.x:.2f}, {p.y:.2f}) 剩余 {dist:.2f} m')


def main():
    ap = argparse.ArgumentParser(description='端到端导航自动测试')
    ap.add_argument('--goal', nargs=2, type=float, default=[1.5, 0.5],
                    metavar=('X', 'Y'), help='目标点（map 坐标，默认 1.5 0.5）')
    ap.add_argument('--yaw', type=float, default=90.0, help='目标朝向（度）')
    ap.add_argument('--init', nargs=2, type=float, default=[-2.0, -0.5],
                    metavar=('X', 'Y'), help='初始位姿（必须与 Gazebo 出生点一致！）')
    ap.add_argument('--init-yaw', type=float, default=0.0, help='初始朝向（度）')
    ap.add_argument('--timeout', type=float, default=150.0, help='导航超时（秒）')
    ap.add_argument('--controller', default=None,
                    help='运行时切换控制器（FollowPath/PID/Stanley/DWB/MPPI/MPC）')
    ap.add_argument('--viz', action='store_true',
                    help='拉起实时可视化面板（地图/轨迹/h(t)/标称vs CBF）')
    ap.add_argument('--slam', action='store_true',
                    help='SLAM 模式：栈以 slam:=True 启动（无 map_server/amcl，'
                         '跳过初始位姿；目标坐标用建图坐标系）')
    args = ap.parse_args()

    rclpy.init()
    tester = E2ENavTester()
    viz_proc = None
    if args.viz:
        viz_script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'live_monitor.py')
        viz_proc = subprocess.Popen(
            [sys.executable, viz_script, '--d-safe', '0.22'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('[VIZ] 实时面板已启动（独立窗口：地图/轨迹/h(t)/速度对比）')
    ok = True
    try:
        ok &= tester.check_odom_tf()
        if not ok:
            sys.exit(1)
        ok &= tester.activate_localization(slam_mode=args.slam)
        if not ok:
            sys.exit(1)
        if not args.slam:
            ok &= tester.set_initial_pose(*args.init, args.init_yaw, True)
            if not ok:
                sys.exit(1)
        else:
            tester.get_logger().info('[INIT] SLAM 模式：建图起点即地图原点 (0,0)，跳过初始位姿')
        ok &= tester.activate_stack()
        if not ok:
            sys.exit(1)
        ok &= tester.wait_map_tf()
        if not ok:
            sys.exit(1)
        if args.controller:
            tester.select_controller(args.controller)
        ok &= tester.navigate_to(args.goal[0], args.goal[1], args.yaw, args.timeout)
    finally:
        rclpy.shutdown()
        if viz_proc is not None:
            viz_proc.terminate()          # 面板随实验结束关闭
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
