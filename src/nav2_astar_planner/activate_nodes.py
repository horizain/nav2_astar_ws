#!/usr/bin/env python3
"""批量查询/激活 nav2 生命周期节点（单进程，避免逐个启动 CLI 的开销）"""
import sys
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition

NODES = [
    'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
    'bt_navigator', 'velocity_smoother', 'collision_monitor', 'waypoint_follower',
    'amcl', 'map_server',
]
STATE_NAMES = {State.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
               State.PRIMARY_STATE_INACTIVE: 'inactive',
               State.PRIMARY_STATE_ACTIVE: 'active',
               State.PRIMARY_STATE_FINALIZED: 'finalized'}


def call(node, srv_type, name, req, timeout=5.0):
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def main():
    activate = '--activate' in sys.argv
    rclpy.init()
    node = Node('lifecycle_batch')
    # 先等一小会儿让发现完成
    rclpy.spin_once(node, timeout_sec=1.0)

    states = {}
    for n in NODES:
        r = call(node, GetState, f'/{n}/get_state', GetState.Request(), timeout=3.0)
        if r is None:
            states[n] = 'n/a'
        else:
            states[n] = STATE_NAMES.get(r.current_state.id, f'id={r.current_state.id}')

    print("== 当前状态 ==")
    for n in NODES:
        print(f"  {n:<20} {states[n]}")

    if activate:
        print("\n== 激活 ==")
        for n in NODES:
            if states[n] == 'active':
                print(f"  {n:<20} 已激活")
                continue
            # 需要先 configure 再 activate
            if states[n] == 'unconfigured':
                call(node, ChangeState, f'/{n}/change_state',
                     ChangeState.Request(transition=Transition(id=Transition.TRANSITION_CONFIGURE)),
                     timeout=8.0)
            call(node, ChangeState, f'/{n}/change_state',
                 ChangeState.Request(transition=Transition(id=Transition.TRANSITION_ACTIVATE)),
                 timeout=8.0)
            r = call(node, GetState, f'/{n}/get_state', GetState.Request(), timeout=3.0)
            now = STATE_NAMES.get(r.current_state.id, '?') if r else 'n/a'
            print(f"  {n:<20} {states[n]} -> {now}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
