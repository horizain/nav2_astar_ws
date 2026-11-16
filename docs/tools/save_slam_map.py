#!/usr/bin/env python3
"""抓取一帧 /map（OccupancyGrid）并保存为 nav2 标准地图文件（PGM + YAML）。

为什么不用 map_saver_cli：它对订阅 QoS/发现时序敏感（本机实测两次超时）；
直接订阅 slam_toolbox 的 /map 更可靠，顺便把地图文件格式讲明白。

地图文件格式（docs/20 §4）：
  lab_built.pgm   灰度图：254=自由 0=占据 205=未知（栅格 0.05m）
  lab_built.yaml  分辨率/原点/占据阈值 + 图像相对路径

用法：python3 docs/tools/save_slam_map.py --out docs/tools/maps/lab_built
"""
import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapGrabber(Node):

    def __init__(self):
        super().__init__('map_grabber')
        self.msg = None
        # slam_toolbox 的 /map 是 transient_local，兼容订阅
        self.create_subscription(OccupancyGrid, 'map', self._cb, 10)

    def _cb(self, m):
        self.msg = m


def save(msg, out_prefix):
    w, h = msg.info.width, msg.info.height
    res, ox, oy = msg.info.resolution, msg.info.origin.position.x, \
        msg.info.origin.position.y

    # OccupancyGrid: -1=未知, 0~100=占据概率 → PGM: 254=自由 0=占据 205=未知
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)
    pgm = np.full((h, w), 205, dtype=np.uint8)
    pgm[data == -1] = 205
    pgm[(data >= 0) & (data < 65)] = 254          # occ < 0.65 → 自由
    pgm[data >= 65] = 0                            # occ ≥ 0.65 → 占据
    # OccupancyGrid 行 0 是 y 最小端；PGM 行 0 是图最上端（y 最大端）→ 翻转
    pgm = np.flipud(pgm)

    pgm_path = out_prefix + '.pgm'
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'# created by save_slam_map.py (slam_toolbox /map)\n'.encode())
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(pgm.tobytes())

    yaml_path = out_prefix + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write(f'image: {os.path.basename(pgm_path)}\n')
        f.write(f'resolution: {res}\n')
        f.write(f'origin: [{ox}, {oy}, 0.0]\n')
        f.write(f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    free = int((pgm == 254).sum())
    occ = int((pgm == 0).sum())
    print(f'已保存: {pgm_path} ({w}x{h} @ {res}m/格)')
    print(f'  原点=({ox:.2f},{oy:.2f})  自由格 {free}  障碍格 {occ}')
    return pgm_path


def main():
    ap = argparse.ArgumentParser(description='抓取 /map 保存为 nav2 地图文件')
    ap.add_argument('--out', default='docs/tools/maps/lab_built')
    args = ap.parse_args()

    rclpy.init()
    n = MapGrabber()
    n.get_logger().info('等待 /map 消息（slam_toolbox 每 5s 发布一次）...')
    deadline = time.time() + 30
    while rclpy.ok() and n.msg is None and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.5)
    if n.msg is None:
        print('30 秒内未收到 /map —— 确认 slam_toolbox 会话在运行')
        return
    out = args.out if os.path.isabs(args.out) else os.path.join(os.getcwd(), args.out)
    save(n.msg, out)
    rclpy.shutdown()


if __name__ == '__main__':
    import time
    main()
