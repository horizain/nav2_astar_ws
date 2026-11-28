#!/usr/bin/env python3
# Copyright 2026 ruijianding
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Offline diagnostic inspector and visualizer for scan_localizer .slocbb blackbox bundles."""

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import math
import os
import struct
import sys
from typing import List, Optional, Tuple

MAGIC = b'SLOCBB01'
FORMAT_VERSION = 1
MAX_BUNDLE_ODOMS = 200000
MAX_BUNDLE_SCANS = 50000
MAX_SCAN_POINTS = 512

TRIGGER_NAMES = {
    0: 'NONE',
    1: 'ENTER_GLOBAL',
    2: 'ENTER_LOST',
    3: 'WALL_PHYS_REJECT',
    4: 'REJECT_STREAK',
    5: 'EXTERNAL_REQUEST',
}

STATE_NAMES = {
    0: 'UNINIT',
    1: 'TRACKING',
    2: 'DEGRADED',
    3: 'LOST',
    4: 'GLOBAL',
}

GATE_FLAGS = [
    (1 << 0, 'kInvalid'),
    (1 << 1, 'kInlier'),
    (1 << 2, 'kScore'),
    (1 << 3, 'kWallPhys'),
    (1 << 4, 'kCov'),
    (1 << 5, 'kMahal'),
]


@dataclass
class OdomRecord:
    """Single odom/IMU sample stored in a .slocbb bundle."""

    stamp_s: float
    x: float
    y: float
    theta: float
    imu_yaw: float
    has_imu: bool


@dataclass
class ScanRecord:
    """Single scan + LocalizerCore diagnostic snapshot stored in a .slocbb bundle."""

    stamp_s: float
    x: float
    y: float
    theta: float
    state: int
    accepted: bool
    score: float
    inlier_ratio: float
    reject_streak: int
    reject_flags: int
    raw_score: float
    raw_inlier: float
    wall_visibility: float
    frontier_visibility: float
    short_ratio: float
    frontier_ratio: float
    mahal_dist: float
    points: List[Tuple[float, float]]


@dataclass
class BlackboxBundle:
    """Parsed .slocbb accident/fault blackbox bundle."""

    trigger_stamp_s: float
    reason: int
    odoms: List[OdomRecord]
    scans: List[ScanRecord]

    @property
    def reason_name(self) -> str:
        return TRIGGER_NAMES.get(self.reason, f'UNKNOWN({self.reason})')


def decode_gate_flags(flags: int) -> str:
    """Return human-readable GateFlag bitmask string."""
    if flags == 0:
        return 'kNone'
    names = [name for bit, name in GATE_FLAGS if (flags & bit) != 0]
    return '|'.join(names) if names else f'0x{flags:x}'


def load_slocbb(path: str) -> BlackboxBundle:
    """Parse a binary .slocbb bundle produced by sloc::BlackboxRecorder."""
    with open(path, 'rb') as f:
        data = f.read()

    header_fmt = '<8sIdBII'
    header_size = struct.calcsize(header_fmt)
    if len(data) < header_size:
        raise ValueError(f'File too small to be a valid .slocbb bundle: {path}')

    magic, version, trig_s, reason, n_odoms, n_scans = struct.unpack_from(header_fmt, data, 0)
    if magic != MAGIC:
        raise ValueError(f'Invalid magic {magic!r}, expected {MAGIC!r}')
    if version != FORMAT_VERSION:
        raise ValueError(f'Unsupported .slocbb format version {version}')
    if n_odoms > MAX_BUNDLE_ODOMS or n_scans > MAX_BUNDLE_SCANS:
        raise ValueError(f'Bundle record counts out of bounds: odoms={n_odoms}, scans={n_scans}')

    offset = header_size
    odom_fmt = '<dddddB'
    odom_size = struct.calcsize(odom_fmt)
    odoms: List[OdomRecord] = []
    for _ in range(n_odoms):
        if offset + odom_size > len(data):
            raise ValueError('Truncated .slocbb file while reading odom records')
        st, ox, oy, oth, iyaw, himu = struct.unpack_from(odom_fmt, data, offset)
        offset += odom_size
        odoms.append(OdomRecord(st, ox, oy, oth, iyaw, bool(himu)))

    scan_fmt = '<ddddBBddiIdddddddH'
    scan_size = struct.calcsize(scan_fmt)
    scans: List[ScanRecord] = []
    for _ in range(n_scans):
        if offset + scan_size > len(data):
            raise ValueError('Truncated .slocbb file while reading scan header')
        vals = struct.unpack_from(scan_fmt, data, offset)
        offset += scan_size
        npts = vals[17]
        if npts > MAX_SCAN_POINTS:
            raise ValueError(f'Scan point count {npts} exceeds limit {MAX_SCAN_POINTS}')
        pts_bytes = npts * 2 * 4
        if offset + pts_bytes > len(data):
            raise ValueError('Truncated .slocbb file while reading scan points')
        pts: List[Tuple[float, float]] = []
        if npts > 0:
            floats = struct.unpack_from(f'<{npts * 2}f', data, offset)
            offset += pts_bytes
            pts = [(floats[2 * i], floats[2 * i + 1]) for i in range(npts)]
        scans.append(
            ScanRecord(
                stamp_s=vals[0],
                x=vals[1],
                y=vals[2],
                theta=vals[3],
                state=vals[4],
                accepted=bool(vals[5]),
                score=vals[6],
                inlier_ratio=vals[7],
                reject_streak=vals[8],
                reject_flags=vals[9],
                raw_score=vals[10],
                raw_inlier=vals[11],
                wall_visibility=vals[12],
                frontier_visibility=vals[13],
                short_ratio=vals[14],
                frontier_ratio=vals[15],
                mahal_dist=vals[16],
                points=pts,
            )
        )
    return BlackboxBundle(trigger_stamp_s=trig_s, reason=reason, odoms=odoms, scans=scans)


def find_trigger_frame_index(bundle: BlackboxBundle) -> int:
    """Return index of scan closest to trigger_stamp_s."""
    if not bundle.scans:
        return -1
    return min(
        range(len(bundle.scans)),
        key=lambda i: abs(bundle.scans[i].stamp_s - bundle.trigger_stamp_s),
    )


def format_summary(bundle: BlackboxBundle, window_frames: int = 8) -> str:
    """Format a human-readable diagnostic report for terminal inspection."""
    lines: List[str] = []
    n_sc = len(bundle.scans)
    n_od = len(bundle.odoms)
    n_acc = sum(1 for s in bundle.scans if s.accepted)
    n_rej = n_sc - n_acc
    dur = (bundle.scans[-1].stamp_s - bundle.scans[0].stamp_s) if n_sc >= 2 else 0.0
    acc_pct = (100.0 * n_acc / n_sc) if n_sc > 0 else 0.0

    lines.append('=== scan_localizer Blackbox Diagnostic Summary ===')
    lines.append(f'Trigger Reason : {bundle.reason_name}')
    lines.append(f'Trigger Stamp  : {bundle.trigger_stamp_s:.3f} s')
    lines.append(f'Window Span    : {dur:.2f} s ({n_sc} scans, {n_od} odoms)')
    lines.append(f'Acceptance     : {n_acc}/{n_sc} ({acc_pct:.1f}%), rejected={n_rej}')

    flag_counts: Counter = Counter()
    state_counts: Counter = Counter()
    for s in bundle.scans:
        state_counts[STATE_NAMES.get(s.state, str(s.state))] += 1
        if not s.accepted:
            for bit, name in GATE_FLAGS:
                if (s.reject_flags & bit) != 0:
                    flag_counts[name] += 1

    st_str = ', '.join(f'{k}={v}' for k, v in state_counts.items())
    lines.append(f'State Counts   : {st_str}')
    if flag_counts:
        fl_str = ', '.join(f'{k}={v}' for k, v in flag_counts.most_common())
        lines.append(f'Reject Causes  : {fl_str}')
    else:
        lines.append('Reject Causes  : none')

    trig_idx = find_trigger_frame_index(bundle)
    if trig_idx >= 0:
        lo = max(0, trig_idx - window_frames // 2)
        hi = min(n_sc, trig_idx + window_frames // 2 + 1)
        lines.append(f'\n--- Frames around trigger (idx {lo}..{hi - 1}, trigger={trig_idx}) ---')
        lines.append(
            f'{"idx":>4} {"dt(s)":>7} {"state":>8} {"acc":>3} {"stk":>3} '
            f'{"score":>6} {"inlr":>6} {"wall":>5} {"shrt":>5} {"mahal":>7}  flags'
        )
        for i in range(lo, hi):
            s = bundle.scans[i]
            mark = '*' if i == trig_idx else ' '
            dt = s.stamp_s - bundle.trigger_stamp_s
            st_name = STATE_NAMES.get(s.state, str(s.state))
            lines.append(
                f'{mark}{i:3d} {dt:+7.2f} {st_name:>8} {int(s.accepted):>3} {s.reject_streak:>3} '
                f'{s.score:6.3f} {s.inlier_ratio:6.3f} {s.wall_visibility:5.2f} '
                f'{s.short_ratio:5.2f} {s.mahal_dist:7.1f}  {decode_gate_flags(s.reject_flags)}'
            )
    return '\n'.join(lines)


def export_csv(bundle: BlackboxBundle, csv_path: str) -> None:
    """Export all scan records and diagnostic fields to a CSV file."""
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'idx', 'stamp_s', 'dt_trigger_s', 'x', 'y', 'theta', 'state', 'accepted',
            'reject_streak', 'reject_flags', 'reject_flags_str', 'score', 'inlier_ratio',
            'raw_score', 'raw_inlier', 'wall_visibility', 'frontier_visibility',
            'short_ratio', 'frontier_ratio', 'mahal_dist', 'num_points',
        ])
        for i, s in enumerate(bundle.scans):
            writer.writerow([
                i, f'{s.stamp_s:.6f}', f'{s.stamp_s - bundle.trigger_stamp_s:.6f}',
                f'{s.x:.5f}', f'{s.y:.5f}', f'{s.theta:.5f}',
                STATE_NAMES.get(s.state, str(s.state)), int(s.accepted),
                s.reject_streak, s.reject_flags, decode_gate_flags(s.reject_flags),
                f'{s.score:.5f}', f'{s.inlier_ratio:.5f}',
                f'{s.raw_score:.5f}', f'{s.raw_inlier:.5f}',
                f'{s.wall_visibility:.5f}', f'{s.frontier_visibility:.5f}',
                f'{s.short_ratio:.5f}', f'{s.frontier_ratio:.5f}',
                f'{s.mahal_dist:.4f}', len(s.points),
            ])


def _load_map_yaml_simple(yaml_path: str) -> Tuple[str, float, float, float]:
    """Minimal ROS map.yaml parser without requiring PyYAML."""
    image_rel = ''
    res = 0.05
    ox, oy = 0.0, 0.0
    with open(yaml_path, 'r', encoding='utf-8') as f:
        for raw_line in f:
            line = raw_line.split('#', 1)[0].strip()
            if not line:
                continue
            if line.startswith('image:'):
                image_rel = line.split(':', 1)[1].strip().strip('"').strip("'")
            elif line.startswith('resolution:'):
                res = float(line.split(':', 1)[1].strip())
            elif line.startswith('origin:'):
                inside = line.split('[', 1)[1].split(']', 1)[0]
                parts = [float(p.strip()) for p in inside.split(',')]
                ox, oy = parts[0], parts[1]
    img_path = image_rel if os.path.isabs(image_rel) else os.path.join(
        os.path.dirname(os.path.abspath(yaml_path)), image_rel
    )
    return img_path, res, ox, oy


def render_diagnostic_plot(
    bundle: BlackboxBundle,
    out_png: str,
    map_yaml: Optional[str] = None,
    frame_idx: Optional[int] = None,
) -> None:
    """Render a 2x2 diagnostic figure (timelines + trigger scan overlay)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    if not bundle.scans:
        raise ValueError('Cannot plot empty blackbox bundle (0 scans)')

    t = np.array([s.stamp_s - bundle.trigger_stamp_s for s in bundle.scans])
    scores = np.array([s.score for s in bundle.scans])
    inliers = np.array([s.inlier_ratio for s in bundle.scans])
    raw_inliers = np.array([s.raw_inlier for s in bundle.scans])
    wall_vis = np.array([s.wall_visibility for s in bundle.scans])
    short_r = np.array([s.short_ratio for s in bundle.scans])
    mahal = np.array([min(s.mahal_dist, 100.0) for s in bundle.scans])
    states = np.array([s.state for s in bundle.scans])

    idx = frame_idx if frame_idx is not None else find_trigger_frame_index(bundle)
    idx = max(0, min(len(bundle.scans) - 1, idx))
    focus = bundle.scans[idx]

    fig, axes = plt.subplots(2, 2, figsize=(15, 10), dpi=160)
    fig.patch.set_facecolor('#ffffff')
    fig.suptitle(
        f'scan_localizer Blackbox Inspector | Trigger: {bundle.reason_name} '
        f'@ t={bundle.trigger_stamp_s:.2f}s ({len(bundle.scans)} scans)',
        fontsize=13.5, fontweight='bold', y=0.98, color='#0f172a',
    )

    # Panel A: Match Score & Inlier Ratio
    ax = axes[0, 0]
    ax.plot(t, scores, color='#0284c7', lw=1.6, label='track_score')
    ax.plot(t, inliers, color='#16a34a', lw=1.5, label='static_inlier')
    ax.plot(t, raw_inliers, color='#94a3b8', lw=1.1, ls='--', label='raw_inlier')
    ax.axvline(0.0, color='#dc2626', ls='--', lw=1.2, label='trigger (t=0)')
    ax.set_title('A. Match Score & Inlier Ratio Timeline', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time Relative to Trigger (s)')
    ax.set_ylabel('Ratio / Score')
    ax.set_ylim(-0.02, 1.05)
    ax.legend(loc='lower left', fontsize=8.5, framealpha=0.9)
    ax.grid(True, alpha=0.5)

    # Panel B: Beam Physics & Mahalanobis Distance
    ax = axes[0, 1]
    ax.plot(t, wall_vis, color='#0f766e', lw=1.6, label='wall_visibility')
    ax.plot(t, short_r, color='#ea580c', lw=1.4, label='short_ratio (dynamic)')
    ax.axvline(0.0, color='#dc2626', ls='--', lw=1.2)
    ax.set_title('B. Beam Physics & Mahalanobis Distance', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time Relative to Trigger (s)')
    ax.set_ylabel('Visibility / Occlusion Ratio')
    ax.set_ylim(-0.02, 1.05)
    ax2 = ax.twinx()
    ax2.plot(t, mahal, color='#7c3aed', lw=1.2, alpha=0.8, label='mahal_dist (capped 100)')
    ax2.set_ylabel('Mahalanobis Distance', color='#7c3aed')
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=8.2, framealpha=0.9)
    ax.grid(True, alpha=0.5)

    # Panel C: State & GateFlag Timeline
    ax = axes[1, 0]
    ax.step(t, states, where='post', color='#1e293b', lw=1.8, label='LocState')
    flag_colors = {
        'kInvalid': '#64748b',
        'kInlier': '#f59e0b',
        'kScore': '#eab308',
        'kWallPhys': '#dc2626',
        'kCov': '#ec4899',
        'kMahal': '#7c3aed',
    }
    for bit_idx, (bit, name) in enumerate(GATE_FLAGS):
        hit_t = [t[i] for i, s in enumerate(bundle.scans) if (s.reject_flags & bit) != 0]
        if hit_t:
            ax.scatter(
                hit_t, [4.35 + 0.18 * bit_idx] * len(hit_t),
                s=18, color=flag_colors[name], label=name, zorder=5,
            )
    ax.axvline(0.0, color='#dc2626', ls='--', lw=1.2)
    ax.set_yticks([0, 1, 2, 3, 4])
    ax.set_yticklabels(['UNINIT', 'TRACKING', 'DEGRADED', 'LOST', 'GLOBAL'], fontsize=8.5)
    ax.set_ylim(-0.3, 5.5)
    ax.set_title('C. State Machine & GateFlag Rejection Timeline', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time Relative to Trigger (s)')
    ax.legend(loc='lower right', fontsize=8.0, ncol=2, framealpha=0.9)
    ax.grid(True, alpha=0.5)

    # Panel D: Trigger Scan Overlay on Map (or local frame)
    ax = axes[1, 1]
    if map_yaml is not None:
        from PIL import Image
        img_path, res, ox, oy = _load_map_yaml_simple(map_yaml)
        img = np.array(Image.open(img_path))
        h_img, w_img = img.shape[:2]
        extent = [ox, ox + w_img * res, oy, oy + h_img * res]
        ax.imshow(img, cmap='gray', origin='upper', extent=extent, alpha=0.88)

    xs = [s.x for s in bundle.scans]
    ys = [s.y for s in bundle.scans]
    ax.plot(xs, ys, color='#0284c7', lw=1.5, alpha=0.85, label='Estimated Trajectory')
    if focus.points:
        c = math.cos(focus.theta)
        s_th = math.sin(focus.theta)
        wx = [focus.x + c * px - s_th * py for px, py in focus.points]
        wy = [focus.y + s_th * px + c * py for px, py in focus.points]
        ax.scatter(wx, wy, s=8, c='#dc2626', alpha=0.85, label=f'Scan idx={idx} ({len(wx)} pts)')
    ax.scatter([focus.x], [focus.y], s=60, c='#16a34a', zorder=6, label='Pose at Focus Frame')
    ax.arrow(
        focus.x, focus.y, 0.8 * math.cos(focus.theta), 0.8 * math.sin(focus.theta),
        head_width=0.25, head_length=0.25, fc='#16a34a', ec='#16a34a', zorder=6,
    )
    ax.set_title(
        f'D. Focus Frame #{idx} (dt={focus.stamp_s - bundle.trigger_stamp_s:+.2f}s, '
        f'{decode_gate_flags(focus.reject_flags)})',
        fontsize=11, fontweight='bold',
    )
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.legend(loc='lower right', fontsize=8.2, framealpha=0.92)
    ax.grid(True, alpha=0.4)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Inspect and visualize scan_localizer .slocbb blackbox bundles offline.'
    )
    parser.add_argument('bundle', help='Path to .slocbb binary blackbox file')
    parser.add_argument('--map', dest='map_yaml', default=None, help='Optional ROS map.yaml path')
    parser.add_argument('--csv', dest='csv_out', default=None, help='Export per-frame CSV path')
    parser.add_argument('--plot', dest='plot_out', default=None, help='Render 2x2 diagnostic PNG')
    parser.add_argument(
        '--frame-idx', type=int, default=None,
        help='Focus scan frame index for Panel D (default: frame closest to trigger)',
    )
    parser.add_argument(
        '--window', type=int, default=10,
        help='Number of frames around trigger to print in terminal table',
    )
    args = parser.parse_args(argv)

    bundle = load_slocbb(args.bundle)
    print(format_summary(bundle, window_frames=args.window))
    if args.csv_out:
        export_csv(bundle, args.csv_out)
        print(f'\nExported CSV -> {args.csv_out}')
    if args.plot_out:
        render_diagnostic_plot(
            bundle, args.plot_out, map_yaml=args.map_yaml, frame_idx=args.frame_idx
        )
        print(f'Rendered plot -> {args.plot_out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
