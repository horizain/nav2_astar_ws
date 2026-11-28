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
"""Unit tests for tools/inspect_blackbox.py (.slocbb parser, summary, CSV export)."""

import csv
import os
import struct
import sys
import tempfile
import unittest

TOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'tools'))
sys.path.insert(0, TOOLS_DIR)

import inspect_blackbox as ib  # noqa: E402


def _write_synthetic_slocbb(path: str) -> None:
    """Write a minimal valid v1 .slocbb binary file matching blackbox_recorder.cpp."""
    with open(path, 'wb') as f:
        # Header: magic, version=1, trigger_stamp=12.5, reason=4 (REJECT_STREAK), 2 odoms, 3 scans
        f.write(struct.pack('<8sIdBII', b'SLOCBB01', 1, 12.5, 4, 2, 3))
        # 2 odom records
        f.write(struct.pack('<dddddB', 12.0, 1.0, 2.0, 0.1, 0.12, 1))
        f.write(struct.pack('<dddddB', 12.5, 1.5, 2.1, 0.2, 0.22, 1))
        # 3 scan records (1 accepted, 2 rejected with kMahal / kWallPhys)
        scans = [
            (12.0, 1.0, 2.0, 0.1, 1, 1, 0.92, 0.88, 0, 0, 0.92, 0.88, 1.0, 1.0, 0.05, 0.0, 1.5,
             [(1.0, 0.0), (0.0, 2.0)]),
            (12.5, 1.5, 2.1, 0.2, 2, 0, 0.78, 0.72, 5, (1 << 5), 0.78, 0.72, 1.0, 1.0, 0.10, 0.0,
             24.3, [(1.1, 0.1)]),
            (12.6, 1.6, 2.2, 0.25, 3, 0, 0.40, 0.35, 6, (1 << 3) | (1 << 1), 0.50, 0.35, 0.30,
             1.0, 0.20, 0.0, 40.0, []),
        ]
        for sc in scans:
            pts = sc[17]
            f.write(struct.pack('<ddddBBddiIdddddddH', *sc[:17], len(pts)))
            for px, py in pts:
                f.write(struct.pack('<ff', px, py))


class InspectBlackboxTest(unittest.TestCase):
    """Verify .slocbb binary parsing, corruption guards, summary, and CSV export."""

    def test_roundtrip_summary_and_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bb_path = os.path.join(tmp, 'fault.slocbb')
            csv_path = os.path.join(tmp, 'fault.csv')
            _write_synthetic_slocbb(bb_path)

            bundle = ib.load_slocbb(bb_path)
            self.assertEqual(bundle.reason_name, 'REJECT_STREAK')
            self.assertAlmostEqual(bundle.trigger_stamp_s, 12.5)
            self.assertEqual(len(bundle.odoms), 2)
            self.assertEqual(len(bundle.scans), 3)
            self.assertTrue(bundle.odoms[0].has_imu)
            self.assertEqual(len(bundle.scans[0].points), 2)
            self.assertEqual(ib.find_trigger_frame_index(bundle), 1)
            self.assertEqual(
                ib.decode_gate_flags(bundle.scans[2].reject_flags), 'kInlier|kWallPhys'
            )

            summary = ib.format_summary(bundle)
            self.assertIn('REJECT_STREAK', summary)
            self.assertIn('kMahal=1', summary)
            self.assertIn('kWallPhys=1', summary)

            ib.export_csv(bundle, csv_path)
            with open(csv_path, 'r', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[1]['reject_flags_str'], 'kMahal')
            self.assertEqual(rows[2]['state'], 'LOST')

    def test_rejects_corrupt_magic_and_truncated_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bad_magic = os.path.join(tmp, 'bad.slocbb')
            with open(bad_magic, 'wb') as f:
                f.write(b'NOTMAGIC' + b'\x00' * 32)
            with self.assertRaises(ValueError):
                ib.load_slocbb(bad_magic)

            trunc = os.path.join(tmp, 'trunc.slocbb')
            with open(trunc, 'wb') as f:
                f.write(struct.pack('<8sIdBII', b'SLOCBB01', 1, 0.0, 1, 10, 0))
            with self.assertRaises(ValueError):
                ib.load_slocbb(trunc)


if __name__ == '__main__':
    unittest.main()
