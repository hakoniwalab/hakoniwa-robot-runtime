"""Regression tests for comparing RPC records against captured fixtures."""

from hashlib import sha256
import gzip
import io
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import check_step_goldens


RECORD = struct.Struct("<QIIQ13d")
FILENAME = "Car-1.bin"


class GoldenComparisonTests(unittest.TestCase):
    def setUp(self):
        work = tempfile.TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.root = Path(work.name)
        self.first = self.root / "run-1"
        self.second = self.root / "run-2"
        self.first.mkdir()
        self.second.mkdir()
        self.warnings = self.root / "golden-warnings.json"
        self.records = [[0, 0, 0, 1000, 1.0, *([0.0] * 12)],
                        [1, 0, 0, 2000, 2.0, *([0.0] * 12)]]
        reference = self.encode()
        (self.root / "Car-1.bin.gz").write_bytes(gzip.compress(reference))
        fixture = self.root / "goldens.json"
        fixture.write_text(json.dumps({
            "_provenance": {"uname_sm": "Darwin arm64", "compiler": "AppleClang 21.0.0"},
            "_tolerance": {"absolute": 1e-9, "by_column": {}, "reason": "comparison test"},
            "sample": {FILENAME: {
                "bytes": len(reference), "sha256": sha256(reference).hexdigest(),
                "payload": "Car-1.bin.gz", "format": "ackermann-body",
            }},
        }))
        reference_patch = patch.object(check_step_goldens, "REFERENCES", fixture)
        reference_patch.start()
        self.addCleanup(reference_patch.stop)
        env = patch.dict(os.environ, {"HAKO_GOLDEN_COMPILER": "fixture compiler"})
        env.start()
        self.addCleanup(env.stop)

    def encode(self):
        return b"".join(RECORD.pack(*row) for row in self.records)

    def verify(self, directory=None):
        (directory or self.first).joinpath(FILENAME).write_bytes(self.encode())
        log = io.StringIO()
        with redirect_stdout(log):
            check_step_goldens.verify("sample", directory or self.first)
        return log.getvalue()

    def test_g1_identical_bytes_without_warning(self):
        log = self.verify()
        self.assertIn("matches sample reference", log)
        self.assertNotIn("WARNING", log)
        if self.warnings.exists():
            self.assertEqual(json.loads(self.warnings.read_text())["warnings"], [])

    def test_g2_warns_for_small_float_difference(self):
        self.records[1][4] += 1e-12
        log = self.verify()
        [warning] = json.loads(self.warnings.read_text())["warnings"]
        self.assertIn("WARNING", log)
        self.assertEqual(warning["test"], "sample")
        self.assertEqual(warning["record"], FILENAME)
        self.assertEqual(warning["column"], "pose.translation.x")
        self.assertAlmostEqual(warning["max_abs_diff"], 1e-12, delta=1e-15)
        self.assertEqual(warning["max_diff_row"], 2)
        self.assertEqual(warning["differing_rows"], 1)
        self.assertEqual(warning["reference"]["uname_sm"], "Darwin arm64")
        self.assertEqual(warning["current"]["compiler"], "fixture compiler")

    def test_g3_rejects_float_outside_tolerance(self):
        self.records[1][4] += 1e-6
        with self.assertRaisesRegex(ValueError, "pose.translation.x.*tolerance"):
            self.verify()

    def test_column_override_uses_its_own_absolute_tolerance(self):
        fixture = check_step_goldens.REFERENCES
        manifest = json.loads(fixture.read_text())
        manifest["_tolerance"]["by_column"]["pose.translation.x"] = 1e-13
        fixture.write_text(json.dumps(manifest))
        self.records[1][4] += 1e-12
        with self.assertRaisesRegex(ValueError, "pose.translation.x.*tolerance"):
            self.verify()

    def test_g4_rejects_missing_row(self):
        self.records.pop()
        with self.assertRaisesRegex(ValueError, "size|row"):
            self.verify()

    def test_g5_rejects_step_or_tick_changes(self):
        for column in (0, 3):
            with self.subTest(column=column):
                self.records[1][column] += 1
                try:
                    with self.assertRaisesRegex(ValueError, "exact fields.*row 2"):
                        self.verify()
                finally:
                    self.records[1][column] -= 1

    def test_g6_rejects_a_one_byte_repeat_difference(self):
        original = self.encode()
        (self.first / FILENAME).write_bytes(original)
        self.records[0][4] = math.nextafter(1.0, math.inf)
        changed = self.encode()
        self.assertEqual(sum(a != b for a, b in zip(original, changed)), 1)
        (self.second / FILENAME).write_bytes(changed)
        with self.assertRaisesRegex(ValueError, "repeatability mismatch"):
            check_step_goldens.verify("sample", self.first, self.second, repeat=True)


if __name__ == "__main__":
    unittest.main()

