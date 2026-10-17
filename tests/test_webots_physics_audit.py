"""Protect the independent replay audit against altered physical and startup evidence."""
import copy
import csv
import gzip
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('physics_audit', ROOT / 'scripts/audit_webots_physics.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class PhysicsAuditTests(unittest.TestCase):
    def setUp(self):
        folder = ROOT / 'results/webots-physics-v1/after/speed-5-clearance-20'
        self.summary = json.loads((folder / 'summary.json').read_text())
        with gzip.open(folder / 'run.csv.gz', 'rt', newline='') as stream:
            self.rows = list(csv.DictReader(stream))

    def test_recorded_telemetry_reconstructs_the_verdict(self):
        self.assertEqual(audit.audit_case(self.summary, self.rows, True), [])

    def test_forged_stopping_margin_is_rejected(self):
        self.summary['minimum_true_distance_m'] += 1
        self.assertTrue(any('minimum_true_distance_m' in e for e in audit.audit_case(self.summary, self.rows, True)))

    def test_hidden_contact_is_rejected(self):
        self.rows[-1]['collision'] = 'True'
        self.assertTrue(any('collision' in e for e in audit.audit_case(self.summary, self.rows, True)))

    def test_unbraked_startup_exception_is_rejected(self):
        self.rows[1]['brake_command'] = '0.0'
        self.assertTrue(any('initialization' in e for e in audit.audit_case(self.summary, self.rows, True)))

    def test_acceleration_before_valid_startup_samples_is_rejected(self):
        self.rows[1]['accel_command'] = '0.1'
        self.assertTrue(any('three valid' in e for e in audit.audit_case(self.summary, self.rows, True)))


if __name__ == '__main__':
    unittest.main()
