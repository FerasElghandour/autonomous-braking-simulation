"""Unit tests for the summary.json schema validator in scripts/validate_results.py."""

import math
import os
import sys
import tempfile
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")
)

import validate_results as vr  # noqa: E402


def make_valid_summary(**overrides):
    summary = {
        "run_id": "run-20260101T000000Z",
        "timestamp": "2026-01-01T00:00:00+00:00",
        "webots_version": "R2025a",
        "python_version": "3.11.15",
        "scenario_config": {"target_speed_mps": 10.0},
        "simulation_completed": True,
        "end_reason": "stopped",
        "car_moved": True,
        "car_stopped": True,
        "collision_occurred": False,
        "sensor_readings_valid": True,
        "braking_triggered": True,
        "emergency_braking_count": 1,
        "minimum_sensor_distance_m": 4.2,
        "minimum_true_distance_m": 3.8,
        "maximum_speed_mps": 10.0,
        "total_simulation_time_s": 22.5,
        "final_speed_mps": 0.0,
        "safe_stopping_distance_m": 3.0,
        "pass": True,
        "failure_reasons": [],
    }
    summary.update(overrides)
    return summary


class TestSchemaValidation(unittest.TestCase):
    def test_valid_summary_has_no_errors(self):
        self.assertEqual(vr.validate_summary_schema(make_valid_summary()), [])

    def test_missing_required_field(self):
        summary = make_valid_summary()
        del summary["pass"]
        errors = vr.validate_summary_schema(summary)
        self.assertTrue(any("pass" in e for e in errors))

    def test_wrong_type_is_rejected(self):
        summary = make_valid_summary(car_moved="yes")  # should be bool, not str
        errors = vr.validate_summary_schema(summary)
        self.assertTrue(any("car_moved" in e for e in errors))

    def test_bool_not_accepted_where_number_expected(self):
        summary = make_valid_summary(maximum_speed_mps=True)
        errors = vr.validate_summary_schema(summary)
        self.assertTrue(any("maximum_speed_mps" in e for e in errors))

    def test_nan_float_is_rejected(self):
        summary = make_valid_summary(maximum_speed_mps=float("nan"))
        errors = vr.validate_summary_schema(summary)
        self.assertTrue(any("NaN" in e or "Infinity" in e for e in errors))

    def test_infinity_float_is_rejected(self):
        summary = make_valid_summary(final_speed_mps=float("inf"))
        errors = vr.validate_summary_schema(summary)
        self.assertTrue(any("Infinity" in e for e in errors))

    def test_nested_nan_in_scenario_config_is_rejected(self):
        summary = make_valid_summary(scenario_config={"bad": float("nan")})
        errors = vr.validate_summary_schema(summary)
        self.assertTrue(any("scenario_config.bad" in e for e in errors))

    def test_none_is_accepted_for_optional_numeric_field(self):
        summary = make_valid_summary(minimum_sensor_distance_m=None)
        errors = vr.validate_summary_schema(summary)
        self.assertEqual(errors, [])


class TestSafetyConditions(unittest.TestCase):
    def test_passing_summary_has_no_safety_errors(self):
        self.assertEqual(vr.check_safety_conditions(make_valid_summary()), [])

    def test_collision_is_a_failure(self):
        errors = vr.check_safety_conditions(make_valid_summary(collision_occurred=True))
        self.assertTrue(any("collision" in e for e in errors))

    def test_car_not_stopped_is_a_failure(self):
        errors = vr.check_safety_conditions(make_valid_summary(car_stopped=False))
        self.assertTrue(any("stop" in e for e in errors))

    def test_final_speed_too_high_is_a_failure(self):
        errors = vr.check_safety_conditions(make_valid_summary(final_speed_mps=0.5))
        self.assertTrue(any("final speed" in e for e in errors))

    def test_insufficient_stopping_margin_is_a_failure(self):
        errors = vr.check_safety_conditions(
            make_valid_summary(minimum_true_distance_m=1.0, safe_stopping_distance_m=3.0)
        )
        self.assertTrue(any("safety threshold" in e for e in errors))

    def test_timeout_end_reason_is_a_failure(self):
        errors = vr.check_safety_conditions(make_valid_summary(end_reason="timeout"))
        self.assertTrue(any("timeout" in e for e in errors))

    def test_self_reported_pass_false_is_flagged(self):
        errors = vr.check_safety_conditions(make_valid_summary(**{"pass": False}))
        self.assertTrue(any("pass=false" in e for e in errors))


class TestRunCsvValidation(unittest.TestCase):
    def test_missing_file_is_an_error(self):
        errors = vr.validate_run_csv("/nonexistent/path/run.csv")
        self.assertTrue(any("not found" in e for e in errors))

    def test_valid_csv_has_no_errors(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
            f.write(",".join(vr.REQUIRED_CSV_COLUMNS) + "\n")
            f.write(",".join(["0"] * len(vr.REQUIRED_CSV_COLUMNS)) + "\n")
            path = f.name
        try:
            self.assertEqual(vr.validate_run_csv(path), [])
        finally:
            os.unlink(path)

    def test_empty_csv_is_an_error(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
            path = f.name
        try:
            errors = vr.validate_run_csv(path)
            self.assertTrue(any("empty" in e for e in errors))
        finally:
            os.unlink(path)

    def test_header_only_csv_is_an_error(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
            f.write(",".join(vr.REQUIRED_CSV_COLUMNS) + "\n")
            path = f.name
        try:
            errors = vr.validate_run_csv(path)
            self.assertTrue(any("no telemetry rows" in e for e in errors))
        finally:
            os.unlink(path)


class TestFindInvalidFloats(unittest.TestCase):
    def test_clean_structure_has_no_errors(self):
        self.assertEqual(vr.find_invalid_floats({"a": 1.0, "b": [1, 2, 3.5]}), [])

    def test_finds_nan_anywhere_in_nested_structure(self):
        errors = vr.find_invalid_floats({"a": {"b": [1.0, float("nan")]}})
        self.assertEqual(len(errors), 1)

    def test_finds_infinity(self):
        errors = vr.find_invalid_floats([float("inf"), -float("inf")])
        self.assertEqual(len(errors), 2)


if __name__ == "__main__":
    unittest.main()
