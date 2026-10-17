#!/usr/bin/env python3
"""Validate results/summary.json and results/run.csv against the milestone-1 acceptance rules.

Prints a clear PASS/FAIL report. Exit code 0 on pass, non-zero on any failure.
This is an independent check: it re-derives the safety verdict from summary.json's own
fields rather than only trusting the "pass" boolean the supervisor wrote.
"""

import argparse
import csv
import json
import math
import os
import sys

REQUIRED_FIELDS = {
    "run_id": str,
    "timestamp": str,
    "webots_version": str,
    "python_version": str,
    "scenario_config": dict,
    "simulation_completed": bool,
    "car_moved": bool,
    "car_stopped": bool,
    "collision_occurred": bool,
    "sensor_readings_valid": bool,
    "braking_triggered": bool,
    "emergency_braking_count": int,
    "minimum_sensor_distance_m": (int, float, type(None)),
    "minimum_true_distance_m": (int, float, type(None)),
    "maximum_speed_mps": (int, float),
    "total_simulation_time_s": (int, float),
    "final_speed_mps": (int, float),
    "safe_stopping_distance_m": (int, float),
    "pass": bool,
    "failure_reasons": list,
}

REQUIRED_CSV_COLUMNS = [
    "step", "sim_time_s", "car_x_m", "true_speed_mps", "controller_speed_mps",
    "sensor_distance_m", "true_distance_m", "ttc_s", "controller_state", "collision",
]


def find_invalid_floats(obj, path="summary"):
    """Recursively scan for NaN/Infinity floats, which must never appear in valid JSON output."""
    errors = []
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            errors.append(f"{path}: invalid float value {obj!r} (NaN/Infinity not allowed in JSON)")
    elif isinstance(obj, dict):
        for k, v in obj.items():
            errors.extend(find_invalid_floats(v, f"{path}.{k}"))
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            errors.extend(find_invalid_floats(v, f"{path}[{i}]"))
    return errors


def validate_summary_schema(summary):
    errors = []
    for field, expected_type in REQUIRED_FIELDS.items():
        if field not in summary:
            errors.append(f"summary.json missing required field: {field}")
            continue
        value = summary[field]
        # bool is a subclass of int in Python; only accept bool where explicitly expected.
        if expected_type is bool:
            if not isinstance(value, bool):
                errors.append(f"summary.json field '{field}' should be bool, got {type(value).__name__}")
        elif isinstance(value, bool):
            errors.append(f"summary.json field '{field}' should be {expected_type}, got bool")
        elif not isinstance(value, expected_type):
            errors.append(f"summary.json field '{field}' should be {expected_type}, got {type(value).__name__}")
    errors.extend(find_invalid_floats(summary))
    return errors


def check_safety_conditions(summary):
    """Independently re-derive the pass/fail verdict from summary.json's own fields."""
    errors = []
    if not summary.get("car_moved"):
        errors.append("car did not move")
    if not summary.get("car_stopped"):
        errors.append("car did not come to a stop")
    if summary.get("collision_occurred"):
        errors.append("a collision occurred")
    if not summary.get("braking_triggered"):
        errors.append("automatic braking was never triggered")
    final_speed = summary.get("final_speed_mps")
    if final_speed is None or final_speed >= 0.1:
        errors.append(f"final speed {final_speed} m/s is not below 0.1 m/s")
    min_true_distance = summary.get("minimum_true_distance_m")
    safe_distance = summary.get("safe_stopping_distance_m")
    if min_true_distance is None or safe_distance is None or min_true_distance < safe_distance:
        errors.append(
            f"minimum true distance {min_true_distance} m did not meet safety threshold {safe_distance} m"
        )
    if not summary.get("sensor_readings_valid", False):
        errors.append("sensor readings were not valid throughout the run")
    if summary.get("end_reason") == "timeout":
        errors.append("simulation ended by timeout, not a controlled stop")
    if not summary.get("pass"):
        errors.append("summary.json itself reports pass=false")
    return errors


def validate_run_csv(csv_path):
    errors = []
    if not os.path.isfile(csv_path):
        return [f"run.csv not found at {csv_path}"]
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            return ["run.csv is empty (no header row)"]
        missing = [c for c in REQUIRED_CSV_COLUMNS if c not in header]
        if missing:
            errors.append(f"run.csv missing expected columns: {missing}")
        row_count = sum(1 for _ in reader)
        if row_count == 0:
            errors.append("run.csv has a header but no telemetry rows")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default_results_dir = os.path.abspath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
    )
    parser.add_argument("--summary", default=os.path.join(default_results_dir, "summary.json"))
    parser.add_argument("--csv", default=os.path.join(default_results_dir, "run.csv"))
    args = parser.parse_args()

    all_errors = []

    if not os.path.isfile(args.summary):
        print(f"FAIL: summary.json not found at {args.summary}")
        return 1

    with open(args.summary) as f:
        raw_text = f.read()
    try:
        summary = json.loads(raw_text)
    except json.JSONDecodeError as e:
        print(f"FAIL: summary.json is not valid JSON: {e}")
        return 1

    all_errors.extend(validate_summary_schema(summary))
    all_errors.extend(check_safety_conditions(summary))
    all_errors.extend(validate_run_csv(args.csv))

    print("=" * 60)
    if all_errors:
        print("RESULT: FAIL")
        print("=" * 60)
        for err in all_errors:
            print(f"  - {err}")
        return 1

    print("RESULT: PASS")
    print("=" * 60)
    print(f"  run_id:                 {summary.get('run_id')}")
    print(f"  car_moved:              {summary.get('car_moved')}")
    print(f"  car_stopped:            {summary.get('car_stopped')}")
    print(f"  collision_occurred:     {summary.get('collision_occurred')}")
    print(f"  braking_triggered:      {summary.get('braking_triggered')}")
    print(f"  emergency_braking_count:{summary.get('emergency_braking_count')}")
    print(f"  minimum_true_distance_m:{summary.get('minimum_true_distance_m')}")
    print(f"  final_speed_mps:        {summary.get('final_speed_mps')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
