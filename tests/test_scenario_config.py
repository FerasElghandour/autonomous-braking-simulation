"""Unit tests for config/scenario.json: required fields present, positive safety thresholds."""

import json
import os
import unittest

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "scenario.json"
)

REQUIRED_FIELDS = [
    "time_step_ms",
    "max_duration_s",
    "target_speed_mps",
    "acceleration_mps2",
    "normal_brake_mps2",
    "emergency_brake_mps2",
    "ttc_normal_threshold_s",
    "ttc_emergency_threshold_s",
    "min_safe_stopping_distance_m",
    "car_start_position_m",
    "obstacle_position_m",
    "sensor_max_range_m",
    "random_seed",
    "results_dir",
    "simulation_mode",
]

POSITIVE_FIELDS = [
    "time_step_ms",
    "max_duration_s",
    "target_speed_mps",
    "acceleration_mps2",
    "normal_brake_mps2",
    "emergency_brake_mps2",
    "ttc_normal_threshold_s",
    "ttc_emergency_threshold_s",
    "min_safe_stopping_distance_m",
    "sensor_max_range_m",
]


class TestScenarioConfig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(CONFIG_PATH) as f:
            cls.config = json.load(f)

    def test_all_required_fields_present(self):
        for field in REQUIRED_FIELDS:
            self.assertIn(field, self.config, f"scenario.json missing field: {field}")

    def test_safety_thresholds_are_positive(self):
        for field in POSITIVE_FIELDS:
            self.assertGreater(self.config[field], 0, f"{field} must be > 0")

    def test_emergency_brake_stronger_than_normal_brake(self):
        self.assertGreater(self.config["emergency_brake_mps2"], self.config["normal_brake_mps2"])

    def test_emergency_ttc_threshold_tighter_than_normal(self):
        self.assertLess(
            self.config["ttc_emergency_threshold_s"], self.config["ttc_normal_threshold_s"]
        )

    def test_obstacle_is_far_enough_from_car_start(self):
        gap = self.config["obstacle_position_m"] - self.config["car_start_position_m"]
        self.assertGreaterEqual(
            gap, 20.0, "obstacle must start far enough away for a non-trivial approach/braking test"
        )

    def test_sensor_range_can_see_braking_trigger_distance(self):
        # The car must be able to detect the obstacle before TTC drops below the normal
        # braking threshold, otherwise braking could never start in time.
        approx_trigger_distance = (
            self.config["target_speed_mps"] * self.config["ttc_normal_threshold_s"]
        )
        self.assertGreaterEqual(self.config["sensor_max_range_m"], approx_trigger_distance)

    def test_results_dir_is_a_relative_path_name(self):
        self.assertFalse(os.path.isabs(self.config["results_dir"]))

    def test_simulation_mode_is_a_known_value(self):
        self.assertIn(self.config["simulation_mode"], ("fast", "real-time", "run"))


if __name__ == "__main__":
    unittest.main()
