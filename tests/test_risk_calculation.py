"""Unit tests for the pure risk-calculation functions in autonomous_car.py.

These import the controller module directly and never touch Webots -- the module's
Webots-dependent code is confined to main(), which is not executed on import.
"""

import math
import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "autonomous_car")
)

import autonomous_car as ac  # noqa: E402


class TestComputeTTC(unittest.TestCase):
    def test_stationary_obstacle_stationary_car(self):
        # closing_speed 0 means we are not approaching -- no collision risk from this alone.
        self.assertEqual(ac.compute_ttc(distance=10.0, closing_speed=0.0), math.inf)

    def test_zero_vehicle_speed(self):
        self.assertEqual(ac.compute_ttc(distance=25.0, closing_speed=0.0), math.inf)

    def test_negative_closing_speed_is_not_approaching(self):
        self.assertEqual(ac.compute_ttc(distance=25.0, closing_speed=-1.0), math.inf)

    def test_normal_approach(self):
        self.assertAlmostEqual(ac.compute_ttc(distance=20.0, closing_speed=10.0), 2.0)

    def test_invalid_distance_nan(self):
        self.assertIsNone(ac.compute_ttc(distance=float("nan"), closing_speed=5.0))

    def test_invalid_distance_inf(self):
        self.assertIsNone(ac.compute_ttc(distance=float("inf"), closing_speed=5.0))

    def test_invalid_distance_negative(self):
        self.assertIsNone(ac.compute_ttc(distance=-1.0, closing_speed=5.0))

    def test_invalid_distance_none(self):
        self.assertIsNone(ac.compute_ttc(distance=None, closing_speed=5.0))

    def test_zero_distance_is_valid_edge_case_immediate_collision(self):
        # distance itself isn't rejected by compute_ttc (that's validate_distance's job);
        # 0 distance with positive closing speed means TTC == 0 (already touching).
        self.assertEqual(ac.compute_ttc(distance=0.0, closing_speed=5.0), 0.0)


class TestValidateDistance(unittest.TestCase):
    def test_valid_reading(self):
        valid, d = ac.validate_distance(15.0, max_range=40.0)
        self.assertTrue(valid)
        self.assertEqual(d, 15.0)

    def test_nan_is_invalid(self):
        valid, _ = ac.validate_distance(float("nan"), max_range=40.0)
        self.assertFalse(valid)

    def test_inf_is_invalid(self):
        valid, _ = ac.validate_distance(float("inf"), max_range=40.0)
        self.assertFalse(valid)

    def test_zero_is_invalid(self):
        valid, _ = ac.validate_distance(0.0, max_range=40.0)
        self.assertFalse(valid)

    def test_negative_is_invalid(self):
        valid, _ = ac.validate_distance(-5.0, max_range=40.0)
        self.assertFalse(valid)

    def test_none_is_invalid(self):
        valid, _ = ac.validate_distance(None, max_range=40.0)
        self.assertFalse(valid)

    def test_clamped_to_max_range(self):
        valid, d = ac.validate_distance(1000.0, max_range=40.0)
        self.assertTrue(valid)
        self.assertEqual(d, 40.0)


class TestValidateSpeed(unittest.TestCase):
    def test_valid_speed(self):
        valid, s = ac.validate_speed(8.5)
        self.assertTrue(valid)
        self.assertEqual(s, 8.5)

    def test_nan_speed_invalid(self):
        valid, s = ac.validate_speed(float("nan"))
        self.assertFalse(valid)
        self.assertEqual(s, 0.0)

    def test_inf_speed_invalid(self):
        valid, s = ac.validate_speed(float("inf"))
        self.assertFalse(valid)

    def test_negative_speed_clamped_not_invalid(self):
        # A small negative reading (e.g. numerical noise near 0) is clamped, not rejected.
        valid, s = ac.validate_speed(-0.01)
        self.assertTrue(valid)
        self.assertEqual(s, 0.0)


class TestRampTowards(unittest.TestCase):
    def test_ramp_up_limited_by_rate(self):
        result = ac.ramp_towards(current=0.0, target=10.0, rate=2.0, dt=1.0)
        self.assertAlmostEqual(result, 2.0)

    def test_ramp_up_does_not_overshoot(self):
        result = ac.ramp_towards(current=9.0, target=10.0, rate=2.0, dt=1.0)
        self.assertAlmostEqual(result, 10.0)

    def test_ramp_down_limited_by_rate(self):
        result = ac.ramp_towards(current=10.0, target=0.0, rate=3.0, dt=1.0)
        self.assertAlmostEqual(result, 7.0)

    def test_ramp_down_does_not_undershoot(self):
        result = ac.ramp_towards(current=1.0, target=0.0, rate=3.0, dt=1.0)
        self.assertAlmostEqual(result, 0.0)

    def test_already_at_target(self):
        self.assertEqual(ac.ramp_towards(current=5.0, target=5.0, rate=2.0, dt=1.0), 5.0)


class TestDecideState(unittest.TestCase):
    def test_sensor_fault_while_moving_overrides_everything(self):
        state = ac.decide_state(
            sensor_valid=False, ttc=None, current_speed=5.0, commanded_speed=5.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
        )
        self.assertEqual(state, ac.STATE_SENSOR_FAULT)

    def test_sensor_invalid_while_stationary_prevents_acceleration(self):
        state = ac.decide_state(
            sensor_valid=False, ttc=None, current_speed=0.0, commanded_speed=0.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
        )
        self.assertEqual(state, ac.STATE_SENSOR_FAULT)

    def test_emergency_braking_below_threshold(self):
        state = ac.decide_state(
            sensor_valid=True, ttc=1.0, current_speed=10.0, commanded_speed=10.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
        )
        self.assertEqual(state, ac.STATE_BRAKING_EMERGENCY)

    def test_normal_braking_below_threshold(self):
        state = ac.decide_state(
            sensor_valid=True, ttc=2.5, current_speed=10.0, commanded_speed=10.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
        )
        self.assertEqual(state, ac.STATE_BRAKING_NORMAL)

    def test_accelerating_below_target(self):
        state = ac.decide_state(
            sensor_valid=True, ttc=math.inf, current_speed=2.0, commanded_speed=2.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
        )
        self.assertEqual(state, ac.STATE_ACCELERATING)

    def test_cruising_at_target(self):
        state = ac.decide_state(
            sensor_valid=True, ttc=math.inf, current_speed=10.0, commanded_speed=10.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
        )
        self.assertEqual(state, ac.STATE_CRUISING)

    def test_braking_is_sticky_even_once_ttc_recovers(self):
        # Once braking has started, TTC recovering above the threshold (because braking
        # itself increases TTC) must not flip the state back to ACCELERATING -- otherwise
        # the controller oscillates every step and the car never actually slows down.
        state = ac.decide_state(
            sensor_valid=True, ttc=8.0, current_speed=5.0, commanded_speed=5.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
            previous_state=ac.STATE_BRAKING_NORMAL,
        )
        self.assertEqual(state, ac.STATE_BRAKING_NORMAL)

    def test_sticky_braking_still_escalates_to_emergency(self):
        state = ac.decide_state(
            sensor_valid=True, ttc=1.0, current_speed=5.0, commanded_speed=5.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
            previous_state=ac.STATE_BRAKING_NORMAL,
        )
        self.assertEqual(state, ac.STATE_BRAKING_EMERGENCY)

    def test_no_sticky_braking_when_not_previously_braking(self):
        # High TTC and no prior braking -- should not spuriously enter a braking state.
        state = ac.decide_state(
            sensor_valid=True, ttc=8.0, current_speed=10.0, commanded_speed=10.0,
            target_speed=10.0, ttc_normal_threshold=3.0, ttc_emergency_threshold=1.2,
            previous_state=ac.STATE_CRUISING,
        )
        self.assertEqual(state, ac.STATE_CRUISING)


class TestSpeedUnitConversion(unittest.TestCase):
    def test_mps_to_kmh(self):
        self.assertAlmostEqual(ac.mps_to_kmh(10.0), 36.0)

    def test_kmh_to_mps(self):
        self.assertAlmostEqual(ac.kmh_to_mps(36.0), 10.0)

    def test_round_trip(self):
        self.assertAlmostEqual(ac.kmh_to_mps(ac.mps_to_kmh(7.3)), 7.3)


class TestBrakeIntensityForState(unittest.TestCase):
    def test_emergency_is_full_brake(self):
        self.assertEqual(ac.brake_intensity_for_state(ac.STATE_BRAKING_EMERGENCY), 1.0)

    def test_sensor_fault_is_full_brake(self):
        self.assertEqual(ac.brake_intensity_for_state(ac.STATE_SENSOR_FAULT), 1.0)

    def test_normal_is_partial_brake(self):
        self.assertGreater(ac.brake_intensity_for_state(ac.STATE_BRAKING_NORMAL), 0.0)
        self.assertLess(ac.brake_intensity_for_state(ac.STATE_BRAKING_NORMAL), 1.0)

    def test_cruising_is_no_brake(self):
        self.assertEqual(ac.brake_intensity_for_state(ac.STATE_CRUISING), 0.0)


if __name__ == "__main__":
    unittest.main()
