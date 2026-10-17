"""autonomous_car controller.

Environment -> sensor reading -> risk calculation -> driving decision -> physical response.

The pure functions below (compute_ttc, validate_distance, validate_speed, ramp_towards,
decide_state) have no Webots dependency and are unit tested directly by
tests/test_risk_calculation.py. Everything that touches the `vehicle`/`controller`
Webots modules is confined to main(), which only runs when Webots launches this file
as a controller process (guarded by `if __name__ == "__main__"`), so importing this
module for tests never requires Webots to be running.
"""

import json
import math
import os
import sys

EPSILON = 1e-3
MPS_TO_KMH = 3.6  # Webots' vehicle.Driver speed API (setCruisingSpeed/getCurrentSpeed) uses
                  # km/h, confirmed from Webots' own bundled autonomous_vehicle.c ("speed_ms *
                  # 3.6 // convert from m/s to km/h" before calling the equivalent C API). All
                  # of this project's own logic stays in SI units (m/s); conversion happens only
                  # at the two points where we call into the Driver API.


def mps_to_kmh(speed_mps):
    return speed_mps * MPS_TO_KMH


def kmh_to_mps(speed_kmh):
    return speed_kmh / MPS_TO_KMH

STATE_ACCELERATING = "ACCELERATING"
STATE_CRUISING = "CRUISING"
STATE_BRAKING_NORMAL = "BRAKING_NORMAL"
STATE_BRAKING_EMERGENCY = "BRAKING_EMERGENCY"
STATE_SENSOR_FAULT = "SENSOR_FAULT"
STATE_STOPPED = "STOPPED"


def load_scenario_config(config_path=None):
    """Load config/scenario.json. Pure I/O, no Webots dependency."""
    if config_path is None:
        config_path = os.environ.get('BRAKING_SCENARIO_CONFIG') or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "scenario.json"
        )
    with open(config_path, "r") as f:
        return json.load(f)


def validate_distance(raw_distance, max_range):
    """Return (is_valid, sanitized_distance). Rejects NaN, +/-inf, and non-positive values."""
    if raw_distance is None:
        return False, max_range
    try:
        d = float(raw_distance)
    except (TypeError, ValueError):
        return False, max_range
    if math.isnan(d) or math.isinf(d):
        return False, max_range
    if d <= 0.0:
        return False, max_range
    return True, min(d, max_range)


def validate_speed(raw_speed):
    """Return (is_valid, sanitized_speed). Rejects NaN and +/-inf. Negative speed clamps to 0."""
    if raw_speed is None:
        return False, 0.0
    try:
        s = float(raw_speed)
    except (TypeError, ValueError):
        return False, 0.0
    if math.isnan(s) or math.isinf(s):
        return False, 0.0
    return True, max(s, 0.0)


def compute_ttc(distance, closing_speed):
    """Pure time-to-collision calculation.

    Returns:
        float seconds until collision, or math.inf if not closing (closing_speed <= 0),
        or None if the distance reading is invalid (caller should fail-safe).
    """
    if distance is None or math.isnan(distance) or math.isinf(distance) or distance < 0:
        return None
    if closing_speed is None or closing_speed <= 0.0:
        return math.inf
    return distance / closing_speed


def ramp_towards(current, target, rate, dt):
    """Move `current` towards `target` by at most rate*dt, never overshooting, never negative."""
    if current < target:
        return min(current + rate * dt, target)
    if current > target:
        return max(current - rate * dt, target)
    return current


def decide_state(sensor_valid, ttc, current_speed, commanded_speed, target_speed,
                  ttc_normal_threshold, ttc_emergency_threshold, previous_state=None):
    """Decide the controller state for this step from validated inputs. Pure function.

    Braking is "sticky" once started: braking itself increases TTC (closing speed drops
    faster than distance closes), so checking the raw TTC threshold fresh every step would
    flip the state back to ACCELERATING within a step or two of every brake pulse, then back
    to braking the step after -- an oscillation that never lets the car actually slow down.
    `previous_state` lets the controller keep decelerating toward a full stop once it has
    started responding to an obstacle, rather than re-accelerating toward it. This is
    correct for milestone 1's single static obstacle (it never becomes safe to speed back
    up towards something that isn't moving out of the way); a scenario with a
    disappearing/passing obstacle would need to release the latch on top of this.
    """
    if not sensor_valid:
        return STATE_SENSOR_FAULT

    obstacle_response_active = previous_state in (
        STATE_BRAKING_NORMAL, STATE_BRAKING_EMERGENCY, STATE_SENSOR_FAULT,
    )

    if ttc is not None and ttc <= ttc_emergency_threshold:
        return STATE_BRAKING_EMERGENCY
    if obstacle_response_active:
        return STATE_BRAKING_NORMAL
    if ttc is not None and ttc <= ttc_normal_threshold:
        return STATE_BRAKING_NORMAL

    if commanded_speed < target_speed - EPSILON:
        return STATE_ACCELERATING
    return STATE_CRUISING


def brake_intensity_for_state(state):
    """Map a controller state to a brake pedal intensity in [0, 1]."""
    if state == STATE_BRAKING_EMERGENCY:
        return 1.0
    if state == STATE_SENSOR_FAULT:
        return 1.0
    if state == STATE_BRAKING_NORMAL:
        return 0.4
    return 0.0


def main():
    from controller import Emitter  # noqa: F401 (Emitter is fetched via driver.getDevice)
    from vehicle import Driver

    config = load_scenario_config()

    driver = Driver()
    timestep_ms = int(config["time_step_ms"])
    dt = timestep_ms / 1000.0

    front_sensor = driver.getDevice("distance sensor front")
    front_sensor.enable(timestep_ms)

    emitter = driver.getDevice("telemetry emitter")

    target_speed = float(config["target_speed_mps"])
    accel_rate = float(config["acceleration_mps2"])
    normal_brake_rate = float(config["normal_brake_mps2"])
    emergency_brake_rate = float(config["emergency_brake_mps2"])
    ttc_normal_threshold = float(config["ttc_normal_threshold_s"])
    ttc_emergency_threshold = float(config["ttc_emergency_threshold_s"])
    sensor_max_range = float(config["sensor_max_range_m"])

    commanded_speed = 0.0
    step_count = 0
    previous_state = None

    driver.setSteeringAngle(0.0)

    while driver.step() != -1:
        step_count += 1

        raw_distance = front_sensor.getValue()
        sensor_valid, distance = validate_distance(raw_distance, sensor_max_range)

        raw_speed_kmh = driver.getCurrentSpeed()
        speed_valid, current_speed = validate_speed(kmh_to_mps(raw_speed_kmh))

        closing_speed = current_speed  # obstacle is static: relative closing speed = our speed
        ttc = compute_ttc(distance, closing_speed) if sensor_valid else None

        state = decide_state(
            sensor_valid and speed_valid, ttc, current_speed, commanded_speed, target_speed,
            ttc_normal_threshold, ttc_emergency_threshold, previous_state=previous_state,
        )
        previous_state = state

        if state in (STATE_BRAKING_EMERGENCY, STATE_SENSOR_FAULT):
            commanded_speed = ramp_towards(commanded_speed, 0.0, emergency_brake_rate, dt)
        elif state == STATE_BRAKING_NORMAL:
            commanded_speed = ramp_towards(commanded_speed, 0.0, normal_brake_rate, dt)
        else:
            commanded_speed = ramp_towards(commanded_speed, target_speed, accel_rate, dt)

        commanded_speed = max(commanded_speed, 0.0)  # never reverse

        if commanded_speed < EPSILON and current_speed < EPSILON and state != STATE_SENSOR_FAULT:
            state = STATE_STOPPED

        brake_intensity = brake_intensity_for_state(state)

        driver.setCruisingSpeed(mps_to_kmh(commanded_speed))
        driver.setSteeringAngle(0.0)
        driver.setBrakeIntensity(brake_intensity)

        packet = {
            "step": step_count,
            "t": driver.getTime(),
            "sensor_distance": distance if sensor_valid else None,
            "sensor_valid": sensor_valid,
            "speed_valid": speed_valid,
            "controller_speed": current_speed,
            "commanded_speed": commanded_speed,
            "ttc": None if ttc is None or math.isinf(ttc) else ttc,
            "accel_command": commanded_speed,
            "brake_command": brake_intensity,
            "state": state,
        }
        if emitter is not None:
            emitter.send(json.dumps(packet))

    return 0


if __name__ == "__main__":
    sys.exit(main())
