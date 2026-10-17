"""experiment_supervisor controller.

Independent ground-truth measurement and pass/fail evaluation for the obstacle-braking
milestone. Runs as a Webots Supervisor process, deliberately decoupled from the car
controller's own self-reported values: position, velocity, and collision are all read
directly from the physics engine via the Supervisor API, not trusted from the car.

The car controller (controllers/autonomous_car/autonomous_car.py) broadcasts its own
per-step decision data (sensor reading, commanded speed, brake command, state) over an
Emitter/Receiver link purely so it can be recorded in results/run.csv alongside the
independent ground truth -- it is never used to decide pass/fail.
"""

import csv
import json
import math
import os
import platform
import sys
from datetime import datetime, timezone

# `controller` is provided by Webots itself and is only importable inside a running Webots
# controller process. Deferred into main() so this module (and its pure functions below) can
# still be imported for unit tests without Webots running.

# Must match the Box size of DEF OBSTACLE in worlds/obstacle_braking.wbt (2 4 2 -> half-extents).
OBSTACLE_HALF_EXTENTS = (1.0, 2.0, 1.0)
CONTACT_EPSILON = 0.05  # metres, numerical tolerance for "is this contact point on the obstacle"
STOPPED_SPEED_THRESHOLD = 0.05  # m/s
CONTROLLER_SILENCE_STEP_LIMIT = 100  # steps of silence after first packet == controller failure


def load_scenario_config(config_path=None):
    if config_path is None:
        config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "scenario.json"
        )
    with open(config_path, "r") as f:
        return json.load(f)


def point_in_obstacle_aabb(point, obstacle_center, half_extents, epsilon):
    for i in range(3):
        if abs(point[i] - obstacle_center[i]) > half_extents[i] + epsilon:
            return False
    return True


def sanitize_json(value):
    """Recursively strip NaN/Infinity (invalid in JSON) down to None."""
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
        return value
    if isinstance(value, dict):
        return {k: sanitize_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [sanitize_json(v) for v in value]
    return value


def main():
    from controller import Supervisor

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    config = load_scenario_config()
    results_dir = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", config["results_dir"]
    ))
    os.makedirs(results_dir, exist_ok=True)

    car_node = supervisor.getFromDef("CAR")
    obstacle_node = supervisor.getFromDef("OBSTACLE")
    if car_node is None or obstacle_node is None:
        print("FATAL: could not find DEF CAR / DEF OBSTACLE in the world", file=sys.stderr)
        return 1

    car_node.enableContactPointsTracking(timestep, True)

    receiver = supervisor.getDevice("telemetry receiver")
    receiver.enable(timestep)

    csv_path = os.path.join(results_dir, "run.csv")
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "step", "sim_time_s", "car_x_m", "car_y_m", "true_speed_mps", "controller_speed_mps",
        "sensor_distance_m", "true_distance_m", "closing_speed_mps", "ttc_s",
        "accel_command", "brake_command", "controller_state", "sensor_valid", "collision",
    ])

    max_duration_s = float(config["max_duration_s"])
    min_safe_stopping_distance = float(config["min_safe_stopping_distance_m"])
    car_start_x = float(config["car_start_position_m"])

    car_moved = False
    car_max_speed = 0.0
    car_stopped = False
    collision_occurred = False
    min_true_distance = math.inf
    min_sensor_distance = math.inf
    initial_true_distance = None
    braking_triggered = False
    emergency_braking_count = 0
    sensor_readings_valid_overall = True
    sensor_distance_history = []
    car_controller_responded = False
    steps_since_last_packet = 0
    last_true_speed = 0.0
    last_car_x = car_start_x
    step_count = 0
    latest_packet = None
    end_reason = None
    screenshot_captured = False
    screenshot_path = os.path.join(results_dir, "final_screenshot.png")

    while supervisor.step(timestep) != -1:
        step_count += 1
        sim_time = supervisor.getTime()

        car_pos = car_node.getPosition()
        car_vel = car_node.getVelocity()
        true_speed = math.sqrt(car_vel[0] ** 2 + car_vel[1] ** 2 + car_vel[2] ** 2)
        car_max_speed = max(car_max_speed, true_speed)
        last_true_speed = true_speed
        last_car_x = car_pos[0]
        if abs(car_pos[0] - car_start_x) > 0.05:
            car_moved = True

        obstacle_pos = obstacle_node.getPosition()
        obstacle_front_x = obstacle_pos[0] - OBSTACLE_HALF_EXTENTS[0]
        true_distance = obstacle_front_x - car_pos[0]
        if initial_true_distance is None:
            initial_true_distance = true_distance
        min_true_distance = min(min_true_distance, true_distance)

        for c in car_node.getContactPoints(True):
            if point_in_obstacle_aabb(c.getPoint(), obstacle_pos, OBSTACLE_HALF_EXTENTS, CONTACT_EPSILON):
                collision_occurred = True
                break

        steps_since_last_packet += 1
        while receiver.getQueueLength() > 0:
            try:
                latest_packet = json.loads(receiver.getString())
                car_controller_responded = True
                steps_since_last_packet = 0
            except (ValueError, TypeError):
                sensor_readings_valid_overall = False
            receiver.nextPacket()

        sensor_distance = controller_speed = ttc = None
        accel_command = brake_command = controller_state = sensor_valid = None
        if latest_packet is not None:
            sensor_distance = latest_packet.get("sensor_distance")
            controller_speed = latest_packet.get("controller_speed")
            ttc = latest_packet.get("ttc")
            accel_command = latest_packet.get("accel_command")
            brake_command = latest_packet.get("brake_command")
            controller_state = latest_packet.get("state")
            sensor_valid = latest_packet.get("sensor_valid")
            if sensor_distance is not None:
                min_sensor_distance = min(min_sensor_distance, sensor_distance)
                sensor_distance_history.append(sensor_distance)
            if sensor_valid is False:
                sensor_readings_valid_overall = False
            if controller_state in ("BRAKING_NORMAL", "BRAKING_EMERGENCY", "SENSOR_FAULT"):
                braking_triggered = True
            if controller_state == "BRAKING_EMERGENCY":
                emergency_braking_count += 1

        if not screenshot_captured and true_distance < 22.0:
            # Capture once the car is close to the obstacle (both clearly framed together),
            # using the world's static camera (worlds/obstacle_braking.wbt DEF VIEWPOINT),
            # centred on the car-to-obstacle zone. Silently skipped in --no-rendering runs,
            # where there is no rendered frame to export.
            try:
                supervisor.exportImage(screenshot_path, 100)
                screenshot_captured = True
            except Exception as exc:  # pragma: no cover - best-effort visual artifact only
                print(f"NOTE: could not export screenshot: {exc}", file=sys.stderr)
                screenshot_captured = True  # don't keep retrying every step

        csv_writer.writerow([
            step_count, round(sim_time, 3), round(car_pos[0], 4), round(car_pos[1], 4),
            round(true_speed, 4), controller_speed if controller_speed is not None else "",
            sensor_distance if sensor_distance is not None else "",
            round(true_distance, 4), round(true_speed, 4),
            ttc if ttc is not None else "",
            accel_command if accel_command is not None else "",
            brake_command if brake_command is not None else "",
            controller_state or "", sensor_valid if sensor_valid is not None else "",
            collision_occurred,
        ])

        if true_speed < STOPPED_SPEED_THRESHOLD and car_moved and sim_time > 2.0:
            car_stopped = True

        if collision_occurred:
            end_reason = "collision"
        elif car_stopped and step_count > 5:
            end_reason = "stopped"
        elif sim_time >= max_duration_s:
            end_reason = "timeout"
        elif car_controller_responded and steps_since_last_packet > CONTROLLER_SILENCE_STEP_LIMIT:
            end_reason = "controller_failure"

        if end_reason:
            break

    csv_file.close()

    failure_reasons = []
    moved_distance = abs(last_car_x - car_start_x)
    if moved_distance < 3.0:
        failure_reasons.append(f"car moved only {moved_distance:.2f} m (< 3 m required)")
    if car_max_speed < 2.0:
        failure_reasons.append(f"car max speed {car_max_speed:.2f} m/s (< 2 m/s required)")
    if not sensor_distance_history:
        failure_reasons.append("front sensor produced no valid readings")
    elif (max(sensor_distance_history) - min(sensor_distance_history)) < 1.0:
        failure_reasons.append("front sensor readings did not change meaningfully")
    if initial_true_distance is not None and (initial_true_distance - min_true_distance) < 5.0:
        failure_reasons.append("obstacle distance did not decrease meaningfully while approaching")
    if not braking_triggered:
        failure_reasons.append("automatic braking was never triggered")
    if not car_stopped:
        failure_reasons.append("car never came to a safe stop")
    if collision_occurred:
        failure_reasons.append("physical collision with the obstacle occurred")
    if last_true_speed >= 0.1:
        failure_reasons.append(f"final speed {last_true_speed:.3f} m/s (>= 0.1 m/s)")
    if min_true_distance < min_safe_stopping_distance:
        failure_reasons.append(
            f"minimum true distance {min_true_distance:.2f} m < configured safety threshold "
            f"{min_safe_stopping_distance:.2f} m"
        )
    if not car_controller_responded:
        failure_reasons.append("car controller never sent telemetry (controller did not start)")
    if end_reason == "timeout":
        failure_reasons.append("simulation timed out without reaching a stop")
    if end_reason == "controller_failure":
        failure_reasons.append("car controller stopped responding mid-run")
    if not sensor_readings_valid_overall:
        failure_reasons.append("at least one invalid sensor/telemetry reading was reported")

    passed = len(failure_reasons) == 0

    summary = {
        "run_id": f"run-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "webots_version": os.environ.get("WEBOTS_VERSION", "R2025a"),
        "python_version": platform.python_version(),
        "scenario_config": config,
        "simulation_completed": end_reason is not None,
        "end_reason": end_reason,
        "car_moved": car_moved,
        "car_stopped": car_stopped,
        "collision_occurred": collision_occurred,
        "sensor_readings_valid": sensor_readings_valid_overall,
        "braking_triggered": braking_triggered,
        "emergency_braking_count": emergency_braking_count,
        "minimum_sensor_distance_m": None if math.isinf(min_sensor_distance) else min_sensor_distance,
        "minimum_true_distance_m": None if math.isinf(min_true_distance) else min_true_distance,
        "maximum_speed_mps": car_max_speed,
        "total_simulation_time_s": supervisor.getTime(),
        "final_speed_mps": last_true_speed,
        "safe_stopping_distance_m": min_safe_stopping_distance,
        "pass": passed,
        "failure_reasons": failure_reasons,
    }

    with open(os.path.join(results_dir, "summary.json"), "w") as f:
        json.dump(sanitize_json(summary), f, indent=2)

    print(f"RESULT: {'PASS' if passed else 'FAIL'} (end_reason={end_reason})")
    for reason in failure_reasons:
        print(f"  - {reason}")

    supervisor.simulationQuit(0 if passed else 1)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
