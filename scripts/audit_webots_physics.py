"""Recompute the recorded Webots matrix checks from compressed per-step telemetry."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import zipfile

DEFAULT = Path(__file__).resolve().parents[1] / 'results/webots-physics-v1'


def audit_case(summary, rows, startup_hold=False):
    errors = []
    if not rows:
        return ['empty telemetry']
    config = summary['scenario_config']
    start = config['car_start_position_m']
    front = config['obstacle_position_m'] - 1.0
    distances = [float(row['true_distance_m']) for row in rows]
    speeds = [float(row['true_speed_mps']) for row in rows]
    positions = [float(row['car_x_m']) for row in rows]
    times = [float(row['sim_time_s']) for row in rows]
    if not all(math.isfinite(v) for series in (distances, speeds, positions, times) for v in series):
        errors.append('non-finite physical telemetry')
    for field, observed in [('minimum_true_distance_m', min(distances)),
                            ('maximum_speed_mps', max(speeds)),
                            ('final_speed_mps', speeds[-1])]:
        if abs(summary[field] - observed) > 0.000051:
            errors.append(field + ' disagrees with telemetry')
    if abs(summary['total_simulation_time_s'] - times[-1]) > 0.00051:
        errors.append('duration disagrees with telemetry')
    for index, row in enumerate(rows, 1):
        if int(row['step']) != index or abs(float(row['sim_time_s']) - index * .032) > .00051:
            errors.append('step/time sequence mismatch')
            break
        if abs(front - float(row['car_x_m']) - float(row['true_distance_m'])) > .00011:
            errors.append('distance does not match world geometry')
            break
    collision = any(row['collision'] == 'True' for row in rows)
    if collision != summary['collision_occurred']:
        errors.append('collision summary disagrees with telemetry')
    moved = any(abs(x - start) > .05 for x in positions)
    if moved != summary['car_moved']:
        errors.append('movement summary disagrees with telemetry')
    valid = True
    initializing_invalid = 0
    valid_streak = 0
    started = False
    for row in rows:
        if not row['controller_state']:
            continue  # first supervisor step can precede the first received packet
        invalid = row['sensor_valid'] == 'False' or row.get('speed_valid') == 'False'
        hold = (startup_hold and row['controller_state'] == 'INITIALIZING'
                and row.get('initialization_complete') == 'False'
                and float(row['sim_time_s']) <= 1.032
                and abs(float(row['car_x_m']) - start) < .05
                and float(row['accel_command']) == 0.
                and float(row['brake_command']) == 1.)
        if startup_hold:
            valid_streak = 0 if invalid else valid_streak + 1
            if not started and float(row['accel_command']) > 0:
                if valid_streak < 3 or row.get('initialization_complete') != 'True':
                    errors.append('acceleration before three valid startup samples')
                started = True
            if row['controller_state'] == 'INITIALIZING' and not hold:
                errors.append('initialization is not a bounded full-brake hold')
        if invalid:
            if hold:
                initializing_invalid += 1
            else:
                valid = False
    # Legacy CSV did not contain speed_valid. Its summary retained the startup fault;
    # the exact old controller is archived, and all nine legacy runs failed to move.
    if startup_hold:
        if valid != summary['sensor_readings_valid']:
            errors.append('validity summary disagrees with telemetry')
        if initializing_invalid != summary['initialization_invalid_packets']:
            errors.append('initialization count disagrees with telemetry')
    sensor = [float(row['sensor_distance_m']) for row in rows if row['sensor_distance_m']]
    braking = any(row['controller_state'] in ('BRAKING_NORMAL', 'BRAKING_EMERGENCY', 'SENSOR_FAULT') for row in rows)
    physical_pass = (abs(positions[-1] - start) >= 3 and max(speeds) >= 2
                     and bool(sensor) and max(sensor) - min(sensor) >= 1
                     and distances[0] - min(distances) >= 5 and braking
                     and moved and times[-1] > 2 and speeds[-1] < .05 and not collision
                     and min(distances) >= config['min_safe_stopping_distance_m']
                     and summary['end_reason'] == 'stopped'
                     and (valid if startup_hold else summary['sensor_readings_valid']))
    if physical_pass != summary['pass']:
        errors.append('pass verdict disagrees with reconstructed acceptance checks')
    return errors


def audit(root=DEFAULT):
    root = Path(root)
    errors, cases = [], []
    digests = json.loads((root / 'artifacts.sha256.json').read_text())
    for path, expected in digests.items():
        if hashlib.sha256((root / path).read_bytes()).hexdigest() != expected:
            errors.append('artifact hash mismatch: ' + path)
    for phase in ('before', 'after', 'repeat'):
        matrix = json.loads((root / f'matrix-{phase}.json').read_text())
        with zipfile.ZipFile(root / f'source-{phase}.zip') as archive:
            for name, expected in matrix['source_sha256'].items():
                if hashlib.sha256(archive.read(name)).hexdigest() != expected:
                    errors.append('executed source hash mismatch: ' + phase + '/' + name)
        for case in matrix['cases']:
            folder = root / phase / case['name']
            summary = json.loads((folder / 'summary.json').read_text())
            with gzip.open(folder / 'run.csv.gz', 'rt', newline='') as stream:
                rows = list(csv.DictReader(stream))
            local_errors = audit_case(summary, rows, startup_hold=phase != 'before')
            if summary != case['summary']:
                local_errors.append('matrix and standalone summary disagree')
            if case['status'] != 'completed' or case['exit_code'] != (0 if summary['pass'] else 1):
                local_errors.append('execution status/exit code mismatch')
            errors.extend(f'{phase}/{case["name"]}: {error}' for error in local_errors)
            cases.append({'phase': phase, 'name': case['name'], 'rows': len(rows),
                          'pass': summary['pass'], 'collision': summary['collision_occurred'],
                          'maximum_speed_mps': summary['maximum_speed_mps'],
                          'minimum_reference_distance_m': summary['minimum_true_distance_m'],
                          'final_speed_mps': summary['final_speed_mps'], 'errors': local_errors})
    return {'cases': cases, 'telemetry_rows': sum(c['rows'] for c in cases), 'errors': errors}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    args = parser.parse_args()
    report = audit(args.root)
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(bool(report['errors']))
