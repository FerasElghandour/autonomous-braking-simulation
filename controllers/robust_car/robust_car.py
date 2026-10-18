"""Sensor-only AEB policies for the cruise-then-hazard validation harness.

The supervisor's event timestamp schedules software sensor faults; neither policy
receives ground-truth distances, object speeds or future motion. Historical code
remains available in autonomous_car. The common frontend recognizes no-hit range.
"""
from collections import deque
import json, math, os, random, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'autonomous_car'))
import autonomous_car as old


def guarded_state(valid, age, distance, speed, previous, max_range, reaction=.35,
                  design_deceleration=3., margin=2.):
    """Conservative static-target stopping envelope; constants are design assumptions."""
    if not valid or age > .160001 or previous == old.STATE_SENSOR_FAULT:
        return old.STATE_SENSOR_FAULT
    if previous in (old.STATE_BRAKING_EMERGENCY,old.STATE_BRAKING_NORMAL):
        return old.STATE_BRAKING_EMERGENCY
    if distance < max_range-.001 and distance <= margin + speed*reaction + speed*speed/(2*design_deceleration):
        return old.STATE_BRAKING_EMERGENCY
    return old.STATE_CRUISING


def delayed_sample(history, now, delay):
    eligible=[row for row in history if row[0] <= now-delay+1e-9]
    return eligible[-1] if eligible else history[0]


def main():
    from vehicle import Driver
    config=old.load_scenario_config();driver=Driver();dt=config['time_step_ms']/1000.
    front=driver.getDevice('distance sensor front');front.enable(config['time_step_ms'])
    emit=driver.getDevice('telemetry emitter');recv=driver.getDevice('scenario receiver');recv.enable(config['time_step_ms'])
    rng=random.Random(config['random_seed']+5000)
    max_range=config['sensor_max_range_m'];target=config['target_speed_mps']
    history=deque(maxlen=100);event=None;previous=None;commanded=0.;valid_count=0;ready=False;step=0
    while driver.step()!=-1:
        step+=1;now=driver.getTime()
        while recv.getQueueLength():
            event=json.loads(recv.getString())['trigger_time'];recv.nextPacket()
        raw=float(front.getValue());history.append((now,raw))
        elapsed=None if event is None else now-event
        delay=config.get('sensor_delay_s',0.) if event is not None else 0.
        measured_at,distance=delayed_sample(history,now,delay)
        if event is not None and config.get('sensor_noise_std_m',0.):
            distance+=rng.gauss(0.,config['sensor_noise_std_m'])
        dropout=(elapsed is not None and config.get('dropout_start_s',1e9)<=elapsed<config.get('dropout_end_s',-1.))
        if dropout:distance=float('nan')
        valid,distance=old.validate_distance(distance,max_range)
        speed_valid,speed=old.validate_speed(old.kmh_to_mps(driver.getCurrentSpeed()))
        valid=valid and speed_valid
        if not ready:
            valid_count=valid_count+1 if valid else 0;ready=valid_count>=3
        age=now-measured_at
        no_hit=valid and distance>=max_range-.001
        ttc=math.inf if no_hit else old.compute_ttc(distance,speed)
        if not ready:
            state=old.STATE_INITIALIZING if now<=1. else old.STATE_SENSOR_FAULT
        elif config['policy']=='legacy_ttc':
            state=old.decide_state(valid,ttc,speed,commanded,target,3.,1.2,previous)
        else:
            state=guarded_state(valid,age,distance,speed,previous,max_range)
        previous=state
        if state in (old.STATE_INITIALIZING,old.STATE_SENSOR_FAULT):commanded=0.
        elif state==old.STATE_BRAKING_EMERGENCY:commanded=old.ramp_towards(commanded,0.,6.,dt)
        elif state==old.STATE_BRAKING_NORMAL:commanded=old.ramp_towards(commanded,0.,3.,dt)
        else:commanded=old.ramp_towards(commanded,target,2.,dt)
        brake=old.brake_intensity_for_state(state)
        driver.setCruisingSpeed(old.mps_to_kmh(commanded));driver.setBrakeIntensity(brake);driver.setSteeringAngle(0.)
        emit.send(json.dumps({'step':step,'t':now,'raw_distance_m':raw,'sensor_distance_m':distance if valid else None,
                             'sensor_valid':valid,'measurement_time_s':measured_at,'measurement_age_s':age,
                             'controller_speed_mps':speed,'commanded_speed_mps':commanded,'brake_command':brake,
                             'state':state,'no_hit':no_hit,'dropout_injected':dropout}))
    return 0

if __name__=='__main__':sys.exit(main())
