"""Software-in-the-loop stress tests of the real controller in a declared 1D plant.

This is a kinematic model, not Webots physics, a vehicle test or certification.
The measured endpoint is collision/clearance until first stop or a 20 s horizon.
"""
import argparse
import csv
import hashlib
import importlib.util
import itertools
import json
import math
import platform
import random
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('braking_controller',ROOT/'controllers/autonomous_car/autonomous_car.py')
controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
CONDITIONS={'nominal':{},'noise':{'noise_sd':.5},'positive_bias':{'bias':5.},
            'dropout':{'dropout':.05},'latency':{'delay_steps':10},
            'low_friction':{'friction':.35},'combined':{'noise_sd':.5,'bias':2.,'delay_steps':10,'friction':.5}}

def simulate(speed,clearance,obstacle_speed,condition,seed,policy='ttc',dt=.032):
    if speed<=0 or clearance<=0 or dt<=0 or condition not in CONDITIONS or policy not in ('ttc','fixed_distance'):
        raise ValueError('Invalid scenario or policy')
    settings=CONDITIONS[condition];rng=random.Random(seed);v=speed;command=speed
    x=0.; obstacle=clearance; minimum=clearance; previous=None; history=[]; triggered=False
    collision=False; stopped=False; max_deceleration=0.; invalid=0
    for step in range(math.ceil(20/dt)):
        true_distance=obstacle-x;history.append(min(40.,true_distance))
        reading=history[max(0,len(history)-1-settings.get('delay_steps',0))]
        reading+=settings.get('bias',0.)+rng.gauss(0,settings.get('noise_sd',0.))
        if rng.random()<settings.get('dropout',0.):reading=float('nan')
        valid,distance=controller.validate_distance(reading,40.)
        invalid+=int(not valid)
        ttc=controller.compute_ttc(distance,v) if valid else None
        if policy=='ttc':
            state=controller.decide_state(valid,ttc,v,command,speed,3.,1.2,previous_state=previous)
            previous=state;brake=controller.brake_intensity_for_state(state)
            rate=6. if state in (controller.STATE_BRAKING_EMERGENCY,controller.STATE_SENSOR_FAULT) else 3.
            command=controller.ramp_towards(command,0. if brake else speed,rate if brake else 2.,dt)
        else:
            triggered=triggered or not valid or distance<=20.
            brake=1. if triggered else 0.;command=0. if triggered else speed
        triggered=triggered or brake>0
        # Ideal longitudinal plant: braking pedal maps to at most 6 m/s^2;
        # friction multiplies that bound. Otherwise track the commanded speed
        # with a bounded 2 m/s^2 accelerator. No drag, steering, tyres or lag.
        acceleration=-6.*settings.get('friction',1.)*brake if brake else max(-3.,min(2.,(command-v)/dt))
        next_v=max(0.,v+acceleration*dt);x+=(v+next_v)*.5*dt;obstacle+=obstacle_speed*dt
        v=next_v;minimum=min(minimum,obstacle-x);max_deceleration=max(max_deceleration,-acceleration)
        collision=minimum<=0
        stopped=v<.05
        if collision or stopped:break
    return {'policy':policy,'speed_mps':speed,'initial_clearance_m':clearance,'obstacle_speed_mps':obstacle_speed,
            'condition':condition,'seed':seed,'collision':collision,'stopped':stopped,'braking_triggered':triggered,
            'minimum_clearance_m':minimum,'final_speed_mps':v,'elapsed_s':(step+1)*dt,'invalid_readings':invalid,
            'max_model_deceleration_mps2':max_deceleration,'safe_stop':bool(stopped and not collision and minimum>=3.)}

def aggregate(rows):
    groups={}
    for policy,condition in itertools.product(('ttc','fixed_distance'),CONDITIONS):
        selected=[r for r in rows if r['policy']==policy and r['condition']==condition]
        groups[policy+'/'+condition]={'runs':len(selected),'collisions':sum(r['collision'] for r in selected),
             'safe_stops':sum(r['safe_stop'] for r in selected),'timeouts':sum(not r['stopped'] and not r['collision'] for r in selected),
             'minimum_clearance_m':min(r['minimum_clearance_m'] for r in selected)}
    return groups

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    if a.out.exists():raise ValueError('Use a new results directory')
    protocol={'speeds_mps':[5.,10.,15.],'initial_clearances_m':[20.,40.,80.],'obstacle_speeds_mps':[-2.,0.,2.],
              'conditions':CONDITIONS,'seeds_by_condition':{c:(list(range(100,110)) if c in ['noise','dropout','combined'] else [100]) for c in CONDITIONS},'policies':['ttc','fixed_distance'],
              'dt_s':.032,'horizon_s':20.,'safe_margin_m':3.,'initial_ego_speed':'already at target cruise speed',
              'fixed_distance_baseline':'full brake once observed clearance <=20m or a sensor fault; sticky',
              'limitations':'Declared 1D ideal plant. Controller TTC deliberately retains static-obstacle assumption (distance/ego speed). Moving obstacles are stress inputs, not a new supported autonomy claim. Collision measured until first stop or horizon, not indefinitely after stopping. Synthetic cases are correlated model scenarios, not independent real-world trials. No Webots or physical-vehicle outcome follows.'}
    a.out.mkdir(parents=True);(a.out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[simulate(v,d,ov,c,s,policy) for v,d,ov,c,policy in itertools.product(protocol['speeds_mps'],protocol['initial_clearances_m'],protocol['obstacle_speeds_mps'],CONDITIONS,protocol['policies']) for s in protocol['seeds_by_condition'][c]]
    with (a.out/'cases.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    result={'validation_level':'software-in-the-loop, 1D kinematic plant','cases':len(rows),'groups':aggregate(rows),
            'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),ROOT/'controllers/autonomous_car/autonomous_car.py']},
            'python':platform.python_version(),'limits':protocol['limitations']}
    (a.out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
