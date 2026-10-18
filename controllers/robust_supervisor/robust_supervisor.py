"""Independently measure at-speed hazard response, geometry, contact and stop.

Targets are scripted kinematic boxes, not a validated traffic or vehicle model.
Vehicle motion, speed, orientation and contact are measured through Webots APIs.
"""
import csv,itertools,json,math,os,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiment_supervisor'))
from experiment_supervisor import point_in_obstacle_aabb


def axis_angle_matrix(axis_angle):
    x,y,z,a=axis_angle;c=math.cos(a);s=math.sin(a);d=1-c
    return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s],
            [y*x*d+z*s,c+y*y*d,y*z*d-x*s],
            [z*x*d-y*s,z*y*d+x*s,c+z*z*d]]


def hull_corners():
    # Pinned R2025a LincolnMKZ collision geometry, with upstream asset terms intact.
    specs=[((1.44,0.,.3),(.0199987,0.,.9998,3.14159),(4.9,1.8,.8)),
           ((1.44,0.,.85),(0.,0.,1.,3.14159),(2.,1.6,.5))]
    points=[]
    for pos,rotation,size in specs:
        norm=math.sqrt(sum(x*x for x in rotation[:3]));r=axis_angle_matrix([*(x/norm for x in rotation[:3]),rotation[3]])
        for signs in itertools.product([-1,1],repeat=3):
            local=[sgn*dim/2 for sgn,dim in zip(signs,size)]
            points.append(tuple(pos[i]+sum(r[i][j]*local[j] for j in range(3)) for i in range(3)))
    return points


HULL=hull_corners()


def forward_extent(orientation):
    return max(sum(orientation[j]*p[j] for j in range(3)) for p in HULL)


def lead_displacement(t,speed,deceleration,brake_after):
    before=min(t,brake_after)*speed
    after=max(0.,t-brake_after)
    if deceleration<=0:return speed*t
    duration=min(after,speed/deceleration)
    return before+speed*duration-.5*deceleration*duration*duration


def outcome(rows,trigger,target):
    active=[r for r in rows if r['active']]
    errors=[]
    if trigger is None:errors.append('cruise condition was not reached')
    if not active:errors.append('hazard phase not executed')
    if active and not .95*target<=active[0]['true_speed_mps']<=1.05*target:errors.append('trigger speed outside frozen tolerance')
    if any(r['contact'] for r in active):errors.append('obstacle contact')
    minimum=min((r['hull_clearance_m'] for r in active),default=None)
    if minimum is not None and minimum<1.:errors.append('collision-hull clearance below 1 m')
    if not rows or rows[-1]['true_speed_mps']>=.1:errors.append('did not stop below 0.1 m/s')
    if active and not any(r['brake_command'] is not None and r['brake_command']>0 for r in active):errors.append('no recorded braking command')
    return {'pass':not errors,'failure_reasons':errors,'minimum_hull_clearance_m':minimum,
            'minimum_reference_distance_m':min((r['reference_distance_m'] for r in active),default=None),
            'trigger_speed_mps':active[0]['true_speed_mps'] if active else None,
            'peak_speed_mps':max((r['true_speed_mps'] for r in rows),default=None),
            'final_speed_mps':rows[-1]['true_speed_mps'] if rows else None,
            'contact':any(r['contact'] for r in active)}


def main():
    from controller import Supervisor
    sup=Supervisor();config=json.loads(Path(os.environ['BRAKING_SCENARIO_CONFIG']).read_text())
    dt=config['time_step_ms']/1000.;car=sup.getFromDef('CAR');obstacle=sup.getFromDef('OBSTACLE')
    car.enableContactPointsTracking(config['time_step_ms'],True)
    recv=sup.getDevice('telemetry receiver');recv.enable(config['time_step_ms']);emitter=sup.getDevice('scenario emitter')
    out=Path(__file__).resolve().parents[2]/'results';out.mkdir(exist_ok=True)
    rows=[];trigger=None;obstacle_origin=None;stable=0;latest={};stopped_count=0;last_packet_time=0.;reason='timeout'
    while sup.step(config['time_step_ms'])!=-1:
        now=sup.getTime();pos=car.getPosition();velocity=car.getVelocity();speed=math.sqrt(sum(v*v for v in velocity[:3]))
        while recv.getQueueLength():
            latest=json.loads(recv.getString());last_packet_time=now;recv.nextPacket()
        if trigger is None:
            stable=stable+1 if .95*config['target_speed_mps']<=speed<=1.05*config['target_speed_mps'] else 0
            if stable>=20:
                trigger=now;obstacle_origin=pos[0]+config['hazard_reference_gap_m']+1.
                obstacle.getField('translation').setSFVec3f([obstacle_origin,0.,1.])
                emitter.send(json.dumps({'trigger_time':trigger}))
        if trigger is not None:
            elapsed=now-trigger
            x=obstacle_origin+lead_displacement(elapsed,config.get('lead_speed_mps',0.),config.get('lead_deceleration_mps2',0.),config.get('lead_brake_after_s',0.))
            obstacle.getField('translation').setSFVec3f([x,0.,1.])
        obstacle_pos=obstacle.getPosition();reference=obstacle_pos[0]-1.-pos[0]
        contact=any(point_in_obstacle_aabb(c.getPoint(),obstacle_pos,(1.,2.,1.),.05) for c in car.getContactPoints(True))
        orientation=car.getOrientation()
        row={'step':len(rows)+1,'sim_time_s':now,'active':trigger is not None,'car_x_m':pos[0],'car_y_m':pos[1],
             'obstacle_x_m':obstacle_pos[0],'true_speed_mps':speed,'reference_distance_m':reference,
             'hull_forward_extent_m':forward_extent(orientation),
             'hull_clearance_m':reference-forward_extent(orientation),'contact':contact,
             'packet_time_s':latest.get('t'),'state':latest.get('state'),'sensor_distance_m':latest.get('sensor_distance_m'),
             'sensor_valid':latest.get('sensor_valid'),'measurement_age_s':latest.get('measurement_age_s'),
             'brake_command':latest.get('brake_command'),'commanded_speed_mps':latest.get('commanded_speed_mps'),
             'dropout_injected':latest.get('dropout_injected'),
             **{f'orientation_{i}':v for i,v in enumerate(orientation)}}
        rows.append(row)
        stopped_count=stopped_count+1 if trigger is not None and speed<.05 else 0
        if contact:reason='contact';break
        if stopped_count>=10:reason='stopped';break
        if now-last_packet_time>1.:reason='controller_silence';break
        if trigger is None and now>=35.:reason='warmup_timeout';break
        if trigger is not None and now-trigger>=20.:reason='hazard_timeout';break
    with (out/'run.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    result={'config':config,'trigger_time_s':trigger,'end_reason':reason,'rows':len(rows),**outcome(rows,trigger,config['target_speed_mps'])}
    if reason not in ['contact','stopped']:
        result['pass']=False;result['failure_reasons'].append(reason)
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result),flush=True);sup.simulationQuit(0)

if __name__=='__main__':main()
