"""Frozen at-speed Webots scenario family; preserve all failed cases and exact sources."""
import argparse,hashlib,itertools,json,os,platform,re,shutil,subprocess,time,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

CASES=[
    dict(name='stationary-10-gap30',target_speed_mps=10.,hazard_reference_gap_m=30.),
    dict(name='stationary-15-gap45',target_speed_mps=15.,hazard_reference_gap_m=45.),
    dict(name='low-friction-15-gap45',target_speed_mps=15.,hazard_reference_gap_m=45.,friction=.35),
    dict(name='short-gap-15-gap18',target_speed_mps=15.,hazard_reference_gap_m=18.),
    dict(name='stress-15-gap9-low-friction',target_speed_mps=15.,hazard_reference_gap_m=9.,friction=.35,stress=True),
    dict(name='moving-lead-15-gap30',target_speed_mps=15.,hazard_reference_gap_m=30.,lead_speed_mps=8.),
    dict(name='braking-lead-15-gap30',target_speed_mps=15.,hazard_reference_gap_m=30.,lead_speed_mps=12.,lead_deceleration_mps2=4.,lead_brake_after_s=1.),
    dict(name='sudden-appearance-10-gap22',target_speed_mps=10.,hazard_reference_gap_m=22.),
    dict(name='delayed-range-15-gap45',target_speed_mps=15.,hazard_reference_gap_m=45.,sensor_delay_s=.256),
    dict(name='range-dropout-15-gap45',target_speed_mps=15.,hazard_reference_gap_m=45.,dropout_start_s=.25,dropout_end_s=.45),
    dict(name='noisy-range-15-gap45',target_speed_mps=15.,hazard_reference_gap_m=45.,sensor_noise_std_m=.5),
]

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')

def prepare(out,development=False):
    if out.exists():raise ValueError('Choose a new directory; failures are retained')
    out.mkdir(parents=True)
    files=[ROOT/'scripts/robust_matrix.py',ROOT/'worlds/obstacle_braking.wbt',*sorted((ROOT/'controllers').glob('*/*.py'))]
    report={'name':'webots-cruise-then-hazard-v2','declared_date':'2026-10-17','source_base_commit':'80d0b38943ed502d697479142b521aaef65ad685',
            'stage':'development' if development else 'frozen-evaluation','physics_seeds':[97] if development else [101,202,303],
            'policies':['legacy_ttc','stopping_envelope'],'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},
            'acceptance':'verified cruise within 5% for 20 steps, stop below 0.1 m/s, no obstacle contact, minimum collision-hull clearance >=1 m; report stress failures too',
            'scope':'Physical ego vehicle with scripted kinematic box targets introduced after measured cruise. Static, moving and decelerating targets; contact-friction parameter changes; software delay, dropout and Gaussian range-noise injection. Not a validated road, lead-vehicle, weather, perception or certification model. Physics seeds are repeats within this small designed family, not independent traffic scenarios.',
            'constants':'stopping envelope: margin 2 m + speed*0.35 s + speed^2/(2*3 m/s^2); full brake; stale threshold 0.16 s. Deceleration is a design assumption, not calibrated friction truth.',
            'geometry_source':'https://github.com/cyberbotics/webots/blob/R2025a/projects/vehicles/protos/lincoln/LincolnMKZ.proto',
            'cases':[]}
    with zipfile.ZipFile(out/'executed-source.zip','w',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(p,p.relative_to(ROOT))
    template=(ROOT/'worlds/obstacle_braking.wbt').read_text()
    selected=CASES[:2] if development else CASES
    for case,seed,policy in itertools.product(selected,report['physics_seeds'],report['policies']):
        name=f'{case["name"]}-seed{seed}-{policy}';project=out/name
        (project/'worlds').mkdir(parents=True);(project/'config').mkdir()
        shutil.copytree(ROOT/'controllers',project/'controllers',ignore=shutil.ignore_patterns('__pycache__'))
        config={**case,'policy':policy,'random_seed':seed,'time_step_ms':32,'sensor_max_range_m':120.,'friction':case.get('friction',.9)}
        text=template.replace('randomSeed 1',f'randomSeed {seed}').replace('        8\n',f'        {config["friction"]}\n')
        text=text.replace('floorSize 400 60','floorSize 2000 60').replace('length 200','length 1000')
        text=text.replace('controller "autonomous_car"','controller "robust_car"').replace('controller "experiment_supervisor"','controller "robust_supervisor"')
        text=text.replace('40 40 0','120 120 0').replace('translation 150 0 1','translation 10000 0 1')
        text=text.replace('name "telemetry emitter"','name "telemetry emitter"').replace('  sensorsSlotCenter [','  sensorsSlotCenter [\n    Receiver { name "scenario receiver" channel 2 }')
        text=text.replace('  children [\n    Receiver {\n      name "telemetry receiver"','  children [\n    Emitter { name "scenario emitter" channel 2 }\n    Receiver {\n      name "telemetry receiver"')
        (project/'worlds/obstacle_braking.wbt').write_text(text);dump(project/'config/scenario.json',config)
        report['cases'].append({'name':name,'case':case['name'],'seed':seed,'policy':policy,'stress':case.get('stress',False),'status':'prepared','config':config})
    dump(out/'protocol.json',report);dump(out/'matrix.json',report)
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--webots',type=Path);p.add_argument('--development',action='store_true');p.add_argument('--timeout',type=int,default=120);a=p.parse_args()
    a.out=a.out.resolve();report=prepare(a.out,a.development)
    if not a.webots:return
    report['runtime']={'path':str(a.webots),'executable_sha256':sha(a.webots),'host':platform.platform(),'declared_version':os.environ.get('WEBOTS_VERSION','unspecified')}
    for row in report['cases']:
        project=a.out/row['name'];config=project/'config/scenario.json'
        command=[str(a.webots),'--batch','--mode=fast','--no-rendering','--minimize','--port=13037',str(project/'worlds/obstacle_braking.wbt')]
        started=time.time()
        try:
            run=subprocess.run(command,env={**os.environ,'BRAKING_SCENARIO_CONFIG':str(config)},stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=a.timeout)
            (project/'webots.log').write_text(run.stdout);row['exit_code']=run.returncode
            summary=project/'results/summary.json'
            if summary.exists():row.update(status='completed',summary=json.loads(summary.read_text()))
            else:row['status']='startup_failed'
        except subprocess.TimeoutExpired as exc:
            (project/'webots.log').write_text(exc.stdout.decode(errors='replace') if isinstance(exc.stdout,bytes) else (exc.stdout or ''));row['status']='timed_out'
        row['wall_seconds']=time.time()-started;dump(a.out/'matrix.json',report)
        print(f'{row["name"]}: {row["status"]} pass={row.get("summary",{}).get("pass")} wall={row["wall_seconds"]:.1f}s',flush=True)
        if row['status']!='completed':break

if __name__=='__main__':main()
