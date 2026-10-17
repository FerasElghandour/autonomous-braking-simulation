"""Prepare or execute separate Webots speed/clearance cases; preserve historical results."""
import argparse
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]

def prepare(output):
    output=Path(output)
    if output.exists():raise ValueError('Choose a new matrix directory')
    output.mkdir(parents=True)
    original=json.loads((ROOT/'config/scenario.json').read_text());world=(ROOT/'worlds/obstacle_braking.wbt').read_text()
    jobs=[]
    for speed,clearance in itertools.product([5.,10.,15.],[20.,40.,80.]):
        name=f'speed-{speed:g}-clearance-{clearance:g}';project=output/name
        (project/'worlds').mkdir(parents=True);(project/'config').mkdir()
        shutil.copytree(ROOT/'controllers',project/'controllers',ignore=shutil.ignore_patterns('__pycache__'))
        config={**original,'target_speed_mps':speed,'car_start_position_m':5.,'obstacle_position_m':5.+clearance+1.,'results_dir':'results','max_duration_s':40.}
        # Clearance is obstacle front face to vehicle reference X, matching the
        # existing supervisor definition; it is not bumper-to-bumper distance.
        text=world.replace('translation 150 0 1',f'translation {config["obstacle_position_m"]:g} 0 1')
        if text==world and config['obstacle_position_m']!=150:raise ValueError('World substitution failed')
        (project/'worlds/obstacle_braking.wbt').write_text(text)
        (project/'config/scenario.json').write_text(json.dumps(config,indent=2)+'\n')
        jobs.append({'name':name,'speed_mps':speed,'initial_reference_clearance_m':clearance,'status':'prepared','project':name})
    protocol={'cases':jobs,'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),ROOT/'worlds/obstacle_braking.wbt',*sorted((ROOT/'controllers').glob('*/*.py'))]},
              'scope':'Nine fresh Webots cases using unchanged static-obstacle geometry and TTC rules, varied cruise targets and initial reference clearances. Not moving-obstacle, weather, learning-based control or vehicle certification.'}
    (output/'matrix.json').write_text(json.dumps(protocol,indent=2)+'\n');return protocol

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--webots',type=Path);p.add_argument('--timeout',type=int,default=90);a=p.parse_args()
    a.out=a.out.resolve();report=prepare(a.out)
    if a.webots:
        for job in report['cases']:
            project=a.out/job['project'];env={**os.environ,'BRAKING_SCENARIO_CONFIG':str(project/'config/scenario.json')}
            command=[str(a.webots),'--batch','--mode=fast','--no-rendering','--minimize','--port=13027',str(project/'worlds/obstacle_braking.wbt')]
            started=time.perf_counter()
            try:
                result=subprocess.run(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=a.timeout)
                (project/'webots.log').write_text(result.stdout);job['exit_code']=result.returncode
                summary=project/'results/summary.json'
                if summary.exists():job.update(status='completed',summary=json.loads(summary.read_text()))
                else:job['status']='startup_failed'
            except subprocess.TimeoutExpired as exc:
                (project/'webots.log').write_text((exc.stdout or b'').decode(errors='replace') if isinstance(exc.stdout,bytes) else (exc.stdout or ''))
                job['status']='timed_out'
            job['wall_seconds']=time.perf_counter()-started
            (a.out/'matrix.json').write_text(json.dumps(report,indent=2)+'\n')
            if job['status']!='completed':
                report['execution_note']='Stopped after first runtime failure; remaining prepared cases were not executed.';break
        (a.out/'matrix.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
