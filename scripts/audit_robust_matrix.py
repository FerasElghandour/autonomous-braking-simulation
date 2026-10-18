"""Replay every retained CSV and reject altered geometry, verdicts or omissions."""
import argparse,csv,gzip,hashlib,importlib.util,io,json,math,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('robust_supervisor',ROOT/'controllers/robust_supervisor/robust_supervisor.py')
model=importlib.util.module_from_spec(spec);spec.loader.exec_module(model)

BOOL=['active','contact','sensor_valid','dropout_injected']
STR=['state']
def records(path):
    if hasattr(path,'read'):
        rows=list(csv.DictReader(path))
    else:
        opener=gzip.open if path.suffix=='.gz' else open
        with opener(path,'rt',newline='') as f:
            rows=list(csv.DictReader(f))
    for row in rows:
        for key,value in row.items():
            if key in STR:continue
            if value=='':row[key]=None
            elif key in BOOL:
                if value not in ['True','False']:raise ValueError('Invalid boolean telemetry')
                row[key]=value=='True'
            else:
                row[key]=float(value)
                if not math.isfinite(row[key]):raise ValueError('Non-finite telemetry')
    return rows

def verify_case(rows,summary):
    if len(rows)!=summary['rows'] or not rows:raise ValueError('Missing telemetry rows')
    config=summary['config'];trigger=summary['trigger_time_s']
    for i,row in enumerate(rows,1):
        if row['step']!=i or abs(row['sim_time_s']-i*.032)>1e-7:raise ValueError('Noncontiguous physics time')
        if row['active']!=(trigger is not None and row['sim_time_s']>=trigger-1e-9):raise ValueError('Altered hazard phase')
        reference=row['obstacle_x_m']-1.-row['car_x_m']
        extent=model.forward_extent([row[f'orientation_{j}'] for j in range(9)])
        if abs(reference-row['reference_distance_m'])>1e-8 or abs(extent-row['hull_forward_extent_m'])>1e-8 or abs(reference-extent-row['hull_clearance_m'])>1e-8:
            raise ValueError('Altered measured clearance')
    active=[r for r in rows if r['active']]
    if active:
        # Supervisor field commands are applied on the next physics step. The
        # trigger row still measures the initial hidden object at X=10000.
        origin=active[0]['car_x_m']+config['hazard_reference_gap_m']+1.
        for index,r in enumerate(active):
            expected=(10000. if index==0 else origin+model.lead_displacement(max(0.,r['sim_time_s']-trigger-.032),config.get('lead_speed_mps',0.),config.get('lead_deceleration_mps2',0.),config.get('lead_brake_after_s',0.)))
            if abs(expected-r['obstacle_x_m'])>1e-7:raise ValueError('Target trajectory mismatch')
    expected=model.outcome(rows,trigger,config['target_speed_mps'])
    if summary['end_reason'] not in ['contact','stopped']:
        expected['pass']=False;expected['failure_reasons'].append(summary['end_reason'])
    for key,value in expected.items():
        actual=summary.get(key)
        if isinstance(value,float):
            if actual is None or abs(value-actual)>1e-8:raise ValueError('Changed summary '+key)
        elif actual!=value:raise ValueError('Changed summary '+key)
    return len(rows)

def audit(root):
    matrix=json.loads((root/'matrix.json').read_text());errors=[];total=0;completed=0
    archives=[zipfile.ZipFile(p) for p in sorted(root.glob('case-artifacts*.zip'))]
    source=root/'executed-source.zip'
    if source.exists():
        with zipfile.ZipFile(source) as z:
            for name,digest in matrix['source_sha256'].items():
                if hashlib.sha256(z.read(name)).hexdigest()!=digest:errors.append('Executed source mismatch: '+name)
    for case in matrix['cases']:
        if case['status']!='completed':errors.append(case['name']+': not completed');continue
        project=root/case['name'];p=project/'results/run.csv'
        if not p.exists():p=p.with_suffix('.csv.gz')
        try:
            prefix=case['name']+'/'
            matches=[z for z in archives if prefix+'results/run.csv' in z.namelist()]
            if archives:
                if len(matches)!=1:raise ValueError('Missing or duplicated case archive')
                archive=matches[0]
                summary=json.loads(archive.read(prefix+'results/summary.json'))
                rows=records(io.StringIO(archive.read(prefix+'results/run.csv').decode()))
                if json.loads(archive.read(prefix+'config/scenario.json'))!=case['config']:raise ValueError('Executed configuration mismatch')
            else:
                summary=json.loads((project/'results/summary.json').read_text());rows=records(p)
            if summary!=case['summary'] or summary['config']!=case['config']:raise ValueError('Matrix/config/summary discrepancy')
            total+=verify_case(rows,summary);completed+=1
        except Exception as exc:errors.append(case['name']+': '+str(exc))
    for archive in archives:archive.close()
    return {'cases_expected':len(matrix['cases']),'cases_audited':completed,'telemetry_rows':total,'errors':errors,'pass':not errors}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True);a=p.parse_args();report=audit(a.results)
    (a.results/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));raise SystemExit(0 if report['pass'] else 1)

if __name__=='__main__':main()
