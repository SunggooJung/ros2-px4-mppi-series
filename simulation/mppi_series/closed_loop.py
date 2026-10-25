"""Chapter 9-2 actual own-model closed loops, including failed temperature cases."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from .controller import Config,MPPIController,dynamics


def case(out,seed,temperature,hover=False):
    cfg=Config(seed=seed,temperature=temperature);controller=MPPIController(cfg)
    state=np.zeros(12);state[2]=3.;rows=[];error=None
    for i in range(240):
        t=i*cfg.dt;goal=np.array([0 if hover or t<2 else 1,0,3.])
        begin=time.perf_counter()
        try:command=controller.step(state,goal)
        except ValueError as failure:error=str(failure);break
        elapsed=(time.perf_counter()-begin)*1000
        state=dynamics(state,command,cfg)
        rows.append([t+cfg.dt,*goal,*state,*command,elapsed,float(np.max(controller.last['weights']))])
    name=f'{"hover" if hover else "move"}-seed{seed}-temp{temperature:g}'
    path=out/(name+'.csv')
    header=['time_s','goal_x','goal_y','goal_z',*['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r'],'c','p_sp','q_sp','r_sp','compute_ms','max_weight']
    with path.open('w') as file:
        writer=csv.writer(file);writer.writerow(header);writer.writerows(rows)
    trace=np.array(rows)
    if not len(trace):raise RuntimeError('No actual closed-loop samples')
    errors=np.linalg.norm(trace[:,4:7]-trace[:,1:4],axis=1)
    window=trace[:,0]>=8.0
    normal=trace[:,0]>=10.0
    final=float(errors[-1]);completed=len(rows)==240
    return {'name':name,'seed':seed,'temperature':temperature,'config':asdict(cfg),'hover':hover,'trace':str(path),
            'trace_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'completed':completed,'error':error,'samples':len(rows),
            'success':completed and final<.15,'final_position_error_m':final,'rmse_all_m':float(np.sqrt(np.mean(errors**2))),
            'rmse_settled_m':float(np.sqrt(np.mean(errors[normal]**2))) if normal.any() else None,
            'settled_rule':'time >= 10s, last 2s of planned 12s; fixed before running, no error-based filtering',
            'compute_mean_ms':float(trace[:,20].mean()),'compute_max_ms':float(trace[:,20].max()),
            'compute_over_dt_count':int((trace[:,20]>cfg.dt*1000).sum()),'max_abs_tilt_rad':float(np.abs(trace[:,10:12]).max())}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True);args=parser.parse_args()
    out=args.output_root/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out.mkdir(parents=True)
    source=out/'source';source.mkdir();hashes={}
    for path in Path(__file__).parent.glob('*.py'):
        shutil.copy2(path,source/path.name);hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    tests=subprocess.run([sys.executable,'-m','unittest','mppi_series.test_controller','-v'],capture_output=True,text=True)
    (out/'tests.log').write_text(tests.stdout+tests.stderr)
    if tests.returncode:raise RuntimeError('Controller consistency checks failed')
    cases=[case(out,seed,temperature) for temperature in [1.,3.,10.] for seed in [7,11,23]]
    cases.extend(case(out,seed,3.,hover=True) for seed in [7,11,23])
    # Recompute metrics from persisted traces, independently of their calculation above.
    for result in cases:
        rows=np.genfromtxt(result['trace'],delimiter=',',skip_header=1)
        e=rows[:,4:7]-rows[:,1:4]
        np.testing.assert_allclose(result['rmse_all_m'],np.sqrt(np.mean(np.sum(e*e,axis=1))),rtol=1e-12)
    repository=Path(__file__).resolve().parents[2];px4=repository/'third_party/px4_sitl/PX4-Autopilot'
    commit=subprocess.check_output(['git','-C',str(px4),'rev-parse','HEAD'],text=True).strip() if px4.is_dir() else None
    package=Path('/opt/ros/jazzy/share/mavros/package.xml')
    gazebo=subprocess.run(['gz','sim','--version'],capture_output=True,text=True,timeout=10) if shutil.which('gz') else None
    environment={'python':platform.python_version(),'numpy':np.__version__,'px4_commit_context':commit,
                 'ros_context':os.environ.get('ROS_DISTRO','not sourced'),
                 'mavros_installed_context':ET.parse(package).getroot().findtext('version') if package.exists() else None,
                 'gazebo_installed_context':gazebo.stdout.strip() if gazebo else None,
                 'scope':'PX4/ROS/MAVROS/Gazebo context only; none are used in this NumPy model experiment'}
    report={'passed':True,'scope':'own_model_closed_loop','episode':'9-2','cases':cases,'source_sha256':hashes,'environment':environment,
            'command':[sys.executable,'-m','mppi_series.closed_loop','--output-root',str(args.output_root)],
            'checked_utc':datetime.now(timezone.utc).isoformat(),'tests_returncode':tests.returncode,
            'limits':'Controller and simulated plant share the assumed model. Not Gazebo/PX4 flight or identified plant performance.',
            'failure_rule':'final 3D position error >= 0.15m at 12s, incomplete trace or controller exception; fixed in code before execution'}
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(out),'passed':True,'cases':len(cases),'successes':sum(c['success'] for c in cases)},indent=2))


if __name__=='__main__':main()
