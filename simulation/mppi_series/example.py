"""Run one complete solve and a prediction; no ROS/Gazebo or vehicle commands."""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import numpy as np
from .controller import Config,MPPIController,dynamics,attitude_target_fields


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,default=Path(__file__).resolve().parents[2]/'publishing/artifacts/mppi_series/9_1')
    args=parser.parse_args();started=datetime.now(timezone.utc)
    out=args.output_root/started.strftime('%Y%m%dT%H%M%S_%fZ');out.mkdir(parents=True)
    source=out/'source';source.mkdir()
    hashes={}
    for p in Path(__file__).parent.glob('*.py'):
        shutil.copy2(p,source/p.name);hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/'command.json').write_text(json.dumps([sys.executable,'-m','mppi_series.example',*sys.argv[1:]],indent=2))
    tests=subprocess.run([sys.executable,'-m','unittest','mppi_series.test_controller','-v'],capture_output=True,text=True)
    (out/'tests.log').write_text(tests.stdout+tests.stderr)
    if tests.returncode:raise RuntimeError('Model/interface contract tests failed')
    cfg=Config();c=MPPIController(cfg);state=np.zeros(12);state[2]=3.;goal=np.array([1.,0.,3.])
    begin=time.perf_counter();command=c.step(state,goal);elapsed=(time.perf_counter()-begin)*1000
    prediction,valid=c.rollout(state,c.last['updated_sequence'][None,:,:])
    if not valid[0]:raise RuntimeError('Weighted sequence left the model validity region')
    checks=[]
    for seed in [0,1,2,3,7]:
        for target in [[0,0,3],[1,0,3],[-1,0,3],[0,1,3],[0,0,2]]:
            other=MPPIController(Config(seed=seed));begin=time.perf_counter();u=other.step(state,target)
            checks.append({'seed':seed,'goal':target,'command':u.tolist(),'compute_ms':(time.perf_counter()-begin)*1000,
                           'valid_candidates':other.last['valid_count']})
    scalar=dynamics(state,cfg.hover,cfg)
    report={'passed':True,'started_utc':started.isoformat(),'finished_utc':datetime.now(timezone.utc).isoformat(),
            'scope':'Independent chapter 9-1 model and complete controller execution. Prediction only; no closed-loop, Gazebo or PX4-command verification.',
            'config':asdict(cfg),'state':state.tolist(),'goal':goal.tolist(),'command':command.tolist(),
            'attitude_target_fields':attitude_target_fields(command),'compute_ms':elapsed,'candidate_count':c.last['candidate_count'],
            'valid_candidates':c.last['valid_count'],'weight_sum':float(c.last['weights'].sum()),
            'hover_one_step_max_error':float(np.max(np.abs(scalar-state))),'cases':checks,
            'python':platform.python_version(),'numpy':np.__version__,'source_sha256':hashes,
            'tests_returncode':tests.returncode,'tests':13,'server_updated':False,'publication_ready':False}
    np.savez(out/'prediction.npz',prediction=prediction[0],weights=c.last['weights'],costs=c.last['costs'],
             updated_sequence=c.last['updated_sequence'],state=state,goal=goal)
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(out),'command':report['command'],'compute_ms':elapsed,'valid_candidates':report['valid_candidates'],'tests':report['tests'],'passed':True},indent=2))


if __name__=='__main__':main()
