"""Actual Gazebo timed-trajectory tracking, two horizon lengths, raw evidence."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from .episode_10_1 import sha, write
from .trajectory import PROFILES

TITLE='ROS2 × PX4로 시작하는 자율비행 드론 제어 (11-1) 목표점 추종에서 궤적 추종으로 확장하기'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workspace-root',type=Path,default=Path(os.environ.get('STACK_MPPI_WORKSPACE',str(Path(__file__).resolve().parents[2]))))
    parser.add_argument('--reproduce-child',action='store_true',help='Run both actual flights, skip duplicate recording and recursive ZIP execution')
    args=parser.parse_args();workspace=args.workspace_root.resolve()
    out=args.output_root.resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    out.mkdir(parents=True)
    application=Path(__file__).resolve().parents[2]
    sources=list((application/'simulation/mppi_series').glob('*.py'))
    sources+=list((application/'simulation/mppi_series/reference_inputs').glob('*'))
    sources += [application/'simulation/scripts/experiment_provenance.py',application/'simulation/config/headless_server.config']
    hashes={}
    for path in sources:
        if not path.is_file():continue
        relative=path.relative_to(application);dest=out/'source'/relative
        dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(path.read_bytes())
        hashes[str(relative)]=sha(dest)
    facts={'passed':False,'scope':'gazebo_trajectory_tracking','episode':'11-1','title':TITLE,
           'hardware_used':False,'flight_verified':False,'mppi_applied_verified':False,
           'checked_utc':datetime.now(timezone.utc).isoformat(),'source_sha256':hashes,
           'trials':[],'cases':[],'seeds':[7],
           'units':{'position':'m','velocity':'m/s','body_rate':'rad/s','thrust':'normalized [0,1]','time':'s'},
           'frames':{'pose_velocity':'ENU map','body_rate':'FLU base_link','FCU':'FRD; MAVROS converts once'},
           'config':{'samples':500,'dt_s':.05,'temperature':3.,'rate_tau_s':.08,
                     'horizon_steps':PROFILES,'reference_radius_m':.5,'reference_duration_s':24.,
                     'handover_duration_s':30.,'evaluation_window_s':[7.,27.],
                     'success_rmse_limit_m':.12,'seed':7,
                     'cost_weights':{'position_running':30,'velocity_running':3,'position_terminal':40,'velocity_terminal':5,
                                     'tilt':.5,'heading':1,'body_rate':.2,'hover_deviation':.03,'input_change':.02}},
           'limitations':['Gazebo SITL, no real hardware.',
                          'PX4 position control only for takeoff/calibration; AUTO.LAND for landing; excluded from MPPI evaluation.',
                          'PX4 still controls inner body rates, allocation and motors.',
                          'Linear local thrust/rate lag are assumptions; live hover command median initializes each trial.',
                          'One attempt per horizon per invocation, fixed seed and order; not a repeated statistical superiority test.',
                          'Receipt matching within +/-100ms is value consistency, not unique latency measurement.',
                          'No obstacle avoidance or disturbance robustness claim.']}
    prior=[]
    for p in sorted(out.parent.glob('*/result.json')):
        if p.parent==out:continue
        d=json.loads(p.read_text())
        prior.append({'output':str(p.parent),'passed':d.get('passed'),
                      'error':d.get('error'),'scope':d.get('scope')})
    write(out/'prior_attempts.json',{'attempts':prior,'note':'Prior failures retained; never merged into current flight metrics.'})
    write(out/'result.json',facts)
    try:
        instructions={}
        for name in ['AGENTS.md','docs/DRONE_MPPI_SERIES_HANDOFF.md','publishing/MPPI_SERIES_DESIGN.md',
                     'publishing/MPPI_EDITORIAL_POLICY.md','publishing/GIF_POLICY.md']:
            source=workspace/name;destination=out/'instructions'/name
            destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(source.read_bytes())
            instructions[name]=sha(destination)
        facts['instruction_sha256']=instructions
        command=[sys.executable,'-m','unittest','mppi_series.test_controller','mppi_series.test_ros_interface','mppi_series.test_ros_flight','-v']
        tests=subprocess.run(command,capture_output=True,text=True)
        (out/'tests.log').write_text(tests.stdout+tests.stderr)
        facts['verification_commands']=[{'command':command,'returncode':tests.returncode,'log':str(out/'tests.log')},
                                        {'python_callable':'mppi_series.trajectory_checks.verify','report':str(out/'meaningful_checks.json')}]
        if tests.returncode:raise RuntimeError('Canonical dynamics/interface/flight guard checks failed')
        from .trajectory_checks import verify
        checks=verify(out);facts['meaningful_checks']=checks
        facts['cases'].extend(checks['cases'])
        from .trajectory_runtime import execute
        from .trajectory_results import analyze
        lock_root=workspace/'simulation/experiment_logs';lock_root.mkdir(parents=True,exist_ok=True)
        with (lock_root/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            for profile in PROFILES:
                trial=out/'trials'/profile;trial.mkdir(parents=True)
                tf=execute(trial,workspace,capture_gif=(profile=='long_horizon' and not args.reproduce_child),profile=profile)
                tf.update(output=str(trial),profile=profile)
                write(trial/'result.json',tf)
                facts['trials'].append(tf)
                try:
                    if 'error' in tf:raise RuntimeError(tf['error'])
                    report,_=analyze(trial,tf)
                    tf.update(passed=True,analysis=report,mppi_applied_verified=True,flight_verified=True,fcu_received_verified=True)
                    facts['cases'].extend(report['cases'])
                except Exception as error:
                    tf.update(passed=False,analysis_error=str(error))
                    facts['cases'].append({'name':profile,'kind':'actual_attempt','success':False,'reason':str(error)})
                write(trial/'result.json',tf);write(out/'result.json',facts)
        # Analysis verifies Odometry -> deterministic calculation -> actual ROS
        # AttitudeTarget -> active airborne FCU rate-Offboard -> received input.
        if not all(t.get('mppi_applied_verified') is True for t in facts['trials']):
            raise RuntimeError('At least one planned flight lacks full actual evidence; attempts retained')
        for name,digest in hashes.items():
            if sha(application/name)!=digest:raise RuntimeError('Executed source changed: '+name)
        facts.update(flight_verified=True,mppi_applied_verified=True,fcu_received_verified=True,
                     px4_commit=facts['trials'][0]['px4_commit'],
                     versions=facts['trials'][0]['environment'],
                     commands=[{'trial':t['profile'],**c} for t in facts['trials']
                               for c in json.loads((Path(t['output'])/'commands.json').read_text())])
        if not args.reproduce_child:
            from .trajectory_article import build
            build(out,facts,workspace)
        facts['passed']=True
    except Exception as error:
        facts.update(passed=False,error=str(error))
    facts['checked_utc']=datetime.now(timezone.utc).isoformat()
    facts['evidence_files']=[str(p.resolve()) for p in out.glob('trials/**/*')
                             if p.is_file() and p.suffix in ['.mcap','.db3','.ulg','.csv','.jsonl','.mp4','.gif']]
    write(out/'result.json',facts)
    print(json.dumps({'output':str(out),'passed':facts['passed']},ensure_ascii=False),flush=True)
    return 0 if facts['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
