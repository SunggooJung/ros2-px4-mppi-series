"""Actual Gazebo static obstacle trials with raw contact/control evidence."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from .episode_10_1 import sha, write
from .obstacles import PROFILES,OBSTACLE
from dataclasses import asdict

TITLE='ROS2 × PX4로 시작하는 자율비행 드론 제어 (11-2) 장애물 비용은 실제로 회피 행동을 만드는가?'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workspace-root',type=Path,default=Path(os.environ.get('STACK_MPPI_WORKSPACE',str(Path(__file__).resolve().parents[2]))))
    parser.add_argument('--reproduce-child',action='store_true',help='Run all three actual flights, skip duplicate recording and recursive ZIP execution')
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
    facts={'passed':False,'scope':'gazebo_static_obstacle_avoidance','episode':'11-2','title':TITLE,
           'hardware_used':False,'flight_verified':False,'mppi_applied_verified':False,
           'checked_utc':datetime.now(timezone.utc).isoformat(),'source_sha256':hashes,
           'trials':[],'cases':[],'seeds':[7],
           'units':{'position':'m','velocity':'m/s','body_rate':'rad/s','thrust':'normalized [0,1]','time':'s'},
           'frames':{'pose_velocity':'ENU map','body_rate':'FLU base_link','FCU':'FRD; MAVROS converts once'},
           'config':{'samples':500,'dt_s':.05,'temperature':3.,'rate_tau_s':.08,
                     'profiles':PROFILES,'obstacle':asdict(OBSTACLE),'reference_endpoint_offset_m':[1.7,.12,2.],
                     'handover_duration_s':30.,'evaluation_window_s':[7.,27.],
                     'seed':7,'arrival_limit_m':.25,'minimum_clearance_m':.10,
                     'cost_weights':{'position_running':30,'velocity_running':3,'position_terminal':40,'velocity_terminal':5,
                                     'tilt':.5,'heading':1,'body_rate':.2,'hover_deviation':.03,'input_change':.02}},
           'limitations':['Gazebo SITL, no real hardware.',
                          'PX4 position control only for takeoff/calibration; AUTO.LAND for landing; excluded from MPPI evaluation.',
                          'PX4 still controls inner body rates, allocation and motors.',
                          'Linear local thrust/rate lag are assumptions; live hover command median initializes each trial.',
                          'One attempt per horizon per invocation, fixed seed and order; not a repeated statistical superiority test.',
                          'Receipt matching within +/-100ms is value consistency, not unique latency measurement.',
                          'Known static geometry only; soft cost is not a collision avoidance guarantee.',
                          'Conservative 0.40m sphere covers x500 collision geometry; clearance violation is distinct from actual contact.',
                          'Same straight timed reference in all cases; endpoint y=0.12m is explicit symmetry breaking.',
                          'No repeated statistical superiority or real-world robustness claim.',
                          'Clearance is sampled from actual Gazebo poses; no between-sample safety guarantee.',
                          'Weak/short-cost cases may contact obstacles or abort; partial verified intervals are retained as failures.',
                          'If AUTO.LAND cannot disarm a collided owned SITL model within 60s, identity-gated test-only force-disarm is attempted and actual disarmed state must be observed; never a hardware procedure.']}
    facts['verification_commands']=[]
    prior=[]
    for p in sorted(out.parent.glob('*/result.json')):
        if p.parent==out:continue
        d=json.loads(p.read_text())
        prior.append({'output':str(p.parent),'passed':d.get('passed'),
                      'error':d.get('error'),'scope':d.get('scope')})
    write(out/'prior_attempts.json',{'attempts':prior,'note':'Prior failures retained; never merged into current flight metrics.'})
    facts['implementation_command']=[sys.executable,'-m','mppi_series.episode_11_2','--output-root',str(args.output_root.resolve()),'--workspace-root',str(workspace)]
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
                                        {'python_callable':'mppi_series.obstacle_checks.verify','report':str(out/'meaningful_checks.json')}]
        if tests.returncode:raise RuntimeError('Canonical dynamics/interface/flight guard checks failed')
        from .obstacle_checks import verify
        checks=verify(out,workspace);facts['meaningful_checks']=checks
        facts['cases'].extend(checks['cases'])
        from .obstacle_runtime import execute
        from .obstacle_results import analyze
        lock_root=workspace/'simulation/experiment_logs';lock_root.mkdir(parents=True,exist_ok=True)
        with (lock_root/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            for profile in PROFILES:
                trial=out/'trials'/profile;trial.mkdir(parents=True)
                tf=execute(trial,workspace,capture_gif=(profile=='inflated_cost' and not args.reproduce_child),profile=profile)
                tf.update(output=str(trial),profile=profile)
                write(trial/'result.json',tf)
                facts['trials'].append(tf)
                try:
                    if 'flight' not in tf:raise RuntimeError(tf.get('error','No flight records'))
                    if tf['flight'].get('final_armed'):raise RuntimeError('Owned SITL did not recover to disarmed')
                    report,_=analyze(trial,tf)
                    tf.update(passed=True,trial_execution_completed=tf['flight'].get('passed',False),analysis=report,mppi_applied_verified=True,flight_verified=True,fcu_received_verified=True)
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
            from .obstacle_article import build
            build(out,facts,workspace)
        for name,digest in hashes.items():
            if sha(application/name)!=digest:raise RuntimeError('Executed source changed during packaging: '+name)
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
