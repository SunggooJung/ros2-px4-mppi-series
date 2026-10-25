"""12-1: repeated actual PX4 position vs MPPI rate/thrust Gazebo comparison."""
import argparse
from datetime import datetime,timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
from .episode_10_1 import sha,write

TITLE='ROS2 × PX4로 시작하는 자율비행 드론 제어 (12-1) MPPI는 얼마나 잘 동작했는가?'
SCHEDULE=[dict(name=f'{i+1}_{method}',method=method,seed=seed,hover_scale=1.)
          for i,seed in enumerate([7,17,27])
          for method in (['px4_position','mppi'] if i%2==0 else ['mppi','px4_position'])]
SCHEDULE += [dict(name='model_hover_plus8percent',method='mppi',seed=7,hover_scale=1.08)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workspace-root',type=Path,default=Path(os.environ.get('STACK_MPPI_WORKSPACE',str(Path(__file__).resolve().parents[2]))))
    parser.add_argument('--reproduce-child',action='store_true',help='Run all seven actual trials; omit recursive ZIP and duplicate recording only')
    args=parser.parse_args();workspace=args.workspace_root.resolve();application=Path(__file__).resolve().parents[2]
    out=args.output_root.resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out.mkdir(parents=True)
    sources=list((application/'simulation/mppi_series').glob('*.py'))+list((application/'simulation/mppi_series/reference_inputs').glob('*'))
    sources += [application/'simulation/scripts/experiment_provenance.py',application/'simulation/config/headless_server.config']
    hashes={}
    for source in sources:
        if not source.is_file():continue
        rel=source.relative_to(application);dest=out/'source'/rel;dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(source.read_bytes());hashes[str(rel)]=sha(dest)
    facts=dict(passed=False,scope='gazebo_repeated_comparison',episode='12-1',title=TITLE,
               checked_utc=datetime.now(timezone.utc).isoformat(),source_sha256=hashes,hardware_used=False,
               flight_verified=False,mppi_applied_verified=False,cases=[],trials=[],seeds=[7,17,27],
               config=dict(schedule=SCHEDULE,paired_repetitions=3,samples=500,horizon=20,dt_s=.05,temperature=3.,rate_tau_s=.08,
                           reference_radius_m=.5,reference_duration_s=30.,evaluation_window_s=[7.,27.],success_rmse_limit_m=.12,
                           compute_budget_ms=50,publish_budget_ms=50,publish_tolerance_ms=60,
                           controlled_model_error='Assumed thrust_hover = measured hover *1.08 in one additional seed-7 flight; physical plant unchanged'),
               units=dict(position='m',velocity='m/s',body_rate='rad/s',thrust='normalized [0,1]',time='s'),
               frames=dict(pose_velocity='ENU map',body_rate='FLU base_link',FCU='NED/FRD; MAVROS transforms once'),
               limitations=['SITL only, no real hardware or general superiority/safety/robustness guarantee.',
                            'Same timed reference, vehicle/world/binary/controller settings and fixed initial hover gates; scheduling and observed initial state/calibrated hover vary.',
                            'Three paired nominal repetitions, alternating order, seeds 7/17/27 affect MPPI sampling only. PX4/Gazebo noise seeds are not controlled.',
                            'Baseline is PX4 position-only Offboard, without external velocity feedforward. MPPI has predicted position/velocity references; this is architecture comparison, not equal-information algorithm isolation.',
                            'Both architectures use PX4 inner rate control/allocation/motors; baseline also uses PX4 position/velocity/attitude loops. Position takeoff and AUTO.LAND excluded.',
                            'MPPI compute time and external baseline dispatch time do not measure the same work; PX4 internal computation time not measured.',
                            'Different input units are reported per component. FCU total variation uses native ULog sampling; neither input change nor squared command is physical energy or jerk.',
                            'Local linear thrust and 0.08s rate lag remain assumptions; one +8% assumed-hover case is sensitivity evidence only.',
                            'Receipt within +/-100ms is value consistency, not unique latency.',
                            'A successful execution can include failed tracking criteria; all verification/flight failures are retained and block readiness.'])
    facts['implementation_command']=[sys.executable,'-m','mppi_series.episode_12_1','--output-root',str(args.output_root.resolve()),'--workspace-root',str(workspace)]
    prior=[]
    for p in sorted(out.parent.glob('*/result.json')):
        if p.parent==out:continue
        try:d=json.loads(p.read_text())
        except Exception:continue
        prior.append(dict(output=str(p.parent),passed=d.get('passed'),error=d.get('error')))
    write(out/'prior_attempts.json',dict(attempts=prior,note='All prior failures retained; separate from this invocation.'))
    write(out/'result.json',facts)
    try:
        instructions={}
        for name in ['AGENTS.md','docs/DRONE_MPPI_SERIES_HANDOFF.md','publishing/MPPI_SERIES_DESIGN.md','publishing/MPPI_EDITORIAL_POLICY.md','publishing/GIF_POLICY.md']:
            dest=out/'instructions'/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((workspace/name).read_bytes());instructions[name]=sha(dest)
        facts['instruction_sha256']=instructions
        cmd=[sys.executable,'-m','unittest','mppi_series.test_controller','mppi_series.test_ros_interface','mppi_series.test_ros_flight','-v']
        tested=subprocess.run(cmd,capture_output=True,text=True);(out/'tests.log').write_text(tested.stdout+tested.stderr)
        facts['verification_commands']=[dict(command=cmd,returncode=tested.returncode,log=str(out/'tests.log'))]
        if tested.returncode:raise RuntimeError('Canonical controller/interface/guard checks failed')
        from .comparison_checks import verify
        facts['meaningful_checks']=verify(out);facts['cases'].extend(facts['meaningful_checks']['cases'])
        from .comparison_runtime import execute
        from .comparison_results import analyze
        lock_root=workspace/'simulation/experiment_logs';lock_root.mkdir(parents=True,exist_ok=True)
        with (lock_root/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            for trial_spec in SCHEDULE:
                trial=out/'trials'/trial_spec['name'];trial.mkdir(parents=True)
                print(json.dumps(dict(stage='actual_trial',**trial_spec)),flush=True)
                tf=execute(trial,workspace,capture_gif=trial_spec['name']=='1_mppi' and not args.reproduce_child,
                           method=trial_spec['method'],seed=trial_spec['seed'],hover_scale=trial_spec['hover_scale'])
                tf.update(trial_spec,output=str(trial));facts['trials'].append(tf);write(trial/'result.json',tf)
                try:
                    if 'error' in tf:raise RuntimeError(tf['error'])
                    if not tf['flight']['passed'] or tf['flight']['final_armed']:raise RuntimeError('Flight incomplete or not disarmed')
                    report=analyze(trial,tf);tf.update(passed=True,analysis=report,flight_verified=True,fcu_received_verified=True)
                    if trial_spec['method']=='mppi':tf['mppi_applied_verified']=True
                    else:tf['baseline_applied_verified']=True
                    facts['cases'].extend(report['cases'])
                except Exception as error:
                    tf.update(passed=False,analysis_error=str(error));(trial/'analysis_failure.txt').write_text(traceback.format_exc())
                    facts['cases'].append(dict(name=trial_spec['name'],kind='actual_attempt',success=False,reason=str(error)))
                write(trial/'result.json',tf);write(out/'result.json',facts)
                print(json.dumps(dict(stage='trial_complete',name=tf['name'],passed=tf['passed'],error=tf.get('analysis_error'))),flush=True)
                if tf.get('flight',{}).get('final_armed'):raise RuntimeError('Owned SITL still armed; no subsequent trials')
        if len(facts['trials'])!=len(SCHEDULE) or not all(t.get('passed') is True for t in facts['trials']):
            raise RuntimeError('At least one planned actual trial lacks complete verified evidence; all attempts preserved')
        binaries={t['px4_binary_sha256'] for t in facts['trials']}
        images={t['environment']['image_id'] for t in facts['trials']}
        conditions=[t['condition_sha256'] for t in facts['trials']]
        if len(binaries)!=1 or len(images)!=1 or any(c!=conditions[0] for c in conditions):
            raise RuntimeError('Runtime binary/image/world/vehicle/server config changed between conditions')
        facts['condition_sha256']=conditions[0]
        facts.update(flight_verified=True,mppi_applied_verified=True,fcu_received_verified=True,
                     px4_commit=facts['trials'][0]['px4_commit'],versions=facts['trials'][0]['environment'],
                     commands=[dict(trial=t['name'],**c) for t in facts['trials'] for c in json.loads((Path(t['output'])/'commands.json').read_text())])
        nominal=[c for c in facts['cases'] if c.get('kind')=='actual_gazebo_repeated_trial' and c['hover_scale']==1]
        facts['comparison_summary']={method:dict(attempts=3,successes=sum(c['success'] for c in nominal if c['method']==method),
                  failures=sum(not c['success'] for c in nominal if c['method']==method),
                  rmse_values_m=[c['rmse_normal_m'] for c in nominal if c['method']==method]) for method in ['px4_position','mppi']}
        for name,digest in hashes.items():
            if sha(application/name)!=digest:raise RuntimeError('Executed source changed: '+name)
        if not args.reproduce_child:
            from .comparison_article import build
            build(out,facts,workspace)
        for name,digest in hashes.items():
            if sha(application/name)!=digest:raise RuntimeError('Executed source changed during packaging: '+name)
        facts['passed']=True
    except Exception as error:
        facts.update(passed=False,error=str(error));(out/'failure_traceback.txt').write_text(traceback.format_exc())
    facts['checked_utc']=datetime.now(timezone.utc).isoformat()
    facts['evidence_files']=[str(p.resolve()) for p in out.glob('trials/**/*') if p.is_file() and p.suffix in ['.mcap','.db3','.ulg','.csv','.jsonl','.mp4','.gif']]
    write(out/'result.json',facts)
    print(json.dumps(dict(output=str(out),passed=facts['passed']),ensure_ascii=False),flush=True)
    return 0 if facts['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
