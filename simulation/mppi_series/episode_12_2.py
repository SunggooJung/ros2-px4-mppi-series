"""12-2: rerun representative published work and audit the entire curriculum.

No browser, publication, timers, credentials, hardware, or unrelated edits.
The fresh-ZIP child runs the identical experiment suite, omitting recursion only.
"""
import argparse
import csv
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback

import numpy as np
from .episode_10_1 import sha, write

TITLE='ROS2 × PX4로 시작하는 자율비행 드론 제어 (12-2) 처음부터 다시 실행하는 자율비행 프로젝트 — 연재 마무리'
SCHEDULE=[dict(name='10_1_live_interface',episode='10-1',kind='interface'),
          dict(name='10_2_hover_move_return',episode='10-2',kind='point'),
          dict(name='11_1_long_horizon',episode='11-1',kind='trajectory',profile='long_horizon'),
          dict(name='11_2_weak_cost',episode='11-2',kind='obstacle',profile='weak_cost'),
          dict(name='11_2_inflated_cost',episode='11-2',kind='obstacle',profile='inflated_cost'),
          dict(name='12_1_seed7_px4_position',episode='12-1',kind='comparison',method='px4_position'),
          dict(name='12_1_seed7_mppi',episode='12-1',kind='comparison',method='mppi')]
INSTRUCTIONS=['AGENTS.md','docs/DRONE_MPPI_SERIES_HANDOFF.md','publishing/MPPI_SERIES_DESIGN.md',
              'publishing/MPPI_EDITORIAL_POLICY.md','docs/BLOG_EDITORIAL_POLICY.md','publishing/GIF_POLICY.md']


def snapshot(out,application):
    files=list((application/'simulation/mppi_series').glob('*.py'))
    files+=list((application/'simulation/mppi_series/reference_inputs').glob('*'))
    files+=list((application/'offboard_control').rglob('*.py'))
    files += [application/p for p in ['offboard_control/package.xml','offboard_control/setup.cfg',
              'offboard_control/resource/offboard_control','simulation/scripts/experiment_provenance.py',
              'simulation/config/headless_server.config']]
    hashes={}
    for source in sorted(set(files)):
        if not source.is_file() or '__pycache__' in source.parts:continue
        rel=source.relative_to(application);dest=out/'source'/rel
        dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read_bytes());hashes[str(rel)]=sha(dest)
    return hashes


def persisted_command(out,command,env=None,cwd=None,timeout=300):
    started=datetime.now(timezone.utc).isoformat()
    with (out/'execution.log').open('w') as log:
        result=subprocess.run(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
    record=dict(command=command,cwd=str(cwd) if cwd else None,started_utc=started,
                finished_utc=datetime.now(timezone.utc).isoformat(),returncode=result.returncode,log=str(out/'execution.log'))
    write(out/'command.json',record)
    if result.returncode:raise RuntimeError('Command failed, see '+str(out/'execution.log'))
    return record


def point_alignment(out,facts):
    """Align the piecewise position reference to original Odometry header times."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from .ros_interface import odometry_state
    commands=np.atleast_1d(np.genfromtxt(out/'commands.csv',delimiter=',',names=True,dtype=None,encoding='utf-8'))
    events=[e for e in facts['flight']['events'] if e['event']=='target_change']
    times=np.array([e['ros_ns'] for e in events],dtype=np.int64)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    records={}
    while reader.has_next():
        topic,raw,_=reader.read_next()
        if not topic.endswith('/local_position/odom'):continue
        msg=deserialize_message(raw,Odometry);ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if not int(commands[0]['ros_ns'])<=ns<=int(commands[-1]['ros_ns']):continue
        i=int(np.searchsorted(times,ns,side='right')-1)
        if i<0:continue
        state=odometry_state(msg);goal=np.array(events[i]['goal'])
        records[ns]=[ns,(ns-int(times[0]))/1e9,events[i]['phase'],*state[:3],*goal,*list(state[:3]-goal)]
    if len(records)<500:raise RuntimeError('Insufficient original point-task aligned Odometry')
    rows=list(v for _,v in sorted(records.items()))
    with (out/'aligned_point_measurements.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['odom_ros_ns','elapsed_s','phase','x','y','z',
            'ref_x','ref_y','ref_z','error_x','error_y','error_z']);writer.writerows(rows)
    e=np.array([r[-3:] for r in rows]);t=np.array([r[1] for r in rows]);normal=np.zeros(len(rows),dtype=bool)
    for event in events:
        start=(event['ros_ns']-times[0])/1e9;normal|=(t>=start+6)&(t<start+10)
    report=dict(samples=len(rows),rmse_all_m=float(np.sqrt(np.mean(np.sum(e*e,axis=1)))),
                rmse_normal_m=float(np.sqrt(np.mean(np.sum(e[normal]*e[normal],axis=1)))),
                normal_samples=int(normal.sum()),max_position_error_m=float(np.linalg.norm(e,axis=1).max()),
                rule='Unique original Odometry header timestamps; event-defined piecewise reference; fixed last 4s of each 10s phase.',
                distinction='Original chapter acceptance retains command-state metrics; these independently aligned Odometry metrics are labelled separately.')
    write(out/'point_alignment.json',report);return report


def run_trial(out,workspace,spec):
    kind=spec['kind']
    if kind in ['interface','point']:
        from .episode_10_1 import execute
        facts=execute(out,workspace,chapter=spec['episode'],capture_gif=False)
    elif kind=='trajectory':
        from .trajectory_runtime import execute
        facts=execute(out,workspace,capture_gif=False,profile=spec['profile'])
    elif kind=='obstacle':
        from .obstacle_runtime import execute
        facts=execute(out,workspace,capture_gif=False,profile=spec['profile'])
    else:
        from .comparison_runtime import execute
        facts=execute(out,workspace,capture_gif=False,method=spec['method'],seed=7,hover_scale=1.)
    facts.update(spec,output=str(out),passed=False)
    write(out/'result.json',facts)
    try:
        if kind=='interface':
            if 'error' in facts:raise RuntimeError(facts['error'])
            from .interface_results import analyze
            report,_,_=analyze(out,facts)
            facts.update(analysis=report,fcu_received_verified=True,flight_verified=False,mppi_applied_verified=False,
                         cases=[dict(name='live_state_command_receipt',success=True),
                                dict(name='injected_stale_state',success=False,expected_failure=True,verification_passed=True,
                                     reason='Commands blocked and PX4 stream-loss flag observed; unarmed interface only.')])
        else:
            if 'flight' not in facts:raise RuntimeError(facts.get('error','No flight record'))
            if facts['flight'].get('final_armed'):raise RuntimeError('Owned SITL not disarmed')
            if kind!='obstacle' and ('error' in facts or not facts['flight'].get('passed')):
                raise RuntimeError(facts.get('error',facts['flight'].get('error','Incomplete flight')))
            if kind=='point':
                from .flight_results import analyze
                report,_=analyze(out,facts);facts['aligned_odometry_analysis']=point_alignment(out,facts)
            elif kind=='trajectory':
                from .trajectory_results import analyze
                report,_=analyze(out,facts)
            elif kind=='obstacle':
                from .obstacle_results import analyze
                report,_=analyze(out,facts)
            else:
                from .comparison_results import analyze
                facts.update(seed=7,hover_scale=1.)
                report=analyze(out,facts)
            facts.update(analysis=report,cases=report['cases'],fcu_received_verified=True,flight_verified=True,
                         mppi_applied_verified=spec.get('method')!='px4_position',
                         baseline_applied_verified=spec.get('method')=='px4_position')
        facts['passed']=True
    except Exception as error:
        facts['analysis_error']=str(error);(out/'analysis_failure.txt').write_text(traceback.format_exc())
    facts['evidence_files']=[str(p.resolve()) for p in out.rglob('*') if p.is_file() and p.suffix in {'.mcap','.db3','.ulg','.csv','.jsonl'}]
    write(out/'result.json',facts)
    return facts


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workspace-root',type=Path,default=Path(os.environ.get('STACK_MPPI_WORKSPACE',str(Path(__file__).resolve().parents[2]))))
    parser.add_argument('--reproduce-child',action='store_true',help='Identical experiments; skip only recursive ZIP rerun')
    args=parser.parse_args();workspace=args.workspace_root.resolve();application=Path(__file__).resolve().parents[2]
    out=args.output_root.resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out.mkdir(parents=True)
    hashes=snapshot(out,application)
    facts=dict(passed=False,scope='reproduction',episode='12-2',title=TITLE,
        checked_utc=datetime.now(timezone.utc).isoformat(),source_sha256=hashes,hardware_used=False,
        flight_verified=False,mppi_applied_verified=False,cases=[],trials=[],commands=[],seeds=[7,11,23],
        config=dict(schedule=SCHEDULE,own_model_temperatures=[1.,3.,10.],own_model_steps=240,
                    representative_only=True,comparison_repetitions_this_run=1,
                    samples=500,dt_s=.05,temperature=3.,rate_tau_s=.08,
                    own_model='Canonical Config() and original 12 seed/temperature/hover cases.',
                    criteria='Original chapter criteria retained; tracking/collision failures are outcomes, not evidence-verification failures.'),
        units=dict(position='m',velocity='m/s',body_rate='rad/s',angles='rad',thrust='normalized [0,1]',time='s'),
        frames=dict(pose_velocity='ENU map',body_rate='FLU base_link',FCU='NED/FRD; MAVROS converts once'),
        limitations=['SITL and own-model computations only; no hardware, superiority, safety or mathematical convergence guarantee.',
          'Representative replay, not every historical installation command or all seven 12-1 comparison trials.',
          'One seed-7 architecture pair here; the three-repetition statistics belong to preserved 12-1 evidence.',
          'Seed controls MPPI sampling only; Gazebo sensor noise, scheduling and initial hover vary.',
          'Linear thrust and rate lag are assumed models; live hover initialization is not plant identification.',
          'Both flight architectures use PX4 inner rate control/allocation; position takeoff and landing are excluded.',
          'Baseline uses external position only; MPPI uses predicted position/velocity; information and hierarchy differ.',
          'Receipt within +/-100ms tests value consistency, not unique transport latency.',
          'Historical pages and links are audited from preserved local evidence, not fresh server access.',
          'External ROS/Docker/PX4 dependencies and previous raw evidence remain required; the ZIP is a runnable source/config bundle, not a full OS image.',
          'Dirty PX4 source is recorded separately from the actual prebuilt binary hash; no claim it was rebuilt from this tree.'])
    facts['implementation_command']=[sys.executable,'-m','mppi_series.episode_12_2','--output-root',str(args.output_root.resolve()),'--workspace-root',str(workspace)]
    prior=[]
    for p in sorted(out.parent.glob('*/result.json')):
        if p.parent==out:continue
        try:d=json.loads(p.read_text())
        except Exception:continue
        prior.append(dict(output=str(p.parent),passed=d.get('passed'),error=d.get('error')))
    write(out/'prior_attempts.json',dict(attempts=prior,note='All prior attempts retained; no mixing with this run.'))
    write(out/'result.json',facts)
    try:
        instruction_hashes={}
        for name in INSTRUCTIONS:
            dest=out/'instructions'/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((workspace/name).read_bytes());instruction_hashes[name]=sha(dest)
        facts['instruction_sha256']=instruction_hashes
        from .reproduction_audit import audit,verify_file,digest
        facts['series_audit']=audit(out,workspace,application)
        # Hash verification must detect a modified archive/source, not only match an intact file.
        check=out/'checks/archive_integrity';check.mkdir(parents=True)
        probe=check/'probe.txt';probe.write_text('retained source\n');expected=digest(probe);verify_file(probe,expected)
        probe.write_text('modified source\n')
        try:verify_file(probe,expected)
        except RuntimeError as error:
            facts['cases'].append(dict(name='altered_source_rejected',kind='executed_integrity_rejection',success=False,
                                      verification_passed=True,expected_failure=True,reason=str(error)))
        else:raise RuntimeError('Integrity check accepted a changed source')
        tests=out/'checks/canonical';tests.mkdir(parents=True)
        facts['commands'].append(persisted_command(tests,[sys.executable,'-m','unittest','mppi_series.test_controller',
               'mppi_series.test_ros_interface','mppi_series.test_ros_flight','-v']))
        from .chapter_7_checks import verify as chapter7
        from .comparison_checks import verify as evaluation
        from .obstacle_checks import verify as geometry
        for name,fn in [('chapter_7',chapter7),('reference_cost_evaluation',evaluation),('obstacle_geometry',lambda p:geometry(p,workspace))]:
            directory=out/'checks'/name;directory.mkdir(parents=True);report=fn(directory)
            facts.setdefault('meaningful_checks',{})[name]=report
            facts['cases'].extend(report.get('cases',[]) if isinstance(report.get('cases'),list) else [])
        for module,name in [('example','9_1_independent'),('closed_loop','9_2_closed_loop')]:
            directory=out/'models'/name;directory.mkdir(parents=True)
            facts['commands'].append(persisted_command(directory,[sys.executable,'-m','mppi_series.'+module,'--output-root',str(directory)]))
            results=list(directory.glob('*/result.json'))
            if len(results)!=1:raise RuntimeError('Missing unique model result')
            model=json.loads(results[0].read_text())
            if model.get('passed') is not True:raise RuntimeError('Incomplete model reproduction')
            facts.setdefault('model_runs',{})[name]=dict(output=str(results[0].parent),result=model)
            if module=='closed_loop':facts['cases'].extend(dict(c,kind='actual_own_model_trial',episode='9-2') for c in model['cases'])
        write(out/'result.json',facts)
        lock_root=workspace/'simulation/experiment_logs';lock_root.mkdir(parents=True,exist_ok=True)
        with (lock_root/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            for spec in SCHEDULE:
                trial=out/'trials'/spec['name'];trial.mkdir(parents=True)
                print(json.dumps(dict(stage='actual_reproduction_trial',**spec)),flush=True)
                tf=run_trial(trial,workspace,spec);facts['trials'].append(tf)
                facts['cases'].extend(dict(c,reproduction_trial=spec['name'],episode=spec['episode']) for c in tf.get('cases',[]))
                for c in json.loads((trial/'commands.json').read_text()) if (trial/'commands.json').exists() else []:
                    facts['commands'].append(dict(c,trial=spec['name']))
                write(out/'result.json',facts)
                print(json.dumps(dict(stage='trial_verified',name=spec['name'],passed=tf['passed'],error=tf.get('analysis_error'))),flush=True)
                if tf.get('flight',{}).get('final_armed'):raise RuntimeError('Owned SITL armed after recovery; no subsequent trials')
                if not tf['passed']:raise RuntimeError('Actual reproduction evidence incomplete: '+spec['name']+'; '+tf.get('analysis_error',tf.get('error','unknown')))
        facts.update(flight_verified=True,mppi_applied_verified=True,fcu_received_verified=True,
                     px4_commit=facts['trials'][0]['px4_commit'],versions=facts['trials'][0]['environment'])
        if len({t['px4_binary_sha256'] for t in facts['trials']})!=1 or len({t['environment']['image_id'] for t in facts['trials']})!=1:
            raise RuntimeError('PX4 binary or image changed during reproduction')
        for name,digest_value in hashes.items():verify_file(application/name,digest_value)
        from .reproduction_article import build
        build(out,facts,workspace,reproduce_child=args.reproduce_child)
        for name,digest_value in hashes.items():verify_file(application/name,digest_value)
        facts['passed']=True
    except Exception as error:
        facts.update(passed=False,error=str(error));(out/'failure_traceback.txt').write_text(traceback.format_exc())
    facts['checked_utc']=datetime.now(timezone.utc).isoformat()
    facts['evidence_files']=[str(p.resolve()) for folder in ['trials','models','checks'] for p in (out/folder).rglob('*')
                             if p.is_file() and p.suffix in {'.mcap','.db3','.ulg','.csv','.jsonl','.npz'}]
    write(out/'result.json',facts)
    print(json.dumps(dict(output=str(out),passed=facts['passed']),ensure_ascii=False),flush=True)
    return 0 if facts['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
