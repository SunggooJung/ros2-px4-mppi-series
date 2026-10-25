"""Bounded actual Gazebo flight with source snapshots and original ULog/MCAP."""
import argparse
from datetime import datetime,timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from .episode_10_1 import execute,sha,write


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workspace-root',type=Path,default=Path(os.environ.get('STACK_MPPI_WORKSPACE',str(Path(__file__).resolve().parents[2]))))
    parser.add_argument('--reproduce-child',action='store_true')
    parser.add_argument('--capture-gif',action='store_true',help='Record the actual Gazebo GUI of this live flight')
    args=parser.parse_args();workspace=args.workspace_root.resolve()
    out=args.output_root.resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out.mkdir(parents=True)
    application=Path(__file__).resolve().parents[2]
    sources=list((application/'simulation/mppi_series').glob('*.py'))+list((application/'offboard_control').rglob('*.py'))
    sources+=list((application/'simulation/mppi_series/reference_inputs').glob('*.txt'))
    sources+=[application/'offboard_control/package.xml',application/'offboard_control/setup.cfg',application/'offboard_control/resource/offboard_control',
              application/'simulation/scripts/experiment_provenance.py',application/'simulation/config/headless_server.config']
    hashes={}
    for path in sources:
        if '__pycache__' in path.parts:continue
        relative=path.relative_to(application);destination=out/'source'/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(path.read_bytes());hashes[str(relative)]=sha(destination)
    facts={'passed':False,'scope':'gazebo_mppi_flight','episode':'10-2','hardware_used':False,
           'flight_verified':False,'mppi_applied_verified':False}
    tests=subprocess.run([sys.executable,'-m','unittest','mppi_series.test_controller','mppi_series.test_ros_interface','mppi_series.test_ros_flight','-v'],capture_output=True,text=True)
    (out/'tests.log').write_text(tests.stdout+tests.stderr)
    try:
        if tests.returncode:raise RuntimeError('Flight/model/state guard checks failed')
        lock_root=workspace/'simulation/experiment_logs';lock_root.mkdir(parents=True,exist_ok=True)
        with (lock_root/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            facts=execute(out,workspace,chapter='10-2',capture_gif=args.capture_gif or not args.reproduce_child)
    except Exception as error:facts['error']=str(error)
    for name,digest in hashes.items():
        if sha(application/name)!=digest:facts['error']='Source changed during execution: '+name
    facts.update(source_sha256=hashes,checked_utc=datetime.now(timezone.utc).isoformat(),tests_returncode=tests.returncode,
                 evidence_files=[str(p) for p in [*out.glob('rosbag/*.mcap'),*out.glob('runtime/log/**/*.ulg'),*out.glob('*.csv')]],
                 config=facts.get('flight',{}).get('config',{}),
                 units={'position':'m','velocity':'m/s','body_rate':'rad/s','collective_thrust':'normalized [0,1]'},
                 frames={'pose':'ENU map','twist':'FLU base_link','PX4_rates':'FRD; MAVROS converts once'},
                 seeds=[7],cases=[],limitations=['Gazebo SITL only; not hardware validation.',
                     'PX4 position control performs takeoff and AUTO.LAND performs landing.',
                     'Only the explicit rate-Offboard interval counts as MPPI flight.',
                     'Hover thrust is initialized from this run; rate lag and local linear thrust map remain assumptions.'])
    write(out/'result.json',facts)
    if 'error' not in facts:
        from .flight_results import finish
        facts=finish(out,facts,workspace,args.reproduce_child)
    print(json.dumps({'output':str(out),'passed':facts['passed']},indent=2),flush=True)
    return 0 if facts['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
