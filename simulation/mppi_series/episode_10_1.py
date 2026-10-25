"""Actual 10-1 ROS/PX4 interface execution; unarmed reception, not MPPI flight."""
import argparse
from datetime import datetime,timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def execute(out,workspace,chapter='10-1',capture_gif=False):
    if chapter not in ['10-1','10-2']:raise ValueError('Unsupported experiment chapter')
    flight=chapter=='10-2'
    from pymavlink import mavutil
    application=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(application/'simulation/scripts'))
    from experiment_provenance import capture
    px4=workspace/'third_party/px4_sitl/PX4-Autopilot'
    runtime=out/'runtime';runtime.mkdir()
    env=dict(os.environ,ROS_DOMAIN_ID='77',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(out/'ros_logs'))
    processes=[];handles=[];commands=[];gcs=None;container_started=False;screen_capture=None
    name='afm-mppi-interface-'+str(os.getpid())
    def launch(command,log):
        commands.append({'command':command,'log':log});write(out/'commands.json',commands)
        handle=(out/log).open('w');handles.append(handle)
        process=subprocess.Popen(command,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    def interrupted(signum,frame):raise RuntimeError('Interrupted by signal '+str(signum))
    old_handlers={s:signal.signal(s,interrupted) for s in [signal.SIGINT,signal.SIGTERM]}
    result={'passed':False,'scope':'gazebo_mppi_flight' if flight else 'live_ros_px4_interface','episode':chapter,
            'started_utc':datetime.now(timezone.utc).isoformat(),'flight_verified':False,
            'hardware_used':False,'mppi_applied_verified':False,
            'px4_instance':7,'ros_domain_id':77,'model':'gz_x500'}
    try:
        subprocess.run([sys.executable,'-c',"import importlib.metadata; importlib.metadata.distribution('ros2cli')"],
                       env=env,check=True,capture_output=True,timeout=10)
        for port in (14547,14587,18577,14287,13037):
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:sock.bind(('0.0.0.0',port))
        image=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','docker-px4-sitl:latest'],text=True).strip()
        provenance=capture(out,workspace,px4,image,[sys.executable,'-m','mppi_series.episode_'+chapter.replace('-','_'),'--output-root',str(out.parent),'--workspace-root',str(workspace)])
        result.update(environment=provenance,px4_commit=provenance['repositories']['px4']['commit'],
                      px4_binary_sha256=provenance['px4_binary_sha256'])
        source_root=Path(__file__).resolve().parents[2]
        # Build in this run only. No unrelated workspace package is rebuilt.
        build=['colcon','--log-base',str(out/'build_log'),'build','--base-paths',str(source_root/'offboard_control'),
               '--build-base',str(out/'build'),'--install-base',str(out/'install')]
        commands.append({'command':build,'log':'build.log'});write(out/'commands.json',commands)
        with (out/'build.log').open('w') as log:
            subprocess.run(build,cwd=source_root,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        cp='/workspace/src/third_party/px4_sitl/PX4-Autopilot';cb=cp+'/build/px4_sitl_default'
        ce={'HEADLESS':'1','PX4_SYS_AUTOSTART':'4001','PX4_SIM_MODEL':'gz_x500','PX4_GZ_WORLD':'default',
            'PX4_GZ_NO_FOLLOW':'1','GZ_DISTRO':'harmonic','GZ_PARTITION':name,'GZ_LOG_PATH':'/experiment/gz_logs',
            'PX4_GZ_MODELS':cp+'/Tools/simulation/gz/models','PX4_GZ_WORLDS':cp+'/Tools/simulation/gz/worlds',
            'GZ_SIM_RESOURCE_PATH':cp+'/Tools/simulation/gz/models',
            'GZ_SIM_SERVER_CONFIG_PATH':'/experiment/server.config',
            'GZ_SIM_SYSTEM_PLUGIN_PATH':cb+'/src/modules/simulation/gz_plugins',
            'PX4_PARAM_SDLOG_MODE':'1','PX4_UXRCE_DDS_PORT':'8897','ROS_DOMAIN_ID':'77'}
        command=['docker','run','--rm','--init','--name',name,'--network','host',
                 '--tmpfs','/.gz:rw,mode=1777','--tmpfs','/.ignition:rw,mode=1777','--user',f'{os.getuid()}:{os.getgid()}',
                 '-v',f'{workspace}:/workspace/src:ro','-v',f'{runtime}:/experiment:rw',
                 '-v',f'{application}/simulation/config/headless_server.config:/experiment/server.config:ro','-w','/experiment']
        for key,value in ce.items():command+=['-e',f'{key}={value}']
        command+=['--entrypoint',cb+'/bin/px4',image,cb+'/etc','-i','7','-d','-w','/experiment']
        px4_process=launch(command,'px4.log');container_started=True
        if capture_gif:
            from .gazebo_capture import GazeboCapture
            screen_capture=GazeboCapture(out,workspace,name)
        gcs=mavutil.mavlink_connection('udpout:127.0.0.1:18577',source_system=254,source_component=190)
        mavros=launch(['ros2','launch','mavros','px4.launch','fcu_url:=udp://127.0.0.1:14547@127.0.0.1:14587',
                       'tgt_system:=8','namespace:=/sitl_check/mavros'],'mavros.log')
        topics=['/sitl_check/mavros/'+t for t in ['state','extended_state','local_position/odom',
                'local_position/pose','local_position/velocity_local','setpoint_raw/attitude','timesync_status']]+['/mppi/reference']
        if flight:topics+=['/sitl_check/mavros/setpoint_position/local','/sitl_check/mavros/setpoint_raw/target_attitude']
        bag=launch(['ros2','bag','record','-s','mcap','-o',str(out/'rosbag'),'--topics',*topics],'rosbag.log')
        # Exercise the installed ros2 launch entry point, not an uninstalled
        # module that merely resembles the published package instructions.
        if flight:write(out/'sitl_owned.json',{'px4_instance':7,'system_id':8,'hardware_used':False,'container':name,'image_id':image})
        command=['bash','-c','source "$1"; exec ros2 launch offboard_control "$3" "output:=$2"',
                 'mppi-experiment',str(out/'install/setup.bash'),str(out),
                 'mppi_flight.launch.py' if flight else 'mppi_interface.launch.py']
        worker=launch(command,'interface.log')
        deadline=time.monotonic()+(300 if flight else 140);heartbeat=0.
        while worker.poll() is None:
            now=time.monotonic()
            if now>=heartbeat:
                gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,mavutil.mavlink.MAV_STATE_ACTIVE)
                heartbeat=now+1
            for _ in range(100):
                if gcs.recv_match(blocking=False) is None:break
            if any(p.poll() is not None for p in [px4_process,mavros,bag]):raise RuntimeError('PX4/MAVROS/recorder exited')
            if now>=deadline:raise RuntimeError('Bounded interface experiment timed out')
            time.sleep(.1)
        interface=json.loads((out/('flight.json' if flight else 'interface.json')).read_text())
        result['flight' if flight else 'interface']=interface
        if worker.returncode or not interface['passed']:raise RuntimeError('Interface node failed; inspect interface.json')
    except Exception as error:result['error']=str(error)
    finally:
        if screen_capture:screen_capture.stop()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if container_started:subprocess.run(['docker','stop','-t','10',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=20)
        for handle in handles:handle.close()
        if gcs:gcs.close()
        for s,handler in old_handlers.items():signal.signal(s,handler)
    if screen_capture and 'error' not in result:
        try:
            result['screen_gif']=str(screen_capture.gif(result['flight']['events']))
            result['screen_capture']=json.loads((out/'screen_capture.json').read_text())
        except Exception as error:result['error']='Actual Gazebo GIF capture failed: '+str(error)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workspace-root',type=Path,default=Path(os.environ.get('STACK_MPPI_WORKSPACE',str(Path(__file__).resolve().parents[2]))))
    parser.add_argument('--reproduce-child',action='store_true')
    args=parser.parse_args();workspace=args.workspace_root.resolve()
    out=args.output_root.resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');out.mkdir(parents=True)
    actual_root=Path(__file__).resolve().parents[2]
    sources=list((actual_root/'simulation/mppi_series').glob('*.py'))+list((actual_root/'offboard_control').rglob('*.py'))
    sources+=list((actual_root/'simulation/mppi_series/reference_inputs').glob('*.txt'))
    sources+=[actual_root/'offboard_control/package.xml',actual_root/'offboard_control/setup.cfg',actual_root/'offboard_control/resource/offboard_control']
    sources+=[actual_root/'simulation/scripts/experiment_provenance.py',actual_root/'simulation/config/headless_server.config']
    hashes={}
    for path in sources:
        if '__pycache__' in path.parts:continue
        relative=path.relative_to(actual_root);dest=out/'source'/relative;dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(path.read_bytes());hashes[str(relative)]=sha(dest)
    tests=subprocess.run([sys.executable,'-m','unittest','mppi_series.test_controller','mppi_series.test_ros_interface','-v'],capture_output=True,text=True)
    (out/'tests.log').write_text(tests.stdout+tests.stderr)
    facts={'passed':False,'scope':'live_ros_px4_interface','episode':'10-1',
           'flight_verified':False,'hardware_used':False,'mppi_applied_verified':False}
    chapter_7=None
    try:
        if tests.returncode:raise RuntimeError('Meaningful model/interface checks failed; inspect tests.log')
        from .chapter_7_checks import verify
        chapter_7=verify(out)
        # Share the baseline experiment's lock, as both use PX4 instance 7.
        lock_root=workspace/'simulation/experiment_logs';lock_root.mkdir(parents=True,exist_ok=True)
        with (lock_root/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            facts=execute(out,workspace)
    except Exception as error:
        facts['error']=str(error)
    for name,digest in hashes.items():
        if sha(actual_root/name)!=digest:facts['error']='Source changed during actual execution: '+name
    facts.update(source_sha256=hashes,checked_utc=datetime.now(timezone.utc).isoformat(),
                 chapter_7_checks=chapter_7,tests_returncode=tests.returncode,
                 verification_commands=[{'command':[sys.executable,'-m','unittest',
                     'mppi_series.test_controller','mppi_series.test_ros_interface','-v'],'log':'tests.log'},
                     {'python_callable':'mppi_series.chapter_7_checks.verify','report':'chapter_7_checks.json'}],
                 evidence_files=[str(p) for p in [*out.glob('rosbag/*.mcap'),*out.glob('runtime/log/**/*.ulg')]],
                 commands=json.loads((out/'commands.json').read_text()) if (out/'commands.json').exists() else [],
                 limitations=['Unarmed Gazebo/PX4 reception; no flight or plant identification.',
                              'Linear thrust/rate-lag prediction remains an assumption.',
                              'This bounded node is not a hardware flight supervisor.'])
    from dataclasses import asdict
    from .controller import Config
    facts.setdefault('config',facts.get('interface',{}).get('config',asdict(Config(temperature=3.,seed=7))))
    facts.setdefault('seeds',[facts['config']['seed']])
    facts.setdefault('units',{'position':'m','velocity':'m/s','body_rates':'rad/s','thrust':'normalized [0,1]'})
    facts.setdefault('frames',{'pose':'ENU map','odom_twist':'FLU base_link','PX4_rates':'FRD; converted once by MAVROS'})
    facts.setdefault('cases',[])
    if 'error' in facts:
        facts['cases'].append({'name':'live ROS/PX4 attempt','success':False,'failure':facts['error']})
    write(out/'result.json',facts)
    if 'error' not in facts:
        from .interface_results import finish
        facts=finish(out,facts,workspace,args.reproduce_child)
    print(json.dumps({'output':str(out),'passed':facts['passed']},indent=2),flush=True)
    return 0 if facts['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
