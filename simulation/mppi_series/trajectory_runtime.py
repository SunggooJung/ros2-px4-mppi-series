"""11-1 owned live PX4/Gazebo runtime; reuse established ports/server/capture."""
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


def execute(out,workspace,chapter='11-1',capture_gif=False,profile='long_horizon'):
    if chapter != '11-1':raise ValueError('Unsupported experiment chapter')
    flight=True
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
    result={'passed':False,'scope':'gazebo_trajectory_tracking','episode':chapter,
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
            class RecordedCapture(GazeboCapture):
                def launch(self,command,env,log):
                    commands.append({'command':command,'log':log,'capture':True,'launched_utc':datetime.now(timezone.utc).isoformat()})
                    write(out/'commands.json',commands)
                    return super().launch(command,env,log)
            screen_capture=RecordedCapture(out,workspace,name)
        gcs=mavutil.mavlink_connection('udpout:127.0.0.1:18577',source_system=254,source_component=190)
        mavros=launch(['ros2','launch','mavros','px4.launch','fcu_url:=udp://127.0.0.1:14547@127.0.0.1:14587',
                       'tgt_system:=8','namespace:=/sitl_check/mavros'],'mavros.log')
        topics=['/sitl_check/mavros/'+t for t in ['state','extended_state','local_position/odom',
                'local_position/pose','local_position/velocity_local','setpoint_raw/attitude','timesync_status']]+['/mppi/reference','/mppi/reference_velocity']
        if flight:topics+=['/sitl_check/mavros/setpoint_position/local','/sitl_check/mavros/setpoint_raw/target_attitude']
        bag=launch(['ros2','bag','record','-s','mcap','-o',str(out/'rosbag'),'--topics',*topics],'rosbag.log')
        # Exercise the installed ros2 launch entry point, not an uninstalled
        # module that merely resembles the published package instructions.
        if flight:write(out/'sitl_owned.json',{'px4_instance':7,'system_id':8,'hardware_used':False,'container':name,'image_id':image})
        command=[sys.executable,'-m','mppi_series.trajectory_flight','--output',str(out),'--profile',profile]
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
            metadata=json.loads((out/'screen_capture.json').read_text())
            from PIL import Image
            with Image.open(result['screen_gif']) as gif:
                duration=0
                for index in range(gif.n_frames):
                    gif.seek(index);duration+=gif.info.get('duration',0)
            metadata.update(original_recording=str(out/'gazebo_screen.mp4'),original_sha256=sha(out/'gazebo_screen.mp4'),
                            gif_duration_ms=duration,capture_end_utc=datetime.now(timezone.utc).isoformat(),
                            run_output=str(out),capture_commands=[c for c in commands if c.get('capture')])
            write(out/'screen_capture.json',metadata)
            result['screen_capture']=metadata
        except Exception as error:result['error']='Actual Gazebo GIF capture failed: '+str(error)
    return result
