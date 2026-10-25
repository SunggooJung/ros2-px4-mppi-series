"""Independent actual ROS/ULog flight analysis; fixed evaluation windows."""
from datetime import datetime,timezone
import csv
from pathlib import Path
import numpy as np
from .episode_10_1 import write
from .controller import Config,MPPIController


def analyze(out,facts):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import AttitudeTarget
    from geometry_msgs.msg import PoseStamped
    from pyulog import ULog
    from .ros_interface import odometry_state
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    odometry={};published={};position_stamps=[];reference={}
    def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    while reader.has_next():
        name,data,_=reader.read_next()
        if name.endswith('/local_position/odom'):
            msg=deserialize_message(data,Odometry);odometry.setdefault(stamp(msg),[]).append(odometry_state(msg))
        elif name.endswith('/setpoint_raw/attitude'):
            msg=deserialize_message(data,AttitudeTarget)
            if msg.type_mask!=128:raise RuntimeError('Incorrect actual rate mask')
            published[stamp(msg)]=[msg.thrust,msg.body_rate.x,msg.body_rate.y,msg.body_rate.z]
        elif name.endswith('/setpoint_position/local'):
            position_stamps.append(stamp(deserialize_message(data,PoseStamped)))
        elif name=='/mppi/reference':
            msg=deserialize_message(data,PoseStamped);p=msg.pose.position;reference[stamp(msg)]=[p.x,p.y,p.z]
    rows=np.atleast_1d(np.genfromtxt(out/'commands.csv',delimiter=',',names=True,dtype=None,encoding='utf-8'))
    if len(rows)<400:raise RuntimeError('Insufficient actual MPPI flight commands')
    names=['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r']
    replay=MPPIController(Config(**facts['flight']['config']));replay_errors=[]
    for row in rows:
        state=np.array([row[n] for n in names]);command=np.array([row[n] for n in ['c','p_sp','q_sp','r_sp']])
        stamp_ns=int(row['ros_ns']);goal=np.array([row[n] for n in ['goal_x','goal_y','goal_z']])
        if stamp_ns not in published or stamp_ns not in reference:raise RuntimeError('Calculated command/reference absent from MCAP')
        if np.max(abs(command-published[stamp_ns]))>1e-6 or np.max(abs(goal-reference[stamp_ns]))>1e-12:raise RuntimeError('Actual published command/reference differs from calculation')
        originals=odometry.get(int(row['odom_ros_ns']),[])
        if not originals or min(np.max(abs(state-v)) for v in originals)>1e-12:raise RuntimeError('Calculated state not backed by original Odometry')
        error=float(np.max(abs(command-replay.step(state,goal))));replay_errors.append(error)
        if error>1e-12:raise RuntimeError('Actual command differs from deterministic MPPI replay')
    first=int(rows[0]['ros_ns']);last=int(rows[-1]['ros_ns'])
    position_overlap=sum(first<=t<=last for t in position_stamps)
    if position_overlap:raise RuntimeError('PX4 position commands overlap MPPI control interval')
    logs=list(out.glob('runtime/log/**/*.ulg'))
    if len(logs)!=1:raise RuntimeError('One original ULog required')
    log=ULog(str(logs[0]));mode=log.get_dataset('vehicle_control_mode').data
    rates=log.get_dataset('vehicle_rates_setpoint').data;status=log.get_dataset('vehicle_status').data;land=log.get_dataset('vehicle_land_detected').data
    sync=log.get_dataset('timesync_status').data
    valid=(sync['source_protocol']==0)&(sync['estimated_offset']<-1e12)
    times=sync['timestamp'][valid];offset=sync['estimated_offset'][valid]
    if len(times)<3:raise RuntimeError('Measured clock offset required')
    rt=rates['timestamp'];indices=[np.searchsorted(v,rt,side='right')-1 for v in [mode['timestamp'],status['timestamp'],land['timestamp'],times]]
    m,s,l,t=[np.maximum(i,0) for i in indices];known=np.all(np.array(indices)>=0,axis=0)
    ros=(rt.astype(float)-offset[t])/1e6
    mask=known&(ros>=first/1e9)&(ros<=last/1e9)
    mask&=mode['flag_control_offboard_enabled'][m].astype(bool)&mode['flag_control_rates_enabled'][m].astype(bool)
    mask&=~mode['flag_control_position_enabled'][m].astype(bool)&~mode['flag_control_attitude_enabled'][m].astype(bool)
    mask&=(status['arming_state'][s]==2)&~land['landed'][l].astype(bool)
    selected=np.flatnonzero(mask)
    if len(selected)<200 or ros[selected[-1]]-ros[selected[0]]<25:raise RuntimeError('Sufficient armed-airborne rate-Offboard evidence missing')
    sent=np.column_stack([rows['ros_ns']/1e9,rows['p_sp'],-rows['q_sp'],-rows['r_sp'],-rows['c']])
    actual=np.column_stack([rates['roll'],rates['pitch'],rates['yaw'],rates['thrust_body[2]']])
    pairs=[]
    for i in selected:
        candidates=np.flatnonzero(abs(sent[:,0]-ros[i])<=.1)
        if not len(candidates):continue
        errs=np.max(abs(sent[candidates,1:]-actual[i]),axis=1);idx=candidates[np.argmin(errs)]
        pairs.append([ros[i],sent[idx,0],*actual[i],*sent[idx,1:],*list(actual[i]-sent[idx,1:])])
    pairs=np.array(pairs)
    fraction=float(np.mean(np.max(abs(pairs[:,10:14]),axis=1)<5e-5)) if len(pairs) else 0
    if len(pairs)<200 or fraction<.95:raise RuntimeError('Clock-aligned FCU command receipt failed')
    with (out/'received_commands.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['received_ros_s','sent_ros_s','received_p','received_q','received_r','received_thrust_z',
                                            'sent_p','sent_q','sent_r','sent_thrust_z','error_p','error_q','error_r','error_c']);writer.writerows(pairs)
    error=np.sqrt(sum((rows[n]-rows['goal_'+n])**2 for n in ['x','y','z']))
    normal=np.zeros(len(rows),dtype=bool);cases=[]
    for name in ['hover','move_x','return']:
        phase=rows['phase']==name;ts=rows['ros_ns']/1e9
        start=next(e['ros_ns']/1e9 for e in facts['flight']['events'] if e['event']=='target_change' and e['phase']==name)
        settle=phase&(ts>=start+6.)&(ts<start+10.)
        if sum(settle)<30:raise RuntimeError('Fixed normal interval lacks samples')
        normal|=settle
        final=float(error[phase][-1]);case={'name':name,'success':final<.35 and float(np.sqrt(np.mean(error[settle]**2)))<.4,
            'samples':int(sum(phase)),'normal_samples':int(sum(settle)),
            'rmse_all_m':float(np.sqrt(np.mean(error[phase]**2))),
            'rmse_normal_m':float(np.sqrt(np.mean(error[settle]**2))),'final_position_error_m':final}
        cases.append(case)
    report={'passed':all(c['success'] for c in cases),'cases':cases,
            'rmse_all_m':float(np.sqrt(np.mean(error**2))),'rmse_normal_m':float(np.sqrt(np.mean(error[normal]**2))),
            'normal_rule':'For each planned 10s goal phase, [target_change+6s, target_change+10s); fixed before execution; no error filtering.',
            'evaluation_scope':'MPPI rate-Offboard only; PX4 takeoff/calibration/landing excluded.',
            'mppi_commands':len(rows),'clock_aligned_pairs':len(pairs),'command_match_fraction':fraction,
            'mppi_airborne_duration_s':float(ros[selected[-1]]-ros[selected[0]]),
            'position_commands_during_mppi':position_overlap,'independent_mppi_replay_max_error':max(replay_errors),
            'compute_mean_ms':float(np.mean(rows['compute_ms'])),'compute_max_ms':float(np.max(rows['compute_ms'])),
            'max_tilt_rad':float(np.max(abs(np.column_stack([rows['roll'],rows['pitch']])))),
            'max_position_error_m':float(np.max(error)),
            'clock_alignment':'ROS us = PX4 ULog timestamp us - forward-held MAVLink timesync_status estimated_offset us.',
            'receipt_rule':'Minimum max-component error among sent samples within +/-100ms; >=95% below 5e-5; value/time-window consistency, not one-to-one latency.',
            'flight_verified':True,'hardware_verified':False}
    write(out/'analysis.json',report)
    if not report['passed']:raise RuntimeError('Preselected tracking limits failed; inspect analysis.json')
    return report,rows


def finish(out,facts,workspace,reproduce_child):
    try:
        analysis,rows=analyze(out,facts)
        facts.update(analysis=analysis,flight_verified=True,mppi_applied_verified=True,
                     fcu_received_verified=True,cases=analysis['cases'],passed=True)
        write(out/'result.json',facts)
        if not reproduce_child:
            from .flight_article import build
            build(out,facts,rows,workspace)
        facts['checked_utc']=datetime.now(timezone.utc).isoformat();write(out/'result.json',facts)
    except Exception as error:
        facts.update(passed=False,analysis_error=str(error));write(out/'result.json',facts)
    return facts
