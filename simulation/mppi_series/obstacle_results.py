"""Independent actual ROS/ULog flight analysis; fixed evaluation windows."""
from datetime import datetime,timezone
import csv
import json
from pathlib import Path
import numpy as np
from .episode_10_1 import write
from .controller import Config
from .obstacles import ObstacleController,reference,horizon_reference,OBSTACLE,DURATION


def analyze(out,facts):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import AttitudeTarget
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from pyulog import ULog
    from .ros_interface import odometry_state
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    odometry={};published={};position_stamps=[];references={};velocities={}
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
            msg=deserialize_message(data,PoseStamped);p=msg.pose.position;references[stamp(msg)]=[p.x,p.y,p.z]
        elif name=='/mppi/reference_velocity':
            msg=deserialize_message(data,TwistStamped);v=msg.twist.linear;velocities[stamp(msg)]=[v.x,v.y,v.z]
    rows=np.atleast_1d(np.genfromtxt(out/'commands.csv',delimiter=',',names=True,dtype=None,encoding='utf-8'))
    if len(rows)<20:raise RuntimeError('Insufficient actual MPPI flight commands')
    names=['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r']
    cfg=Config(**facts['flight']['config']);replay=ObstacleController(cfg,facts['flight']['profile']);replay_errors=[]
    plans=[json.loads(line) for line in (out/'predicted_references.jsonl').read_text().splitlines()]
    if len(plans)!=len(rows):raise RuntimeError('Lost predicted reference records')
    epoch=facts['flight']['reference_epoch_ros_ns'];origin=facts['flight']['origin']
    for row,plan_record in zip(rows,plans):
        state=np.array([row[n] for n in names]);command=np.array([row[n] for n in ['c','p_sp','q_sp','r_sp']])
        stamp_ns=int(row['ros_ns']);goal=np.array([row[n] for n in ['goal_x','goal_y','goal_z']])
        if stamp_ns not in published or stamp_ns not in references:raise RuntimeError('Calculated command/reference absent from MCAP')
        if np.max(abs(command-published[stamp_ns]))>1e-6 or np.max(abs(goal-references[stamp_ns]))>1e-12:raise RuntimeError('Actual published command/reference differs from calculation')
        originals=odometry.get(int(row['odom_ros_ns']),[])
        if not originals or min(np.max(abs(state-v)) for v in originals)>1e-12:raise RuntimeError('Calculated state not backed by original Odometry')
        elapsed=(int(row['plan_ros_ns'])-epoch)/1e9
        times,plan=horizon_reference(elapsed,origin,cfg)
        if plan_record['ros_ns']!=stamp_ns or np.max(abs(plan-np.array(plan_record['position_velocity_enu'])))>1e-12 or np.max(abs(times-plan_record['times_s']))>1e-12:raise RuntimeError('Predicted horizon reference mismatch')
        now_ref=reference(elapsed,origin)
        if np.max(abs(now_ref[:3]-goal))>1e-12 or stamp_ns not in velocities or np.max(abs(now_ref[3:]-velocities[stamp_ns]))>1e-12:raise RuntimeError('Actual live position/velocity reference mismatch')
        error=float(np.max(abs(command-replay.step(state,plan))));replay_errors.append(error)
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
    if len(selected)<20 or ros[selected[-1]]-ros[selected[0]]<1:raise RuntimeError('Sufficient armed-airborne rate-Offboard evidence missing')
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
    if len(pairs)<20 or fraction<.95:raise RuntimeError('Clock-aligned FCU command receipt failed')
    with (out/'received_commands.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['received_ros_s','sent_ros_s','received_p','received_q','received_r','received_thrust_z',
                                            'sent_p','sent_q','sent_r','sent_thrust_z','error_p','error_q','error_r','error_c']);writer.writerows(pairs)
    # Evaluate each distinct original measurement at ITS ROS timestamp,
    # using the analytic reference there; never align to latest command time.
    measured=[]
    for stamp_ns,states in sorted(odometry.items()):
        elapsed=(stamp_ns-epoch)/1e9
        if not 0<=elapsed<30. or not first<=stamp_ns<=last:continue
        state=states[-1];goal=reference(elapsed,origin)
        measured.append([stamp_ns,elapsed,*state,*goal,*list(state[:3]-goal[:3]),*list(state[3:6]-goal[3:])])
    data=np.array(measured)
    if len(data)<20:raise RuntimeError('Insufficient distinct reference-aligned Odometry measurements')
    with (out/'aligned_measurements.csv').open('w') as handle:
        w=csv.writer(handle);w.writerow(['odom_ros_ns','elapsed_s',*names,'ref_x','ref_y','ref_z','ref_vx','ref_vy','ref_vz',
                                        'error_x','error_y','error_z','error_vx','error_vy','error_vz']);w.writerows(measured)
    error=np.linalg.norm(data[:,20:23],axis=1)
    velocity_error=np.linalg.norm(data[:,23:26],axis=1)
    normal=(data[:,1]>=7.)&(data[:,1]<27.)
    if sum(normal)==0:normal=np.ones(len(data),dtype=bool)
    rmse=float(np.sqrt(np.mean(error[normal]**2)))
    period=np.diff(rows['ros_ns'])/1e9
    case={'name':facts['flight']['profile'],'kind':'actual_gazebo_obstacle_trial','success':False,
          'criterion':'Position RMSE <0.12m on fixed [7,27)s after handover; chosen before flight.',
          'samples':len(data),'normal_samples':int(sum(normal)),
          'horizon_steps':cfg.horizon,'horizon_s':cfg.horizon*cfg.dt,
          'rmse_all_m':float(np.sqrt(np.mean(error**2))), 'rmse_normal_m':rmse,
          'velocity_rmse_m_s':float(np.sqrt(np.mean(velocity_error[normal]**2))),
          'max_position_error_m':float(np.max(error)),
          'compute_mean_ms':float(np.mean(rows['compute_ms'])),'compute_max_ms':float(np.max(rows['compute_ms'])),
          'compute_overrun_count':int(sum(rows['compute_ms']>50)),
          'publish_period_mean_s':float(np.mean(period)),'publish_period_max_s':float(np.max(period)),
          'publish_period_over_60ms_count':int(sum(period>.06))}
    from .obstacle_checks import analyze_geometry
    geometry=analyze_geometry(out,facts,data,first,last)
    case.update(geometry)
    case['success']=geometry['success'] and facts['flight']['passed']
    case['completed_full_task']=facts['flight']['passed']
    case['execution_failure']=facts['flight'].get('error')
    case['evaluated_duration_s']=float(data[-1,1]-data[0,1])
    case['criterion']='No actual contacts, minimum conservative clearance >=0.10m, final goal error <0.25m; fixed before flight.'
    report={'passed':True,'cases':[case],'normal_rule':'Available measurements in fixed [7,27)s; if aborted before 7s, whole available interval, explicitly labelled; no error filtering.',
            'fixed_window_complete':bool(data[-1,1]>=27.),'partial_flight':not facts['flight']['passed'],
            'evaluation_scope':'Unique original Odometry timestamps during MPPI rate-Offboard only; analytic position/velocity reference at measurement timestamp.',
            'mppi_commands':len(rows),'clock_aligned_pairs':len(pairs),'command_match_fraction':fraction,
            'mppi_airborne_duration_s':float(ros[selected[-1]]-ros[selected[0]]),
            'position_commands_during_mppi':position_overlap,'independent_mppi_replay_max_error':max(replay_errors),
            'clock_alignment':'ROS us = PX4 timestamp us - forward-held measured MAVLink estimated_offset us.',
            'receipt_rule':'Value consistency within +/-100ms, >=95% under 5e-5; not one-to-one latency.',
            'flight_verified':True,'hardware_verified':False}
    write(out/'analysis.json',report)
    return report,data
