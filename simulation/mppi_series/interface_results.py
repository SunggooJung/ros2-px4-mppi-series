"""Independent persisted ROS/ULog comparison and reproducible 10-1 materials."""
from datetime import datetime,timezone
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile
import numpy as np
from .episode_10_1 import sha,write


def analyze(out,facts):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from mavros_msgs.msg import AttitudeTarget
    from nav_msgs.msg import Odometry
    from pyulog import ULog
    from .ros_interface import odometry_state
    from .controller import Config, MPPIController
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    commands=[];odom_states={};sent_by_stamp={};raw_odom=[]
    while reader.has_next():
        name,raw,_=reader.read_next()
        if name.endswith('/local_position/odom'):
            msg=deserialize_message(raw,Odometry)
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            state=odometry_state(msg)
            odom_states.setdefault(stamp,[]).append(state)
            raw_odom.append([stamp,_,*state])
        if name.endswith('/setpoint_raw/attitude'):
            msg=deserialize_message(raw,AttitudeTarget)
            if msg.type_mask!=128:raise RuntimeError('Unexpected command mask')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            commands.append([stamp,msg.body_rate.x,-msg.body_rate.y,-msg.body_rate.z,-msg.thrust])
            stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            sent_by_stamp[stamp_ns]=np.array([msg.thrust,msg.body_rate.x,msg.body_rate.y,msg.body_rate.z])
    command=np.array(commands)
    odom_count=len(raw_odom)
    if odom_count<20 or len(command)<100:raise RuntimeError('Insufficient persisted live state/commands')
    with (out/'live_odometry.csv').open('w') as file:
        writer=csv.writer(file)
        writer.writerow(['odom_ros_ns','bag_receive_ns','x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r'])
        writer.writerows(raw_odom)
    rows=np.atleast_1d(np.genfromtxt(out/'commands.csv',delimiter=',',names=True,dtype=None,encoding='utf-8'))
    state_names=['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r']
    config=Config(**facts['interface']['config']);replay=MPPIController(config)
    state_errors=[];published_errors=[];replay_errors=[];replay_rows=[]
    for row in rows:
        state=np.array([row[n] for n in state_names],dtype=float)
        candidates=odom_states.get(int(row['odom_ros_ns']),[])
        if not candidates:raise RuntimeError('Calculated state has no original Odometry in MCAP')
        state_error=min(float(np.max(abs(s-state))) for s in candidates)
        original=np.array([row[n] for n in ['c','p_sp','q_sp','r_sp']])
        published=sent_by_stamp.get(int(row['ros_ns']))
        if published is None:raise RuntimeError('Calculated command has no actual AttitudeTarget in MCAP')
        goal=np.array([row[n] for n in ['goal_x','goal_y','goal_z']])
        replayed=replay.step(state,goal)
        publication_error=float(np.max(abs(published-original)))
        replay_error=float(np.max(abs(replayed-original)))
        state_errors.append(state_error);published_errors.append(publication_error);replay_errors.append(replay_error)
        replay_rows.append([row['ros_ns'],row['odom_ros_ns'],state_error,publication_error,replay_error,*original,*published])
    with (out/'calculation_message_alignment.csv').open('w') as file:
        writer=csv.writer(file)
        writer.writerow(['command_ros_ns','odom_ros_ns','state_error','publication_error','replay_error',
                         'calculated_c','calculated_p','calculated_q','calculated_r',
                         'published_c','published_p','published_q','published_r'])
        writer.writerows(replay_rows)
    if max(state_errors)>1e-12 or max(replay_errors)>1e-12 or max(published_errors)>1e-6:
        raise RuntimeError('Live state/calculation/published-message correspondence failed')
    logs=list(out.glob('runtime/log/**/*.ulg'))
    if len(logs)!=1:raise RuntimeError('Expected one complete original ULog')
    log=ULog(str(logs[0]),message_name_filter_list=['vehicle_rates_setpoint','vehicle_control_mode','vehicle_status','vehicle_land_detected','timesync_status','failsafe_flags'])
    mode=log.get_dataset('vehicle_control_mode').data
    rates=log.get_dataset('vehicle_rates_setpoint').data
    status=log.get_dataset('vehicle_status').data;land=log.get_dataset('vehicle_land_detected').data
    sync=log.get_dataset('timesync_status').data
    # Source 0 is MAVLink. PX4 estimated_offset is (FCU boot time - ROS time)
    # in microseconds. It is a measured clock offset, not a best-fit shift.
    sync_mask=(sync['source_protocol']==0)&(sync['estimated_offset']<-1e12)
    sync_t=sync['timestamp'][sync_mask];sync_offset=sync['estimated_offset'][sync_mask]
    if len(sync_t)<3:raise RuntimeError('Measured MAVLink time synchronization missing')
    time_us=rates['timestamp'];mi=np.searchsorted(mode['timestamp'],time_us,side='right')-1
    si=np.searchsorted(status['timestamp'],time_us,side='right')-1
    li=np.searchsorted(land['timestamp'],time_us,side='right')-1
    ti=np.searchsorted(sync_t,time_us,side='right')-1
    known=(mi>=0)&(si>=0)&(li>=0)&(ti>=0)
    m=np.maximum(mi,0);s=np.maximum(si,0);l=np.maximum(li,0);t=np.maximum(ti,0)
    mask=known&mode['flag_control_offboard_enabled'][m].astype(bool)&mode['flag_control_rates_enabled'][m].astype(bool)
    mask&=~mode['flag_control_position_enabled'][m].astype(bool)&~mode['flag_control_attitude_enabled'][m].astype(bool)
    if np.any(status['arming_state']==2) or np.any(~land['landed'].astype(bool)):
        raise RuntimeError('Unarmed interface experiment unexpectedly armed or airborne')
    selected=np.flatnonzero(mask)
    if len(selected)<100:raise RuntimeError('No sufficient FCU rate-Offboard reception')
    rx_time=(time_us[selected].astype(float)-sync_offset[t[selected]])/1e6
    rx=np.column_stack([rates['roll'][selected],rates['pitch'][selected],rates['yaw'][selected],rates['thrust_body[2]'][selected]])
    comparisons=[]
    for timestamp,value in zip(rx_time,rx):
        candidates=np.flatnonzero(np.abs(command[:,0]-timestamp)<=.1)
        if not len(candidates):continue
        error=np.max(np.abs(command[candidates,1:]-value),axis=1)
        match=candidates[int(np.argmin(error))]
        comparisons.append([timestamp,command[match,0],*(value-command[match,1:]),*value,*command[match,1:]])
    pairs=np.array(comparisons)
    if len(pairs)<100:raise RuntimeError('Clock-aligned transmission/reception pairs missing')
    matched=np.max(np.abs(pairs[:,2:6]),axis=1)<5e-5
    fraction=float(np.mean(matched))
    if fraction<.95:raise RuntimeError('FCU command match below preselected 95% rule: '+str(fraction))
    with (out/'received_commands.csv').open('w') as file:
        writer=csv.writer(file);writer.writerow(['received_ros_s','sent_ros_s','error_p','error_q','error_r','error_thrust_z',
                                                'received_p_frd','received_q_frd','received_r_frd','received_thrust_z',
                                                'sent_p_frd','sent_q_frd','sent_r_frd','sent_thrust_z']);writer.writerows(pairs)
    events=facts['interface']['events']
    entered=next(e['ros_ns'] for e in events if e['event']=='rate_offboard_observed')/1e9
    injected=next(e['ros_ns'] for e in events if e['event']=='inject_stale_state')/1e9
    blocked=next(e['ros_ns'] for e in events if e['event']=='command_blocked')/1e9
    after=int(np.sum(command[:,0]>blocked+1e-5))
    if after:raise RuntimeError('Commands continued after stale-state block')
    flags=log.get_dataset('failsafe_flags').data
    flags_ros=(flags['timestamp'].astype(float)-sync_offset[-1])/1e6
    loss=flags['offboard_control_signal_lost'][(flags_ros>=blocked+.5)&(flags_ros<=blocked+3.2)]
    if not len(loss) or not np.any(loss):raise RuntimeError('PX4 did not record stream loss after stale-state block')
    position=np.column_stack([rows[n] for n in ['x','y','z']]);reference=np.column_stack([rows[n] for n in ['goal_x','goal_y','goal_z']])
    error=np.linalg.norm(position-reference,axis=1);time_s=rows['ros_ns']/1e9
    normal=(time_s>=entered+1)&(time_s<injected)
    report={'passed':True,'odom_messages':odom_count,'published_commands_in_bag':len(command),
            'calculated_commands_verified_against_bag':len(rows),
            'calculated_state_max_error':max(state_errors),
            'calculated_published_max_component_error':max(published_errors),
            'independent_mppi_replay_max_error':max(replay_errors),
            'fcu_rate_offboard_samples':len(selected),'clock_aligned_pairs':len(pairs),
            'clock_alignment':'PX4 timesync_status source_protocol=0; ROS us = ULog timestamp us - estimated_offset us; forward-held last measured offset',
            'match_rule':'For each received sample select minimum component error among transmitted stamps within +/-100ms; >=95% pairs must have max component error <5e-5. Fixed before running.',
            'command_match_fraction':fraction,'max_component_error':np.max(np.abs(pairs[:,2:6]),axis=0).tolist(),
            'rate_offboard_duration_s':float((time_us[selected[-1]]-time_us[selected[0]])/1e6),
            'armed_samples':int(np.sum(status['arming_state']==2)),'airborne_samples':int(np.sum(~land['landed'].astype(bool))),
            'commands_after_stale_block':after,'state_age_max_s':float(np.max(rows['state_age_s'])),
            'px4_offboard_signal_loss_verified':True,'mode_after_stream_stop':facts['interface']['mode_after_stream_stop'],
            'compute_mean_ms':float(np.mean(rows['compute_ms'])),'compute_max_ms':float(np.max(rows['compute_ms'])),
            'compute_over_50ms_count':int(np.sum(rows['compute_ms']>50)),
            'ground_reference_position_rmse_all_m':float(np.sqrt(np.mean(error**2))),
            'ground_reference_position_rmse_normal_m':float(np.sqrt(np.mean(error[normal]**2))),
            'normal_rule':'Offboard observation +1s until stale-state injection, selected by events before any error filtering.',
            'position_rmse_scope':'Unarmed stationary vehicle versus requested positions. A recorded reference difference, NOT flight tracking performance.',
            'flight_verified':False,'clock_offset_span_us':float(np.ptp(sync_offset))}
    intervals=np.diff(rows['ros_ns'].astype(float))/1e9
    live=np.asarray(raw_odom)
    report.update(config=facts['interface']['config'],
                  prediction_dt_s=config.dt,target_compute_period_s=config.dt,target_publish_period_s=config.dt,
                  publish_interval_mean_s=float(np.mean(intervals)),
                  publish_interval_max_s=float(np.max(intervals)),
                  ground_position_max_excursion_m=float(np.max(np.linalg.norm(live[:,2:5]-live[0,2:5],axis=1))),
                  measured_ground_body_rate_max_rad_s=float(np.max(np.abs(live[:,11:14]))),
                  measured_response_scope='Unarmed ground Odometry retained; no actuated response or flight claim',
                  stale_injection_to_block_s=blocked-injected)
    write(out/'analysis.json',report)
    return report,rows,pairs


def materials(out,facts,analysis,rows,pairs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    origin=float(pairs[0,0]);fig,axes=plt.subplots(4,1,figsize=(10,8),sharex=True)
    for i,label in enumerate(['p FRD [rad/s]','q FRD [rad/s]','r FRD [rad/s]','thrust body Z [1]']):
        axes[i].plot(pairs[:,0]-origin,pairs[:,6+i],label='PX4 ULog received',linewidth=1)
        axes[i].plot(pairs[:,0]-origin,pairs[:,10+i],label='MAVROS sent (converted)',linewidth=.8,alpha=.65)
        axes[i].set_ylabel(label);axes[i].grid(alpha=.2)
    axes[0].legend();axes[-1].set_xlabel('Time in rate-Offboard [s]');fig.tight_layout()
    fig.savefig(out/'interface_commands.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(10,7))
    t=rows['monotonic_s']-rows['monotonic_s'][0]
    axes[0].plot(t,rows['compute_ms']);axes[0].axhline(50,color='r',linestyle='--',label='50 ms budget')
    axes[0].set_ylabel('MPPI compute [ms]');axes[0].legend();axes[0].grid(alpha=.2)
    axes[1].plot(pairs[:,0]-origin,np.max(np.abs(pairs[:,2:6]),axis=1));axes[1].set_ylabel('Max command component error')
    axes[1].set_xlabel('Time [s]');axes[1].grid(alpha=.2);fig.tight_layout()
    fig.savefig(out/'interface_timing.png',dpi=150);plt.close(fig)


def finish(out,facts,workspace,reproduce_child):
    try:
        analysis,rows,pairs=analyze(out,facts)
        materials(out,facts,analysis,rows,pairs)
        facts.update(passed=bool(reproduce_child),analysis=analysis,fcu_received_verified=True,
                     phase='verified_live_interface' if reproduce_child else 'preparing_verified_bundle',
                     config=facts['interface']['config'],seeds=[facts['interface']['config']['seed']],
                     mppi_applied_verified=False,
                     cases=[{'name':'live MPPI state-to-FCU interface','success':True},
                            {'name':'injected stale state','success':False,'verification_passed':True,
                             'expected_failure':True,'command_rejected':True,'px4_signal_loss_detected':True,
                             'failure':'Odometry callback deliberately frozen; stale commands blocked'}],
                     units={'position':'m','velocity':'m/s','body_rates':'rad/s','thrust':'normalized [0,1]'},
                     frames={'pose':'ENU map','odom_twist':'FLU base_link','PX4_rates':'FRD; converted once by MAVROS'})
        write(out/'result.json',facts)
        if not reproduce_child:
            from .interface_article import build
            build(out,facts,workspace)
        facts['passed']=True
        facts['phase']='verified_live_interface' if reproduce_child else 'complete'
        facts['checked_utc']=datetime.now(timezone.utc).isoformat()
        facts['evidence_files']=[str(p.resolve()) for p in sorted(out.rglob('*'))
                                 if p.is_file() and p.suffix in {'.mcap','.ulg','.csv'}]
        write(out/'result.json',facts)
    except Exception as error:
        facts.update(passed=False,analysis_error=str(error));write(out/'result.json',facts)
    return facts
