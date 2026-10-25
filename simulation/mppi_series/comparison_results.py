"""12-1 independent baseline receipt, aligned error and explicit-unit input metrics."""
import csv
import json
from pathlib import Path
import numpy as np
from .episode_10_1 import write
from .trajectory import reference

NAMES=['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r']


def error_metrics(data):
    error=np.linalg.norm(data[:,20:23],axis=1)
    velocity=np.linalg.norm(data[:,23:26],axis=1)
    window=(data[:,1]>=7)&(data[:,1]<27)
    if window.sum()<200 or data[window,1].max()-data[window,1].min()<19.8:
        raise RuntimeError('Fixed [7,27)s evaluation window incomplete')
    return dict(rmse_all_m=float(np.sqrt(np.mean(error**2))),
                rmse_normal_m=float(np.sqrt(np.mean(error[window]**2))),
                velocity_rmse_m_s=float(np.sqrt(np.mean(velocity[window]**2))),
                max_position_error_m=float(error.max()),max_normal_error_m=float(error[window].max()),
                samples=len(data),normal_samples=int(window.sum()))


def aligned_clock(log,stamps):
    sync=log.get_dataset('timesync_status').data
    valid=(sync['source_protocol']==0)&(sync['estimated_offset']<-1e12)
    times=sync['timestamp'][valid];offset=sync['estimated_offset'][valid]
    if len(times)<3:raise RuntimeError('Measured MAVLink clock offset required')
    indices=np.searchsorted(times,stamps,side='right')-1
    return (stamps.astype(float)-offset[np.maximum(indices,0)])/1e6,indices>=0


def baseline(out,facts):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from .ros_interface import odometry_state
    from pyulog import ULog
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    odometry={};published={};refs={};vels={};rate_count=0
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    while reader.has_next():
        name,raw,_=reader.read_next()
        if name.endswith('/local_position/odom'):
            m=deserialize_message(raw,Odometry);odometry.setdefault(stamp(m),[]).append(odometry_state(m))
        elif name.endswith('/setpoint_position/local') or name=='/mppi/reference':
            m=deserialize_message(raw,PoseStamped);p=m.pose.position
            (published if name.endswith('/setpoint_position/local') else refs)[stamp(m)]=[p.x,p.y,p.z]
        elif name=='/mppi/reference_velocity':
            m=deserialize_message(raw,TwistStamped);v=m.twist.linear;vels[stamp(m)]=[v.x,v.y,v.z]
        elif name.endswith('/setpoint_raw/attitude'):rate_count+=1
    if rate_count:raise RuntimeError('Baseline unexpectedly publishes external rate commands')
    rows=np.atleast_1d(np.genfromtxt(out/'commands.csv',delimiter=',',names=True,dtype=None,encoding='utf-8'))
    if len(rows)<400:raise RuntimeError('Insufficient baseline commands')
    epoch=facts['flight']['reference_epoch_ros_ns'];origin=facts['flight']['origin']
    for row in rows:
        ns=int(row['ros_ns']);state=np.array([row[n] for n in NAMES])
        goal=reference((int(row['plan_ros_ns'])-epoch)/1e9,origin)
        for records,expected in [(published,goal[:3]),(refs,goal[:3]),(vels,goal[3:])]:
            if ns not in records or np.max(abs(expected-records[ns]))>1e-12:
                raise RuntimeError('Baseline actual ROS position/reference mismatch')
        originals=odometry.get(int(row['odom_ros_ns']),[])
        if not originals or min(np.max(abs(state-v)) for v in originals)>1e-12:
            raise RuntimeError('Baseline state lacks original Odometry')
    paths=list(out.glob('runtime/log/**/*.ulg'))
    if len(paths)!=1:raise RuntimeError('One original baseline ULog required')
    log=ULog(str(paths[0]));sp=log.get_dataset('trajectory_setpoint').data
    stamps=sp['timestamp'];ros,known=aligned_clock(log,stamps)
    mode=log.get_dataset('vehicle_control_mode').data
    status=log.get_dataset('vehicle_status').data;land=log.get_dataset('vehicle_land_detected').data
    ids=[np.searchsorted(d['timestamp'],stamps,side='right')-1 for d in [mode,status,land]]
    m,s,l=[np.maximum(i,0) for i in ids]
    mask=known&np.all(np.array(ids)>=0,axis=0)&(ros>=rows[0]['ros_ns']/1e9)&(ros<=rows[-1]['ros_ns']/1e9)
    for flag in ['flag_control_offboard_enabled','flag_control_position_enabled','flag_control_attitude_enabled','flag_control_rates_enabled']:
        mask &=mode[flag][m].astype(bool)
    mask&=(status['arming_state'][s]==2)&~land['landed'][l].astype(bool)
    selected=np.flatnonzero(mask)
    # PX4 default trajectory_setpoint logging is 5Hz (~150/30s), not rate-setpoint 20Hz.
    if len(selected)<100 or ros[selected[-1]]-ros[selected[0]]<25:
        raise RuntimeError('Baseline airborne position-Offboard interval missing')
    sent=np.column_stack([rows['ros_ns']/1e9,rows['goal_y'],rows['goal_x'],-rows['goal_z']])
    actual=np.column_stack([sp['position[0]'],sp['position[1]'],sp['position[2]']])
    pairs=[]
    for i in selected:
        candidates=np.flatnonzero(abs(sent[:,0]-ros[i])<=.1)
        if not len(candidates):continue
        errors=np.max(abs(sent[candidates,1:]-actual[i]),axis=1);j=candidates[np.argmin(errors)]
        pairs.append([ros[i],sent[j,0],*actual[i],*sent[j,1:],*list(actual[i]-sent[j,1:])])
    pairs=np.array(pairs)
    fraction=float(np.mean(np.max(abs(pairs[:,8:11]),axis=1)<5e-5)) if len(pairs) else 0
    if len(pairs)<100 or fraction<.95:raise RuntimeError('Baseline ENU->NED FCU receipt mismatch')
    with (out/'received_positions.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['received_ros_s','sent_ros_s','received_n','received_e','received_d',
                                  'sent_n','sent_e','sent_d','error_n','error_e','error_d']);w.writerows(pairs)
    measured=[]
    for ns,states in sorted(odometry.items()):
        elapsed=(ns-epoch)/1e9
        if not 0<=elapsed<30:continue
        state=states[-1];goal=reference(elapsed,origin)
        measured.append([ns,elapsed,*state,*goal,*list(state[:3]-goal[:3]),*list(state[3:6]-goal[3:])])
    data=np.array(measured)
    with (out/'aligned_measurements.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['odom_ros_ns','elapsed_s',*NAMES,'ref_x','ref_y','ref_z','ref_vx','ref_vy','ref_vz',
                                  'error_x','error_y','error_z','error_vx','error_vy','error_vz']);w.writerows(measured)
    return {'passed':True,'flight_verified':True,'baseline_applied_verified':True,
            'clock_aligned_pairs':len(pairs),'command_match_fraction':fraction,
            'position_airborne_duration_s':float(ros[selected[-1]]-ros[selected[0]]),
            'logged_trajectory_period_median_s':float(np.median(np.diff(ros[selected]))),
            'position_receipt_minimum_samples':100,
            'external_rate_commands':rate_count,'cases':[]},data


def analyze(out,facts):
    from pyulog import ULog
    method=facts['flight']['method']
    if method=='mppi':
        from .trajectory_results import analyze as mppi_analyze
        report,data=mppi_analyze(out,facts)
    else:report,data=baseline(out,facts)
    rows=np.atleast_1d(np.genfromtxt(out/'commands.csv',delimiter=',',names=True,dtype=None,encoding='utf-8'))
    # Input changes are per component, not a norm combining different units.
    inputs=np.column_stack([rows[n] for n in (['c','p_sp','q_sp','r_sp'] if method=='mppi' else ['goal_x','goal_y','goal_z'])])
    delta=np.diff(inputs,axis=0)
    labels=['c','p_sp','q_sp','r_sp'] if method=='mppi' else ['x_sp','y_sp','z_sp']
    units=['1','rad/s','rad/s','rad/s'] if method=='mppi' else ['m','m','m']
    period=np.diff(rows['ros_ns'])/1e9
    c=error_metrics(data)
    c.update(name=facts['name'],method=method,seed=facts['flight']['seed'],kind='actual_gazebo_repeated_trial',
             hover_scale=facts['flight']['hover_scale'],success=c['rmse_normal_m']<.12,
             criterion='Predeclared position RMSE <0.12m in [7,27)s; no low-error sample selection.',
             compute_samples=len(rows),publish_intervals=len(rows)-1,compute_mean_ms=float(np.mean(rows['compute_ms'])),compute_p95_ms=float(np.percentile(rows['compute_ms'],95)),
             compute_max_ms=float(np.max(rows['compute_ms'])),compute_overrun_count=int(sum(rows['compute_ms']>50)),
             compute_overrun_fraction=float(np.mean(rows['compute_ms']>50)),
             publish_period_mean_s=float(period.mean()),publish_period_p95_s=float(np.percentile(period,95)),
             publish_period_max_s=float(period.max()),publish_period_over_50ms_count=int(sum(period>.05)),
             publish_period_over_50ms_fraction=float(np.mean(period>.05)),
             publish_period_over_60ms_count=int(sum(period>.06)),publish_period_over_60ms_fraction=float(np.mean(period>.06)),
             compute_scope='MPPI optimization' if method=='mppi' else 'Current analytic reference generation only; PX4 internal compute not measured',
             input_change={label:{'unit':unit,'total_variation':float(np.sum(abs(delta[:,i]))),
                                        'step_rms':float(np.sqrt(np.mean(delta[:,i]**2)))} for i,(label,unit) in enumerate(zip(labels,units))})
    # Also retain the common FCU rate/thrust interface from both architectures.
    log=ULog(str(next(out.glob('runtime/log/**/*.ulg'))));rates=log.get_dataset('vehicle_rates_setpoint').data
    times,known=aligned_clock(log,rates['timestamp']);epoch=facts['flight']['reference_epoch_ros_ns']/1e9
    selected=known&(times>=epoch)&(times<epoch+30)
    actual=np.column_stack([rates['roll'][selected],-rates['pitch'][selected],-rates['yaw'][selected],-rates['thrust_body[2]'][selected]])
    if len(actual)<200:raise RuntimeError('Common FCU rate/thrust data missing')
    with (out/'fcu_inner_setpoints.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['ros_s','p_sp_flu','q_sp_flu','r_sp_flu','c']);w.writerows(np.column_stack([times[selected],actual]))
    c['fcu_inner_setpoint_variation']={label:{'unit':unit,'total_variation':float(np.sum(abs(np.diff(actual[:,i]))))}
                                     for i,(label,unit) in enumerate(zip(['p_sp','q_sp','r_sp','c'],['rad/s']*3+['1']))}
    c['fcu_inner_setpoint_samples']=len(actual)
    c['fcu_inner_setpoint_period_median_s']=float(np.median(np.diff(times[selected])))
    c['fcu_inner_setpoint_note']='Different native ULog sampling intervals; total variation over 30s, no mixed-unit sum or energy interpretation.'
    report.update(cases=[c],evaluation_scope='Unique original Odometry at its timestamp, same analytic reference, fixed [7,27)s.',
                  receipt_rule='Value consistency within +/-100ms, >=95% under 5e-5; not unique latency.')
    write(out/'analysis.json',report)
    return report
