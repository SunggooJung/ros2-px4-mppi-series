"""10-2: position-controlled takeoff, measured-hover initialization, MPPI flight.

Requires the owned runtime/PX4 instance 7 and system ID 8. AUTO.LAND and state
gates bound this SITL experiment; this is not a hardware flight supervisor.
"""
import csv
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np
from .controller import Config, MPPIController, attitude_target_fields
from .ros_interface import odometry_state


def flight_gate(state,connected,armed,mode,state_age,header_age,compute_s=0.):
    if not connected or not armed or mode!='OFFBOARD':return 'lost_flight_state'
    if not 0<=state_age<=.25 or not -.05<=header_age<=.5:return 'stale_state'
    if not 0<=compute_s<=.05:return 'compute_overrun'
    if not np.isfinite(state).all():return 'invalid_state'
    if state[2]<.5 or state[2]>4 or np.linalg.norm(state[:2])>4:return 'flight_envelope'
    if np.max(np.abs(state[6:8]))>.6:return 'excessive_tilt'
    return None


def run(output,ros_args=None):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import AttitudeTarget,State,ExtendedState
    from mavros_msgs.srv import SetMode,CommandBool
    from pymavlink import mavutil
    # Refuse a launch without the owned Gazebo/PX4 runtime marker.
    marker=json.loads((output/'sitl_owned.json').read_text())
    if marker.get('px4_instance')!=7 or marker.get('system_id')!=8 or marker.get('hardware_used') is not False:
        raise RuntimeError('Owned isolated SITL marker required')
    rclpy.init(args=ros_args);node=rclpy.create_node('mppi_sitl_flight')
    ns='/sitl_check/mavros';status=State();extended=ExtendedState();odom=None;arrival=0.
    rate_feedback=[];events=[];phase='connect';position_stream=False;next_position=0.
    result={'passed':False,'events':events,'hardware_used':False,'target_phases':[]}
    def on_odom(msg):
        nonlocal odom,arrival
        odom=msg;arrival=time.monotonic()
    def on_state(msg):
        nonlocal status
        status=msg
    def on_extended(msg):
        nonlocal extended
        extended=msg
    def on_rate(msg):
        if odom is not None:
            s=odometry_state(odom)
            rate_feedback.append((time.monotonic(),msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                                  msg.thrust,*s[:6],*s[6:8]))
    node.create_subscription(Odometry,ns+'/local_position/odom',on_odom,qos_profile_sensor_data)
    node.create_subscription(State,ns+'/state',on_state,10)
    node.create_subscription(ExtendedState,ns+'/extended_state',on_extended,10)
    node.create_subscription(AttitudeTarget,ns+'/setpoint_raw/target_attitude',on_rate,qos_profile_sensor_data)
    position=node.create_publisher(PoseStamped,ns+'/setpoint_position/local',10)
    rates=node.create_publisher(AttitudeTarget,ns+'/setpoint_raw/attitude',10)
    reference=node.create_publisher(PoseStamped,'/mppi/reference',10)
    mode=node.create_client(SetMode,ns+'/set_mode');arm=node.create_client(CommandBool,ns+'/cmd/arming')
    target=PoseStamped();target.header.frame_id='map';target.pose.orientation.w=1.
    target.pose.position.z=2.
    file=(output/'commands.csv').open('w');writer=csv.writer(file)
    writer.writerow(['ros_ns','odom_ros_ns','monotonic_s','phase','mode','armed','landed',
                     'x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r',
                     'goal_x','goal_y','goal_z','c','p_sp','q_sp','r_sp','compute_ms','state_age_s'])
    def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    def event(name,**extra):
        events.append({'event':name,'ros_ns':node.get_clock().now().nanoseconds,'monotonic_s':time.monotonic(),**extra})
        print(name,extra,flush=True)
    def tick():
        nonlocal next_position
        rclpy.spin_once(node,timeout_sec=.005)
        if position_stream and time.monotonic()>=next_position:
            target.header.stamp=node.get_clock().now().to_msg();position.publish(target)
            next_position=time.monotonic()+.05
    def until(predicate,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            tick()
            if predicate():event(label);return
        raise RuntimeError('Timeout: '+label)
    def request(client,req):
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Service unavailable')
        future=client.call_async(req);until(future.done,5,'service_response')
        return future.result()
    def set_mode(name):
        req=SetMode.Request();req.custom_mode=name
        if not request(mode,req).mode_sent:raise RuntimeError('Mode rejected: '+name)
    try:
        until(lambda:status.connected and odom is not None and time.monotonic()-arrival<.25,100,'live_state_ready')
        if status.armed:raise RuntimeError('Owned experiment must start disarmed')
        origin=odometry_state(odom)[:3].copy()
        target.pose.position.x=float(origin[0]);target.pose.position.y=float(origin[1])
        position_stream=True;phase='px4_takeoff'
        warmup=time.monotonic()+10
        while time.monotonic()<warmup:tick()
        set_mode('OFFBOARD');until(lambda:status.mode=='OFFBOARD',10,'position_offboard')
        req=CommandBool.Request();req.value=True
        if not request(arm,req).success:raise RuntimeError('Arming rejected')
        until(lambda:status.armed,10,'armed')
        stable=None;end=time.monotonic()+50
        while time.monotonic()<end:
            tick();s=odometry_state(odom)
            if not status.armed or status.mode!='OFFBOARD':raise RuntimeError('Takeoff state lost')
            good=np.linalg.norm(s[:3]-[origin[0],origin[1],2.])<.2 and np.linalg.norm(s[3:6])<.15
            stable=(stable or time.monotonic()) if good else None
            if stable and time.monotonic()-stable>=4:break
        else:raise RuntimeError('Stable position-controlled hover not reached')
        now=time.monotonic()
        calibration=[v for v in rate_feedback if now-v[0]<=3 and .1<v[2]<.9
                     and np.linalg.norm(v[6:9])<.2 and max(abs(v[9]),abs(v[10]))<.1]
        if len(calibration)<15:raise RuntimeError('Insufficient live PX4 hover-thrust samples')
        hover=float(np.median([v[2] for v in calibration]))
        with (output/'hover_calibration.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['monotonic_s','ros_ns','c','x','y','z','vx','vy','vz','roll','pitch']);w.writerows(calibration)
        cfg=Config(samples=500,horizon=20,temperature=3.,thrust_hover=hover,seed=7)
        controller=MPPIController(cfg);result.update(config=asdict(cfg),hover_calibration_samples=len(calibration))
        event('px4_takeoff_complete',hover_thrust=hover)
        # Stop the position publisher before the first rate command. ROS bag
        # and PX4 control-mode flags independently verify the handover.
        position_stream=False;handover=time.monotonic();next_step=handover
        event('mppi_handover')
        phases=[('hover',0.,0.),('move_x',.5,0.),('return',0.,0.)]
        duration=10.
        for index,(phase,dx,dy) in enumerate(phases):
            goal=np.array([origin[0]+dx,origin[1]+dy,2.])
            event('target_change',phase=phase,goal=goal.tolist(),duration_s=duration)
            result['target_phases'].append({'phase':phase,'goal':goal.tolist(),'duration_s':duration})
            phase_end=handover+(index+1)*duration
            while time.monotonic()<phase_end:
                tick();current=time.monotonic()
                if current<next_step:continue
                next_step=current+cfg.dt
                s=odometry_state(odom);now_ns=node.get_clock().now().nanoseconds
                reason=flight_gate(s,status.connected,status.armed,status.mode,current-arrival,(now_ns-stamp(odom))/1e9)
                if reason:raise RuntimeError('MPPI blocked: '+reason)
                before=time.perf_counter();command=controller.step(s,goal);compute=time.perf_counter()-before
                reason=flight_gate(s,status.connected,status.armed,status.mode,time.monotonic()-arrival,
                                   (node.get_clock().now().nanoseconds-stamp(odom))/1e9,compute)
                if reason:raise RuntimeError('MPPI blocked after compute: '+reason)
                fields=attitude_target_fields(command);msg=AttitudeTarget();msg.header.stamp=node.get_clock().now().to_msg()
                msg.header.frame_id='base_link';msg.type_mask=128;msg.orientation.w=1.;msg.thrust=fields['thrust_01']
                msg.body_rate.x,msg.body_rate.y,msg.body_rate.z=fields['body_rate_flu_rad_s'];rates.publish(msg)
                ref=PoseStamped();ref.header=msg.header;ref.header.frame_id='map';ref.pose.orientation.w=1.
                ref.pose.position.x,ref.pose.position.y,ref.pose.position.z=goal;reference.publish(ref)
                writer.writerow([stamp(msg),stamp(odom),time.monotonic(),phase,status.mode,status.armed,extended.landed_state,
                                 *s,*goal,*command,compute*1000,current-arrival]);file.flush()
        event('mppi_sequence_complete');phase='px4_landing';set_mode('AUTO.LAND')
        until(lambda:not status.armed and extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND,60,'landed_and_disarmed')
        result['passed']=True
    except Exception as error:
        result['error']=str(error);event('failed',reason=str(error));position_stream=False
        if status.armed:
            try:
                set_mode('AUTO.LAND');until(lambda:not status.armed,60,'failure_recovery_disarmed')
            except Exception as recovery:result['recovery_error']=str(recovery)
    finally:
        file.close();result.update(final_mode=status.mode,final_armed=status.armed)
        (output/'flight.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1
