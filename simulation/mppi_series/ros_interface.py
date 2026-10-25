"""10-1: live MAVROS Odometry -> model state -> normalized thrust/FLU rates.

This bounded interface experiment never arms a vehicle. PX4 SITL is required;
the stream is NOT a complete flight supervisor for hardware or chapter 10-2.
"""
import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np
from .controller import Config, MPPIController, attitude_target_fields


def quaternion_rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all():raise ValueError('Invalid quaternion')
    norm=np.linalg.norm(q)
    if not .5<norm<1.5:raise ValueError('Quaternion norm outside supported range')
    x,y,z,w=q/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def odometry_state(message):
    # MAVROS local_position/odom twist is body FLU, even though pose is ENU.
    if message.header.frame_id!='map' or message.child_frame_id!='base_link':
        raise ValueError('Expected map pose / base_link body twist')
    p=message.pose.pose.position;q=message.pose.pose.orientation
    v=message.twist.twist.linear;w=message.twist.twist.angular
    R=quaternion_rotation([q.x,q.y,q.z,q.w])
    euler=[math.atan2(R[2,1],R[2,2]),math.asin(np.clip(-R[2,0],-1,1)),
           math.atan2(R[1,0],R[0,0])]
    state=np.r_[[p.x,p.y,p.z],R@np.array([v.x,v.y,v.z]),euler,[w.x,w.y,w.z]]
    if not np.isfinite(state).all():raise ValueError('Non-finite state')
    return state


def gate(*,connected,armed,state_age_s,header_age_s,compute_s,budget_s=.05):
    if not connected:return 'disconnected'
    if armed:return 'unexpected_armed_vehicle'
    if not 0<=state_age_s<=.25 or not -.05<=header_age_s<=.5:return 'stale_state'
    if not 0<=compute_s<=budget_s:return 'compute_overrun'
    return None


def run(output,ros_args=None):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import AttitudeTarget,State,ExtendedState
    from mavros_msgs.srv import SetMode

    rclpy.init(args=ros_args);node=rclpy.create_node('mppi_rate_interface_check')
    ns='/sitl_check/mavros';odom=None;arrival=0.;status=State();extended=ExtendedState()
    frozen=False;events=[];cfg=Config(temperature=3.,seed=7)
    controller=MPPIController(cfg)
    result={'passed':False,'armed_command_sent':False,'config':asdict(cfg),'events':events}
    def on_odom(msg):
        nonlocal odom,arrival
        if not frozen:odom=msg;arrival=time.monotonic()
    def on_state(msg):
        nonlocal status
        status=msg
    def on_extended(msg):
        nonlocal extended
        extended=msg
    node.create_subscription(Odometry,ns+'/local_position/odom',on_odom,qos_profile_sensor_data)
    node.create_subscription(State,ns+'/state',on_state,10)
    node.create_subscription(ExtendedState,ns+'/extended_state',on_extended,10)
    pub=node.create_publisher(AttitudeTarget,ns+'/setpoint_raw/attitude',10)
    reference=node.create_publisher(PoseStamped,'/mppi/reference',10)
    mode=node.create_client(SetMode,ns+'/set_mode')
    def stamp_ns(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    def event(name,**extra):
        events.append({'event':name,'monotonic_s':time.monotonic(),
                       'ros_ns':node.get_clock().now().nanoseconds,**extra})
        print(name,flush=True)
    def set_mode(name):
        if not mode.wait_for_service(timeout_sec=5):raise RuntimeError('Mode service unavailable')
        req=SetMode.Request();req.custom_mode=name;future=mode.call_async(req)
        deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
        if not future.done() or not future.result().mode_sent:raise RuntimeError('Mode rejected: '+name)
    output.mkdir(parents=True,exist_ok=True)
    file=(output/'commands.csv').open('w');writer=csv.writer(file)
    writer.writerow(['ros_ns','odom_ros_ns','monotonic_s','phase','mode','armed','landed',
                     *['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r'],
                     'goal_x','goal_y','goal_z','c','p_sp','q_sp','r_sp','compute_ms','state_age_s'])
    try:
        deadline=time.monotonic()+100
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if status.connected and odom is not None and time.monotonic()-arrival<.25:break
        else:raise RuntimeError('No live FCU state / Odometry')
        if status.armed:raise RuntimeError('Refuse interface test on armed FCU')
        initial=odometry_state(odom);origin=initial[:3].copy()
        event('live_state_ready',frame=odom.header.frame_id,child_frame=odom.child_frame_id)
        start=time.monotonic();next_step=start;offboard_requested=False;offboard_begin=None
        suppression_begin=None;last_publish=None;published=0;blocked=0
        while time.monotonic()-start<32:
            rclpy.spin_once(node,timeout_sec=.005);current=time.monotonic()
            if current<next_step:continue
            next_step=current+cfg.dt
            if current-start>=5 and not offboard_requested:
                set_mode('OFFBOARD');offboard_requested=True;event('offboard_requested')
            if status.mode=='OFFBOARD' and offboard_begin is None:
                offboard_begin=current;event('rate_offboard_observed')
            if offboard_requested and offboard_begin is None and current-start>15:
                raise RuntimeError('FCU did not enter OFFBOARD')
            if offboard_begin and current-offboard_begin>=12 and not frozen:
                frozen=True;event('inject_stale_state')
            phase=0 if offboard_begin is None else min(3,int((current-offboard_begin)/3))
            offset=[(0.,0.,.5),(.5,0.,.5),(0.,.5,.5),(-.5,-.5,.5)][phase]
            goal=origin+offset
            now_ns=node.get_clock().now().nanoseconds
            state_age=current-arrival;header_age=(now_ns-stamp_ns(odom))/1e9
            reason=gate(connected=status.connected,armed=status.armed,state_age_s=state_age,
                        header_age_s=header_age,compute_s=0)
            if reason:
                blocked+=1
                if suppression_begin is None:
                    suppression_begin=current;event('command_blocked',reason=reason)
                if reason!='stale_state':raise RuntimeError('Command blocked: '+reason)
                if current-suppression_begin>=3:
                    # A disarmed PX4 can retain OFFBOARD in the displayed
                    # nav state. Verify its loss flag independently in ULog.
                    event('stale_block_observed',mode=status.mode)
                    result.update(passed=True,stale_state_block_verified=True,
                                  mode_after_stream_stop=status.mode,published_commands=published,
                                  blocked_cycles=blocked,commands_after_stale_block=0,
                                  last_publish_monotonic_s=last_publish)
                    break
                continue
            state=odometry_state(odom);before=time.perf_counter()
            command=controller.step(state,goal);elapsed=time.perf_counter()-before
            reason=gate(connected=status.connected,armed=status.armed,state_age_s=time.monotonic()-arrival,
                        header_age_s=(node.get_clock().now().nanoseconds-stamp_ns(odom))/1e9,
                        compute_s=elapsed,budget_s=cfg.dt)
            if reason:raise RuntimeError('No command after rejected calculation: '+reason)
            fields=attitude_target_fields(command);msg=AttitudeTarget()
            msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='base_link'
            msg.type_mask=fields['type_mask'];msg.orientation.w=1.
            msg.thrust=fields['thrust_01']
            msg.body_rate.x,msg.body_rate.y,msg.body_rate.z=fields['body_rate_flu_rad_s']
            pub.publish(msg);last_publish=time.monotonic();published+=1
            ref=PoseStamped();ref.header=msg.header;ref.header.frame_id='map';ref.pose.orientation.w=1.
            ref.pose.position.x,ref.pose.position.y,ref.pose.position.z=goal;reference.publish(ref)
            writer.writerow([stamp_ns(msg),stamp_ns(odom),last_publish,phase,status.mode,status.armed,
                             extended.landed_state,*state,*goal,*command,elapsed*1000,state_age])
            file.flush()
        if not result['passed']:raise RuntimeError('Bounded interface sequence incomplete')
    except Exception as error:
        result['error']=str(error);event('failed',reason=str(error))
    finally:
        file.close()
        # No arming request exists in this node. Leave the disarmed FCU out of
        # Offboard after both success and failure; the experiment stops it next.
        if status.connected and not status.armed:
            try:set_mode('POSCTL')
            except Exception as error:result['cleanup_error']=str(error)
        result.update(final_mode=status.mode,final_armed=status.armed)
        (output/'interface.json').write_text(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path)
    raise SystemExit(run(parser.parse_args().output))
