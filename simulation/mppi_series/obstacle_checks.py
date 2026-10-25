"""Source-backed geometry/cost checks, world generation and measured clearance."""
from dataclasses import asdict
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .obstacles import OBSTACLE,PROFILES,ObstacleController,horizon_reference,reference
from .controller import Config
from .episode_10_1 import write,sha


def build_world(runtime,workspace):
    default=workspace/'third_party/px4_sitl/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
    root=ET.fromstring(default.read_text(encoding='utf-8-sig'));world=root.find('world')
    o=OBSTACLE
    model=ET.fromstring(f'''<model name="known_obstacle"><static>true</static><pose>{o.x} {o.y} {o.height/2} 0 0 0</pose>
<link name="column"><collision name="column_collision"><geometry><cylinder><radius>{o.radius}</radius><length>{o.height}</length></cylinder></geometry></collision>
<visual name="column_visual"><geometry><cylinder><radius>{o.radius}</radius><length>{o.height}</length></cylinder></geometry><material><ambient>0.9 0.2 0.1 1</ambient><diffuse>0.9 0.2 0.1 1</diffuse></material></visual>
<sensor name="contact" type="contact"><always_on>true</always_on><update_rate>30</update_rate><topic>/obstacle/contacts</topic><contact><collision>column_collision</collision></contact></sensor>
</link></model>''')
    world.append(model)
    ground=world.find("model[@name='ground_plane']/link")
    ground.append(ET.fromstring('<sensor name="ground_contact" type="contact"><always_on>true</always_on><update_rate>30</update_rate><contact><collision>collision</collision></contact></sensor>'))
    dest=runtime/'worlds/default.sdf';dest.parent.mkdir(parents=True);ET.ElementTree(root).write(dest,encoding='utf-8',xml_declaration=True)
    write(runtime/'world_geometry.json',{'obstacle':asdict(o),'base_world_sha256':sha(default),'executed_world_sha256':sha(dest),'world':'default'})
    return dest


def verify(out,workspace):
    source=workspace/'third_party/px4_sitl/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf'
    model=ET.parse(source).getroot().find('model');bounds=[]
    for link in model.findall('link'):
        lp=np.array([float(v) for v in link.findtext('pose','0 0 0 0 0 0').split()[:3]])
        for collision in link.findall('collision'):
            size=np.array([float(v) for v in collision.findtext('geometry/box/size').split()])
            cp=np.array([float(v) for v in collision.findtext('pose','0 0 0 0 0 0').split()[:3]])
            bound=float(np.linalg.norm(lp)+np.linalg.norm(cp)+np.linalg.norm(size/2))
            bounds.append({'link':link.attrib['name'],'collision':collision.attrib['name'],'sphere_bound_m':bound,'size_m':size.tolist()})
    maximum=max(b['sphere_bound_m'] for b in bounds)
    assert maximum<=OBSTACLE.vehicle_radius
    points=np.array([[.85,0,2],[.85,.52,2],[.85,.62,2],[0,0,2]])
    clearance=OBSTACLE.clearance(points);penalty=OBSTACLE.penalty(points)
    np.testing.assert_allclose(clearance,[-.52,0.,.1,.33],atol=1e-12)
    assert penalty[0]>penalty[1]>penalty[2]>penalty[3]
    straight=reference(np.linspace(0,30,601),[0,0,0]);assert min(OBSTACLE.clearance(straight[:,:3]))<0
    cfg=Config(horizon=40,temperature=3.);c=ObstacleController(cfg,'inflated_cost')
    # Same base state/input quantities, change ONLY cylinder clearance.
    trajectories=np.zeros((2,40,12));trajectories[:,:,2]=2
    trajectories[0,:,:3]=[.85,0,2];trajectories[1,:,:3]=[.85,.8,2]
    p=OBSTACLE.penalty(trajectories[:,:,:3]);assert p[0].sum()>p[1].sum()
    cases=[{'name':'collision_geometry_enclosed','success':True,'max_sphere_bound_m':maximum},
           {'name':'known_distance_and_margin','success':True},
           {'name':'straight_reference_intersects_inflation','success':True}]
    for name,point in [('nan_geometry',[np.nan,0,2]),('malformed_geometry',[0,2])]:
        try:OBSTACLE.clearance(point)
        except ValueError:cases.append({'name':name,'kind':'rejected_failure_input','success':False,'expected_rejection':True,'verification_passed':True})
        else:raise AssertionError('Invalid geometry accepted')
    # Retain actual execution of a small counterexample: weighted averaging
    # of two safe opposite-side trajectories can lie inside the cylinder.
    candidates=np.array([[.85,.8,2],[.85,-.8,2]])
    assert min(OBSTACLE.clearance(candidates))>0 and OBSTACLE.clearance(candidates.mean(axis=0))<0
    cases.append({'name':'safe_candidate_mean_can_collide','kind':'executed_failure_counterexample','success':False,'verification_passed':True})
    examples={'points_m':points.tolist(),'clearance_m':clearance.tolist(),'penalty':penalty.tolist(),
              'candidate_safe_points_m':candidates.tolist(),'unsafe_mean_m':candidates.mean(axis=0).tolist()}
    write(out/'cost_examples.json',examples)
    snapshot=out/'geometry_source/x500_base.sdf';snapshot.parent.mkdir();snapshot.write_bytes(source.read_bytes())
    report={'passed':True,'cases':cases,'collision_bounds':bounds,'vehicle_radius_m':OBSTACLE.vehicle_radius,
            'model_sdf_sha256':sha(snapshot),'source':str(source),'obstacle':asdict(OBSTACLE),
            'scope':'Executed geometry/cost checks; not an actual flight or safety guarantee.'}
    write(out/'meaningful_checks.json',report);return report


def analyze_geometry(out,facts,data,first,last):
    o=OBSTACLE
    world=ET.parse(out/'runtime/worlds/default.sdf').getroot().find('world')
    column=world.find("model[@name='known_obstacle']")
    pose=[float(v) for v in column.findtext('pose').split()]
    assert pose[:3]==[o.x,o.y,o.height/2]
    assert float(column.findtext('link/collision/geometry/cylinder/radius'))==o.radius
    assert float(column.findtext('link/collision/geometry/cylinder/length'))==o.height
    scene=json.loads((out/'gazebo_scene.json').read_text())
    actual=next(m for m in scene['model'] if m['name']=='known_obstacle')
    # Original server response is preserved; geometry must agree with cost.
    pos=actual['pose']['position']
    np.testing.assert_allclose([pos.get('x',0),pos.get('y',0),pos.get('z',0)],[o.x,o.y,o.height/2],atol=1e-9)
    cylinders=[v['geometry']['cylinder'] for l in actual['link'] for v in l.get('visual',[]) if 'cylinder' in v.get('geometry',{})]
    assert cylinders and all(abs(c['radius']-o.radius)<1e-9 and abs(c['length']-o.height)<1e-9 for c in cylinders)
    counts=json.loads((out/'observer_counts.json').read_text())
    if set(counts.get('verified_topics',{}))!={'ground','cylinder'}:raise RuntimeError('Live contact publisher identity missing')
    if counts['gazebo_contacts']<10 or counts['gazebo_poses']<100:raise RuntimeError('Live Gazebo contact/pose observation missing')
    poses=[]
    for line in (out/'gazebo_poses.jsonl').read_text().splitlines():
        d=json.loads(line)
        if not first<=d['received_wall_ns']<=last:continue
        for p in d['message'].get('pose',[]):
            if p.get('name')=='x500_7':
                pos=p['position'];poses.append([d['received_wall_ns'],pos.get('x',0),pos.get('y',0),pos.get('z',0)])
    if len(poses)<100:raise RuntimeError('Actual Gazebo drone pose measurements missing')
    truth=np.array(poses);clear=o.clearance(truth[:,1:4]);measured=o.clearance(data[:,2:5])
    contact_records=[];active_messages=0
    for line in (out/'gazebo_contacts.jsonl').read_text().splitlines():
        d=json.loads(line)
        if not first<=d['received_wall_ns']<=last:continue
        active_messages+=1
        for c in d['message'].get('contact',[]):
            names=str(c.get('collision1',''))+' '+str(c.get('collision2',''))
            if 'known_obstacle' in names and 'x500' in names:contact_records.append(d)
    # Ground contacts before/after handover establish the live sensor stream.
    # Message count is contact reports, not distinct collision events.
    write(out/'obstacle_contact_reports.json',{'reports':contact_records,'count':len(contact_records),'monitor_total_messages':counts['gazebo_contacts'],'active_messages':active_messages})
    with (out/'gazebo_clearance.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['received_wall_ns','x','y','z','body_clearance_m','margin_clearance_m'])
        w.writerows([list(row)+[float(d),float(d-o.margin)] for row,d in zip(truth,clear)])
    # Align ground truth with measured Odometry by receive wall timestamp;
    # this comparison includes asynchronous transport, not a latency estimate.
    idx=np.searchsorted(data[:,0],truth[:,0]);idx=np.clip(idx,0,len(data)-1)
    diff=np.linalg.norm(truth[:,1:4]-data[idx,2:5],axis=1)
    if float(np.median(diff))>.15:raise RuntimeError('Gazebo/ENU origin or measured pose inconsistent')
    goal=reference(30.,facts['flight']['origin'])[:3]
    final_error=float(np.linalg.norm(data[-1,2:5]-goal))
    minimum=float(np.min(clear));reports=len(contact_records)
    report={'minimum_body_clearance_m':minimum,'minimum_margin_clearance_m':minimum-o.margin,
            'odometry_minimum_body_clearance_m':float(np.min(measured)),
            'collision_contact_report_count':reports,'collision_observed':reports>0,
            'inflated_clearance_violation_samples':int(np.sum(clear<o.margin)),
            'conservative_overlap_samples':int(np.sum(clear<0)),
            'final_goal_error_m':final_error,'max_lateral_deviation_m':float(np.max(abs(truth[:,2]))),
            'ground_truth_samples':len(truth),'ground_truth_odom_median_difference_m':float(np.median(diff)),
            'world_geometry_verified':True,'contact_monitor_verified':True,
            'success':minimum>=o.margin and reports==0 and final_error<.25,
            'outcome':('contact_collision' if reports else 'clearance_violation' if minimum<o.margin else 'stalled' if final_error>=.25 else 'avoided_and_arrived'),
            'clearance_definition':'horizontal center distance minus physical cylinder radius minus conservative rotation-invariant 0.40m x500 sphere; subtract 0.10m separately for margin.'}
    report['contact_observation_scope']='Sampled reports from live cylinder sensor; ground contacts verify pipeline; no mathematical between-sample safety guarantee.'
    if not facts['flight']['passed']:report['outcome']='aborted_'+report['outcome']
    write(out/'geometry_analysis.json',report);return report

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--workspace-root',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);verify(args.output,args.workspace_root)
