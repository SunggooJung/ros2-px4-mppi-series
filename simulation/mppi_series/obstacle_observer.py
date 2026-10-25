"""Retain actual Gazebo world poses, contacts and server scene metadata."""
import argparse
import json
import signal
import threading
import time
from pathlib import Path
from google.protobuf.json_format import MessageToDict
from gz.transport13 import Node
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene

def run(out):
    node=Node();stop=threading.Event();lock=threading.Lock()
    paths={name:(out/(name+'.jsonl')).open('w') for name in ['gazebo_contacts','gazebo_poses']}
    counts={name:0 for name in paths}
    verified_topics={};last_checkpoint=0.
    def checkpoint():
        temporary=out/'observer_counts.tmp'
        value={**counts,'verified_topics':dict(verified_topics),'checkpoint_wall_ns':time.time_ns()}
        temporary.write_text(json.dumps(value,indent=2))
        temporary.replace(out/'observer_counts.json')
    def callback(name):
        def received(message):
            nonlocal last_checkpoint
            if stop.is_set():return
            record={'received_wall_ns':time.time_ns(),'received_monotonic_s':time.monotonic(),
                    'message':MessageToDict(message,preserving_proto_field_name=True)}
            with lock:
                paths[name].write(json.dumps(record)+'\n');paths[name].flush();counts[name]+=1
                now=time.monotonic()
                if now-last_checkpoint>=.5:
                    checkpoint();last_checkpoint=now
        return received
    node.subscribe(Contacts,'/world/default/model/ground_plane/link/link/sensor/ground_contact/contact',callback('gazebo_contacts'))
    node.subscribe(Pose_V,'/world/default/pose/info',callback('gazebo_poses'))
    node.subscribe(Contacts,'/world/default/model/known_obstacle/link/column/sensor/contact/contact',callback('gazebo_contacts'))
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,lambda *_:stop.set())
    try:
        while not stop.is_set():
            for label,topic in [('cylinder','/world/default/model/known_obstacle/link/column/sensor/contact/contact'),('ground','/world/default/model/ground_plane/link/link/sensor/ground_contact/contact')]:
                publishers,_=node.topic_info(topic)
                types=[p.msg_type_name for p in publishers]
                if 'gz.msgs.Contacts' in types:
                    verified_topics[label]={'topic':topic,'publisher_types':types,'observed_wall_ns':time.time_ns()}
            if not (out/'gazebo_scene.json').exists():
                ok,msg=node.request('/world/default/scene/info',Empty(),Empty,Scene,1000)
                if ok:(out/'gazebo_scene.json').write_text(json.dumps(MessageToDict(msg,preserving_proto_field_name=True),indent=2))
            with lock:checkpoint()
            stop.wait(.2)
    finally:
        with lock:
            checkpoint()
            for f in paths.values():f.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
