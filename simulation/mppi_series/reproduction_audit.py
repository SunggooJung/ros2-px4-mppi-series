"""Read-only, source-backed audit of the published curriculum and its evidence.

Historical measurements are inventoried, never merged into new trial metrics.
This checks preserved publication records; it does not query or edit the server.
"""
from pathlib import Path
import hashlib
import json
import re
import shutil

from .episode_10_1 import write

POSTS = [('1',83),('2',85),('3',84),('4',86),('5',87),('6',88),
         ('7-1',89),('7-2',90),('8-1',92),('8-2',93),('9-1',96),
         ('9-2',100),('10-1',102),('10-2',106),('11-1',107),('11-2',123),('12-1',130)]
CORE = {'9-1':['controller.py'], '9-2':['controller.py','closed_loop.py'],
        '10-1':['controller.py','ros_interface.py'],
        '10-2':['controller.py','ros_interface.py','ros_flight.py'],
        '11-1':['controller.py','trajectory.py','trajectory_flight.py'],
        '11-2':['controller.py','obstacles.py','obstacle_flight.py'],
        '12-1':['controller.py','trajectory.py','comparison_flight.py']}


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def verify_file(path, expected):
    if not path.is_file():raise RuntimeError('Missing retained evidence: '+str(path))
    actual=digest(path)
    if actual!=expected:raise RuntimeError('Retained source hash mismatch: '+str(path))
    return actual


def audit(out,workspace,application):
    preserved=out/'prior_evidence';preserved.mkdir()
    registry_path=workspace/'publishing/artifacts/mppi_daily/registry.json'
    registry=json.loads(registry_path.read_text())
    shutil.copy2(registry_path,preserved/'registry.json')
    rows=[];issues=[]
    for episode,post_id in POSTS:
        row=dict(episode=episode,post_id=post_id,url=f'https://sunggoo.tistory.com/{post_id}',
                 publication_observation='preserved local evidence; no current server query',
                 proposed_server_revision_applied=False)
        if episode in CORE:
            if registry['episodes'][episode]['post_id']!=post_id:raise RuntimeError('Curriculum ID mismatch: '+episode)
            if registry['episodes'][episode]['phase']!='published':raise RuntimeError('Prior chapter not recorded published: '+episode)
            if episode=='9-1':
                root=workspace/'publishing/artifacts/mppi_series/9_1/20261002T080546_781565Z'
                state_path=workspace/'publishing/artifacts/mppi_series/publication_9_1/state.json'
            else:
                state_path=workspace/f'publishing/artifacts/mppi_daily/episodes/{episode}/state.json'
                state=json.loads(state_path.read_text());root=Path(state['evidence_run'])
            facts=json.loads((root/'result.json').read_text())
            if facts.get('passed') is not True:raise RuntimeError('Incomplete historical experiment: '+episode)
            target=preserved/episode;target.mkdir()
            for src in [root/'result.json',state_path]:shutil.copy2(src,target/src.name)
            source_matches={};changes={}
            for rel,expected in facts['source_sha256'].items():
                # Early 9-1/9-2 snapshots use flat module names.
                source=root/'source'/rel
                source_matches[rel]=verify_file(source,expected)
                current=application/rel if '/' in rel else application/'simulation/mppi_series'/rel
                if current.is_file() and digest(current)!=expected:
                    changes[rel]=dict(historical=expected,current=digest(current))
            core_checks={}
            for name in CORE[episode]:
                rel='simulation/mppi_series/'+name
                expected=facts['source_sha256'].get(rel,facts['source_sha256'].get(name))
                if expected is None:raise RuntimeError('No historical core source hash: '+episode+'/'+name)
                # A representative re-execution must use the published core.
                # Added audit/article modules are allowed; changed dynamics are not.
                verify_file(application/rel,expected);core_checks[rel]=expected
            raw=[]
            candidates=list(root.glob('*.csv'))+list(root.glob('prediction.npz'))
            candidates+=list(root.glob('rosbag/*.mcap'))+list(root.glob('rosbag/*.db3'))+list(root.glob('runtime/log/**/*.ulg'))
            for trial in facts.get('trials',[]):
                tr=Path(trial['output'])
                candidates+=list(tr.glob('rosbag/*.mcap'))+list(tr.glob('rosbag/*.db3'))+list(tr.glob('runtime/log/**/*.ulg'))
                candidates+=list(tr.glob('*.csv'))+list(tr.glob('*.jsonl'))+list(tr.glob('provenance/manifest.json'))
            if episode in ['10-1','10-2','11-1','11-2','12-1']:
                if not any(p.suffix=='.ulg' for p in candidates) or not any(p.suffix in ['.mcap','.db3'] for p in candidates):
                    raise RuntimeError('Historical Gazebo raw evidence missing: '+episode)
            for p in sorted(set(candidates)):
                if not p.is_file():raise RuntimeError('Missing historical raw data: '+str(p))
                raw.append(dict(path=str(p),sha256=digest(p),bytes=p.stat().st_size))
            article=root/'article.md'
            if not article.is_file():
                article=workspace/('publishing/drafts/series_9_1_controller.md' if episode=='9-1'
                                  else 'publishing/drafts/series_9_2_closed_loop.md')
            if not article.is_file():raise RuntimeError('Historical manuscript missing: '+episode)
            shutil.copy2(article,target/'article.md')
            text=article.read_text();asset_path=root/'publication_assets.json'
            assets=[]
            if asset_path.is_file():
                asset=json.loads(asset_path.read_text());shutil.copy2(asset_path,target/asset_path.name)
                for p in [Path(asset['archive']),*[Path(p) for p in asset['figures']]]:
                    if not p.is_file():raise RuntimeError('Missing historical publication asset: '+str(p))
                    assets.append(dict(path=str(p),sha256=digest(p),bytes=p.stat().st_size))
            old_cases=[*facts.get('cases',[]),*[c for t in facts.get('trials',[]) for c in t.get('analysis',{}).get('cases',[])]]
            row.update(evidence_root=str(root),scope=facts['scope'],source_snapshot_verified=source_matches,
                       representative_core_hashes=core_checks,current_source_differences=changes,
                       raw_evidence=raw,publication_assets=assets,
                       historical_failed_case_names=sorted({c.get('name','unnamed') for c in old_cases if c.get('success') is False}),
                       title=text.splitlines()[0],preserved_manuscript=str(target/'article.md'),
                       manuscript_sha256=digest(target/'article.md'),
                       internal_links=sorted(set(re.findall(r'https://sunggoo\.tistory\.com/\d+',text))),
                       historical_config=facts.get('config',facts.get('interface',facts.get('flight',{})).get('config',{})),
                       historical_px4_commit=facts.get('px4_commit',facts.get('environment',{}).get('px4_commit_context')))
        else:
            paths=[workspace/f'publishing/artifacts/blog_remodel/20261007/articles/{post_id}.txt',
                   workspace/f'publishing/artifacts/series_9_1_revision/access_recheck/public_{post_id}.txt']
            source=next((p for p in paths if p.is_file()),None)
            if source is None:raise RuntimeError('Missing preserved early-chapter text: '+str(post_id))
            target=preserved/f'{post_id}.txt';shutil.copy2(source,target);text=target.read_text()
            row.update(preserved_manuscript=str(target),manuscript_source=str(source),manuscript_sha256=digest(target),
                       internal_links=sorted(set(re.findall(r'https://sunggoo\.tistory\.com/\d+',text))),
                       executable_verification={'1':'runtime version/config/provenance inventory','2':'actual MAVROS/PX4/ROS bag',
                          '3':'actual mode/arming and stream-loss guards','4':'fresh colcon offboard_control build/launch',
                          '5':'temperature/weights examples','6':'SI to normalized reference bridge',
                          '7-1':'source-backed motor allocation/units','7-2':'rotation/Euler/inertia checks',
                          '8-1':'clipped sampling/weights/horizon examples','8-2':'geometry/cost/counterexample checks'}[episode])
        rows.append(row)
    issues.extend([
        dict(post_ids=[89,90],issue='Old signed motor output, dimensional mixer error, rank 3; body rate integrated as Euler rate and inertia cross term omitted.',
             action='Retain canonical source-backed corrections; no claim that server revision was applied.'),
        dict(post_ids=[83],issue='Preserved chapter index predates final completion and uses 1/12 framing.',
             action='Prepare chapter/part index and final-link proposal; new post ID resolved only by publisher.'),
        dict(post_ids=[130],issue='Seven-trial comparison must not be replaced by the representative seed-7 rerun.',
             action='Preserve original statistics, add reproduction link and three-sentence summary proposal.'),
        dict(post_ids=[87,92,93],issue='Temperature convention, clipped update, input variation versus jerk/energy require canonical terminology.',
             action='Link executed numerical examples and static obstacle clearance checks.')])
    report=dict(passed=True,chapters=rows,review_findings=issues,
                scope='Preserved source/raw/asset/curriculum audit, plus explicit old-text corrections; not a new public-page or link HTTP-status verification.',
                public_posts_modified=False,registry_sha256=digest(registry_path))
    write(out/'series_audit.json',report)
    return report
