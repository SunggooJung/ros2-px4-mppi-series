"""Capture the source/binary/environment actually selected before an experiment."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as file:
        for block in iter(lambda: file.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def capture(run, src, px4, image_id, invocation):
    root = run / 'provenance'
    root.mkdir()
    manifest = {'captured_utc': datetime.now(timezone.utc).isoformat(), 'invocation': invocation,
                'image_id': image_id, 'files': {}, 'repositories': {},
                'scope': 'Application experiment code/config, PX4 source identity/diffs/untracked files, executable/build runtime hashes and Gazebo assets. A dirty source tree is not proof that its existing binary was rebuilt.'}
    def preserve(path, relative):
        destination = root / 'source' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination)
        manifest['files'][str(relative)] = digest(destination)
    for path in sorted((src/'simulation/scripts').glob('*')):
        if path.is_file():
            preserve(path, path.relative_to(src))
    for path in sorted((src/'simulation/config').rglob('*')):
        if path.is_file():
            preserve(path, path.relative_to(src))
    for label, repo in [('px4',px4),('gazebo',px4/'Tools/simulation/gz')]:
        def git(*args):
            return subprocess.check_output(['git','-C',str(repo),*args])
        manifest['repositories'][label] = {'path':str(repo),'commit':git('rev-parse','HEAD').decode().strip(),
                                           'status':git('status','--short').decode(),
                                           'submodules':git('submodule','status','--recursive').decode()}
        (root/(label+'.patch')).write_bytes(git('diff','HEAD','--binary'))
        files=git('ls-files','--others','--exclude-standard','-z').split(b'\0')
        for name in files:
            if name:
                path=repo / name.decode()
                if path.is_file():
                    preserve(path, Path('untracked')/label/name.decode())
    runtime = px4/'build/px4_sitl_default'
    # Archive the actual executable, init scripts, plugins and assets used by
    # this bounded gz_x500/default experiment, independently of dirty source.
    with tarfile.open(root/'runtime_assets.tar.gz','w:gz') as archive:
        for path in [runtime/'bin/px4',runtime/'etc',runtime/'src/modules/simulation/gz_plugins',
                     px4/'Tools/simulation/gz/models/x500',px4/'Tools/simulation/gz/models/x500_base',
                     px4/'Tools/simulation/gz/worlds/default.sdf']:
            archive.add(path, arcname=str(path.relative_to(px4)))
    manifest['px4_binary_sha256']=digest(runtime/'bin/px4')
    manifest['runtime_assets_sha256']=digest(root/'runtime_assets.tar.gz')
    manifest['host_packages']=subprocess.check_output(['dpkg-query','-W','gz-sim8-cli','ros-jazzy-ros-base',
                                                      'ros-jazzy-mavros','ros-jazzy-mavros-msgs'],text=True)
    manifest['container_packages']=subprocess.check_output(['docker','run','--rm','--entrypoint','dpkg-query',
                                                           image_id,'-W','gz-sim8-cli','libgz-sim8'],text=True)
    manifest['python_packages']=subprocess.check_output(['/usr/bin/python3','-c',
        "import importlib.metadata,json; print(json.dumps({x:importlib.metadata.version(x) for x in ['numpy','matplotlib','pyulog','pymavlink']}))"],text=True).strip()
    (root/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return manifest
