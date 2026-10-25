"""12-2 measured static figures, Korean manuscript and fresh source-ZIP replay."""
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

import numpy as np
from .episode_10_1 import sha,write
from .episode_12_2 import TITLE,SCHEDULE

README='''# 12-2: representative reproduction and series audit

This is a runnable application source/config bundle, not a full OS image.
Required external environment: Ubuntu 24.04, ROS 2 Jazzy, MAVROS 2.14,
colcon/ros2cli, rosbag2 MCAP, numpy/matplotlib/pyulog/pymavlink, Docker,
the recorded docker-px4-sitl image and existing built PX4/Gazebo workspace.
PX4 binary, image ID, packages, model/world assets and dirty source diffs
are captured separately under each trial/provenance. No rebuild is claimed.
The workspace also supplies the prior published evidence for the read-only
curriculum audit. Those large prior logs and the OS image are not in this ZIP.

From this freshly extracted directory:
```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.episode_12_2 --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```
Default execution runs canonical/source-backed checks, the published 9-1
independent solve and all twelve own-model 9-2 cases; then a live unarmed
10-1 interface/stream-loss check and six actual flights: 10-2 point task,
11-1 H20 timed trajectory, 11-2 weak/inflated obstacle cost, and one seed-7
12-1 PX4-position/MPPI pair. It then extracts its ZIP to a fresh directory
and executes this same entry point with --reproduce-child. That option skips
only recursive ZIP replay. It still runs every check and actual experiment
and produces its own manuscript, two static figures and source ZIP.

Busy SITL lock/ports are refused. Only owned instance7/system8/domain77
servers are used; no hardware, browser, timers, credentials or publication.
The installed offboard_control package is rebuilt inside the run directory.
MPPI sends normalized thrust and FLU rates with mask128; MAVROS performs
FLU->FRD once. Position takeoff/calibration and landing are excluded.
The baseline is a separately labelled PX4 position-controller experiment.

passed means execution/evidence/packaging verification succeeded. Outcome
failures remain cases.success=false, including weak obstacle costs, baseline
RMSE and deliberately invalid inputs. An incomplete unverified trial blocks
readiness. All prior and current failures remain in the UTC-unique output.
Final stdout is a JSON object with absolute output and passed fields.

The result contains commands, source snapshots/hashes, instruction copies,
series_audit.json, ULog/MCAP, measured state/command/reference alignment,
config/seeds/frames/units, companion_revision.md and reproduction logs.
The six flights are representative, not a repeat of all 12-1 statistics.
Use the older chapter's entry point/ZIP to repeat its complete seven trials.
No GIF is generated; all figures derive from retained raw measurements.
'''


def companion(out,facts):
    rows={c['post_id']:c for c in facts['series_audit']['chapters']}
    c7=facts['meaningful_checks']['chapter_7']
    text=f'''# 12-2 연결 글 로컬 보완안

대상은 정확히 /83과 /130 두 글입니다. 기존 원문·ID·원본 로그·공개 범위를 유지합니다.
이 실행에서는 서버 조회·수정·비공개 저장·공개 전환을 하지 않았습니다.
새 12-2의 ID는 게시 담당 과정에서 확정하며 여기서는 추측하지 않습니다.

## 1. 전체 목차 /83

대상 ID: 83, https://sunggoo.tistory.com/83.
원문 근거: `prior_evidence/83.txt`, SHA-256 `{rows[83]['manuscript_sha256']}`.
이 보존 원문은 2026-10-07 조회 자료이며 현재 서버와 동일하다고 주장하지 않습니다.
기존 장별 목차의 예정 표기와 ‘12편’ 표현은 본편 게시물 수와 장 번호를 혼동할 수 있습니다.
공개 적용 전 최신 서버 원문을 다시 보존하고 차이를 비교합니다.

제안: 1~12장을 유지하되 -1/-2를 실제 회차로 나누고 9-1 /96 → 9-2 /100 →
10-1 /102 → 10-2 /106 → 11-1 /107 → 11-2 /123 → 12-1 /130 순서를 명시합니다.
12-2는 게시 검증 전까지 예정이며, 검증 후 확정 URL로만 연결합니다. /98은
PX4 기본 위치 제어의 비공개 보충 자료이며 본편 9-2와 혼합하거나 공개로 바꾸지 않습니다.
끝에서 임의 13장을 예고하지 않고 실기체·인지·동적 장애물은 이번 검증 밖이라고 설명합니다.

제안 세 줄 요약:
- 이 연재는 MPPI 계산에서 Gazebo 제어까지 이어지는 12장 학습 과정입니다.
- 최종 입력은 정규화 추력과 몸체 각속도 목표이며 PX4가 내부 제어를 맡습니다.
- 각 단계의 코드와 로그를 확인하면 검증 범위에 맞게 프로젝트를 다시 실행할 수 있습니다.

기술 근거: `series_audit.json`의 17개 본편 지도, 핵심 소스 일치, 이번 실제 mode/수신/응답 기록입니다.
7-1 /89와 7-2 /90의 교정 검사는 기존 배분 rank {c7['old_mixer_rank']}와 수정 rank
{c7['corrected_mixer_rank']}, 100조건의 회전/Euler/관성 교차항 재검증을 포함합니다.
옛 회전행렬은 맞으며 몸체 rate를 Euler rate로 그대로 적분하는 부분을 고쳐야 합니다.
SI 추력[N]·토크[N m]·모터 속도[rad/s]에서 정규화 추력[1]·FLU rate[rad/s] 목표로
넘어가는 이유는 PX4가 rate 제어·배분을 계속 맡기 때문입니다. 토크의 이름만 바꾸지 않습니다.
이 자료는 교정 근거이며 /89·/90에 새 수정이 적용됐다는 서버 증거는 아닙니다.

## 2. 비교 평가 /130 (12-1)

대상 ID: 130, https://sunggoo.tistory.com/130.
원문 근거: `prior_evidence/12-1/article.md`, SHA-256 `{rows[130]['manuscript_sha256']}`와
`prior_evidence/12-1/result.json`, 현재 기존 결과의 원본 ULog·MCAP 경로는 `series_audit.json`에 있습니다.
옛 글은 명목 세 쌍과 +8% 호버 가정의 추가 한 번을 다뤘으며 결과/조건을 보존합니다.
이번 seed7 한 쌍을 그 세 반복의 대체 통계처럼 쓰지 않고, 대표 재현이 끝난 12-2의
확정 URL과 새 ZIP/로그 구조를 뒤쪽에 연결하는 제안입니다.

제안 세 줄 요약:
- 같은 궤적 과제에서 PX4 위치 제어와 MPPI 추력·각속도 제어를 비교합니다.
- 원본 로그의 고정 평가 구간과 실패 시도를 함께 남겨 오차와 계산 시간을 측정합니다.
- 독자는 서로 다른 제어 계층과 입력 단위를 구별하며 제한된 비교를 해석할 수 있습니다.

두 방식 모두 PX4 내부 rate 제어와 모터 배분을 사용하며, 기준에는 외부 속도 feedforward를
주지 않습니다. 계산 시간은 MPPI 최적화와 기준의 참조 생성 시간으로 측정 범위가 다릅니다.
입력 변화 비용을 물리 jerk나 실측 에너지라고 부르지 않습니다. 50ms 목표와 60ms
발행 허용 집계도 구별합니다. 고정 [7,27)초 RMSE와 모든 실패를 원래 기록 그대로 유지합니다.

## 3. 시각자료·링크·적용 상태

목차는 정적 표가 적합하고 /130의 기존 실제 GIF는 원본을 보존한 채 검토합니다.
이번 재현 결과는 정적 오차/최소 이격거리 그래프이므로 GIF를 추가할 이유가 없습니다.
문서의 이전/다음·제목·단위 점검은 보존 자료 기준이며 실제 링크의 현재 HTTP 상태 검증은
게시 담당 단계에서 수행합니다. 비공개 /97·/98은 보호합니다.
서버 반영 상태: 미실행. 자동 공개 수정: 없음.
'''
    (out/'companion_revision.md').write_text(text)
    return text


def figures(out,facts):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':12})
    trials={t['name']:t for t in facts['trials']}
    fig,axes=plt.subplots(4,1,figsize=(7.5,13))
    model=facts['model_runs']['9_2_closed_loop']['result']
    for case in model['cases']:
        if case['hover'] or case['seed']!=7:continue
        d=np.genfromtxt(case['trace'],delimiter=',',names=True)
        error=np.sqrt(sum((d[a]-d['goal_'+a])**2 for a in ['x','y','z']))
        axes[0].plot(d['time_s'],error,label=f"Own model T={case['temperature']:g}")
    axes[0].set_ylabel('Position error [m]');axes[0].set_xlabel('Model time [s]')
    point=np.genfromtxt(Path(trials['10_2_hover_move_return']['output'])/'aligned_point_measurements.csv',delimiter=',',names=True)
    error=np.sqrt(sum(point['error_'+a]**2 for a in ['x','y','z']))
    axes[1].plot(point['elapsed_s'],error,label='10-2 measured Odometry')
    for start in [0,10,20]:axes[1].axvspan(start+6,start+10,color='green',alpha=.1)
    axes[1].set_ylabel('Position error [m]');axes[1].set_xlabel('Point task time [s]')
    for name,label in [('11_1_long_horizon','11-1 MPPI H20'),('12_1_seed7_px4_position','12-1 PX4 position'),('12_1_seed7_mppi','12-1 MPPI')]:
        root=Path(trials[name]['output']);d=np.genfromtxt(root/'aligned_measurements.csv',delimiter=',',names=True)
        error=np.sqrt(sum(d['error_'+a]**2 for a in ['x','y','z']))
        axes[2].plot(d['elapsed_s'],error,label=label)
        cmd=np.genfromtxt(root/'commands.csv',delimiter=',',names=True)
        axes[3].plot(cmd['elapsed_s'],cmd['compute_ms'],label=label)
    axes[2].axvspan(7,27,color='green',alpha=.1);axes[2].set_ylabel('Position error [m]');axes[2].set_xlabel('Circle reference time [s]')
    axes[3].axhline(50,color='r',linestyle='--',label='50ms budget');axes[3].set_ylabel('External calculation [ms]');axes[3].set_xlabel('Circle reference time [s]')
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=10,loc='upper right')
    fig.tight_layout();first=out/'reproduction_tracking.png';fig.savefig(first,dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(7.5,9))
    from .obstacles import OBSTACLE
    for name,label,color in [('11_2_weak_cost','Weak cost','tab:red'),('11_2_inflated_cost','Inflated cost','tab:blue')]:
        d=np.genfromtxt(Path(trials[name]['output'])/'gazebo_clearance.csv',delimiter=',',names=True)
        axes[0].plot(d['x'],d['y'],color=color,label=label)
        axes[1].plot((d['received_wall_ns']-d['received_wall_ns'][0])/1e9,d['body_clearance_m'],color=color,label=label)
    axes[0].add_patch(plt.Circle((OBSTACLE.x,OBSTACLE.y),OBSTACLE.radius,color='gray',label='Physical cylinder'))
    axes[0].add_patch(plt.Circle((OBSTACLE.x,OBSTACLE.y),OBSTACLE.radius+OBSTACLE.vehicle_radius+OBSTACLE.margin,fill=False,linestyle='--',label='Center exclusion radius'))
    axes[0].set_aspect('equal',adjustable='datalim');axes[0].set_xlabel('Gazebo x [m]');axes[0].set_ylabel('Gazebo y [m]')
    axes[1].axhline(.1,color='green',linestyle='--',label='Required 0.10m margin');axes[1].axhline(0,color='gray')
    axes[1].set_xlabel('Time in observed flight interval [s]');axes[1].set_ylabel('Conservative body clearance [m]')
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=10)
    fig.tight_layout();second=out/'reproduction_clearance.png';fig.savefig(second,dpi=140);plt.close(fig)
    return [first,second]


def archive(out,facts,companion_text):
    path=out/'series_12_2_code.zip'
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted((out/'source').rglob('*')):
            if p.is_file():bundle.write(p,str(p.relative_to(out/'source')))
        for p in sorted((out/'prior_evidence').rglob('*')):
            if p.is_file():bundle.write(p,str(p.relative_to(out)))
        bundle.write(out/'series_audit.json','series_audit.json')
        bundle.writestr('README.md',README);bundle.writestr('companion_revision.md',companion_text)
    return path


def reproduce(out,facts,path,workspace):
    root=out/'fresh_archive_extraction';root.mkdir()
    with zipfile.ZipFile(path) as bundle:bundle.extractall(root)
    matches=all(sha(root/name)==digest for name,digest in facts['source_sha256'].items())
    if not matches:raise RuntimeError('Fresh ZIP extracted source mismatch')
    env={**os.environ,'PYTHONPATH':str(root/'simulation')+':'+os.environ.get('PYTHONPATH',''),
         'MPLCONFIGDIR':str(out/'reproduction_matplotlib')}
    command=[sys.executable,'-m','mppi_series.episode_12_2','--workspace-root',str(workspace),
             '--output-root',str(out/'archive_reproduction'),'--reproduce-child']
    print(json.dumps(dict(stage='fresh_zip_identical_actual_reproduction',planned_live_attempts=len(SCHEDULE))),flush=True)
    with (out/'archive_reproduction.log').open('w') as log:
        child=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
        try:child.wait(timeout=1800)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:child.wait(timeout=90)
            except subprocess.TimeoutExpired:child.kill();child.wait()
    results=sorted((out/'archive_reproduction').glob('*/result.json'))
    result=json.loads(results[-1].read_text()) if results else {}
    report=dict(passed=child.returncode==0 and result.get('passed') is True and result.get('source_sha256')==facts['source_sha256']
                and len(result.get('trials',[]))==len(SCHEDULE)
                and result.get('versions',{}).get('px4_binary_sha256')==facts['versions']['px4_binary_sha256'],
                archive_sha256=sha(path),command=command,cwd=str(root),fresh_extraction=True,
                source_snapshot_matches=matches,actual_live_sitl_rerun=True,actual_attempts=len(result.get('trials',[])),
                live_interface_attempts=sum(t['kind']=='interface' for t in result.get('trials',[])),
                actual_flights=sum(t['kind']!='interface' for t in result.get('trials',[])),
                flight_verified=result.get('flight_verified',False),returncode=child.returncode,
                result=str(results[-1]) if results else None,log=str(out/'archive_reproduction.log'),
                checked_utc=datetime.now(timezone.utc).isoformat(),
                outcome_rule='Nondeterministic measurements/outcomes are retained separately; exact floating-point equality is not required.')
    write(out/'archive_reproduction.json',report)
    if not report['passed']:raise RuntimeError('Fresh ZIP identical experiment replay incomplete; retained report/log')
    return report


def manuscript(out,facts):
    trials={t['name']:t for t in facts['trials']}
    model=facts['model_runs']['9_2_closed_loop']['result'];mc=model['cases']
    point=trials['10_2_hover_move_return']['aligned_odometry_analysis']
    interface=trials['10_1_live_interface']['analysis']
    audit=facts['series_audit'];c7=facts['meaningful_checks']['chapter_7']
    table=[]
    for t in facts['trials']:
        for c in t.get('analysis',{}).get('cases',[]):
            value=c.get('rmse_normal_m');maximum=c.get('max_normal_error_m',c.get('max_position_error_m'))
            table.append(f"| {t['name']}/{c['name']} | {value:.4f} | {maximum:.4f} | {'통과' if c['success'] else '실패'} |") if value is not None and maximum is not None else None
    table='\n'.join(table)
    obstacle='\n'.join(f"| {t['profile']} | {t['analysis']['cases'][0]['minimum_body_clearance_m']:.4f} | {t['analysis']['cases'][0]['collision_contact_report_count']} | {t['analysis']['cases'][0]['final_goal_error_m']:.4f} | {t['analysis']['cases'][0]['outcome']} |"
                        for t in facts['trials'] if t['kind']=='obstacle')
    mapping='\n'.join(f"| [{r['episode']}]({r['url']}) | {r.get('scope',r.get('executable_verification'))} |" for r in audit['chapters'])
    version=facts['versions'];proof=facts.get('archive_reproduction',{})
    child_note=('새 ZIP 추출본에서도 동일한 비무장 연결 1회와 실제 비행 6회, 독립 계산 및 자체 모델 12조건을 다시 실행했습니다.'
                if proof.get('passed') else '이 결과는 새 ZIP 추출본의 하위 재현 실행입니다. 같은 실험을 모두 실행했으며 재귀적인 ZIP 재실행만 생략했습니다.')
    timing=[]
    for t in facts['trials']:
        if t['kind']=='interface':continue
        cmd=np.atleast_1d(np.genfromtxt(Path(t['output'])/'commands.csv',delimiter=',',names=True))
        gaps=np.diff(cmd['ros_ns'])/1e9
        timing.append(f"| {t['name']} | {np.mean(cmd['compute_ms']):.3f} / {np.max(cmd['compute_ms']):.3f} | {int(sum(cmd['compute_ms']>50))} | {np.mean(gaps)*1000:.2f} / {np.max(gaps)*1000:.2f} | {int(sum(gaps>.05))} / {int(sum(gaps>.06))} |")
    timing='\n'.join(timing)
    text=f'''# {TITLE}

## 세 줄 요약

- MPPI 계산부터 Gazebo 제어까지 대표 실험을 다시 실행하며 연재를 마무리합니다.
- 원본 상태와 명령, FCU 수신 로그를 연결하고 새 ZIP 추출본에서도 같은 검증을 수행합니다.
- 독자는 코드와 설정의 역할을 구별하고 실패를 포함한 재현 결과를 해석할 수 있습니다.

처음부터 프로젝트를 다시 실행했을 때 어느 단계까지 확인할 수 있을까요? `episode_12_2.py`로 계산·폐루프·실제 ROS 연결과 Gazebo 대표 과제를 다시 실행했습니다. 독립 모델 12조건 중 {sum(c['success'] for c in mc)}조건이 추종 기준을 통과했으며, 실제 로그로 계산한 호버링·이동 과제의 전체 위치 RMSE는 {point['rmse_all_m']:.4f}m였습니다. 실패 과제와 증거 검증의 실패를 구별하는 SITL 재현입니다.

[[FIGURE_TRACKING]]

그림 1. 이번 실행의 자체 모델 원본 CSV와 실제 Odometry의 시각에 맞춘 위치 오차, 외부 계산 시간을 보여 드립니다. 초록 음영은 코드에 미리 정한 평가 구간입니다. 자체 모델과 Gazebo 결과는 서로 다른 패널로 구분하며 기준의 참조 생성 시간은 PX4 내부 계산 시간이 아닙니다.

이전 [12-1 비교 평가 /130](https://sunggoo.tistory.com/130)은 세 반복의 제한적인 비교였습니다. 이번에는 같은 핵심 소스의 대표 조건을 다시 실행하고, [전체 목차 /83](https://sunggoo.tistory.com/83)의 앞 장과 어떤 증거로 이어지는지 정리합니다. 기본 설치나 Offboard 이륙 설명을 반복하기보다 결과 폴더를 읽는 방법에 집중합니다.

## 1. 프로젝트는 상태에서 명령과 응답으로 연결됩니다

상태는 `[x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r]`입니다. 위치·선속도는 ENU 세계 좌표[m, m/s], 자세는 FLU 몸체에서 ENU로 향하는 ZYX 각[rad], p·q·r은 FLU 몸체 각속도[rad/s]입니다. 몸체 각속도를 Euler 각의 변화율로 그대로 적분하지 않습니다.

MPPI가 반환하는 `[c,p_sp,q_sp,r_sp]`는 정규화 추력[무차원]과 몸체 각속도 목표[rad/s]입니다. `controller.py`가 예측·샘플링·비용·갱신을 담당하고, `ros_interface.py`가 실제 Odometry를 같은 계약으로 바꿉니다. mask128 `AttitudeTarget`에는 FLU rate를 그대로 넣습니다. MAVROS가 FRD로 한 번 변환하며 PX4는 내부 rate 제어·제어 배분·모터 출력을 계속 맡습니다.

7편의 SI 추력[N]·토크[N m]·모터 회전속도[rad/s]와 이 목표 입력은 같은 물리량이 아닙니다. `rigid_body_reference.py`는 SI 수식 점검용이고 실제 연결에는 검증된 정규화/rate 모델을 사용합니다. 옛 모터 예제의 배분 rank는 {c7['old_mixer_rank']}, 수정 참조는 {c7['corrected_mixer_rank']}였습니다. 보존된 원문을 100조건으로 실행해 회전행렬은 맞고, 몸체 rate의 Euler 적분과 누락된 관성 교차항을 고쳐야 한다는 근거를 다시 얻었습니다. 옛 코드를 그대로 비행에 넣거나 토크의 이름만 rate로 바꾸지 않습니다.

추력 관계 `T/m=9.81*c/c_hover`와 0.08초 rate 시간상수는 근사 가정입니다. 매 실제 비행의 정지 호버 명령으로 c_hover를 초기화하지만 추력 곡선이나 PX4 전체 응답을 식별한 것은 아닙니다.

## 2. 이번 실행 범위와 환경을 고정합니다

정확한 PX4 commit은 `{facts['px4_commit']}`이며 실행 바이너리 SHA-256은 `{version['px4_binary_sha256']}`입니다. Docker 이미지 ID는 `{version['image_id']}`입니다. 저장소의 dirty diff와 실제 실행 바이너리는 별도로 보존했습니다. 현재 소스 트리에서 이 바이너리를 새로 빌드했다고 주장하지 않습니다.

```text
{version['host_packages'].strip()}
Python packages: {version['python_packages']}
```

기존 Ubuntu 24.04·ROS 2 Jazzy 환경, MAVROS·MCAP·Docker·빌드된 PX4/Gazebo 작업공간이 필요합니다. ZIP에는 이번 애플리케이션 소스와 설정, 앞 장의 원고·감사 자료가 들어 있습니다. 운영체제·Docker 이미지·모든 과거 대용량 로그가 함께 들어 있는 배포 이미지는 아닙니다. 정확한 world/model/init/plugin 자산과 바이너리는 실제 trial의 `provenance/runtime_assets.tar.gz`, 버전·diff는 `provenance/manifest.json`에 남습니다.

이번 순서는 독립 계산, 자체 모델 12조건, 비무장 실제 인터페이스, 호버링·이동·복귀, H20 시간 궤적, 약한/팽창 장애물 비용, seed7 기준/MPPI 한 쌍입니다. sample500·dt0.05초·온도3·기존 노이즈와 비용을 실제 비행에 유지합니다. 자체 모델은 원래 Config와 온도1·3·10, 시드7·11·23을 사용합니다. 실제 물리 기체 비행은 없습니다.

## 3. 새 폴더에서 같은 진입점을 실행합니다

ZIP을 빈 폴더에 푼 뒤 다음 명령을 실행합니다. `--workspace-root`는 외부 PX4 런타임과 이전 근거가 있는 기존 프로젝트를 가리킵니다. `PYTHONPATH`의 맨 앞은 방금 푼 코드로 두므로 저장소의 다른 애플리케이션 코드로 바뀌지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.episode_12_2 \\
  --workspace-root /home/sg/stack/src \\
  --output-root "$PWD/experiment_logs"
```

마지막 표준 출력은 `{{"output":"절대경로/UTC_고유폴더", "passed":true}}` 형태입니다. 이 경로의 `result.json`에서 먼저 passed와 scope=reproduction을 확인합니다. 파일 생성이나 프로세스 종료만으로 비행 성공을 판단하지 않습니다.

실행기는 기존 SITL lock과 포트를 확인하고 소유한 instance7·system8·ROS domain77의 서버만 사용합니다. `offboard_control`은 해당 실행 폴더에서 빌드하고 실제 설치 launch를 거칩니다. MPPI 비행 평가 중에는 PX4 위치 입력을 보내지 않습니다. PX4 위치 제어를 쓴 이륙·호버 초기화와 AUTO.LAND는 평가에서 제외합니다. 기준 비교 비행은 별도 서버에서 위치 제어로 실행하며 MPPI 적용 비행이라고 표시하지 않습니다.

기본 실행은 새 ZIP을 또 다른 폴더에 풀어 같은 진입점을 다시 실행합니다. `--reproduce-child`는 이 재귀 단계만 생략하는 내부 재현 옵션이며 실제 실험과 원본 저장은 모두 수행합니다. {child_note}

## 4. 증거가 이어져야 적용 비행입니다

`trials/이름/commands.csv`에는 원본 Odometry 시각, 계산 상태, 추력·rate 목표, 실제 명령 시각과 계산 시간이 있습니다. `rosbag/*.mcap`의 실제 Odometry와 `AttitudeTarget`를 대조하고, 같은 시드·상태·미래 참조로 MPPI를 결정적으로 재생합니다. ULog에서는 무장·공중·rate-Offboard, 활성 rate 제어, 비활성 위치·자세 제어와 실제 받은 rate/thrust를 확인합니다. 관측한 timesync offset으로 ROS와 FCU 시각을 맞춘 값 검사는 ±100ms 후보 중 성분 오차5e-5 미만이 95% 이상이어야 합니다. 이 검사는 일대일 전송 지연 측정이 아닙니다.

10-1 비무장 재현에서 실제 계산·메시지 {interface['calculated_commands_verified_against_bag']}개를 확인했고 FCU 일치율은 {interface['command_match_fraction']*100:.2f}%였습니다. 상태를 의도적으로 오래되게 만드는 검사를 실행해 명령 차단과 PX4 stream-loss 기록을 확인했습니다. 이는 연결/차단 검증이며 비무장 상태의 위치 차이를 비행 추종 성능으로 부르지 않습니다.

이번 10-2의 고유 Odometry 시각별 고정 목표 평가 RMSE는 전체 {point['rmse_all_m']:.4f}m, 각 10초 구간 마지막 4초는 {point['rmse_normal_m']:.4f}m, 최대 오차는 {point['max_position_error_m']:.4f}m였습니다. 원래 10-2의 판정은 계산 시점 상태 기준이며, `aligned_point_measurements.csv`에서 독립적으로 계산한 이번 숫자와 구별합니다.

| 대표 과제 | 고정 구간 RMSE[m] | 최대 오차[m] | 원래 과제 기준 |
|---|---|---|---|
{table}

원 궤적의 구간은 참조 시작 뒤 [7,27)초입니다. 장애물 과제가 중단되면 같은 구간에서 확보된 표본만 사용하고 partial_flight를 남깁니다. 불완전 구간을 완전한 30초 결과라고 부르지 않습니다. 시간 궤적은 현재 목표 한 점 대신 예측 지평선의 위치·속도 참조를 사용하는지 실제 `predicted_references.jsonl`로 검증합니다.

## 5. 실패도 다시 실행해 남깁니다

[[FIGURE_CLEARANCE]]

그림 2. 이번 두 장애물 trial의 실제 Gazebo pose 원본으로 그린 경로와 기체 여유 거리입니다. 원은 물리 원통과 중심이 피해야 하는 팽창 반경을 구분합니다. 음수 보수적 여유 거리는 포락 구의 겹침이며 실제 접촉 여부는 원본 contact 기록에서 별도로 확인합니다. 정적 그래프이므로 녹화 화면이나 실제 기체 영상이라고 표현하지 않습니다.

| 장애물 비용 | 최소 기체 여유[m] | 접촉 보고 수 | 최종 목표 오차[m] | 결과 |
|---|---|---|---|---|
{obstacle}

반지름0.12m 원통과 x500을 감싼 0.40m 구, 추가 여유0.10m를 사용합니다. 비용의 geometry와 실제 월드/scene를 대조하며 최소 거리는 실제 Gazebo pose로 계산합니다. 접촉 보고 수는 서로 다른 충돌 사건의 수가 아닙니다. 약한 비용에서 충돌·여유 침범·정체가 발생하면 그대로 실패로 기록합니다. 안전한 두 후보의 평균이 장애물 안에 들어갈 수 있다는 작은 수치 반례도 실행했습니다. soft 비용은 충돌 회피 보장이 아닙니다.

자체 모델 12조건 중 {sum(not c['success'] for c in mc)}조건이 최종 오차0.15m 기준을 통과하지 못했습니다. 온도 변화의 성공만 고르지 않았습니다. 제어기와 자체 모델이 같은 가정을 공유하므로 이 성공을 독립 Gazebo 검증으로 확장하지 않습니다. 의도적 invalid 참조·불완전 평가 구간·변조 소스도 실제로 거부하는지 검사하고 사례 종류를 구별합니다.

`passed=true`는 이 모든 실행·증거 검증과 묶음 재현이 끝났다는 뜻입니다. 과제의 실패는 `cases.success=false`입니다. 실제 명령·상태·ULog의 대응이 빠지거나 소유 서버가 무장 상태로 남으면 passed=false로 끝내고 자료를 보존합니다.

## 6. 시간·입력·비교의 한계를 읽습니다

| 실제 trial | 외부 계산 평균/최대[ms] | 계산50ms 초과 | 발행 간격 평균/최대[ms] | 발행50ms/60ms 초과 |
|---|---|---|---|---|---|
{timing}

예측 적분 간격·계산 목표 주기·발행 간격은 서로 다른 값입니다. 실제 OS 스케줄링의 초과를 숨기지 않습니다. 입력 변화량도 c는 무차원, rate는 rad/s, 기준 위치는 m로 성분별 보고합니다. 이를 한 단위처럼 합하거나 물리 에너지·jerk라고 해석하지 않습니다.

12-1의 세 반복·추가 모델 불일치 결과는 원래 로그와 원고로 유지합니다. 이번 한 쌍은 대표 재현이며 새 통계적 우월성의 근거가 아닙니다. 기준에는 외부 속도 feedforward가 없고 MPPI에는 미래 위치·속도 정보가 있습니다. 두 방식 모두 PX4 내부 rate 제어를 사용하며 순수 알고리즘만 떼어낸 비교가 아닙니다.

## 7. 앞 장의 개념을 파일과 연결합니다

| 기존 본편 | 이번 감사·재현에서 연결한 근거 |
|---|---|
{mapping}

`series_audit.json`은 17개 앞선 본편의 보존 원고, 후반부의 실행 소스·원본 로그·첨부 경로와 해시를 연결합니다. 각 대표 과제의 핵심 코드가 당시 실행 소스와 같은지 확인했으며 새 감사·집필 모듈은 따로 기록합니다. 과거 원문을 현재 서버 본문과 같다고 추정하지 않고, 링크 목록의 정합 점검과 현재 HTTP 접속 검증을 구별합니다.

5·8편의 온도 `exp(-(S-S_min)/temperature)`, clipped 후보 갱신, 비용 offset 불변성과 미래 참조 역할은 작은 실행 예제로 다시 확인했습니다. 8-2의 입력 변화 비용은 jerk·실측 에너지가 아니며 장애물에는 기체 크기와 여유 거리가 필요합니다. 1~4편의 환경·연결·Offboard·패키지 설명은 실제 버전, mode/stream-loss 기록과 새 경로 빌드/launch로 연결했습니다. 오래된 설치 명령 전부를 새 OS에서 검증했다고 주장하지 않습니다.

목차 /83과 비교 평가 /130의 구체적인 세 줄 요약·마지막 연결 제안은 `companion_revision.md`에 있습니다. 기존 원문과 정확한 ID, 변경 이유와 근거를 함께 남겼으며 서버에 적용하지 않았습니다. /97·/98의 비공개 보호 범위도 유지합니다.

## 8. 재현 자료와 연재의 마무리

[[SOURCE_ARCHIVE]]

압축 파일에는 이번 실행의 Python 소스, offboard_control 패키지, server config, 재현 README와 앞 장의 근거 지도·로컬 보완안이 들어 있습니다. 실행 결과 폴더에는 아래 구조로 원본을 보존합니다.

```text
result.json                  # 전체 판정, 입력/좌표계/한계, 소스 해시
source/                      # 실행 소스와 설정의 사본
series_audit.json             # 과거 본편의 버전·원고·로그·첨부 지도
checks/                      # SI/회전/비용/참조/평가/변조 거부 검사
models/                      # 독립 계산과 12조건 자체 모델 원본
trials/                      # 비무장 연결과 여섯 실제 비행의 원본
  */rosbag/*.mcap             # 실제 ROS 상태와 발행 명령
  */runtime/log/**/*.ulg     # FCU mode, 수신 입력, 응답
  */provenance/              # 실제 바이너리/자산/버전/diff
archive_reproduction.json     # 새 추출 경로 명령·판정
archive_reproduction.log      # 하위 재현 표준 출력과 오류
companion_revision.md         # 기존 글 로컬 보완안
```

실행한 코드와 단위·시각·실제 제어 계층을 연결해 읽는 것이 이 연재의 도착점입니다. 12-2에서 확정 목차를 마무리하며 임의 후속 회차를 만들지 않습니다. 실제 기체, 인지·SLAM·동적 장애물, 바람과 강인성 보장은 이번 결과 밖이며 별도의 조건과 실험이 필요합니다.

코드 근거: ZIP의 `controller.py`, `ros_interface.py`, `rigid_body_reference.py`,
`trajectory.py`, `obstacles.py`와 각 `*_results.py`의 검증 구현을 참고해 주세요.
공식 구현 근거는 설치 환경의 MAVROS 변환 소스와 기록한 PX4 commit 수신 경로이며,
실행 시각·정확한 파일 해시·원본 참조는 `series_audit.json`과 trial provenance에 있습니다.
'''
    if text.count('[[FIGURE_')!=2 or text.count('[[SOURCE_ARCHIVE]]')!=1:raise RuntimeError('Manuscript marker contract failed')
    (out/'article.md').write_text(text)


def build(out,facts,workspace,reproduce_child=False):
    text=companion(out,facts);images=figures(out,facts);bundle=archive(out,facts,text)
    if not reproduce_child:facts['archive_reproduction']=reproduce(out,facts,bundle,workspace)
    else:facts['archive_reproduction']=dict(recursive_execution_skipped=True,reason='Fresh-extraction child ran the identical full suite; parent retains its verification report.')
    assets=dict(figures=[str(p.resolve()) for p in images],archive=str(bundle.resolve()),
                figure_sha256={str(p.resolve()):sha(p) for p in images},archive_sha256=sha(bundle),
                visual_decisions={'reproduction_tracking.png':dict(format='static',reason='Compare fixed evaluation windows and units across retained runs.'),
                                  'reproduction_clearance.png':dict(format='static',reason='Read exact path geometry, minimum clearance and failure outcome without motion.')},
                reproduction_report=str(out/'archive_reproduction.json') if not reproduce_child else None,
                reproduction_commands=README[README.index('```bash')+8:README.index('```',README.index('```bash')+8)].strip(),
                gif_created=False,raw_flight_evidence_retained=True)
    write(out/'publication_assets.json',assets);facts['publication_assets']=assets
    facts['companion_revision']=str(out/'companion_revision.md');manuscript(out,facts)
