"""11-2 actual measured figures, Korean manuscript and fresh live ZIP replay."""
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import zipfile
import numpy as np
from .episode_10_1 import write,sha
from .obstacles import OBSTACLE,PROFILES

README='''# 11-2 known static obstacle cost experiment

External dependencies: the existing Ubuntu 24.04/ROS 2 Jazzy/MAVROS/MCAP
installation, NumPy, Matplotlib, pyulog, pymavlink, gz.transport13 and gz.msgs10,
built PX4/Gazebo Harmonic workspace, Docker image docker-px4-sitl:latest,
ffmpeg and /home/sg/.local/share/stack-series-tools/usr/bin/Xvfb for recording.
Exact binary, packages, model/world and Docker IDs are retained in provenance.
This bundle includes application code and config, not the external installation.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.obstacle_checks --workspace-root /home/sg/stack/src --output "$PWD/examples"
/usr/bin/python3 -m mppi_series.episode_11_2 --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

The final stdout JSON includes the absolute output directory. The entry point
runs three actual Gazebo flights, checks original MCAP commands/Odometry and
ULog FCU receipt/control modes, measures actual contact reports and conservative
clearance, records its own Gazebo GUI, builds a source ZIP and executes all three
actual flights from a fresh extraction. The isolated runtime owns PX4 instance
7/system 8/ROS domain 77 and refuses busy ports or the existing experiment lock.
PX4 position control performs takeoff; only the explicit MPPI body-rate/thrust
interval is evaluated. AUTO.LAND is used for completion or safety abort. If a collided owned SITL
model cannot land/disarm within 60s, an identity-gated test-only force-disarm
is requested and actual disarmed state must be observed. The trial still fails.
All attempts are retained. passed means evidence/execution verification, not
that every avoidance trial succeeded. A gate abort is a failed trial; its
partial interval is clearly labelled and is verified if sufficient original
evidence and disarmed recovery exist. --reproduce-child skips only recursive
ZIP packaging and duplicate recording; it still runs all actual flights.
All normal invocations include a genuinely animated same-run screen GIF.
'''


def companion(out,facts):
    example=json.loads((out/'cost_examples.json').read_text())
    text=f'''# 11-2에 연결할 8-2편 교정안 — 서버 미반영

대상은 [8-2 /93](https://sunggoo.tistory.com/93) 한 글입니다. 보존 원문은 `source/simulation/mppi_series/reference_inputs/93_public_20261007_from_preserved.txt`입니다. 이는 이전 서버 확인 때 보존한 사본이며 이번 실행에서 최신 서버 본문을 다시 조회한 것으로 쓰지 않습니다. 기존 URL의 원본 백업·같은 ID 저장·재접속은 별도 발행 과정의 작업입니다. 이 자료는 아직 서버에 적용되지 않았습니다. 기존 글의 GIF 추가도 12-2 공개 검증 뒤입니다.

## 1. 원래 장애물 코드와 교정 이유

원문 `obstacle_cost`는 `penetration = obs_radius - center_distance`가 양수일 때만 비용을 부과합니다. 기체 중심이 원기둥 안에 들어가기 전에는 0이며 기체 크기·여유 거리도 없습니다. ‘거리에 반비례’라는 문장은 실제 `exp(5*penetration)-1` 식과 다릅니다. 중심 거리만으로 충돌 회피를 보장한다고 읽히지 않도록 고칩니다.

제안: `d = center_distance - obstacle_radius - vehicle_radius`, `d_margin = d - margin`을 사용합니다. 11-2의 원기둥 반경은 0.12m, 기체 반경은 충돌 box와 rotor 배치를 감싸는 0.40m, 여유는 0.10m입니다. 도식은 `companion_clearance.png`, 계산 예제는 `cost_examples.json`, 실제 소스 기반 기체 경계 검사는 `meaningful_checks.json`입니다. 실제 접촉과 보수적 envelope 침범은 별도 값으로 보여 드립니다.

```python
import numpy as np
from mppi_series.obstacles import OBSTACLE
points = np.array([[.85, 0., 2.], [.85, .52, 2.], [.85, .62, 2.]])
print(OBSTACLE.clearance(points))  # 실행 결과는 cost_examples.json에 보존
print(OBSTACLE.penalty(points))
```

작은 예제의 실제 이격거리[m]: `{example['clearance_m']}`입니다. 두 개의 안전한 반대쪽 후보를 평균하면 원기둥 가운데로 갈 수 있는 반례도 직접 실행했습니다. 예측 후보의 낮은 비용이 실제 명령의 안전 보장은 아닙니다.

## 2. 비용 스케일·입력·jerk·에너지 용어

원문은 `sum(u**2)`를 모터 속도 비용과 에너지 절약으로 설명하고 입력 차이를 `jerk_cost`라 부릅니다. 최종 입력은 무차원 c와 FLU rad/s의 rate 목표입니다. 수정 제안은 NOISE로 성분별 스케일을 맞춘 호버링 입력 편차와 입력 변화량 비용입니다. 물리 jerk는 가속도의 시간 미분[m/s³]이며 입력 차이와 다릅니다. 입력 제곱을 실제 배터리 에너지[J]라고 부르려면 전압·전류·시간 측정이 추가로 필요합니다.

```mermaid
flowchart LR
    A[실제 원기둥 형상] --> D[중심 거리에서 장애물·기체 반경 빼기]
    B[기체 충돌 형상] --> D
    D --> E[여유 거리·영향 구간을 적용한 soft 비용]
    E --> F[다른 위치·속도·입력 비용과 합산]
    F --> G[후보 가중 평균·첫 추력/rate 목표]
    G --> H[실제 Gazebo 상태·접촉·이격거리 검사]
```

근거는 실행한 `obstacles.py`, `obstacle_checks.py`, `obstacle_results.py`와 원본 Gazebo/MCAP/ULog입니다. 원래 비용이 잘못된 코드라는 결론보다 ‘점 기체의 내부 침범 비용을 유한 크기 기체의 실제 회피 검증으로 확장해야 한다’는 차이를 설명합니다. soft 비용에는 충돌 회피 보장이 없습니다.
'''
    (out/'companion_revision.md').write_text(text)
    return text


def build(out,facts,workspace):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from .episode_11_2 import TITLE
    trials=facts['trials'];cases=[t['analysis']['cases'][0] for t in trials]
    fig,axes=plt.subplots(3,1,figsize=(10,11))
    for trial in trials:
        root=Path(trial['output']);d=np.genfromtxt(root/'aligned_measurements.csv',delimiter=',',names=True)
        truth=np.genfromtxt(root/'gazebo_clearance.csv',delimiter=',',names=True)
        label=trial['profile']
        axes[0].plot(d['elapsed_s'],np.sqrt(sum(d['error_'+n]**2 for n in ['x','y','z'])),label=label)
        axes[1].plot(truth['x'],truth['y'],label=label)
        axes[2].plot((truth['received_wall_ns']-trial['flight']['reference_epoch_ros_ns'])/1e9,truth['body_clearance_m'],label=label)
    o=OBSTACLE
    axes[1].plot(d['ref_x'],d['ref_y'],'k--',label='Identical straight reference')
    axes[1].add_patch(Circle((o.x,o.y),o.radius,color='red',alpha=.5,label='Actual cylinder'))
    axes[1].add_patch(Circle((o.x,o.y),o.radius+o.vehicle_radius+o.margin,fill=False,color='red',linestyle='--',label='Inflated clearance'))
    axes[0].set_xlabel('Time after handover [s]');axes[0].set_ylabel('Measured timed-reference error [m]')
    axes[1].set_xlabel('ENU x [m]');axes[1].set_ylabel('ENU y [m]');axes[1].axis('equal')
    axes[2].axhline(.1,color='red',linestyle='--',label='Required 0.10m margin')
    axes[2].axhline(0,color='black',linestyle=':',label='Conservative body overlap')
    axes[2].set_xlabel('Time after handover [s]');axes[2].set_ylabel('Ground-truth body clearance [m]')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'obstacle_error.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,5))
    for radius,label in [(o.radius,'Cylinder'),(o.radius+o.vehicle_radius,'Body envelope'),(o.radius+o.vehicle_radius+o.margin,'Required margin')]:
        ax.add_patch(Circle((0,0),radius,fill=False,label=label))
    ax.set_xlim(-.9,.9);ax.set_ylim(-.9,.9);ax.set_aspect('equal');ax.legend();ax.set_xlabel('Illustrative offset x [m]');ax.set_ylabel('Illustrative offset y [m]')
    fig.tight_layout();fig.savefig(out/'companion_clearance.png',dpi=140);plt.close(fig)
    companion_text=companion(out,facts)
    captured=next(t for t in trials if t['profile']=='inflated_cost')
    gif=Path(captured.get('screen_gif',''))
    if not gif.is_file() or not gif.resolve().is_relative_to(out):raise RuntimeError('Same-run animated Gazebo screen GIF missing')
    screen=captured['screen_capture'];gif_asset=out/'gazebo_obstacle.gif';shutil.copy2(gif,gif_asset)
    archive=out/'series_11_2_code.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted((out/'source').rglob('*')):
            if p.is_file():bundle.write(p,str(p.relative_to(out/'source')))
        bundle.writestr('README.md',README);bundle.writestr('companion_revision.md',companion_text)
    with tempfile.TemporaryDirectory(prefix='stack-11-2-reproduce-') as directory:
        root=Path(directory)
        with zipfile.ZipFile(archive) as bundle:bundle.extractall(root)
        matches=all(sha(root/name)==digest for name,digest in facts['source_sha256'].items())
        if not matches:raise RuntimeError('ZIP snapshot mismatch')
        env={**os.environ,'PYTHONPATH':str(root/'simulation')+':'+os.environ.get('PYTHONPATH',''),'MPLCONFIGDIR':str(out/'reproduction_matplotlib')}
        command=[sys.executable,'-m','mppi_series.episode_11_2','--workspace-root',str(workspace),'--output-root',str(out/'archive_reproduction'),'--reproduce-child']
        with (out/'archive_reproduction.log').open('w') as log:
            child=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=1000)
            except subprocess.TimeoutExpired:
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=90)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        results=sorted((out/'archive_reproduction').glob('*/result.json'));reproduced=json.loads(results[-1].read_text()) if results else {}
        proof={'passed':child.returncode==0 and reproduced.get('passed') is True and reproduced.get('source_sha256')==facts['source_sha256'],
               'source_zip_sha256':sha(archive),'command':command,'fresh_extraction':True,'actual_live_sitl_rerun':True,
               'flight_verified':reproduced.get('flight_verified',False),'returncode':child.returncode,
               'result':str(results[-1]) if results else None,'source_snapshot_matches':matches,'checked_utc':datetime.now(timezone.utc).isoformat()}
        write(out/'archive_reproduction.json',proof)
        if not proof['passed']:raise RuntimeError('Fresh ZIP actual obstacle flight reproduction failed')
    write(out/'publication_assets.json',{'figures':[str(gif_asset),str(out/'obstacle_error.png')],'archive':str(archive),
          'figure_sha256':{str(gif_asset):sha(gif_asset),str(out/'obstacle_error.png'):sha(out/'obstacle_error.png')},
          'gif_metadata':screen,'reproduction_report':str(out/'archive_reproduction.json')})
    table='\n'.join(f"| {c['name']} | {c['horizon_s']:.2f} | {PROFILES[c['name']]['weight']:.0f} | {c['minimum_body_clearance_m']:.4f} | {c['collision_contact_report_count']} | {c['final_goal_error_m']:.4f} | {c['outcome']} / {'통과' if c['success'] else '실패'} |" for c in cases)
    errors='\n'.join(f"| {c['name']} | {c['rmse_all_m']:.4f} | {c['max_position_error_m']:.4f} | {c['compute_mean_ms']:.3f} | {c['compute_max_ms']:.3f} | {c['compute_overrun_count']} | {c['publish_period_max_s']:.4f} | {c['evaluated_duration_s']:.2f} / {'완료' if c['completed_full_task'] else '중단'} |" for c in cases)
    receipts='\n'.join(f"- {t['profile']}: 계산 명령 {t['analysis']['mppi_commands']}개, FCU 대조 {t['analysis']['clock_aligned_pairs']}쌍, 값 일치 {t['analysis']['command_match_fraction']*100:.2f}%, 결정적 계산 재생 차이 {t['analysis']['independent_mppi_replay_max_error']:.3g}." for t in trials)
    outcomes='; '.join(f"{c['name']}: {c['outcome']}" for c in cases)
    gates='\n'.join(f"- {t['profile']}: {t['flight'].get('error','30초 MPPI 과제 완료')}; 복구={t['flight'].get('recovery','AUTO.LAND 뒤 실제 비무장 확인')}; 최종 무장={t['flight']['final_armed']}." for t in trials)
    text=f'''# {TITLE}

장애물 비용을 넣으면 실제로 우회하며 목표에 도착할까요? 같은 직선 참조를 사용한 Gazebo SITL에서 비용 가중치와 예측 길이를 바꾸고, 실제 접촉과 최소 이격거리·목표 오차를 따로 측정했습니다. 이번 실행의 판정은 {outcomes}였으며, 비용이 크다는 이유만으로 안전이나 목표 도달을 보장하지는 않습니다.

[[FIGURE_GAZEBO]]

GIF 1. 이번 inflated_cost 조건과 같은 실행 서버의 실제 Gazebo GUI 녹화입니다. MPPI 인계 1초 뒤부터 연속 27초를 3배속으로 보여 드립니다. {screen['gif_frames']}프레임·{screen['gif_duration_ms']/1000:.1f}초이며 SITL입니다. 원본 MP4, 녹화·변환 명령, 자른 구간, 서버 partition·실행 경로, SHA-256은 `screen_capture.json`에 있습니다. 궤적을 다시 그린 영상이나 실기체 촬영이 아닙니다.

[전체 목차](https://sunggoo.tistory.com/83#drone-series-toc)의 11-2편입니다. [11-1 궤적 추종](https://sunggoo.tistory.com/107)의 시간별 미래 참조와 [10-2 실제 입력 연결](https://sunggoo.tistory.com/106)을 재사용합니다. 선수 개념은 [8-2 비용 함수](https://sunggoo.tistory.com/93)이며 설치·기본 Offboard 절차는 여기서 반복하지 않습니다.

## 1. 중심이 바깥에 있어도 기체는 부딪힐 수 있습니다

장애물은 세계 ENU (0.85,0)m에 둔 반경 0.12m·높이 5m의 알려진 정적 원기둥입니다. 비용용 정보를 실제 월드 SDF의 collision/visual과 맞추고, 실행 서버의 Scene 응답으로 위치·반경·높이를 재확인했습니다. 센서 인지·SLAM·동적 장애물 탐지는 없습니다. 원기둥은 허용 비행 높이 0.5~4m 전체를 막으므로 수평 우회를 평가합니다.

기체는 x500_base의 base·다리·rotor collision box를 감싸는 회전 불변 반경 0.40m의 구로 보수적으로 표현했습니다. 각 link 위치와 collision 중심 거리, box 반대각선 반 길이의 합을 검사했으며 최대 경계는 {max(b['sphere_bound_m'] for b in facts['meaningful_checks']['collision_bounds']):.6f}m였습니다. 이는 실제 접촉 형상과 같다는 가정이 아니라 모든 충돌 box를 포함하는 여유 있는 envelope입니다. propeller의 Gazebo collision box는 실제 실기체 보호 공간을 식별한 값이 아닙니다.

$$d=\\lVert p_{{xy}}-o_{{xy}}\\rVert-r_{{obs}}-r_{{vehicle}},\\quad d_{{margin}}=d-0.10$$

d<0이면 보수적 기체 구가 겹치고 d<0.10m이면 지정한 여유 거리를 만족하지 못합니다. 이 두 판정과 Gazebo의 실제 collision contact는 서로 다릅니다. 중심이 원기둥 바깥이라는 사실만으로 여유 거리가 확보됐다고 하지 않습니다.

## 2. 장애물 내부에 들어가기 전부터 비용을 줍니다

영향 거리 L=0.25m를 두고 `max(0, 1-d_margin/L)²`를 사용합니다. 여유 경계 안팎에서 연속이며 더 가까운 후보의 비용이 커집니다. 지수 폭발이나 무한 장벽이 아니라 유한한 soft 비용입니다.

```python
import numpy as np
from mppi_series.obstacles import OBSTACLE
points = np.array([[.85, .52, 2.], [.85, .62, 2.], [0., 0., 2.]])
print(OBSTACLE.clearance(points))
print(OBSTACLE.penalty(points))
```

첫 점은 기체 envelope가 원기둥에 닿는 거리이고 두 번째는 0.10m 여유를 확보한 거리입니다. 이 실행 예제의 실제 값은 `cost_examples.json`에 있습니다. 장애물 비용은 `weight × (dt × Σ penalty + terminal_penalty)`로 기존 위치·속도·자세·입력 비용에 더합니다. 위치·속도·기울기·입력 비용 스케일은 11-1과 같으며 λ=3, 샘플 500, dt=0.05s, 시드 7을 유지합니다. 입력 성분은 NOISE로 나눕니다. 입력 변화량 비용을 물리 jerk[m/s³] 또는 배터리 에너지[J]로 해석하지 않습니다.

## 3. 미리 그린 우회 경로는 주지 않습니다

인계 후 3초 정지, 20초 동안 cubic smoothstep으로 x=시작점+1.7m·y=시작점+0.12m까지 직선 이동, 마지막 7초 정지라는 같은 참조를 모든 조건에 줍니다. y=0.12m는 대칭을 약하게 깨는 명시적 설정입니다. 이 직선은 여전히 팽창 원기둥을 통과하며 우회 waypoint는 없습니다. 예측 첫 상태는 t+dt의 목표와 비교하고 미래 H스텝 전체의 위치·속도 참조를 사용합니다.

```python
import numpy as np
from mppi_series.controller import Config
from mppi_series.obstacles import ObstacleController, horizon_reference, PROFILES
profile = 'inflated_cost'
cfg = Config(horizon=PROFILES[profile]['horizon'], samples=500,
             dt=.05, temperature=3., seed=7)
state = np.zeros(12); state[2] = 2.
times, plan = horizon_reference(10., [0., 0., 0.], cfg)
command = ObstacleController(cfg, profile).step(state, plan)
print(times, command)
```

이 작은 예제는 자체 모델 계산입니다. 실제 비행 진입점은 7절에 있습니다. `[c,p_sp,q_sp,r_sp]`에서 c는 무차원 정규화 추력이고 rate 목표는 몸체 FLU rad/s입니다. MAVROS mask 128이 FLU→FRD를 한 번 변환하며 PX4는 내부 rate 제어·제어 배분·모터 출력을 계속 수행합니다. SI 토크나 모터 회전속도를 rate 목표로 이름만 바꾼 것이 아닙니다.

## 4. 실제 제어 인계와 접촉 기록을 검증합니다

각 조건은 별도의 소유된 PX4 instance 7/system 8/Gazebo 서버에서 실행했습니다. 이륙·초기 호버링은 PX4 위치 제어로 수행하고 최근 정지 호버링의 정규화 명령 중앙값으로 c_hover를 초기화했습니다. 이는 추력 곡선 식별값이 아니며 rate_tau=0.08s와 국소 선형 추력 관계는 모델 가정입니다. MPPI 구간에는 위치 목표 발행을 끄고 추력/rate 입력만 적용하며 위치 제어·AUTO.LAND 구간은 평가에서 제외했습니다.

{receipts}

MCAP의 원본 Odometry가 계산 상태와 일치하고 실제 `AttitudeTarget`가 계산 명령과 일치하는지 확인했습니다. 같은 상태·시드·미래 참조로 MPPI를 다시 계산했습니다. ULog timesync offset으로 시간 창을 맞춰 FCU의 무장·공중·rate-Offboard·위치/자세 제어 비활성 및 수신 rate/thrust 값을 검사했습니다. ±100ms 창 안에서 성분 오차 5e-5 미만 비율이 95% 이상이어야 합니다. 이것은 값 전달 확인이며 일대일 지연 측정은 아닙니다.

Gazebo 서버 pose 메시지를 별도로 기록해 실제 월드 위치의 이격거리를 계산하고 Odometry의 ENU 원점과 대조했습니다. 원기둥 contact sensor의 실제 기본 토픽과 지면 contact sensor를 구독했습니다. 지면–기체 접촉이 관측된 기록으로 같은 실행의 contact 관측 경로가 동작했는지 확인하며, 원기둥–기체 contact report만 충돌 보고로 셉니다. report 수는 별개의 충돌 사건 횟수가 아닙니다. 비접촉 관측이 모든 미래의 충돌 방지를 보장하지도 않습니다.

## 5. 우회·정체·충돌을 따로 판정합니다

성공 기준은 실행 전에 정한 ‘최소 보수적 기체 이격거리 ≥0.10m, 원기둥 실제 접촉 0 report, 마지막 목표 오차 <0.25m, 30초 과제 완료’입니다. 각 조건 한 번씩 고정 순서이며 반복 통계·성능 우월성 검증은 아닙니다. 별도 호버링 초기화와 OS 스케줄링은 완전히 동일한 측정값이 아닙니다.

| 조건 | 예측 [s] | 장애물 가중치 | 최소 기체 이격 [m] | 접촉 report | 마지막 목표 오차 [m] | 관측/사전 판정 |
|---|---|---|---|---|---|---|
{table}

`avoided_and_arrived`는 기준을 만족한 경우, `stalled`는 여유를 확보했지만 도착 기준을 만족하지 못한 경우입니다. `clearance_violation`은 보수적 여유 기준 실패이며 실제 접촉과 구별합니다. `contact_collision`은 원기둥–기체 접촉이 기록된 경우입니다. 안전 gate로 조기 종료된 조건은 과제 실패입니다. `result.json`의 passed는 필요한 실행·증거 대조가 완료됐다는 뜻이며 모든 시도의 회피 성공을 의미하지 않습니다.

[[FIGURE_ERROR]]

그림 2. 원본 Odometry의 측정 시각마다 계산한 시간 참조 오차, 실제 Gazebo 세계 궤적, 보수적 기체 이격거리입니다. 점선 원은 장애물·기체 반경·0.10m 여유를 합친 경계입니다. 같은 입력/측정 로그에서 계산했으며 원본은 `aligned_measurements.csv`, `gazebo_clearance.csv`에 있습니다.

## 6. 회피에는 비용·예측·모델·시간 제약이 함께 작용합니다

| 조건 | 전체 오차 RMSE [m] | 최대 오차 [m] | 평균 계산 [ms] | 최대 계산 [ms] | 계산 50ms 초과 | 최대 발행 간격 [s] | 실제 평가 길이 [s] / 과제 |
|---|---|---|---|---|---|---|---|
{errors}

직선 참조를 우회하면 추종 오차가 증가할 수 있으므로 RMSE만으로 회피 품질을 판정하지 않습니다. 조기 종료 조건의 전체 RMSE는 완료 조건보다 짧은 실제 구간에서 계산됐으며 같은 길이의 통계라고 비교하지 않습니다. 고정 [7,27)초 보조 평가의 완결 여부도 analysis.json에 남겼습니다. 오차가 작은 표본만 고르지 않았습니다.

{gates}

상태 나이·실제 연결/무장/OFFBOARD·높이 0.5~4m·수평 4m·tilt 0.6rad·계산 50ms를 검사하고 거부하면 실패를 기록한 뒤 AUTO.LAND를 요청합니다. 충돌한 모델이 60초 안에 착륙·비무장하지 못하면 소유된 instance 7/system 8/SITL 표식을 다시 확인한 뒤 시험 전용 강제 비무장을 요청하고 실제 상태 메시지의 비무장을 확인합니다. 이 경우에도 과제는 실패이며 복구 명령·ACK·상태·오류를 보존합니다. 이 gate와 강제 종료를 실제 하드웨어 안전 장치라고 해석하지 않습니다. 실제 Gazebo 접촉·중단이 발생할 수 있으며 모든 실패 로그를 보존합니다.

기하 검사에서는 안전한 좌우 후보를 평균하면 장애물 가운데로 향하는 반례도 실행했습니다. clipping과 결정적 기준 후보를 포함하는 교육용 sampling controller이며 정확한 MPPI 분포·전역 최적성·수학적 충돌 회피 보장은 주장하지 않습니다. 실제 속도·자세·rate 지연이 예측과 다르면 비용이 예상한 궤적과 실제 궤적도 다릅니다.

## 7. 같은 소스를 새 경로에서 실제 실행합니다

실제 PX4 commit은 `{facts['px4_commit']}`입니다. 선택한 바이너리 SHA-256, dirty source diff·x500 모델·기본 월드 자산·Docker image ID·ROS/MAVROS/Gazebo/Python 버전은 각 trial의 `provenance/manifest.json`에 있습니다. 이번에 생성해 실행한 장애물 월드는 `runtime/worlds/default.sdf`와 `runtime/world_geometry.json`으로 별도 보존했습니다. 소스 diff만으로 바이너리를 재빌드했다고 주장하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.obstacle_checks \\
  --workspace-root /home/sg/stack/src --output "$PWD/examples"
/usr/bin/python3 -m mppi_series.episode_11_2 \\
  --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

ZIP은 응용 소스·설정·실행 설명을 포함하며 외부 ROS/PX4/Gazebo 설치는 이미 준비돼 있어야 합니다. 새 디렉토리에 ZIP을 풀고 문서화한 episode 진입점으로 세 실제 비행·원본 MCAP/ULog·geometry/receipt 분석을 재실행했습니다. 자식 실행은 중복 녹화와 재귀 ZIP 생성만 생략합니다. `archive_reproduction.json`·로그에 실제 결과와 소스 해시 일치를 남겼으며 결과 수치의 bit 단위 동일성을 요구하지 않습니다. 마지막 stdout JSON의 output은 고유 UTC 결과 디렉토리의 절대 경로입니다.

[[SOURCE_ARCHIVE]]

8-2 /93의 내부 침범 비용·기체 크기·입력 변화/jerk·에너지 용어 교정안은 `companion_revision.md`에 있습니다. 이번 구현 작업은 기존 글을 서버에서 고치지 않았습니다. 근거는 실행 소스와 원본 서버 메시지·비행 로그이며 구체적인 적용 범위는 각 결과 JSON에 분리해 두었습니다.

## 8. 다음 질문: 같은 조건을 반복하면 결론이 유지될까요?

알려진 정적 월드에서 soft 비용과 예측 길이에 따른 실제 행동·실패를 확인했습니다. 동적 장애물·인지 오차·외란·실기체·MPPI 우월성까지 검증한 결과는 아닙니다. 다음 12-1편(예정)에서는 같은 과제·조건의 반복 실험, PX4 내부 제어 사용 범위, RMSE·최대 오차·입력 단위·계산 시간·주기 초과를 함께 비교하겠습니다.
'''
    (out/'article.md').write_text(text)
    facts.update(archive_reproduction=proof,publication_assets=str(out/'publication_assets.json'),
                 companion_revision={'post_ids':[93],'status':'local_proposal_only','file':str(out/'companion_revision.md')})
