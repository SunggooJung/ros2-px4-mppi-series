"""11-1 Korean manuscript/assets, small companion examples, fresh live ZIP run."""
from datetime import datetime, timezone
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
from .episode_10_1 import write, sha
from .episode_11_1 import TITLE


README='''# 11-1 timed Gazebo trajectory tracking

External runtime: Ubuntu 24.04, ROS 2 Jazzy, MAVROS 2.14, rosbag2 MCAP,
NumPy, Matplotlib, pyulog, pymavlink, Docker image docker-px4-sitl, and the
already-built PX4/Gazebo workspace. ffmpeg and the verified Xvfb path
/home/sg/.local/share/stack-series-tools/usr/bin/Xvfb are used for recording.
The ZIP contains executable application source and the server config, not
the external ROS/PX4 binary installation. Its exact identity is in provenance.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.trajectory_checks --output "$PWD/examples"
/usr/bin/python3 -m mppi_series.episode_11_1 --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

The episode owns instance 7/system 8/domain 77 and refuses busy ports or
the existing experiment lock. It runs two real flights with 10/20 prediction
steps and all other prescribed controller/reference parameters held fixed.
Each trial separately measures live hover-command calibration. PX4 controls
takeoff and landing; only the 30s body-rate/thrust interval is evaluated.
Each normal run records its own actual Gazebo GUI, creates a ZIP and repeats
both actual flights from a fresh extraction. --reproduce-child disables
recursive packaging and duplicate recording ONLY; both actual flights,
meaningful checks and original MCAP/ULog analysis still run. All attempts
remain on disk, including verification failures and tracking-limit failures.
The last stdout JSON has an absolute output directory.
'''


def companion(out, facts):
    checks=facts['meaningful_checks']
    text=f'''# 11-1에 연결할 5·8-1편 교정 제안

대상은 [5편 /87](https://sunggoo.tistory.com/87), [8-1편 /92](https://sunggoo.tistory.com/92) 두 글입니다.
이번 작업은 로컬 교정 자료 작성입니다. 서버 저장·개정·공개 반영은 수행하지 않았습니다.
기존 글의 GIF 추가는 12-2 공개 검증 이후입니다. 아래 도식은 정적 설명 자료이며 신규 비행 GIF를 재사용하지 않습니다.

## 1. /87: 온도와 비용 가중치, 주기·예측 길이

확인한 원문은 `reference_inputs/87_public_20261006.html`에 보존했습니다.
기존 설명에는 비용에 온도를 곱하는 지수 표기와 최저 후보 선택을 먼저 설명하는 흐름,
모델 불일치에 대한 강인성 단정, 예측 스텝 수·50Hz·예측 시간의 혼용이 있습니다.
제안: 온도를 나누는 규약으로 통일하고, 평가한 후보 입력의 가중 평균을 설명합니다.
λ를 비용에 곱하는 표기는 역온도 규약이므로 같은 λ로 혼용하면 증감 해석이 반대입니다.
작은 λ는 최소 비용 후보에 집중하고 큰 λ는 더 균등해집니다. 비용 가중치는 무엇을 벌할지,
온도는 주어진 비용 차이를 얼마나 강하게 구분할지 결정합니다. 재계산을 한다는 이유만으로
외란·센서 오차에 대한 강인성이나 전역 최적성을 보장하지 않습니다.
명령 주기 0.05s는 20Hz이고, H=10/20이면 예측 길이는 0.5/1.0s입니다.

직관용 계산은 비행·운동모델 측정이 아닙니다. 고정 비용 `[0,1,3]`에서
온도 .1/1/10의 첫 후보 가중치는 각각 {checks['weights'][0][0]:.6f},
{checks['weights'][1][0]:.6f}, {checks['weights'][2][0]:.6f}로 실제 계산했습니다.
실행 명령: `/usr/bin/python3 -m mppi_series.trajectory_checks --output examples`.
원본은 `small_examples_raw.npz`, 검사 근거는 `meaningful_checks.json`입니다.

## 2. /92: 후보 clipping, warm start, 입력 계약과 시간별 목표

현재 원문은 `reference_inputs/92_public_20261006.html`에 보존했습니다.
보충 문단은 입력 단위 불일치와 effective noise를 이미 경고하지만, 아래 실행 예제는
clipping 이전 노이즈 반환·0 초기화/끝단을 유지하며 미래 시퀀스를 버려도 된다고 설명합니다.
제안: 예제 본체도 평가한 후보의 가중 평균으로 바꾸고 warm start에서 남은 H-1 입력을
다음 주기의 초기 탐색 중심으로 보존합니다. 끝단은 호버링 `[c_hover,0,0,0]`로 채웁니다.
SI 추력·토크 및 모터 rad/s를 정규화 추력·몸체 FLU rate rad/s로 이름만 바꾸지 않습니다.
9~11편은 별도의 내부 rate 근사 모델이며 PX4가 내부 rate 제어와 배분을 맡습니다.

작은 clipping 예제에서 원 후보 `[.2,1.1]`, 제한 `[.1,.9]`, 가중치 `[.25,.75]`를 사용했습니다.
평가 후보 `[.2,.9]`의 평균은 {checks['clipped_weighted_average']:.3f}이고,
원 후보의 평균은 {checks['raw_weighted_average']:.3f}입니다. 이는 설명용 숫자이며 FCU 측정이 아닙니다.
샘플 N은 후보 개수, H는 스텝 수, H·dt는 미래 시간 길이입니다.
시간별 참조는 H×6의 위치·속도 배열로 만들고 적분 후 상태와 `t+(i+1)dt`를 비교합니다.
속도 비용은 `||v-v_ref||²`이며 궤적 추종에서 항상 정지하도록 벌하지 않습니다.
입력 제곱·입력 변화 비용을 실측 에너지·물리 jerk와 동일시하지 않습니다.

```python
import numpy as np
from mppi_series.controller import MPPIController, Config
from mppi_series.trajectory import horizon_reference
costs = np.array([0., 1., 3.])
for temperature in [.1, 1., 10.]:
    print(temperature, MPPIController.weights(costs, temperature))
times, ref = horizon_reference(11., [0., 0., 0.], Config(horizon=10))
print(times[0], times[-1], ref.shape)  # 11.05, 11.5, (10,6)
```

```mermaid
flowchart LR
    A[측정 상태 ENU 위치·속도 / FLU rate] --> B[H스텝 예측]
    R[각 미래 시각의 위치·속도 참조] --> C[시간을 맞춘 비용]
    B --> C --> D[온도로 가중치 계산] --> E[평가한 후보의 가중 평균]
    E --> F[첫 추력·rate 목표를 PX4에 전달]
    E --> G[나머지 H-1 입력을 다음 탐색 중심으로 이동]
```

근거: `controller.py`, `trajectory.py`, `trajectory_checks.py` 실행 소스와 SHA-256,
실행 결과 `meaningful_checks.json`, 정적 도식 `companion_temperature.png`.
동일 고정 목표 비용은 정규화/rate 모델의 기존 비용과 차이 {checks['constant_reference_canonical_cost_error']:.3g},
참조 위치 수치미분과 참조 속도의 최대 차이는 {checks['analytic_velocity_derivative_max_error']:.3g}입니다.
이 자료를 별도 검토 후 기존 URL에 반영해야 합니다. 현재는 교정안 준비 상태입니다.
'''
    (out/'companion_revision.md').write_text(text)
    return text


def build(out, facts, workspace):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    flights=[c for c in facts['cases'] if c.get('kind')=='actual_gazebo_trajectory_trial']
    fig,axes=plt.subplots(3,1,figsize=(10,10))
    for trial in facts['trials']:
        root=Path(trial['output']);d=np.genfromtxt(root/'aligned_measurements.csv',delimiter=',',names=True)
        rows=np.genfromtxt(root/'commands.csv',delimiter=',',names=True)
        error=np.sqrt(sum(d['error_'+axis]**2 for axis in ['x','y','z']))
        label=trial['profile']+' H='+str(trial['flight']['config']['horizon'])
        axes[0].plot(d['elapsed_s'],error,label=label)
        axes[1].plot(d['x'],d['y'],label=label)
        axes[2].plot(rows['elapsed_s'],rows['compute_ms'],label=label)
    axes[0].axvspan(7,27,color='green',alpha=.12,label='Fixed evaluation window')
    axes[0].set_ylabel('Position error [m]');axes[0].set_xlabel('Time after MPPI handover [s]')
    axes[1].plot(d['ref_x'],d['ref_y'],'k--',label='Timed reference (last run origin)')
    axes[1].set_xlabel('ENU x [m]');axes[1].set_ylabel('ENU y [m]');axes[1].axis('equal')
    axes[2].axhline(50,color='r',linestyle='--',label='Compute limit 50ms')
    axes[2].set_ylabel('Compute [ms]');axes[2].set_xlabel('Time after MPPI handover [s]')
    for axis in axes:axis.grid(alpha=.2);axis.legend()
    fig.tight_layout();fig.savefig(out/'trajectory_error.png',dpi=150);plt.close(fig)
    checks=facts['meaningful_checks'];fig,axis=plt.subplots(figsize=(8,4))
    for t,w in zip(checks['temperatures'],checks['weights']):axis.plot([0,1,3],w,'o-',label='temperature='+str(t))
    axis.set_xlabel('Fixed illustrative cost');axis.set_ylabel('Normalized weight');axis.legend();axis.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'companion_temperature.png',dpi=140);plt.close(fig)
    companion_text=companion(out,facts)
    captured=facts['trials'][1];gif=Path(captured.get('screen_gif',''))
    if not gif.is_file() or not gif.resolve().is_relative_to(out):raise RuntimeError('Same-run actual Gazebo GIF missing')
    screen=captured['screen_capture']
    gif_asset=out/'gazebo_trajectory.gif';shutil.copy2(gif,gif_asset)
    archive=out/'series_11_1_code.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted((out/'source').rglob('*')):
            if p.is_file():bundle.write(p,str(p.relative_to(out/'source')))
        bundle.writestr('README.md',README)
        bundle.writestr('companion_revision.md',companion_text)
    # Create a genuinely fresh extracted application tree. Runtime dependencies
    # remain explicit and read-only in workspace; flight commands import ZIP code.
    with tempfile.TemporaryDirectory(prefix='stack-11-1-reproduce-') as directory:
        root=Path(directory)
        with zipfile.ZipFile(archive) as bundle:bundle.extractall(root)
        matches=all(sha(root/name)==digest for name,digest in facts['source_sha256'].items())
        if not matches:raise RuntimeError('ZIP source snapshot mismatch')
        env={**os.environ,'PYTHONPATH':str(root/'simulation')+':'+os.environ.get('PYTHONPATH',''),
             'MPLCONFIGDIR':str(out/'reproduction_matplotlib')}
        command=[sys.executable,'-m','mppi_series.episode_11_1','--workspace-root',str(workspace),
                 '--output-root',str(out/'archive_reproduction'),'--reproduce-child']
        with (out/'archive_reproduction.log').open('w') as log:
            child=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=700)
            except subprocess.TimeoutExpired:
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=90)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        results=sorted((out/'archive_reproduction').glob('*/result.json'))
        reproduced=json.loads(results[-1].read_text()) if results else {}
        proof={'passed':child.returncode==0 and reproduced.get('passed') is True
                        and reproduced.get('source_sha256')==facts['source_sha256'],
               'source_zip_sha256':sha(archive),'command':command,'fresh_extraction':True,
               'actual_live_sitl_rerun':True,'flight_verified':reproduced.get('flight_verified',False),
               'returncode':child.returncode,'result':str(results[-1]) if results else None,
               'source_snapshot_matches':matches,'checked_utc':datetime.now(timezone.utc).isoformat()}
        write(out/'archive_reproduction.json',proof)
        if not proof['passed']:raise RuntimeError('Fresh source ZIP actual flight reproduction failed')
    write(out/'publication_assets.json',{'figures':[str(gif_asset),str(out/'trajectory_error.png')],'archive':str(archive),
                                        'figure_sha256':{str(gif_asset):sha(gif_asset),str(out/'trajectory_error.png'):sha(out/'trajectory_error.png')},
                                        'gif_metadata':screen,'reproduction_report':str(out/'archive_reproduction.json')})
    table='\n'.join(f"| {c['name']} | {c['horizon_s']:.1f} | {c['rmse_all_m']:.4f} | {c['rmse_normal_m']:.4f} | {c['velocity_rmse_m_s']:.4f} | {c['max_position_error_m']:.4f} | {'통과' if c['success'] else '실패'} |" for c in flights)
    timing='\n'.join(f"| {c['name']} | {c['compute_mean_ms']:.3f} | {c['compute_max_ms']:.3f} | {c['compute_overrun_count']} | {c['publish_period_mean_s']:.4f} | {c['publish_period_max_s']:.4f} | {c['publish_period_over_60ms_count']} |" for c in flights)
    delta=flights[1]['rmse_normal_m']-flights[0]['rmse_normal_m']
    env=facts['versions'];trial=captured['flight'];cfg=trial['config']
    receipts='\n'.join(f"- {t['profile']}: 실제 명령 {t['analysis']['mppi_commands']}개, FCU 대조 {t['analysis']['clock_aligned_pairs']}쌍, 값 일치 {t['analysis']['command_match_fraction']*100:.2f}%, 계산 재생 최대 차이 {t['analysis']['independent_mppi_replay_max_error']:.3g}." for t in facts['trials'])
    text=f'''# {TITLE}

목표점을 바꾸는 것과 정해진 시간에 원 궤도를 따라가는 것은 어떻게 다를까요? 이번에는 `trajectory.py`에서 미래 각 스텝의 위치·속도 참조를 만들고, 실제 Gazebo에서 MPPI 추력·몸체 각속도 목표로 반지름 0.5m의 원을 추종했습니다. 예측 길이 0.5초와 1.0초의 고정 평가 구간 위치 RMSE는 각각 {flights[0]['rmse_normal_m']:.4f}m, {flights[1]['rmse_normal_m']:.4f}m였으며 실물 비행 결과는 아닙니다.

[[FIGURE_GAZEBO]]

GIF 1. 긴 예측 조건의 이번 Gazebo SITL 실행 화면입니다. 같은 실행 서버에 연결한 GUI를 녹화하고 MPPI 제어 인계 1초 뒤부터 연속 27초를 잘라 3배속으로 보여 드립니다. {screen['gif_frames']}개 프레임, {screen['gif_duration_ms']/1000:.1f}초이며 원본 동영상·녹화 명령·변환 명령·구간·해시는 `screen_capture.json`에 보존했습니다. 위치 로그를 다시 그린 화면이 아니며 실기체 촬영도 아닙니다.

이 글은 [전체 목차](https://sunggoo.tistory.com/83#drone-series-toc)의 11-1편입니다. [10-2의 목표점 이동](https://sunggoo.tistory.com/106)에서 확인한 입력 연결을 유지하고 참조와 비용만 궤적 추종에 맞춥니다. [5편의 직관](https://sunggoo.tistory.com/87), [8-1편의 샘플링·가중 평균](https://sunggoo.tistory.com/92)을 실행 예제와 연결합니다.

## 1. 경로 위에 있는 것과 시간에 맞춰 있는 것

원에 가까이 있는 드론이라도 목표보다 뒤처져 있으면 궤적 추종 오차가 있습니다. 여기서는 최근접 경로점 대신 측정 시각의 참조 위치를 사용합니다. 속도도 같은 방향과 크기로 맞춰야 하므로, 기존의 정지 목표 속도 비용 `||v||²`를 `||v-v_ref(t)||²`로 바꿉니다.

높이는 ENU z=2m로 고정하고, 중심은 시작 위치에서 x 방향으로 0.5m 이동한 곳입니다. MPPI 인계 후 3초는 호버링, 다음 24초는 한 바퀴, 마지막 3초는 정지입니다. 처음과 마지막 4초 동안 각속도를 cubic smoothstep으로 바꾸므로 위치·속도 참조가 연속이고 갑자기 속도가 뛰지 않습니다. 순항 각속도는 2π/20 rad/s, 최대 참조 선속도는 약 0.157m/s입니다. 이들은 설정값이고 측정 성능 수치가 아닙니다.

$$p_{{ref}}(t)=[x_0+0.5(1-\\cos\\theta(t)), y_0+0.5\\sin\\theta(t), 2]$$
$$v_{{ref}}(t)=[0.5\\sin\\theta(t)\\dot\\theta(t), 0.5\\cos\\theta(t)\\dot\\theta(t), 0]$$

## 2. 현재 목표 하나를 미래 전체에 복사하지 않습니다

예측 모델은 상태를 dt만큼 적분한 뒤 저장합니다. 따라서 첫 예측 상태는 현재 t가 아니라 t+dt의 목표와 비교합니다. 긴 예측이면 더 먼 시각의 목표 위치와 속도가 들어갑니다.

```python
import numpy as np
from mppi_series.controller import Config
from mppi_series.trajectory import horizon_reference, TrajectoryController
cfg = Config(horizon=20, dt=0.05, samples=500, temperature=3., seed=7)
times, future_ref = horizon_reference(11., [0., 0., 0.], cfg)
state = np.zeros(12); state[2] = 2.
command = TrajectoryController(cfg).step(state, future_ref)
print(times[0], times[-1], future_ref.shape, command)
```

이 작은 예제는 계산만 수행합니다. 실제 비행은 뒤의 episode 진입점을 실행해야 합니다. `future_ref`는 H×6 배열로 각 행에 ENU 위치[m]와 속도[m/s]를 담습니다. 명령은 `[c,p_sp,q_sp,r_sp]`이며 c는 무차원, 몸체 FLU rate는 rad/s입니다. MAVROS가 FLU→FRD를 한 번 변환하고 PX4가 내부 rate 제어·제어 배분·모터 출력을 담당합니다. 토크나 모터 속도를 rate 명령으로 이름만 바꾼 구조가 아닙니다.

## 3. 비용과 온도를 분리해서 읽습니다

이번 누적 비용은 `dt × Σ(30 위치오차² + 3 속도오차² + 자세·rate·입력 항)`이고 끝단에는 `40 위치오차² + 5 속도오차²`를 더합니다. 기울기는 0.35rad로 나누어 평가하고 입력 편차와 변화량은 기존 NOISE로 나눕니다. 입력 변화량은 후보 시퀀스 안의 변화이며 물리 jerk·실측 에너지가 아닙니다. 기존 Gaussian proposal 보정과 후보 제한을 유지하므로 정확한 경로적분 분포나 전역 최적성을 보장한다고 하지 않습니다.

가중치는 `exp(-(S-S_min)/temperature)`를 정규화합니다. 온도가 작으면 낮은 비용 후보에 더 집중합니다. 비용 항의 가중치는 무엇을 벌할지를 정하며 온도는 비용 차이를 얼마나 구분할지를 정합니다. 이번 두 비행은 온도 3, 샘플 500, dt 0.05s, 동일 노이즈·비용·시드 7을 유지하고 H만 10→20으로 바꿨습니다. 예측 길이를 늘리면 미래 목표뿐 아니라 합산하는 비용의 시간 길이도 달라집니다.

독립 검사는 참조 위치의 수치미분과 참조 속도를 대조했고 최대 차이는 {checks['analytic_velocity_derivative_max_error']:.3g}였습니다. 고정 위치·0속도 참조에서는 기존 비용과 차이가 {checks['constant_reference_canonical_cost_error']:.3g}였고, 미래 목표나 참조 속도만 바꾸어도 비용이 바뀌는 것을 검사했습니다. 잘못된 H×3 참조와 NaN 참조는 실제로 거부했습니다. 이 입력 거부 두 건은 비행 실패가 아니라 실행한 검사의 실패 입력입니다.

## 4. 실제로 적용된 MPPI 비행만 평가합니다

각 조건은 새 PX4/Gazebo 서버에서 시작했습니다. 이륙·초기 호버링은 PX4 위치 제어이고, 최근 3초의 정지 호버링 명령 중앙값으로 c_hover를 초기화했습니다. 두 조건의 c_hover는 {facts['trials'][0]['flight']['config']['thrust_hover']:.6f}, {facts['trials'][1]['flight']['config']['thrust_hover']:.6f}입니다. 이는 호버링 때 사용된 정규화 명령이며 모터 추력 곡선의 식별값은 아닙니다. `rate_tau=0.08s`와 국소 선형 추력 관계도 가정입니다.

MPPI 인계 후 위치 명령을 끄고 mask 128의 `AttitudeTarget`만 제어입력으로 보냅니다. `/mppi/reference`와 `/mppi/reference_velocity`는 기록용 참조이며 PX4 위치 제어 토픽이 아닙니다. 두 비행 모두 평가 구간에서 PX4 위치 목표 명령은 0개였습니다. AUTO.LAND 착륙도 평가에서 제외했습니다.

{receipts}

원본 MCAP에서 계산에 쓴 Odometry·실제 발행한 참조·추력/rate 명령을 대조하고, 같은 상태·참조·시드로 MPPI를 결정적으로 재생했습니다. ULog의 timesync offset으로 시각을 맞춰 무장·공중·rate Offboard 활성 및 위치·자세 제어 비활성을 확인했습니다. 수신 값마다 ±100ms 이내 송신 후보와 비교하고 최대 성분 오차 5e-5 미만이 95% 이상인지를 검사했습니다. 이는 시간 창 안의 값 전달 검증이며 일대일 지연 측정은 아닙니다.

## 5. 측정 시각에 맞춘 오차와 튜닝 결과

각 비행의 중복 없는 원본 Odometry 시각마다 `reference(t_odom)`을 계산했습니다. 명령 시각의 목표와 조금 이전 측정값을 무조건 빼지 않습니다. 원본 측정·참조·오차는 `aligned_measurements.csv`, 명령·상태 시각은 `commands.csv`, 매 계산의 미래 참조는 `predicted_references.jsonl`에 보존했습니다.

전체 구간은 인계 후 [0,30)초이며 고정 추종 평가 구간은 [7,27)초입니다. 최초 호버링 3초와 가속 4초를 제외하지만 감속은 포함합니다. 오차가 작은 표본을 고르지 않았습니다. 성공 기준은 고정 구간 위치 RMSE 0.12m 미만으로 미리 정했고 판정 실패도 표와 로그에 남깁니다. `result.json`의 passed는 실행·증거 검증 완료이고 모든 추종 조건 성공을 뜻하지 않습니다.

| 조건 | 예측 [s] | 전체 RMSE [m] | 고정 구간 RMSE [m] | 속도 RMSE [m/s] | 최대 위치 오차 [m] | 사전 기준 |
|---|---|---|---|---|---|---|
{table}

긴 예측과 짧은 예측의 고정 구간 RMSE 차이(긴−짧은)는 {delta:+.4f}m였습니다. 예측을 늘리면 항상 좋아진다고 결론내릴 수 없습니다. 조건마다 한 번씩 고정 순서로 실행했으며 호버링 초기화·상태와 물리 스케줄링은 완전히 같은 숫자가 아닙니다. 시간 기준 오차에는 지연에 따른 영향도 들어 있지만 별도의 위상 지연 추정이나 인과 분해는 하지 않았습니다. 여러 시드·반복 비교는 12-1의 과제입니다.

[[FIGURE_ERROR]]

그림 2. 보존된 원본 Odometry에서 계산한 시간별 참조 위치 오차, 실제 ENU 궤적과 계산 시간입니다. 녹색 음영은 실행 전에 정한 7~27초 구간이며 50ms 선은 계산 예산입니다. 각 비행의 기준 원점은 따로 기록했습니다.

## 6. 계산 시간과 실제 발행 주기는 다릅니다

| 조건 | 평균 계산 [ms] | 최대 계산 [ms] | 계산 50ms 초과 | 평균 발행 간격 [s] | 최대 발행 간격 [s] | 간격 60ms 초과 |
|---|---|---|---|---|---|---|
{timing}

50ms는 예측 적분 간격이자 명령 계산의 예산입니다. 노드는 최신 상태를 수신하고 이전 계산 시작 후 50ms 이상일 때 다시 계산하므로 실제 발행 간격이 정확히 50ms라는 보장은 없습니다. 표는 ROS header 시각 차이에서 측정한 간격입니다. 계산 시간 증가와 추종 오차는 함께 확인해야 합니다.

연결·무장·OFFBOARD 상태, 수신 상태 나이 0.25s, header 나이 0.5s, 계산 50ms, 높이 0.5~4m·수평 4m·tilt 0.6rad를 검사합니다. 거부 시 실패 자료를 쓰고 무장 상태에서는 AUTO.LAND를 요청합니다. 이 제한을 실제 비행 중 모두 주입하여 안전성을 입증한 것은 아닙니다. 기존 상태 검사와 이번 참조 입력 검사를 실행했습니다.

## 7. 압축 파일을 새 경로에 풀어 실행하기

PX4 commit은 `{facts['px4_commit']}`이며 실제 선택한 바이너리 SHA-256과 dirty 소스 diff·Gazebo 자산·Docker image ID는 각 trial의 `provenance/manifest.json`에 있습니다. 호스트 패키지는 아래와 같습니다.

```text
{env['host_packages'].strip()}
```

이 ZIP은 이미 설치된 ROS·MAVROS·Gazebo·컴파일된 PX4·Docker 런타임을 사용합니다. 응용 소스와 headless 설정을 포함하며 외부 의존성을 새로 설치하는 파일은 아닙니다. 자세한 명령과 조건은 ZIP의 README에 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.trajectory_checks --output "$PWD/examples"
/usr/bin/python3 -m mppi_series.episode_11_1 \\
  --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

마지막 JSON의 `output`은 UTC 고유 결과 폴더의 절대 경로입니다. 이번 ZIP을 새 디렉토리에 풀고 문서화된 진입점으로 두 실제 비행·원본 MCAP/ULog·독립 분석을 재실행했습니다. `archive_reproduction.json`과 로그에 결과를 보존했습니다. 자식 실행은 중복 녹화만 생략하며 실제 비행은 생략하지 않습니다. 결과가 bit 단위로 같은 것까지 요구하지는 않습니다.

[[SOURCE_ARCHIVE]]

5·8-1편의 온도·clipping·warm start와 입력 정의 교정안, 작은 실행 예제·정적 도식도 결과 폴더에 준비했습니다. 기존 URL의 서버 반영은 별도 검토·저장 작업이며 이번 구현에서 적용했다고 기록하지 않습니다.

## 8. 다음 질문: 비용은 장애물 회피 행동도 만들까요?

이번에는 시간별 미래 위치·속도 참조를 실제 MPPI rate-Offboard 비행으로 연결하고 측정 시각에 맞춘 오차를 확인했습니다. 장애물·외란·큰 기동·실기체 성능이나 MPPI 우월성을 검증한 결과는 아닙니다. 다음 11-2편(예정)은 알려진 정적 장애물의 월드 형상과 기체 크기·여유 거리를 맞추고 최소 이격거리·충돌·실패 사례를 따로 측정하겠습니다.
'''
    (out/'article.md').write_text(text)
    facts['archive_reproduction']=proof
    facts['publication_assets']=str(out/'publication_assets.json')
    facts['companion_revision']={'post_ids':[87,92],'status':'local_proposal_only','file':str(out/'companion_revision.md')}
