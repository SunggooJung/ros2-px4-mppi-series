"""12-1 Korean comparison manuscript, measured plots, actual fresh ZIP rerun."""
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
from .episode_10_1 import sha,write
from .episode_12_1 import TITLE

README='''# 12-1 repeated actual Gazebo comparison

External dependencies: existing Ubuntu 24.04 / ROS 2 Jazzy / MAVROS 2.14,
MCAP rosbag2, numpy, matplotlib, pyulog, pymavlink, Docker image docker-px4-sitl,
and built PX4/Gazebo workspace. Source/config are bundled; external binary,
world assets, image ID and dirty source are recorded per trial in provenance.
Recording uses ffmpeg, gz sim GUI and the established Xvfb executable
/home/sg/.local/share/stack-series-tools/usr/bin/Xvfb on an owned :98 display.

From the extracted ZIP directory:
```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.episode_12_1 --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```
The default performs seven actual flights and then seven more from a fresh
ZIP extraction. --reproduce-child omits recursive packaging and duplicate
recording only, and still executes every actual flight and evidence analysis.
Each trial resets instance 7/system 8/domain 77 in a fresh owned server and
refuses busy ports/the existing SITL lock. Three paired repetitions use the
same 30s timed circle, seeds 7/17/27, alternating architecture order. The
extra seed-7 MPPI trial changes assumed hover by +8%; the plant is unchanged.
Takeoff/calibration and AUTO.LAND are excluded from evaluation. There is
no hardware execution, credential access, browser or timer modification.
Success criterion: 3D position RMSE <.12m at unique Odometry timestamps in
[7,27)s. A failed tracking criterion is a retained outcome; incomplete flight
or raw evidence verification causes passed=false and a nonzero exit.
All raw ULog, MCAP, commands, predicted references, aligned measurement CSV,
receipt checks, screen originals and failures remain in the output directory.
Final stdout JSON contains its absolute UTC-unique output directory.
'''


def companion(out,facts):
    text='''# 12-1과 직접 연결되는 두 글의 로컬 보완안

서버 글 수정·저장·공개 전환은 수행하지 않았습니다. /98은 비공개 보호 자료입니다.
기존 글 GIF 추가는 12-2 공개 검증 이후이며 이번 제안에는 포함하지 않습니다.

## 1. 보충 기준 비행 /98 (비공개 유지)

정확한 대상 ID: 98, https://sunggoo.tistory.com/98.
원문 근거: `prior_evidence/series_9_2.md`는 보존된 원고이며 현재 서버 본문 조회는 하지 않았습니다.
옛 원고의 전체 무장 OFFBOARD 구간은 이륙을 포함하고, 정상 구간은 무장 또는
목표 변경 후 3초가 지난 표본입니다. 정상 RMSE 0.597080m에는 상승 응답이 남습니다.
원문은 이를 PX4 기본 위치 제어와 추정 위치의 오차로 이미 한정했습니다.
그 경고를 유지하고, 이 수치를 12-1의 MPPI RMSE와 직접 뺄 수 없다고 보충합니다.
12-1은 이륙 후 안정 호버링에서 시작하는 30초 시간 참조와 고정 [7,27)초를 사용합니다.
기존 숫자를 새 숫자로 덮어쓰거나 과거의 비교 조건이 같았다고 표현하지 않습니다.
기본 제어기의 위치·속도·자세·rate·배분 계층과 MPPI의 외부 추력/rate 목표 구조를 함께 설명합니다.

## 2. 궤적 추종 /107

정확한 대상 ID: 107, https://sunggoo.tistory.com/107.
원문 근거: `prior_evidence/11_1_article.md` 및 `prior_evidence/11_1_result.json`.
기존 글은 H=10/20 각각 한 번, 시드7·고정 순서이며 반복 통계 비교가 아니라고 설명했습니다.
이 설명을 유지하고, H20을 고정한 12-1의 세 반복 비교로 연결합니다.
기존 글의 계산 시간 50ms 예산과 발행 간격 60ms 허용 집계를 구별하고,
12-1에서는 실제 50ms 초과와 60ms 초과를 모두 보고하며 계산과 발행 주기를 혼용하지 않습니다.
MPPI의 계산 시간은 최적화 시간, PX4 기준의 외부 시간은 현재 analytic 참조 생성 시간입니다.
PX4 내부 계산 시간을 재지 않고 어느 방식이 계산적으로 빠르다고 결론내리지 않습니다.

## 3. 실행 가능한 평가 예제와 근거

```python
import numpy as np
error = np.array([[.03, .04, 0.], [.03, .04, 0.]])
print(np.sqrt(np.mean(np.sum(error**2, axis=1))))  # .05 m, 설명용 수치
```

`comparison_checks.py`는 이 3D RMSE, 고정 구간 밖 과도응답, 실패 기준,
불완전 평가 구간 거부와 참조 시각 이동을 실제 검사합니다.
`comparison_results.py`는 원본 Odometry 시각에서 참조를 다시 계산하며,
원본 MCAP 명령/상태와 ULog 활성 제어/수신 값을 대조합니다.
이 자료의 SHA-256, 실제 trial CSV/MCAP/ULog와 `analysis.json`이 수치 근거입니다.
새 URL에 연결할 글 ID는 발행 담당 과정에서 확정하며 여기서는 추측하지 않습니다.
'''
    (out/'companion_revision.md').write_text(text)
    return text


def build(out,facts,workspace):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,10))
    for trial in facts['trials']:
        root=Path(trial['output']);d=np.genfromtxt(root/'aligned_measurements.csv',delimiter=',',names=True)
        commands=np.genfromtxt(root/'commands.csv',delimiter=',',names=True)
        label=trial['name'];color='tab:blue' if trial['method']=='px4_position' else 'tab:orange'
        if trial['hover_scale']!=1:color='tab:red'
        error=np.sqrt(sum(d['error_'+a]**2 for a in ['x','y','z']))
        axes[0].plot(d['elapsed_s'],error,color=color,alpha=.65,label=label)
        origin=trial['flight']['origin']
        axes[1].plot(d['x']-origin[0],d['y']-origin[1],color=color,alpha=.65,label=label)
        axes[2].plot(commands['elapsed_s'],commands['compute_ms'],color=color,alpha=.65,label=label)
    axes[0].axvspan(7,27,color='green',alpha=.10);axes[0].set_ylabel('Measured reference error [m]')
    axes[0].set_xlabel('Time after reference epoch [s]')
    axes[1].plot(d['ref_x']-origin[0],d['ref_y']-origin[1],'k--',label='Timed reference')
    axes[1].set_xlabel('ENU x relative to origin [m]');axes[1].set_ylabel('ENU y relative to origin [m]');axes[1].axis('equal')
    axes[2].axhline(50,color='r',linestyle='--');axes[2].set_ylabel('External compute/dispatch [ms]')
    axes[2].set_xlabel('Time after reference epoch [s]')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8,ncol=2)
    fig.tight_layout();fig.savefig(out/'comparison_error.png',dpi=150);plt.close(fig)
    prior=out/'prior_evidence';prior.mkdir()
    prior_sources={'series_9_2.md':workspace/'publishing/drafts/series_9_2.md',
                   '11_1_article.md':workspace/'publishing/artifacts/mppi_daily/episodes/11-1/experiments/20261006T092804_895121Z/article.md',
                   '11_1_result.json':workspace/'publishing/artifacts/mppi_daily/episodes/11-1/experiments/20261006T092804_895121Z/result.json'}
    records={}
    for name,source in prior_sources.items():
        destination=prior/name;shutil.copy2(source,destination)
        records[name]=dict(source=str(source),preserved=str(destination),sha256=sha(destination))
    write(prior/'manifest.json',records);facts['prior_chapter_evidence']=records
    text=companion(out,facts)
    captured=next(t for t in facts['trials'] if t['name']=='1_mppi')
    gif=Path(captured.get('screen_gif',''))
    if not gif.is_file() or not gif.resolve().is_relative_to(out):raise RuntimeError('Same-run Gazebo GIF missing')
    screen=captured['screen_capture'];asset=out/'gazebo_comparison.gif';shutil.copy2(gif,asset)
    archive=out/'series_12_1_code.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted((out/'source').rglob('*')):
            if p.is_file():bundle.write(p,str(p.relative_to(out/'source')))
        bundle.writestr('README.md',README);bundle.writestr('companion_revision.md',text)
        for p in sorted(prior.iterdir()):bundle.write(p,'prior_evidence/'+p.name)
    print(json.dumps(dict(stage='fresh_zip_actual_reproduction',trials=7)),flush=True)
    with tempfile.TemporaryDirectory(prefix='stack-12-1-reproduce-') as directory:
        root=Path(directory)
        with zipfile.ZipFile(archive) as bundle:bundle.extractall(root)
        matches=all(sha(root/name)==digest for name,digest in facts['source_sha256'].items())
        if not matches:raise RuntimeError('ZIP source hash mismatch')
        env={**os.environ,'PYTHONPATH':str(root/'simulation')+':'+os.environ.get('PYTHONPATH',''),
             'MPLCONFIGDIR':str(out/'reproduction_matplotlib')}
        cmd=[sys.executable,'-m','mppi_series.episode_12_1','--workspace-root',str(workspace),
             '--output-root',str(out/'archive_reproduction'),'--reproduce-child']
        with (out/'archive_reproduction.log').open('w') as log:
            child=subprocess.Popen(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=900)
            except subprocess.TimeoutExpired:
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=90)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        results=sorted((out/'archive_reproduction').glob('*/result.json'))
        reproduced=json.loads(results[-1].read_text()) if results else {}
        proof=dict(passed=child.returncode==0 and reproduced.get('passed') is True and reproduced.get('source_sha256')==facts['source_sha256'],
                   source_zip_sha256=sha(archive),command=cmd,fresh_extraction=True,actual_live_sitl_rerun=True,
                   actual_trials=len(reproduced.get('trials',[])),flight_verified=reproduced.get('flight_verified',False),
                   source_snapshot_matches=matches,returncode=child.returncode,result=str(results[-1]) if results else None,
                   checked_utc=datetime.now(timezone.utc).isoformat())
        write(out/'archive_reproduction.json',proof)
        if not proof['passed']:raise RuntimeError('Fresh ZIP seven-flight reproduction failed; retained report/log')
    write(out/'publication_assets.json',dict(figures=[str(asset),str(out/'comparison_error.png')],archive=str(archive),
          figure_sha256={str(p):sha(p) for p in [asset,out/'comparison_error.png']},gif_metadata=screen,
          reproduction_report=str(out/'archive_reproduction.json')))
    cases=[c for c in facts['cases'] if c.get('kind')=='actual_gazebo_repeated_trial']
    table='\n'.join(f"| {c['name']} | {c['seed']} | {c['rmse_all_m']:.4f} | {c['rmse_normal_m']:.4f} | {c['max_normal_error_m']:.4f} | {c['velocity_rmse_m_s']:.4f} | {'통과' if c['success'] else '실패'} |" for c in cases)
    timing='\n'.join(f"| {c['name']} | {c['compute_mean_ms']:.3f}/{c['compute_p95_ms']:.3f}/{c['compute_max_ms']:.3f} | {c['compute_overrun_count']} | {c['publish_period_mean_s']*1000:.2f}/{c['publish_period_p95_s']*1000:.2f}/{c['publish_period_max_s']*1000:.2f} | {c['publish_period_over_50ms_fraction']*100:.1f}% | {c['publish_period_over_60ms_count']} |" for c in cases)
    inputs='\n'.join(f"| {c['name']} | {label} | {v['unit']} | {v['total_variation']:.5f} | {v['step_rms']:.5f} |" for c in cases for label,v in c['input_change'].items())
    summary=facts['comparison_summary'];extra=cases[-1]
    receipt='\n'.join(f"- {t['name']}: 실제 FCU 대조 {t['analysis']['clock_aligned_pairs']}쌍, 일치율 {t['analysis']['command_match_fraction']*100:.2f}%." for t in facts['trials'])
    means={method:float(np.mean(s['rmse_values_m'])) for method,s in summary.items()}
    article=f'''# {TITLE}

같은 시간 참조를 따라갈 때 MPPI는 어느 정도 오차를 내며 실패는 얼마나 생길까요? `episode_12_1.py`로 PX4 기본 위치 제어와 MPPI 추력·몸체 rate 제어를 각각 세 번 실행했습니다. 이번 Gazebo SITL 고정 평가 구간 평균 RMSE는 PX4 {means['px4_position']:.4f}m, MPPI {means['mppi']:.4f}m였으며, 제어 계층과 참조 정보가 다른 제한된 비교입니다.

[[FIGURE_GAZEBO]]

GIF 1. 첫 시드7 MPPI 비교 비행의 실제 Gazebo 화면입니다. 같은 실행 서버의 GUI 녹화를 인계 1초 뒤부터 27초 잘라 3배속으로 표시했습니다. {screen['gif_frames']}프레임·{screen['gif_duration_ms']/1000:.1f}초이며 원본 MP4·서버 partition·시각·녹화/변환 명령·해시는 `screen_capture.json`에 있습니다. 실물 비행이나 로그를 다시 그린 도해가 아닙니다.

[11-1 /107](https://sunggoo.tistory.com/107)의 시간별 미래 참조를 그대로 사용하고 [11-2 /123](https://sunggoo.tistory.com/123)의 장애물 실험과는 별도인 default 월드에서 비교합니다. 설치·기본 Offboard 설명은 앞 글을 참조하며, 이번 핵심은 조건과 평가 기준을 맞추는 일입니다.

## 1. 같은 과제와 다른 제어 계층

반지름 0.5m 원, 높이 ENU 2m, 초기 3초 정지·24초 한 바퀴·마지막 3초 정지의 30초 과제입니다. 첫/마지막 가감속은 기존 smoothstep을 사용합니다. 매 trial은 새 x500/PX4 서버, 같은 바이너리·Docker 이미지·월드·이륙 및 안정 호버 판정으로 시작합니다. 실측 초기 위치·속도·호버 명령과 스케줄링은 숫자까지 같지 않아 그대로 보존합니다.

| 방식 | 외부 명령 | PX4가 맡는 범위 |
|---|---|---|
| 기준 | 시간별 ENU 위치 목표[m] | 위치·속도·자세·몸체 rate·제어 배분·모터 |
| MPPI | 정규화 추력 c와 FLU 몸체 rate 목표[rad/s] | 몸체 rate·제어 배분·모터 |

기준 방식도 실제 비행으로 따로 실행합니다. MPPI trial의 이륙·호버 초기화에는 PX4 위치 제어를 쓰지만 평가 전 끕니다. 평가 중 mask128 `AttitudeTarget`만 제어 토픽에 발행하며 MAVROS가 FLU→FRD를 한 번 변환합니다. `/mppi/reference`와 속도 참조는 기록용이며 PX4 위치 입력이 아닙니다.

PX4 기준에는 외부 속도 feedforward를 보내지 않고, MPPI는 미래 위치·속도를 비용에 사용합니다. 과제는 같아도 사용 정보·계층이 같지 않으므로 순수 알고리즘 우열 비교로 해석하지 않습니다. 두 방식 모두 PX4 내부 rate 제어를 사용합니다.

## 2. 반복 횟수와 실패 기준을 미리 정합니다

시드7·17·27로 세 쌍을 실행하며 순서는 PX4→MPPI, MPPI→PX4, PX4→MPPI입니다. 시드는 MPPI 후보 난수만 제어하고 Gazebo/PX4 센서 난수는 고정하지 않았습니다. MPPI는 샘플500, H20, dt0.05s, 온도3, rate 시간상수0.08s와 기존 비용·노이즈를 고정합니다. 매 비행 최근 정지 호버 명령 중앙값으로 c_hover를 초기화하며 이를 추력 곡선 식별로 부르지 않습니다.

평가 구간은 [7,27)초, 성공 기준은 3차원 위치 RMSE<0.12m입니다. 초반 정지·가속은 제외하고 감속은 포함하며 오차가 작은 표본을 선택하지 않습니다. 30초 전체 RMSE와 고정 구간 최대 오차도 함께 남깁니다. 원본 Odometry의 header 시각마다 analytic 참조를 다시 계산하므로 최신 명령 시각과 이전 측정을 섞지 않습니다. 이는 추정 위치의 참조 오차이고 Gazebo 참값 대비 추정 정확도는 아닙니다.

```python
import json, sys
from pathlib import Path
import numpy as np
from mppi_series.trajectory import reference
trial = Path(sys.argv[1])
flight = json.loads((trial / "flight.json").read_text())
data = np.genfromtxt(trial / "aligned_measurements.csv", delimiter=",", names=True)
elapsed = data["elapsed_s"]  # 원본 Odometry 시각에서 계산한 초
ref = reference(elapsed, flight["origin"])
measured = np.column_stack([data[axis] for axis in ["x", "y", "z"]])
error = measured - ref[:, :3]
mask = (elapsed >= 7.) & (elapsed < 27.)
print(np.sqrt(np.mean(np.sum(error[mask]**2, axis=1))))
```

이 예제를 파일로 저장하고 첫 인자로 실제 trial 폴더를 주면 RMSE를 다시 계산합니다. `comparison_results.py`가 완전한 실행 구현입니다. 30Hz 원본 표본마다 동일 가중치를 주며 시간 적분 가중 RMSE는 아닙니다. 자체 검사에서는 (.03,.04,0)m 오차의 RMSE .05m, 고정 구간 밖 과도응답, 실패 한계, 불완전 구간 거부와 잘못된 참조 시각을 검증했습니다. 설명용 숫자를 비행 수치로 섞지 않습니다.

## 3. 실제 반복 결과

| 시도 | 시드 | 전체 RMSE[m] | 고정 RMSE[m] | 고정 최대[m] | 속도 RMSE[m/s] | 사전 기준 |
|---|---|---|---|---|---|---|
{table}

기준 세 번 중 {summary['px4_position']['successes']}번 통과·{summary['px4_position']['failures']}번 실패, MPPI 세 번 중 {summary['mppi']['successes']}번 통과·{summary['mppi']['failures']}번 실패였습니다. 실패 기준 결과를 제거하지 않았습니다. 세 번의 평균은 이 실행들의 기술통계이며 모집단 유의성이나 다른 과제의 우월성을 뜻하지 않습니다. 과거 /98은 이륙·계단 목표를 포함한 다른 평가 구간이므로 그 RMSE를 이 표와 직접 비교하지 않습니다.

[[FIGURE_ERROR]]

그림 2. 같은 실행의 원본 Odometry에서 계산한 시간별 오차와 원점 기준 실제 궤적, 외부 계산 시간을 보여 드립니다. 음영은 사전 고정한 7~27초 구간입니다. 빨강은 추가 모델 불일치 조건이며 명목 세 반복의 평균에 섞지 않습니다.

## 4. 제한적인 모델 불일치 한 번

물리 기체·월드는 바꾸지 않고 시드7에서 MPPI가 가정하는 c_hover만 측정 중앙값의 1.08배로 바꿨습니다. `T/m=9.81*c/c_hover` 모델에서 같은 c의 예측 가속도가 달라지는 실험이며 질량·페이로드·바람 측정은 아닙니다. 추가 고정 구간 RMSE는 {extra['rmse_normal_m']:.4f}m, 최대 오차 {extra['max_normal_error_m']:.4f}m로 사전 기준 {'통과' if extra['success'] else '실패'}였습니다. 한 번의 민감도 관측이며 강인성 보장이나 원인 전체의 식별은 아닙니다.

## 5. 명령이 FCU에 실제로 적용되었는지 확인합니다

원본 MCAP의 Odometry, 실제 발행 위치 또는 `AttitudeTarget`, 기록용 위치·속도 참조를 계산 로그와 대조합니다. MPPI는 같은 시드·상태·미래 참조로 전체 명령을 결정적으로 재생합니다. ULog의 측정 timesync offset으로 ROS와 FCU 시각을 맞추고, 무장·공중·Offboard와 각 구조의 활성 제어 플래그를 검사합니다. MPPI 구간의 위치 명령은 없어야 하고 위치·자세 제어는 비활성이어야 합니다. 기준은 외부 rate 명령이 없어야 하고 위치·자세·rate 제어가 활성이어야 합니다.

{receipt}

수신 값은 ±100ms 창의 송신 후보와 성분 최대 차이 5e-5 미만이 95% 이상인지 검사합니다. 이는 값 전달 검증이지 일대일 전송 지연 측정은 아닙니다. 기준 위치 명령의 ENU→NED와 MPPI FLU→FRD 부호도 원본 수신값으로 확인합니다. 완전한 비행·증거 검증이 실패하면 `passed=false`이며 원본 시도는 남습니다. 추종 기준 실패는 `cases.success=false`로 별도 기록합니다.

## 6. 계산·발행 시간과 입력 변화를 구별합니다

| 시도 | 외부 평균/p95/최대[ms] | 계산>50ms | 발행 평균/p95/최대[ms] | 간격>50ms | 간격>60ms 횟수 |
|---|---|---|---|---|---|---|
{timing}

MPPI 열은 실제 최적화 시간이며 기준의 외부 시간은 현재 analytic 참조 생성만 잽니다. PX4 내부 계산 시간을 재지 않았으므로 외부 숫자로 전체 제어기의 계산 속도를 비교하지 않습니다. 50ms 예산 초과와 60ms 허용선 초과를 둘 다 기록합니다. 현재 루프는 이전 계산 시작 50ms 이후 재개하므로 callback·스케줄링으로 50ms를 조금 넘는 간격도 그대로 집계합니다.

| 시도 | 외부 입력 | 단위 | 30초 총 변화량 Σ abs(Δu) | 스텝 변화 RMS |
|---|---|---|---|---|
{inputs}

서로 다른 물리량을 더한 입력 norm으로 순위를 만들지 않습니다. 공통 FCU 내부 rate/thrust 값도 `fcu_inner_setpoints.csv`에 FLU·무차원으로 보존하고 성분별 변화량을 기록합니다. 내부 ULog 샘플 간격은 구조에 따라 달라 총 변화량의 해석에도 제한이 있습니다. 입력 변화는 물리 jerk나 전기 에너지 측정이 아닙니다.

## 7. 버전과 새 경로 재현

PX4 commit은 `{facts['px4_commit']}`입니다. 동일 바이너리 SHA-256, dirty 소스 diff, 모델·월드·init 설정, Docker image ID와 정확한 호스트/컨테이너/패키지 버전은 각 `provenance/manifest.json`과 runtime archive에 보존했습니다.

```text
{facts['versions']['host_packages'].strip()}
```

ZIP은 응용 소스·설정·README를 포함하며 외부 ROS/PX4 설치를 대체하지 않습니다. 새 디렉토리에 풀어 아래 entry point로 같은 일곱 비행·검사·MCAP/ULog 분석을 실행했고 재현 보고서를 보존했습니다. 자식에서는 중복 녹화와 재귀 포장만 생략합니다. 비행 수치를 bit 단위로 같게 만드는 검증은 아닙니다.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
/usr/bin/python3 -m mppi_series.episode_12_1 \\
  --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

최종 JSON의 `output`은 UTC 고유 결과 디렉토리입니다. `commands.csv`, 미래 참조 JSONL, 측정/오차 CSV, 원본 MCAP·ULog, 원본 MP4·GIF 변환 정보, 검사 실패·재현 로그를 연결해서 읽을 수 있습니다.

[[SOURCE_ARCHIVE]]

## 8. 다음 질문: 연재 전체를 다시 실행할 수 있을까요?

이번에는 같은 참조에서 세 반복과 제한적 모델 오차 한 번을 평가했습니다. 큰 기동·바람·동적 장애물·실기체·통계적 우월성은 검증하지 않았습니다. /98·/107의 조건·평가 용어 보완안은 `companion_revision.md`에 준비했고 서버 반영은 수행하지 않았습니다. 다음 12-2편(예정)은 대표 공개 실험을 실제 재실행하고 코드·설정·버전·로그를 연재 전체와 연결해 완결하겠습니다.
'''
    if article.count('[[FIGURE_')!=2 or article.count('[[SOURCE_ARCHIVE]]')!=1:raise RuntimeError('Article marker contract violated')
    (out/'article.md').write_text(article)
    facts.update(archive_reproduction=proof,publication_assets=str(out/'publication_assets.json'),
                 companion_revision=dict(post_ids=[98,107],status='local_proposal_only',file=str(out/'companion_revision.md')))
