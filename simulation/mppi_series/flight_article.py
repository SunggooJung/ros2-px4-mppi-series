"""10-2 manuscript and figures from actual flight, with a fresh ZIP flight."""
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import zipfile
import numpy as np
from .episode_10_1 import sha,write

TITLE='Gazebo에서 MPPI로 호버링하고 목표점으로 이동하기 (ROS 2·PX4 드론 제어 10-2)'


def build(out,facts,rows,workspace):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a=facts['analysis'];cfg=facts['config'];t=(rows['ros_ns']-rows['ros_ns'][0])/1e9
    fig,axes=plt.subplots(2,1,figsize=(10,8))
    axes[0].plot(rows['x'],rows['y'],label='Measured ENU trajectory')
    axes[0].scatter(rows['goal_x'][::150],rows['goal_y'][::150],marker='x',color='r',label='Goal')
    axes[0].set_xlabel('ENU x [m]');axes[0].set_ylabel('ENU y [m]');axes[0].legend();axes[0].grid(alpha=.2)
    for name in ['x','y','z']:
        axes[1].plot(t,rows[name],label=name+' measured')
        axes[1].step(t,rows['goal_'+name],where='post',linestyle='--',alpha=.65,label=name+' goal')
    axes[1].set_xlabel('Time after MPPI handover [s]');axes[1].set_ylabel('Position [m]');axes[1].legend(ncol=3);axes[1].grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'flight_trajectory.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(10,7))
    error=np.sqrt(sum((rows[n]-rows['goal_'+n])**2 for n in ['x','y','z']))
    axes[0].plot(t,error,label='Position error norm')
    for i in range(3):axes[0].axvspan(i*10+6,i*10+10,color='green',alpha=.12)
    axes[0].set_ylabel('Position error [m]');axes[0].legend();axes[0].grid(alpha=.2)
    axes[1].plot(t,rows['compute_ms'],label='MPPI calculation');axes[1].axhline(50,color='r',linestyle='--',label='50 ms budget')
    axes[1].set_ylabel('Compute [ms]');axes[1].set_xlabel('Time after MPPI handover [s]');axes[1].legend();axes[1].grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'flight_error.png',dpi=150);plt.close(fig)
    companion=workspace/'publishing/drafts/chapter_7_correction.md'
    companion_text=('''# 10-2와 앞 회차의 연결

이 문서는 기존 글의 교정 제안입니다. 서버 개정 완료를 뜻하지 않습니다.
1~4편의 설치·기본 Offboard는 이번 원고에서 반복하지 않고 기존 글로
연결합니다. 6~7편의 SI 추력·토크 모델과 9~10편의 정규화 추력/rate
목표 모델은 입력과 제어 범위가 다릅니다. PX4가 내부 rate 제어와 모터
배분을 맡습니다. 아래 교정안을 /89·/90에 저장했음을 주장하지 않습니다.
이번 측정은 호버링 때 PX4가 보내던 정규화 추력 명령의 중앙값이며
전체 추력 곡선 또는 rate 시간상수 식별이 아닙니다.

'''+(companion.read_text() if companion.exists() else ''))
    (out/'companion_revision.md').write_text(companion_text)
    archive=out/'series_10_2_code.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted((out/'source').rglob('*')):
            if p.is_file():bundle.write(p,str(p.relative_to(out/'source')))
        bundle.writestr('companion_revision.md',companion_text)
        bundle.writestr('README.md','''# Gazebo MPPI flight — 10-2

Ubuntu 24.04, ROS 2 Jazzy, MAVROS 2.14, rosbag2 MCAP, NumPy,
Matplotlib, pyulog, pymavlink, colcon and the existing compiled PX4/Gazebo
workspace plus Docker image docker-px4-sitl are required. Application source,
the provenance helper and headless server configuration are included here;
PX4 binaries/Gazebo assets and ROS are external runtime dependencies.
Use the same offboard_control/ and simulation/ relative layout.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
python3 -m unittest mppi_series.test_controller mppi_series.test_ros_interface mppi_series.test_ros_flight -v
python3 -m mppi_series.episode_10_2 --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

The runner owns isolated PX4 instance 7/system ID 8, ROS domain 77 and the
/sitl_check/mavros namespace, refusing busy UDP ports and the shared SITL
lock. It actually builds this package in a unique run directory and executes
ros2 launch offboard_control mppi_flight.launch.py. PX4 performs takeoff;
MPPI then provides normalized thrust and body-rate targets for 30 seconds.
AUTO.LAND performs landing. This is SITL only, not hardware supervision.
All source snapshots, original ULog, MCAP, command/state CSV, failures and
clock-aligned analysis are retained. Normal runs rerun this ZIP in a fresh
directory once; --reproduce-child skips recursion, not the live flight.
''')
    with tempfile.TemporaryDirectory(prefix='stack-10-2-reproduce-') as directory:
        root=Path(directory)
        with zipfile.ZipFile(archive) as bundle:bundle.extractall(root)
        matches=all(sha(root/name)==digest for name,digest in facts['source_sha256'].items())
        if not matches:raise RuntimeError('ZIP source snapshot mismatch')
        env={**os.environ,'PYTHONPATH':str(root/'simulation')+':'+os.environ.get('PYTHONPATH',''),
             'MPLCONFIGDIR':str(out/'reproduction_matplotlib')}
        command=[sys.executable,'-m','mppi_series.episode_10_2','--workspace-root',str(workspace),
                 '--output-root',str(out/'archive_reproduction'),'--reproduce-child']
        with (out/'archive_reproduction.log').open('w') as log:
            child=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=360)
            except subprocess.TimeoutExpired:
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=90)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        results=sorted((out/'archive_reproduction').glob('*/result.json'))
        reproduced=json.loads(results[-1].read_text()) if results else {}
        proof={'passed':child.returncode==0 and reproduced.get('passed') is True and reproduced.get('source_sha256')==facts['source_sha256'],
               'source_zip_sha256':sha(archive),'command':command,'fresh_extraction':True,
               'actual_live_sitl_rerun':True,'flight_verified':reproduced.get('flight_verified',False),
               'returncode':child.returncode,'result':str(results[-1]) if results else None,
               'source_snapshot_matches':matches,'checked_utc':datetime.now(timezone.utc).isoformat()}
        write(out/'archive_reproduction.json',proof)
        if not proof['passed']:raise RuntimeError('Source ZIP fresh live flight reproduction failed')
    gif=Path(facts['screen_gif']) if facts.get('screen_gif') else None
    if gif and (not gif.is_file() or not gif.resolve().is_relative_to(out)):
        raise RuntimeError('Actual same-run Gazebo GIF missing')
    figures=[gif or out/'flight_trajectory.png',out/'flight_error.png']
    write(out/'publication_assets.json',{'figures':[str(p) for p in figures],'archive':str(archive)})
    table='\n'.join(f"| {c['name']} | {c['rmse_all_m']:.4f} | {c['rmse_normal_m']:.4f} | {c['final_position_error_m']:.4f} | 통과 |" for c in a['cases'])
    prior=[]
    for p in sorted(out.parent.glob('*/result.json')):
        if p.parent==out:continue
        d=json.loads(p.read_text())
        if not d.get('passed'):prior.append(p.parent.name+': '+str(d.get('error') or d.get('analysis_error')))
    write(out/'prior_attempts.json',{'failed_attempts':prior,'note':'Failures may be preparation/capture failures; they do not all indicate failed MPPI flight.'})
    prior_text=(f'앞선 시도 중 실행 준비나 화면 자료 검증에서 실패한 시도 {len(prior)}회를 보존했습니다. 이 시도들은 이번 수치에서 제외했습니다. 화면 기록에 실패한 시도를 MPPI 제어 성능 실패로 해석하지 않았으며, 상세 사유는 `prior_attempts.json`에 남겼습니다.'
                if prior else '보존된 선행 시도에서 실패 기록은 없었습니다. 이것이 다른 조건에서도 항상 성공한다는 뜻은 아닙니다.')
    caption1=('GIF 1. 이번 Gazebo SITL 비행의 실제 화면을 녹화했습니다. MPPI 호버링·0.5m 이동·복귀 구간에서 연속 27초를 잘라 3배속으로 보여 줍니다. 실물 드론 촬영이 아니며, 이륙과 착륙 구간은 포함하지 않았습니다.'
              if gif else '그림 1. MPPI로 제어를 넘긴 뒤의 ENU 궤적과 목표·측정 위치입니다. 이륙과 착륙은 제외했으며, x 방향 0.5m 이동 후 처음 호버링 위치로 돌아오는 세 구간을 보여 줍니다.')
    text=f'''# {TITLE}

이번에는 MPPI가 계산한 추력과 각속도 목표를 Gazebo의 비행 중인 PX4에 적용합니다. 이륙은 PX4 위치 제어기로 수행하고, 공중에서 MPPI로 제어를 넘겨 호버링·0.5m 이동·복귀를 실행했습니다. MPPI 구간의 전체 위치 RMSE는 {a['rmse_all_m']:.4f}m, 미리 정한 정상 구간에서는 {a['rmse_normal_m']:.4f}m였습니다. 실물 드론 결과는 아닙니다.

[[FIGURE_TRAJECTORY]]

{caption1}

이 글은 [전체 목차](https://sunggoo.tistory.com/83#drone-series-toc)의 10-2편입니다. [9-2의 자체 운동모델 실험](https://sunggoo.tistory.com/100)과 [10-1의 ROS 2·PX4 입력 연결](https://sunggoo.tistory.com/102)을 실제 Gazebo 비행으로 이어갑니다. 설치와 기본 Offboard 절차는 앞선 2~4편을 참고하시면 됩니다.

## 1. 어느 구간을 MPPI 비행으로 볼 것인가

연결된 토픽이 있다는 것만으로 MPPI가 기체를 제어했다고 볼 수는 없습니다. 이번 실행은 PX4 위치 제어로 높이 2m에 도달하고 안정된 호버링을 확인한 뒤, 위치 목표 발행을 끕니다. 그다음 `AttitudeTarget`의 정규화 추력과 몸체 각속도 목표만 발행합니다. 착륙에서는 다시 PX4 `AUTO.LAND`를 사용합니다.

| 구간 | 목표와 입력을 선택하는 쪽 | 이번 평가에 포함 |
|---|---|---|
| 이륙·초기 호버링 | PX4 위치·자세 제어 | 제외 |
| 호버링·0.5m 이동·복귀 | MPPI의 추력/rate 목표 → PX4 내부 rate 제어·모터 배분 | 포함 |
| 착륙 | PX4 AUTO.LAND | 제외 |

ROS bag에서는 MPPI 구간에 위치 명령이 {a['position_commands_during_mppi']}개였음을 확인했습니다. PX4 ULog에서도 무장·공중 상태와 rate Offboard 활성, 위치·자세 제어 비활성을 함께 확인했습니다. 그러므로 기준 PX4 위치 제어 비행의 오차를 MPPI 성능으로 가져온 결과가 아닙니다.

## 2. 호버링 추력 가정을 현재 기체에 맞추기

9~10-1편 모델의 `c_hover=0.6`은 비행에서 식별한 값이 아니었습니다. 이번에는 PX4로 호버링하는 동안 MAVROS `setpoint_raw/target_attitude`에서 전달받은 정규화 추력 명령을 모았습니다. 최근 3초 안의 표본 중 속도 크기 0.2m/s 미만, roll·pitch 절댓값 0.1rad 미만인 표본을 사용했습니다. {facts['flight']['hover_calibration_samples']}개 표본의 중앙값은 {cfg['thrust_hover']:.6f}였습니다.

이 값은 그때의 기체·높이·환경에서 PX4가 사용한 호버링 명령입니다. 추력 N을 직접 측정하거나 전체 모터 추력 곡선을 구한 것이 아닙니다. 예측 모델은 여전히 `T(c)=m g c/c_hover`의 국소 선형 관계와 `rate_tau=0.08s`의 가정을 사용합니다. 큰 기동이나 다른 기체에 그대로 적용할 근거는 없습니다. [7-1의 SI 추력·토크 모델](https://sunggoo.tistory.com/89)과는 입력 단위와 PX4가 맡는 범위를 구별해야 합니다.

```python
hover = float(np.median([sample[2] for sample in calibration]))
cfg = Config(samples=500, horizon=20, dt=0.05,
             temperature=3.0, thrust_hover=hover, rate_tau=0.08, seed=7)
controller = MPPIController(cfg)
```

## 3. 상태와 제어입력은 10-1의 계약을 유지합니다

위치·선속도는 ENU, 몸체 각속도는 FLU이며 각각 m·m/s·rad/s입니다. MAVROS Odometry의 선속도는 몸체 좌표이므로 quaternion 회전행렬로 세계 좌표에 옮깁니다. 각속도는 몸체 FLU 그대로 사용합니다. MPPI는 `[c, p_sp, q_sp, r_sp]`를 선택하고, MAVROS가 FLU→FRD 변환을 한 번 수행합니다.

```python
position_stream = False  # 이전 위치 명령의 동시 발행을 끝냅니다.
command = controller.step(state, goal)
msg.type_mask = 128
msg.thrust = float(command[0])
msg.body_rate.x, msg.body_rate.y, msg.body_rate.z = command[1:]
rate_publisher.publish(msg)
```

위 코드는 핵심 흐름이며, 전체 실행 파일은 ZIP의 `ros_flight.py`입니다. 제어입력을 토크로 해석하거나 모터 회전속도를 추력 필드에 넣지 않습니다. PX4는 내부 각속도 제어와 모터 출력을 담당합니다.

## 4. 원본 로그에서 실제 제어 구간을 확인하기

MPPI가 계산에 사용한 상태를 원본 Odometry와 대조하고, 계산한 명령을 실제 ROS bag의 `AttitudeTarget`과 비교했습니다. 같은 시드·상태·목표로 계산을 재생했을 때 최대 차이는 {a['independent_mppi_replay_max_error']:.3g}였습니다. PX4 ULog의 명령 시각은 MAVLink timesync의 측정된 clock offset으로 ROS 시각에 맞췄습니다.

수신 값마다 ±100ms 안의 송신 값 중 성분 오차가 가장 작은 후보를 선택합니다. 성분 최대 오차 5e-5 미만인 쌍이 95% 이상이어야 한다는 기준을 실행 전에 정했습니다. 이번 {a['clock_aligned_pairs']}개 비교 쌍의 일치 비율은 {a['command_match_fraction']*100:.2f}%였습니다. 이는 시간 창 안의 명령 전달·부호 변환 일치이며, 정확한 일대일 지연 측정은 아닙니다. 확인된 공중 rate Offboard 구간은 {a['mppi_airborne_duration_s']:.2f}초였습니다.

## 5. 목표점 오차와 정상 구간을 나누어 읽기

각 목표는 10초씩 유지했습니다. 정상 구간은 각 목표 전환 후 6초부터 10초 직전까지로 실행 전에 고정했습니다. 오차가 작은 표본만 골라내지 않았으며, 전체 RMSE에는 목표점 변경 직후의 과도 응답도 포함합니다.

위치 오차는 각 제어 시각의 목표와 그때까지 수신한 가장 최근 Odometry를 비교했습니다. 측정 시각과 명령 시각은 CSV에 따로 남기며, 상태는 다음 표본이 오기까지 유지한 값입니다. 연속 궤적을 보간한 평가는 아닙니다. 이번 명령 표본에서 상태 수신 나이의 최대는 {float(np.max(rows['state_age_s'])):.4f}초였습니다.

| 구간 | 전체 RMSE [m] | 정상 RMSE [m] | 마지막 위치 오차 [m] | 사전 기준 |
|---|---|---|---|---|
{table}
| 세 구간 전체 | {a['rmse_all_m']:.4f} | {a['rmse_normal_m']:.4f} | — | — |

마지막 위치 오차 0.35m 미만, 정상 구간 RMSE 0.4m 미만을 각 구간의 통과 기준으로 사용했습니다. 비교 조건은 호버링, ENU x 방향 0.5m 이동, 복귀이며 샘플 500개·예측 20스텝·dt 0.05초·온도 3·시드 7입니다. 여러 시드·거리·외란의 성능 비교는 아직 하지 않았습니다.

[[FIGURE_ERROR]]

그림 2. MPPI 비행 구간의 위치 오차 크기와 계산 시간입니다. 녹색 음영은 각 목표를 바꾼 뒤 6~10초의 고정 평가 구간이며, 50ms 선은 명령 계산의 시간 예산입니다.

## 6. 실패 판정과 착륙 절차

계산 전후에 연결·무장·Offboard 상태와 상태 시각을 확인합니다. 상태 수신 나이 0.25초 초과, header 나이 0.5초 초과, 계산 50ms 초과, 높이 0.5~4m 밖, 수평 원점 거리 4m 초과, roll·pitch 0.6rad 초과이면 MPPI 명령을 보내지 않고 실패를 기록합니다. 무장 상태에서는 AUTO.LAND를 요청하고 해제를 확인합니다. 실제로 이 모든 장애를 비행 중 주입하여 시험한 것은 아닙니다. 상태·범위 판정은 단위 검사로 확인했으며, 10-1의 오래된 상태 주입은 비무장 연결 시험이었습니다.

이번 평균 계산 시간은 {a['compute_mean_ms']:.3f}ms, 최대는 {a['compute_max_ms']:.3f}ms였습니다. 정상 실행에서는 MPPI 구간을 마친 뒤 AUTO.LAND로 착륙하고 무장 해제를 확인했습니다. 보존된 선행 시도 기록은 다음과 같습니다.

{prior_text}

이 감독 절차는 독립된 SITL 실험용이며 실물 비행의 안전성을 보장하지 않습니다.

## 7. 같은 코드로 다시 실행하기

4편의 `offboard_control` 패키지 구조에 별도 `mppi_flight` 실행 항목과 launch를 추가했습니다. 10-1의 비무장 검사와 실행 항목을 나눴으며, 실행 디렉토리 안에서 실제 패키지 빌드와 `ros2 launch`를 사용했습니다. 기존 이륙 스크립트를 덮어쓰지 않습니다.

| 환경 | 이번 실행 |
|---|---|
| 호스트·ROS | Ubuntu 24.04·ROS 2 Jazzy |
| MAVROS·Gazebo | MAVROS 2.14.0·호스트 Harmonic 8.12.0·컨테이너 8.11.0 |
| PX4 commit | {facts['px4_commit']} |
| PX4 바이너리 SHA-256 | {facts['px4_binary_sha256']} |
| 격리 | PX4 instance 7·system ID 8·ROS_DOMAIN_ID=77 |

ZIP에는 응용 소스·패키지·provenance 수집 코드·headless Gazebo 설정이 들어 있습니다. 기존 PX4 빌드·Gazebo 자산·Docker 이미지 `docker-px4-sitl`, ROS·MAVROS·rosbag2 MCAP·colcon, NumPy·Matplotlib·pyulog·pymavlink가 별도로 필요합니다. 압축 해제 디렉토리에서 다음 명령을 실행합니다.

화면 GIF를 만들 때는 ffmpeg와 Xvfb도 필요합니다. 기록용 가상 화면에서 Gazebo GUI를 같은 서버에 연결하고 실제 화면을 녹화합니다. 이 환경의 화면 표시에는 Ogre 렌더러를 사용했습니다. 물리·제어 실험은 같은 Gazebo 서버에서 실행하며, GIF를 위치 로그로 다시 그리지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
python3 -m unittest mppi_series.test_controller mppi_series.test_ros_interface mppi_series.test_ros_flight -v
python3 -m mppi_series.episode_10_2 \\
  --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

`--workspace-root`는 이 호스트의 기존 PX4/Gazebo 런타임 위치입니다. 출력마다 소스·명령·버전·원본 ULog·ROS bag·호버링 표본·CSV와 분석을 보존합니다. 공유 잠금과 전용 UDP 포트가 이미 사용 중이면 실행을 거부합니다. 이번 ZIP도 새 디렉토리에 풀어 패키지를 다시 빌드하고 실제 Gazebo 비행을 재실행했습니다. 재실행에도 같은 성공 기준을 적용했으며, 수치가 완전히 동일할 것을 요구하지는 않았습니다.

[[SOURCE_ARCHIVE]]

함께 들어 있는 앞 회차 교정안은 서버 반영 완료를 뜻하지 않습니다. 6~7편의 SI 모델과 이후 정규화 추력/rate 모델의 연결 관계를 정리한 자료입니다.

## 8. 이번 결과와 다음 실험

10-1에서는 비무장 상태에서 명령 전달을 확인했고, 이번에는 공중의 PX4 rate Offboard 구간에서 MPPI 명령이 실제 적용되는 것을 확인했습니다. 작은 목표점 이동과 복귀까지 검증했으나 실물 드론, 큰 기동, 외란, 장애물 회피까지 검증한 결과는 아닙니다.

다음 11-1편에서는 고정 목표점 대신 시간에 따라 바뀌는 참조 위치를 사용합니다. 목표점 유지와 궤적 추종의 오차 정의를 나누고, 참조 시각을 맞춘 로그로 결과를 비교하겠습니다.
'''
    (out/'article.md').write_text(text)
    (out/'results_ko.md').write_text(f'''# 10-2 실제 Gazebo MPPI 비행

원본 실행: {out}
입력: 정규화 추력과 FLU 몸체 rate 목표. PX4 내부 rate 제어·모터 배분.
PX4 이륙·착륙을 제외한 30초 MPPI 구간의 전체 RMSE {a['rmse_all_m']:.6f}m,
고정 6~10초 구간 RMSE {a['rmse_normal_m']:.6f}m.
원본 ULog/MCAP의 공중 rate Offboard·실제 명령 수신과 계산 재생 확인.
그림·ZIP 실제 비행 재현 통과. 실물 드론·다중 시드·외란 검증은 미실행.
구현·실험·원고 준비이며 서버 저장/발행은 별도 검증이 필요합니다.
''')
