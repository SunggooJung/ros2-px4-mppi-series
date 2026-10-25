"""Build chapter 10-1 from actual retained measurements and rerun its ZIP."""
import json
import os
import signal
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile
from .episode_10_1 import sha,write

TITLE='MPPI의 계산 결과를 PX4 명령으로 연결하기 (ROS 2·PX4 드론 제어 10-1)'


def build(out,facts,workspace):
    a=facts['analysis']
    correction=workspace/'publishing/drafts/chapter_7_correction.md'
    companion='''# 10-1과 연결된 7편 보완안

대상: 7-1 ID 89, 7-2 ID 90. 이 파일은 검증된 교정 제안이며 서버 저장
완료의 증거가 아닙니다. SI 힘·토크·모터 속도와 최종 정규화 추력/몸체
rate 목표를 구분합니다. PX4가 내부 rate 제어와 모터 배분을 맡습니다.

근거: publishing/artifacts/content_strategy/chapter_7_audit/model_verification.json
의 실제 100조건 검사·새 ZIP 경로 재현, 이번 test_ros_interface.py의
몸체 전방 속도/yaw90도 변환 및 body rates 유지 검사.

'''+(correction.read_text() if correction.exists() else '')
    c=facts['chapter_7_checks']
    companion+=f'''

## 이번 10-1 실행에서 재검사한 근거와 수정 대상

서버 수정 상태: 미실행. 다음 두 글만 직접 연결된 교정 대상으로 정리합니다.

| 글 ID | 기존 주장/코드 | 제안하는 수정 | 이번 실행 근거 |
|---|---|---|---|
| 89 (7-1) | 추력/토크 배분 결과를 모터 rad/s로 반환하고 회전 부호를 곱한 뒤 음수 속도를 자름 | 모터 힘 Fi[N]를 먼저 풀고 omega=sqrt(Fi/k_f)로 변환합니다. 속도는 비음수 크기, 요 반작용 부호는 별도입니다. X 배치에서 tau_x=sum(y_i Fi), tau_y=-sum(x_i Fi), tau_z=sum(sigma_i kappa Fi)를 사용합니다. | 보존 원문 호버링 출력 {c['old_hover_output_labeled_rad_s']}, 실제 재계산 추력 {c['old_implied_thrust_N']:.12f}N 대 요청 {c['requested_hover_N']:.2f}N, 구/수정 mixer rank {c['old_mixer_rank']}/{c['corrected_mixer_rank']} |
| 90 (7-2) | 몸체 p,q,r를 Euler 변화율로 직접 적분하고 omega_dot=tau/I로 일반 회전식을 설명 | 원문의 올바른 몸체→세계 ZYX 회전행렬은 유지합니다. Euler 변화율 변환과 I omega_dot=tau−omega×(I omega)를 적용합니다. | 100조건 회전행렬 오차 {c['rotation_max_error']:.3g}, 올바른/기존 Rdot 오차 {c['correct_euler_derivative_max_error']:.3g}/{c['old_body_as_euler_derivative_max_error']:.3g}, 관성 결합항 100조건 검사 |

교정의 SI 입력은 [T,tau_x,tau_y,tau_z]이며 N·N m입니다. 최종 ROS/PX4
입력은 [c_norm,p_sp,q_sp,r_sp]이며 1·rad/s입니다. MPPI 예측 모델도 내부
rate 응답의 1차 지연 근사로 바뀌었습니다. 토크와 rate 목표를 이름만
바꾸거나 모터 rad/s를 정규화 추력에 넣지 않습니다. MAVROS는 FLU→FRD를
한 번 수행하고 PX4는 내부 rate 제어와 모터 배분을 맡습니다.

이번 검사는 chapter_7_checks.json, chapter_7_raw.csv 및 실행 소스
chapter_7_checks.py에 보존했습니다. 원문 조각 reference_inputs/89_code_6.txt,
reference_inputs/90_code_9.txt, reference_inputs/90_code_5.txt는 기존 publishing/artifacts/content_strategy/
chapter_7_audit 사본과 동일하며 SHA-256은 다음과 같습니다.

```json
{json.dumps(c['source_sha256'],ensure_ascii=False,indent=2)}
```

이 수치는 실행한 참조 수학 예제이며 Gazebo 추력/모터 식별이 아닙니다.
SI 모델과 정규화 모델의 한 단계 병진/Euler 일치 오차는
{c['si_normalized_translation_euler_max_error']:.3g}입니다. 내부 rate/토크
동역학이 같다는 주장은 하지 않습니다. 7편 글에 교정이 이미 반영되었다고
표시하지 않으며, 별도 편집 절차가 같은 ID 저장과 서버 재접속을 검증해야 합니다.
'''
    (out/'companion_revision.md').write_text(companion)
    readme='''# MPPI ROS/PX4 interface — 10-1

The package structure extends chapter 4's offboard_control. The original
takeoff script is not included or replaced; this is a separate unarmed test.
The canonical mppi_series controller is installed in the same distribution.

Prerequisites: Ubuntu 24.04, sourced ROS 2 Jazzy, NumPy, pyulog, pymavlink,
Matplotlib, rosbag2 MCAP, colcon, the existing compiled PX4/Gazebo runtime and
docker-px4-sitl image in the workspace. See the original experiment's exact
provenance, binary hash and image digest. The ZIP contains application source,
not PX4/Gazebo binaries. Runtime defaults to /home/sg/stack/src on this host;
use --workspace-root for a different existing runtime workspace.
The provenance helper and headless Gazebo server configuration are included
under simulation/scripts and simulation/config and are loaded from this ZIP,
not from the external runtime workspace. The runner creates the shared
simulation/experiment_logs lock directory in that workspace if it is absent.

From the extracted ZIP directory:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
python3 -m unittest mppi_series.test_controller mppi_series.test_ros_interface -v
python3 -m mppi_series.episode_10_1 --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

This rebuilds only offboard_control in the unique experiment directory and
actually runs ros2 launch offboard_control mppi_interface.launch.py. It starts
an isolated headless PX4 instance 7, MAVROS and ROS bag, never requests arming,
then compares live AttitudeTarget commands with PX4 ULog. The test injects
stale state, suppresses commands and checks the PX4 signal-loss flag. The
disarmed FCU may still display OFFBOARD. No airborne-failsafe claim is made.

Each normal run reruns the ZIP in a fresh extraction once. The internal
--reproduce-child flag disables this recursion, not the actual live experiment.
All attempts and logs are kept; a failed guard is not published.
'''
    archive=out/'series_10_1_code.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted((out/'source').rglob('*')):
            if path.is_file():bundle.write(path,str(path.relative_to(out/'source')))
        bundle.writestr('README.md',readme)
        bundle.writestr('companion_revision.md',companion)
        bundle.write(out/'chapter_7_checks.json','verification/chapter_7_checks.json')
        bundle.write(out/'chapter_7_raw.csv','verification/chapter_7_raw.csv')
    with tempfile.TemporaryDirectory(prefix='stack-10-1-reproduce-') as directory:
        extracted=Path(directory)
        with zipfile.ZipFile(archive) as bundle:bundle.extractall(extracted)
        extraction_matches=all(sha(extracted/name)==digest for name,digest in facts['source_sha256'].items())
        if not extraction_matches:raise RuntimeError('Extracted source hashes differ from executed source snapshot')
        env=dict(os.environ,PYTHONPATH=str(extracted/'simulation')+':'+os.environ.get('PYTHONPATH',''),
                 MPLCONFIGDIR=str(out/'reproduction_matplotlib'))
        command=[sys.executable,'-m','mppi_series.episode_10_1','--workspace-root',str(workspace),
                 '--output-root',str(out/'archive_reproduction'),'--reproduce-child']
        timeout_error=None
        with (out/'archive_reproduction.log').open('w') as log:
            child=subprocess.Popen(command,cwd=extracted,env=env,stdout=log,
                                   stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=240)
            except subprocess.TimeoutExpired:
                timeout_error='Fresh-extraction live reproduction exceeded 240 seconds'
                # Let the episode's signal handler save failure evidence and
                # stop its independently grouped SITL/MAVROS/recorder workers.
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=90)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                log.write('\n'+timeout_error+'\n')
        results=sorted((out/'archive_reproduction').glob('*/result.json'))
        reproduced=json.loads(results[-1].read_text()) if results else {}
        report={'passed':timeout_error is None and child.returncode==0 and reproduced.get('passed') is True,
                'source_zip_sha256':sha(archive),'command':command,
                'fresh_extraction':True,'actual_live_sitl_rerun':True,
                'returncode':child.returncode,'result':str(results[-1]) if results else None,
                'fcu_received_verified':reproduced.get('fcu_received_verified',False),
                'source_snapshot_matches':extraction_matches,
                'rerun_source_matches':reproduced.get('source_sha256')==facts['source_sha256'],
                'checked_utc':datetime.now(timezone.utc).isoformat(),'error':timeout_error}
        report['passed']=report['passed'] and report['rerun_source_matches']
        write(out/'archive_reproduction.json',report)
        if not report['passed']:raise RuntimeError('Source ZIP did not reproduce the actual live ROS/PX4 checks')
    figures=[out/'interface_commands.png',out/'interface_timing.png']
    write(out/'publication_assets.json',{'figures':[str(p) for p in figures],'archive':str(archive)})
    prior=[]
    for path in sorted(out.parent.glob('*/result.json')):
        if path.parent==out:continue
        old=json.loads(path.read_text())
        if not old.get('passed'):
            item={'path':str(path),'error':old.get('error') or old.get('analysis_error')}
            if (path.parent/'interface.json').exists():item['interface']=json.loads((path.parent/'interface.json').read_text())
            if (path.parent/'rosbag.log').exists():item['recorder_log_excerpt']=(path.parent/'rosbag.log').read_text()[-1400:]
            prior.append(item)
    facts['prior_failed_attempts']=prior
    facts['archive_reproduction']=report
    facts['checked_utc']=datetime.now(timezone.utc).isoformat()
    write(out/'result.json',facts)
    manuscript=f'''# {TITLE}

이번에는 9편의 MPPI 제어기를 ROS 2 노드에 넣고, 실제 MAVROS 상태에서 계산한 정규화 추력과 몸체 각속도 목표를 PX4로 보냅니다. Gazebo SITL의 PX4 수신 로그와 {a['clock_aligned_pairs']}개 표본을 비교하여 허용 오차 내 일치율 {a['command_match_fraction']*100:.2f}%를 확인했습니다. 기체는 무장하지 않았으므로 이 결과는 입출력 연결 검증이며, 비행 성능은 다음 편에서 확인합니다.

[[FIGURE_TRAJECTORY]]

그림 1. MAVROS로 발행한 명령을 FRD로 변환한 값과 PX4 ULog의 수신 값을 겹쳤습니다. 두 곡선의 일치는 명령 전달과 좌표계 변환의 근거이며, 기체가 목표점을 따라갔다는 뜻은 아닙니다.

9-1의 [독립 제어기](https://sunggoo.tistory.com/96)와 9-2의 [자체 모델 폐루프 실험](https://sunggoo.tistory.com/100)을 실행한 뒤 읽으시면 좋습니다. 설치와 기본 이륙은 [2편](https://sunggoo.tistory.com/85)·[3편](https://sunggoo.tistory.com/84), 패키지 구조는 [4편](https://sunggoo.tistory.com/86)을 참조합니다. [전체 목차](https://sunggoo.tistory.com/83#drone-series-toc)에서 이번 글의 위치를 확인하실 수 있습니다.

## 1. MPPI와 PX4 사이의 입력 계약

7편의 모터 모델은 총 추력 N과 토크 N·m를 모터 회전속도로 바꾸는 단계였습니다. 이번 구조에서는 PX4가 내부 각속도 제어와 모터 배분을 맡습니다. MPPI가 토크를 계산한 다음 이름만 각속도로 바꾸는 방식이 아닙니다. 9편에서 입력과 예측 모델을 함께 바꾼 이유가 여기에 있습니다.

| 항목 | 노드에서 사용하는 정의 | PX4 연결 |
|---|---|---|
| 위치·선속도 | 세계 ENU, m·m/s | MAVROS 상태를 사용 |
| 자세 | FLU에서 ENU로 향하는 quaternion을 ZYX Euler로 변환 | 9편 예측 모델의 상태 |
| 몸체 각속도 | FLU p·q·r, rad/s | 오일러 각 변화율과 구별 |
| 제어입력 | [c, p_sp, q_sp, r_sp] | c는 무차원 정규화 추력 |
| 제어 범위 | MPPI는 예측·입력 선택 | PX4는 rate 제어·모터 배분 |

`c_hover=0.6`과 rate 시간상수 0.08초는 9편 모델의 가정입니다. 이번에는 무장하지 않았으므로 실제 추력 곡선이나 rate 응답을 식별하지 않았습니다. [7-1](https://sunggoo.tistory.com/89)·[7-2](https://sunggoo.tistory.com/90)의 SI 모터 모델과 최종 rate 모델을 같은 입력으로 읽지 않도록 구분해야 합니다.

## 2. 상태 메시지의 선속도 좌표계를 확인하기

MAVROS 2.14.0의 `local_position/odom`은 위치·자세를 map ENU로, twist를 base_link FLU로 제공합니다. 따라서 선속도를 그대로 9편 상태의 vx·vy·vz에 넣으면 몸체 방향에 따라 다른 속도를 넣게 됩니다. 노드는 quaternion에서 회전행렬을 구한 뒤 선속도만 세계 좌표계로 회전합니다. 몸체 각속도는 FLU 그대로 둡니다.

```python
# message = 실제 MAVROS Odometry, R = quaternion_rotation(q)
linear_body = np.array([v.x, v.y, v.z])
linear_enu = R @ linear_body
body_rate_flu = np.array([w.x, w.y, w.z])
```

회전행렬과 Euler 변환 전체 구현은 ZIP의 `ros_interface.py`에 있습니다. 의미를 확인하는 검사에서는 yaw=90도, 몸체 전방 속도 1m/s를 넣었을 때 ENU 속도가 [0, 1, 0]이 되고, 몸체 각속도는 변하지 않는지 확인했습니다. 알 수 없는 frame 이름, 영 quaternion, NaN 상태도 거부합니다. 이것은 좌표계 단위 검사이며 실제 비행 측정과 구별합니다.

공식 구현 근거: [MAVROS 2.14.0 local_position 소스](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/local_position.cpp).

## 3. AttitudeTarget의 네 필드를 채우기

발행 토픽은 `/sitl_check/mavros/setpoint_raw/attitude`입니다. `type_mask=128`은 자세 목표를 무시하고 몸체 rate와 추력을 사용하는 계약입니다. quaternion은 단위 값으로 채우되 자세 명령으로 사용하지 않습니다.

```python
msg = AttitudeTarget()
msg.type_mask = 128
msg.orientation.w = 1.0
msg.thrust = float(command[0])
msg.body_rate.x = float(command[1])
msg.body_rate.y = float(command[2])
msg.body_rate.z = float(command[3])
publisher.publish(msg)
```

메시지에는 FLU 값을 그대로 넣습니다. MAVROS가 FRD로 한 번 변환하여 PX4에서는 [p_sp, -q_sp, -r_sp]와 body Z 추력 -c가 기록됩니다. 노드에서 미리 q·r의 부호를 바꾸면 두 번 변환됩니다. 이번에는 발행한 정규화 추력과 PX4의 수신 값이 일치하는지를 로그로 확인했습니다. 다른 환경에서는 MAVROS `thrust_scaling` 파라미터도 확인해야 합니다.

공식 구현 근거: [MAVROS setpoint_raw 소스](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/setpoint_raw.cpp), [실행 PX4 커밋의 수신 코드](https://github.com/PX4/PX4-Autopilot/blob/{facts['px4_commit']}/src/modules/mavlink/mavlink_receiver.cpp).

## 4. 기존 패키지 구조에 노드와 launch를 추가하기

이 저장소에는 4편에서 설명한 `offboard_control` 패키지 실물이 없어서, 같은 ament_python 구조를 복원하고 `mppi_interface` 실행 항목과 launch 파일을 추가했습니다. 기존 이륙 스크립트를 덮어쓰거나 함께 실행하지 않습니다. 9편의 `mppi_series` 계산부는 이 패키지의 설치에 포함하여 같은 소스를 사용합니다.

이 진입점에는 NumPy·Matplotlib·pyulog·pymavlink, rosbag2의 MCAP 플러그인과 colcon이 필요합니다. 아래 버전의 기존 PX4/Gazebo 런타임과 Docker 이미지도 있어야 합니다. 응용 소스 ZIP만으로 PX4 설치가 대체되지는 않습니다.

ZIP을 푼 디렉토리에서 다음 명령을 실행합니다. ROS 환경을 읽은 뒤 PYTHONPATH를 추가할 때 기존 값을 유지해야 합니다. 처음 시도의 기록에는 ros2cli 패키지 메타데이터를 찾지 못한 오류가 남아 있습니다. 이 실패 기록과 아래 환경 구성으로 다시 실행한 결과를 따로 보존했습니다.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/simulation:$PYTHONPATH"
python3 -m unittest mppi_series.test_controller mppi_series.test_ros_interface -v
python3 -m mppi_series.episode_10_1 \\
  --workspace-root /home/sg/stack/src --output-root "$PWD/experiment_logs"
```

실험 진입점은 이번 실행 디렉토리에만 패키지를 빌드하고 headless PX4·MAVROS·ROS bag을 시작합니다. 그 뒤 설치된 `ros2 launch offboard_control mppi_interface.launch.py output:=실행경로`를 호출합니다. 출력의 마지막 `output`이 원본 로그와 분석 파일이 들어 있는 경로입니다. 단순히 Python 모듈이 실행되는지만 검사한 것이 아니라 패키지 빌드와 ROS 2 launch 진입점도 실제로 사용했습니다.

| 실행 환경 | 이번에 기록한 값 |
|---|---|
| 호스트 | Ubuntu 24.04, ROS 2 Jazzy |
| MAVROS | 2.14.0 |
| Gazebo | 호스트 Harmonic 8.12.0, 실행 컨테이너 8.11.0 |
| PX4 source commit | {facts['px4_commit']} |
| PX4 실행 바이너리 SHA-256 | {facts['px4_binary_sha256']} |
| 연결 | PX4 SITL instance 7, ROS_DOMAIN_ID=77, 별도 namespace |

소스가 dirty 상태인 것과 바이너리를 다시 빌드한 것은 다릅니다. 실제 실행한 바이너리·이미지 digest·설정·소스 diff는 provenance에 별도로 보존했습니다. ZIP에는 응용 코드·패키지·provenance 수집 코드·headless 서버 설정이 들어 있으며, 실행에는 이 ZIP 안의 파일을 사용합니다. PX4·Gazebo 바이너리는 포함하지 않습니다. 위 workspace 경로에는 third_party/px4_sitl/PX4-Autopilot의 기존 빌드·Gazebo 모델·월드가 있어야 하고 Docker 이미지와 ROS 의존성도 별도로 필요합니다. 다른 PC에서는 이 런타임과 의존성을 준비한 뒤 경로를 바꾸셔야 합니다. 실험 간 충돌을 막는 simulation/experiment_logs 잠금 디렉토리는 진입점이 생성합니다.

## 5. 예측 간격과 실제 계산·발행 주기

예측 적분 간격은 0.05초, 예측 길이는 20칸, 샘플은 500개, 온도는 3, 시드는 7입니다. 노드는 0.05초마다 실제 상태로 MPPI를 계산하고, 통과한 계산의 첫 입력을 한 번 발행합니다. 이는 20Hz를 목표로 하는 루프이며 하드 실시간 보장은 아닙니다. 계산 초과 시 오래된 입력을 계속 발행하는 별도 송신 스레드는 두지 않았습니다.

계산 전에 연결 상태·수신 경과시간·메시지 시각을 검사하고, 계산 뒤에도 다시 검사합니다. 수신 후 0.25초 초과, 메시지 시각이 0.5초보다 오래됨, 50ms 계산 예산 초과, 비정상 상태·출력에서는 해당 명령을 발행하지 않습니다. 이번 진입점은 무장된 FCU도 거부하는 제한된 인터페이스 실험입니다. 실제 비행에서 중단 후 어떤 제어기가 기체를 맡을지는 10-2의 제어 인계와 별도로 구현해야 합니다.

[[FIGURE_ERROR]]

그림 2. 실제 MPPI 계산 시간과 시각을 맞춰 비교한 명령 성분 오차입니다. 50ms 선은 이번 노드의 계산 예산이며, 데이터 전달·스케줄링을 포함한 전체 제어 지연의 상한은 아닙니다.

## 6. 발행 값과 PX4 수신 값을 로그로 비교하기

기체는 끝까지 무장하지 않았습니다. 실제 상태에서 네 구간의 목표 위치를 주어 MPPI 계산과 명령 발행을 확인했지만, 목표 위치를 향한 이동은 평가하지 않았습니다. CSV에는 목표와 측정 위치를 함께 보존했습니다.

| 확인 항목 | 실제 기록 |
|---|---|
| 원본 bag의 Odometry | {a['odom_messages']}개 |
| 원본 bag의 AttitudeTarget | {a['published_commands_in_bag']}개 |
| 원본 상태·계산·발행 메시지 대응 검사 | {a['calculated_commands_verified_against_bag']}개 |
| 동일 시드 MPPI 독립 재계산 최대 차이 | {a['independent_mppi_replay_max_error']:.3g} |
| 시각 정렬한 PX4 수신 표본 | {a['clock_aligned_pairs']}개 |
| 허용 오차 내 명령 일치율 | {a['command_match_fraction']*100:.2f}% |
| rate-Offboard 수신 구간 | {a['rate_offboard_duration_s']:.3f}초 |
| 계산 시간 평균 / 최대 | {a['compute_mean_ms']:.3f} / {a['compute_max_ms']:.3f}ms |
| 50ms 계산 초과 | {a['compute_over_50ms_count']}회 |
| 상태 만료로 차단한 뒤 발행 | {a['commands_after_stale_block']}개 |
| 무장 / 공중 상태 표본 | {a['armed_samples']} / {a['airborne_samples']}개 |

CSV의 계산 입력 상태를 같은 원본 Odometry 시각의 메시지에서 다시
구성했습니다. 상태 최대 차이는 {a['calculated_state_max_error']:.3g}이며,
같은 시드로 MPPI를 독립 재실행한 뒤 계산된 네 성분과 실제 bag에 기록된
AttitudeTarget을 명령 시각으로 연결했습니다. 계산/발행 최대 성분 차이는
{a['calculated_published_max_component_error']:.3g}입니다. 메시지의 float32
추력 표현에 따른 반올림을 포함하여 10^-6 이내인지 검사합니다.

실제 발행 간격의 평균/최대는 {a['publish_interval_mean_s']:.6f}/
{a['publish_interval_max_s']:.6f}초입니다. 전체 Odometry에서 초기 위치 대비
최대 이동은 {a['ground_position_max_excursion_m']:.6f}m였으며, 원본 상태는
live_odometry.csv에도 보존했습니다. 비무장 기체의 지상 상태 기록이므로
MPPI에 의해 기체가 움직였다는 증거로 해석하지 않습니다.

ULog의 MAVLink timesync_status가 기록한 시각 차이를 사용해 PX4 부팅 시각을 ROS 시각으로 옮겼습니다. 각 수신 표본에 대해 ±100ms 안에 발행된 후보 중 네 성분의 최대 차이가 가장 작은 명령을 선택합니다. 선택한 쌍의 최대 차이가 5×10^-5 미만인 비율이 95% 이상이어야 통과합니다. 이 규칙을 실행 전에 코드에 고정했습니다. 이는 지정한 시간 창에서 값이 일치하는지를 보는 검사입니다. 로그 샘플링과 시각 추정이 포함되므로 이 비교에서 얻은 시간차를 통신 지연의 정밀 측정으로 해석하지 않습니다.

전체 목표·측정 위치 차이의 3차원 RMSE는 {a['ground_reference_position_rmse_all_m']:.6f}m입니다. Offboard 관측 1초 뒤부터 상태 만료 주입 직전까지 고정한 구간은 {a['ground_reference_position_rmse_normal_m']:.6f}m입니다. 무장하지 않은 정지 기체와 이동 목표 사이의 차이이므로 두 값은 비행 추종 성능이 아닙니다. 다음 편에서는 실제 제어 인계 뒤의 움직임에 대해 RMSE를 평가합니다.

## 7. 상태가 끊겼을 때 실제로 관측한 동작

노드의 상태 갱신을 의도적으로 멈춘 뒤, 수신 경과시간이 기준을 넘으면 명령을 차단했습니다. 원본 bag에서 차단 이후 발행이 0개였고, ULog의 offboard_control_signal_lost 플래그가 켜진 것을 확인했습니다. 이때 무장하지 않은 PX4의 모드 표시는 {a['mode_after_stream_stop']}로 남았습니다.

처음에는 신호 손실 뒤 모드 표시도 반드시 바뀐다고 판정하여 실험이 실패했습니다. 실제 로그에는 신호 손실이 기록됐으므로, 이번에는 노드의 발행 중단과 PX4의 신호 손실 검출을 각각 확인합니다. 무장 상태의 비행 failsafe가 검증됐다는 뜻은 아닙니다. 계산 지연·잘못된 frame·NaN 거부는 별도의 코드 검사로 확인했으며 실제 비행 장애 실험으로 표현하지 않습니다.

원본 ULog·MCAP·commands.csv·received_commands.csv·analysis.json을 보존했습니다. 응용 소스를 ZIP으로 만든 뒤 새 디렉토리에 풀어 패키지를 다시 빌드하고 실제 SITL 검사를 한 번 더 실행했습니다. 재현 통과는 같은 수치의 반복이나 다양한 조건의 비행 성공을 뜻하지 않습니다.

## 8. 실행 파일과 다음 편

[[SOURCE_ARCHIVE]]

ZIP의 README에 실행 명령과 필요 환경이 있습니다. 독립 모델 검사 13개와 상태·메시지·발행 차단 검사 4개, 7편 참조 모델 100조건을 함께 실행하며, 실패한 원본 로그는 성공 실행으로 덮어쓰지 않습니다. 7편의 모터 배분 및 Euler rate 교정 근거도 companion_revision.md에 따로 정리했습니다. 이 파일이 준비된 것과 기존 글의 서버 개정이 완료된 것은 구별합니다.

이번에는 실제 상태에서 MPPI를 계산하여 ROS 2 노드로 발행하고 PX4의 수신 값까지 확인했습니다. 다음 10-2에서는 기존 이륙 절차 뒤 MPPI가 제어를 맡는 구간을 정하고, Gazebo 호버링과 목표점 이동을 원본 로그로 분석하겠습니다.
'''
    begin=manuscript.index('| 실행 환경 |');end=manuscript.index('\n\n소스가 dirty',begin)
    environment_table=manuscript[begin:end]
    manuscript=manuscript[:begin]+manuscript[end:]
    manuscript=manuscript.replace('ZIP을 푼 디렉토리에서 다음 명령을 실행합니다.',environment_table+'\n\nZIP을 푼 디렉토리에서 다음 명령을 실행합니다.')
    (out/'article.md').write_text(manuscript)
