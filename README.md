# ROS 2 × PX4 자율비행 드론 제어: MPPI 연재 코드

[블로그 연재 목차](https://sunggoo.tistory.com/83)의 9-1~12-2편에 대응하는 제어기, ROS 2 연결, Gazebo/PX4 SITL 실험 소스입니다. 글과 코드를 함께 읽을 수 있도록 아래 표에서 각 회차의 핵심 파일로 연결합니다. 연재는 12-2에서 완결되었습니다.

## 선행 회차에서 이어 읽기

1~8편은 설치, 개념, 초기 예제를 설명합니다. 아래 파일은 그 내용을 후반 연재에서 발전시킨 **참조 구현**입니다. 초기 글의 코드 블록을 그대로 옮긴 파일은 아닙니다. 특히 7~8편 예제의 입력·좌표계와 9편 이후의 `[c_norm, p_sp, q_sp, r_sp]` 인터페이스를 그대로 연결해서 실행하면 안 됩니다.

### 왜 코드가 다른가?

연재 중 제어기가 PX4에 넘기는 **명령의 수준**을 확정했습니다. 3~4편의 위치 목표, 7편의 힘·토크와 모터 배분, 8편의 샘플링 의사코드는 각각 다른 학습 단계입니다. 9편부터의 실제 MPPI 구현은 MAVROS `AttitudeTarget`의 몸체 각속도 목표와 정규화 추력을 사용합니다. PX4가 그 아래의 각속도 제어와 모터 배분을 수행하므로, 앞부분의 토크나 모터 속도 네 개를 후반 코드의 입력 네 개로 이름만 바꿔 넣을 수 없습니다.

| 구간 | 입력과 목적 | 이 저장소에서 읽을 파일 |
| --- | --- | --- |
| 3~4편 | 위치 목표를 보내는 Offboard 입문 | `offboard_control/`은 **후반 rate 제어 패키지**이며 글의 위치 목표 노드와 같은 코드가 아닙니다. |
| 7-1·7-2편 | `[T, τx, τy, τz]` (N, N·m)에서 설명용 모터 힘·속도와 강체 운동 계산 | [`rigid_body_reference.py`](simulation/mppi_series/rigid_body_reference.py). 모터 계수는 설명용이며 Gazebo 기체에서 식별한 값이 아닙니다. |
| 8-1·8-2편 | 샘플 경로와 비용 항을 단계별로 보여 주는 예제 | [`controller.py`](simulation/mppi_series/controller.py)는 입력·상태·제약을 통일한 **후속 통합 구현**입니다. [`obstacles.py`](simulation/mppi_series/obstacles.py)는 11-2의 장애물 확장입니다. |
| 9-1~12-2편 | `[c_norm, p_sp, q_sp, r_sp]` (무차원, FLU rad/s)로 PX4에 목표 전달 | [`controller.py`](simulation/mppi_series/controller.py), [`ros_interface.py`](simulation/mppi_series/ros_interface.py), [`ros_flight.py`](simulation/mppi_series/ros_flight.py) 순서로 확인하세요. |

**초기 글의 오류도 있습니다.** 7-1편의 게시된 모터 함수는 모터 힘을 회전속도로 바꾸는 제곱근 변환이 없고 회전 방향 부호를 속도에 곱한 뒤 음수를 잘라냅니다. 예시 호버 요청 19.62 N에 대한 게시 함수 출력 `[4.905, 4.905, 0, 0]`을 rad/s로 해석하면 총 추력은 약 0.000411 N입니다. 7-2편에는 몸체 각속도 `(p,q,r)`를 Euler 각 변화율과 동일하게 적분하는 부분이 있습니다. 이 둘은 같은 값이 아닙니다. 위 참조 구현은 이 부분을 바로잡은 별도 모델이며, 당시 게시된 코드가 그대로 실행됐다는 뜻이 아닙니다. 검사는 **수학·Python 모델**에 한정되고 PX4·Gazebo 비행이나 실기체 검증은 아닙니다.

아래 명령은 7편의 설명용 파라미터에서 2 kg 기체의 호버 힘을 네 모터에 균등 배분해 약 757.48 rad/s를 얻는 계산입니다. **9편 MPPI 명령을 생성하거나 PX4에 발행하지 않습니다.**

```bash
PYTHONPATH=simulation /usr/bin/python3 - <<'PY'
from mppi_series.rigid_body_reference import SIParams, allocate, realized_wrench
p = SIParams()
omega = allocate([p.mass * 9.81, 0, 0, 0], p)
print(omega)
print(realized_wrench(omega, p))
PY
```

| 회차 | 블로그 글 | 이어 볼 코드 |
| --- | --- | --- |
| 1 | [환경 설정](https://sunggoo.tistory.com/83) | 설치 명령은 글 본문에 있습니다. |
| 2 | [PX4·Gazebo·MAVROS 연결](https://sunggoo.tistory.com/85) | [`mppi_interface.launch.py`](offboard_control/launch/mppi_interface.launch.py): 후반 MPPI 연결용 launch |
| 3 | [Offboard 기초](https://sunggoo.tistory.com/84) | [`offboard_control/`](offboard_control/): 후반 rate Offboard 패키지. 글의 위치 명령 예제와 인터페이스가 다릅니다. |
| 4 | [ROS 2 패키지 구성](https://sunggoo.tistory.com/86) | [`offboard_control/`](offboard_control/): 연재 후반의 실제 패키지 구조 |
| 5 | [MPPI의 직관](https://sunggoo.tistory.com/87) | [`controller.py`](simulation/mppi_series/controller.py): 완성된 샘플링·가중 업데이트 |
| 6 | [쿼드로터 운동모델](https://sunggoo.tistory.com/88) | [`rigid_body_reference.py`](simulation/mppi_series/rigid_body_reference.py): SI 단위 참조 계산 |
| 7-1 | [모터 배분](https://sunggoo.tistory.com/89) | [`rigid_body_reference.py`](simulation/mppi_series/rigid_body_reference.py): 힘·토크와 모터 회전속도의 단위를 구분한 참조 계산 |
| 7-2 | [회전행렬·각속도](https://sunggoo.tistory.com/90) | [`rigid_body_reference.py`](simulation/mppi_series/rigid_body_reference.py): 회전·각속도 참조 계산 |
| 8-1 | [MPPI 샘플링](https://sunggoo.tistory.com/92) | [`controller.py`](simulation/mppi_series/controller.py): 통합된 샘플링 제어기 |
| 8-2 | [비용 함수](https://sunggoo.tistory.com/93) | [`controller.py`](simulation/mppi_series/controller.py), [`obstacles.py`](simulation/mppi_series/obstacles.py): 통합 비용 함수와 후속 장애물 확장 |

## 구현·실험 회차

| 회차 | 블로그 글 | 핵심 코드 |
| --- | --- | --- |
| 9-1 | [MPPI Controller 구현](https://sunggoo.tistory.com/96) | [`controller.py`](simulation/mppi_series/controller.py), [`example.py`](simulation/mppi_series/example.py) |
| 9-2 | [폐루프 제어](https://sunggoo.tistory.com/100) | [`closed_loop.py`](simulation/mppi_series/closed_loop.py) |
| 10-1 | [MPPI와 PX4 연결](https://sunggoo.tistory.com/102) | [`ros_interface.py`](simulation/mppi_series/ros_interface.py), [`mppi_node.py`](offboard_control/offboard_control/mppi_node.py) |
| 10-2 | [Gazebo 호버링·목표점 이동](https://sunggoo.tistory.com/106) | [`ros_flight.py`](simulation/mppi_series/ros_flight.py), [`mppi_flight_node.py`](offboard_control/offboard_control/mppi_flight_node.py) |
| 11-1 | [궤적 추종](https://sunggoo.tistory.com/107) | [`trajectory.py`](simulation/mppi_series/trajectory.py), [`trajectory_runtime.py`](simulation/mppi_series/trajectory_runtime.py) |
| 11-2 | [장애물 비용·회피 실험](https://sunggoo.tistory.com/123) | [`obstacles.py`](simulation/mppi_series/obstacles.py), [`obstacle_runtime.py`](simulation/mppi_series/obstacle_runtime.py) |
| 12-1 | [MPPI와 PX4 제어 비교](https://sunggoo.tistory.com/130) | [`comparison_runtime.py`](simulation/mppi_series/comparison_runtime.py), [`comparison_results.py`](simulation/mppi_series/comparison_results.py) |
| 12-2 | [전체 재실행·재현성 점검](https://sunggoo.tistory.com/151) | [`episode_12_2.py`](simulation/mppi_series/episode_12_2.py), [`reproduction_audit.py`](simulation/mppi_series/reproduction_audit.py) |

## 코드 구조

- `simulation/mppi_series/controller.py`: 자체 동역학 모델과 MPPI 샘플링 제어기. 입력은 `[c_norm, p_sp, q_sp, r_sp]`로, 정규화 추력과 FLU 몸체 rate 목표(rad/s)입니다.
- `simulation/mppi_series/ros_interface.py`, `ros_flight.py`: 상태 수신, 명령 생성과 비행 게이트. PX4의 내부 rate 제어와 모터 배분은 MPPI 코드와 별도입니다.
- `offboard_control/`: ROS 2 패키지의 노드와 launch 파일.
- `simulation/mppi_series/episode_*.py`: 당시 회차별 실험 실행기. 분석, 원고 생성, 증거 기록 코드도 당시 소스 상태를 추적할 수 있도록 포함했습니다.

## 빠른 코드 검사

Ubuntu 24.04, Python 3, NumPy와 ROS 2 Jazzy 환경에서 다음 단위 검사를 실행할 수 있습니다. 비행 검증이나 PX4 빌드 검사는 아닙니다.

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=simulation /usr/bin/python3 -m unittest \
  mppi_series.test_controller \
  mppi_series.test_ros_interface \
  mppi_series.test_ros_flight
```

Gazebo 비행 재실행에는 별도의 PX4-Autopilot 체크아웃, MAVROS, Gazebo 리소스, 빌드된 ROS 2 패키지와 원본 실험 설정이 필요합니다. 회차 실행기 일부는 원래 작업 공간의 `publishing/` 증거·원고와 지침을 읽도록 작성되었습니다. 이 저장소는 해당 로컬 자료, 원본 ULog/ROS bag, 게시 자동화, 계정 정보를 포함하지 않으므로 전체 실험이 이 저장소만으로 재현된다고 보장하지 않습니다. 각 글에 첨부된 재현 자료와 환경 명세를 함께 확인해 주세요.

## 출처와 버전

소스는 연재 실험 프로젝트의 2026-10-25 스냅샷에서 분리했습니다. 코드를 바꾼 뒤 결과가 달라지면 해당 글의 측정값을 새 코드의 측정값으로 간주하지 마세요. 이 저장소의 커밋 ID와 실험 결과에 기록된 코드·PX4 버전을 대조해야 합니다.
