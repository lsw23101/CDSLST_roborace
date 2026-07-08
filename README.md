# CDSLST_roborace

F1TENTH(RoboRacer) 차량을 ROS2 Humble + `f1tenth_gym_ros` 시뮬레이터 위에서, **미리 만들어둔 전역 경로(global path)를 Nav2의 MPPI(Model Predictive Path Integral) 컨트롤러로 추종**하게 만드는 프로젝트입니다.

이 저장소에는 우리가 직접 만든 ROS2 패키지(`src/f1tenth_mppi_nav`)와 트랙 경로 데이터(`paths/`)만 들어있습니다. 시뮬레이터 본체(`f1tenth_gym_ros`, `f1tenth_gym`)와 트랙 맵 데이터(`f1tenth_racetracks`)는 각각 별도의 공식 저장소를 그대로 클론해서 씁니다 (아래 설치 방법 참고).

## 전체 구조 요약

세 개의 서로 다른 성격의 컴포넌트가 토픽/액션으로 느슨하게 연결되어 있습니다: **① 외부에서 클론해오는 시뮬레이터**, **② ROS2 Humble에 이미 내장되어 있는 Nav2 표준 컴포넌트**, **③ 이 저장소에서 직접 만든 연결 코드**.

```
┌───────────────────────────┐   /scan, /map, /ego_racecar/odom (map→base_link tf 포함, ground truth)
│  ① f1tenth_gym_ros          │ ───────────────────────────────────────────┐
│  (외부 클론, 경량 물리        │                                              │
│   시뮬레이터 + RViz2 시각화)  │ ◄──────────────── /drive (AckermannDriveStamped)
└───────────────────────────┘                                              │
                                                                             │
┌───────────────────────────────────────────┐   ┌─────────────────────────┴──┐
│  ③ src/f1tenth_mppi_nav  (이 저장소)          │   │  ② nav2_controller_server    │
│                                                │   │  (ROS2 Humble 표준 설치,      │
│  paths/*.csv                                  │   │   ros-humble-navigation2     │
│      │                                        │   │   패키지 그대로 사용,          │
│      ▼                                        │   │   직접 구현한 코드 아님)       │
│  path_follower ──(FollowPath 액션 goal)───────▶│   │                             │
│                                                │   │  플러그인: nav2_mppi_        │
│                              /cmd_vel(Twist) ◄─┼───┤  controller::MPPIController  │
│                                    │           │   │  (motion_model: Ackermann)   │
│                                    ▼           │   └─────────────────────────────┘
│                        cmd_vel_to_ackermann     │
│                                    │            │
│                                    ▼            │
│                       /drive (AckermannDriveStamped) ──▶ (①로 다시)
└────────────────────────────────────────────────┘
```

- **① 시뮬레이터 (외부)**: `f1tenth_gym_ros` — 별도 저장소를 그대로 클론. 라이다 스캔, 맵, ground-truth 오도메트리(`map→ego_racecar/base_link` tf)를 퍼블리시하고 `/drive` 명령을 받아 차량을 움직입니다.
- **② MPPI 컨트롤러 (ROS2 Humble 내장)**: `nav2_mppi_controller`는 저희가 만든 게 아니라 `sudo apt install ros-humble-navigation2`로 설치되는 **Nav2의 표준 컨트롤러 플러그인**입니다. `controller_server`라는 Nav2 표준 노드(역시 내장)가 이 플러그인을 로드해서 실행합니다. 저희는 이걸 직접 실행(`ros2 run nav2_controller controller_server`)하고, `src/f1tenth_mppi_nav/config/nav2_params.yaml`로 파라미터(속도 제한, motion model, critic 가중치 등)만 설정해서 씁니다 — MPPI 알고리즘 자체의 코드는 한 줄도 저희가 작성하지 않았습니다.
- **③ 연결 코드 (이 저장소)**: ①과 ②는 원래 서로 호환되게 설계된 게 아니라서, 이 둘을 잇는 최소한의 코드만 저희가 작성했습니다.
  - `path_follower`: 미리 만든 전역 경로(csv)를 읽어서 ②의 `controller_server`가 노출하는 `follow_path` 액션에 직접 전송 (Nav2의 `bt_navigator`/`planner_server`는 안 씀 — 목표 지점을 매번 지정할 필요가 없어짐)
  - `cmd_vel_to_ackermann`: ②가 표준으로 출력하는 `geometry_msgs/Twist`(`/cmd_vel`)를 ①이 요구하는 `AckermannDriveStamped`(`/drive`)로 변환 (자전거 모델 기반)
- **맵/전역 경로**: 맵은 `f1tenth_racetracks`가 제공하는 실제 F1 서킷(Silverstone)을 그대로 쓰고, 전역 경로도 그 저장소가 제공하는 정확한 centerline 데이터를 `scripts/convert_racetrack_csv.py`로 변환해서 **오프라인으로 한 번만** 만들어 `paths/silverstone_track.csv`에 저장해둔 것입니다.

---

## 1. 환경 요구사항

- Ubuntu 22.04 (WSL2도 가능 — 이 프로젝트는 Windows 11 + WSL2 Ubuntu-22.04에서 개발/테스트되었습니다)
- ROS2 Humble
- (WSLg 또는 X11 등으로) RViz2 GUI를 띄울 수 있는 환경

시뮬레이터가 순정 최신 Gazebo(gz-sim)가 아니라 `f1tenth_gym_ros`가 제공하는 자체 경량 물리엔진 + RViz2 시각화를 사용한다는 점 참고하세요. 실제 Gazebo 물리엔진 연동은 향후 과제입니다.

---

## 2. 설치

### 2.1 ROS2 패키지 설치

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-ackermann-msgs ros-humble-joint-state-publisher-gui ros-humble-xacro
```

> ROS 저장소 GPG 키가 만료되어 `apt update`에서 서명 에러가 나면:
> ```bash
> sudo rm -f /etc/apt/sources.list.d/ros2.list
> curl -sSL -o /tmp/ros2-apt-source.deb https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb
> sudo apt install /tmp/ros2-apt-source.deb
> sudo apt update
> ```

### 2.2 워크스페이스 구성

```bash
mkdir -p ~/roboracer_ws/src
cd ~/roboracer_ws

# 1) 시뮬레이터 (ROS2 브릿지)
git clone https://github.com/f1tenth/f1tenth_gym_ros.git src/f1tenth_gym_ros

# 2) 시뮬레이터가 쓰는 물리엔진 라이브러리 (pip 패키지, 워크스페이스 루트에 아무데나)
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym && pip3 install -e . && cd ..

# 3) 실제 F1 트랙 맵 + centerline 데이터
git clone --depth 1 https://github.com/f1tenth/f1tenth_racetracks.git

# 4) 이 저장소 (우리가 만든 MPPI 패키지 + 경로 데이터)
git clone https://github.com/lsw23101/CDSLST_roborace.git /tmp/cdslst_roborace
cp -r /tmp/cdslst_roborace/src/f1tenth_mppi_nav src/
cp -r /tmp/cdslst_roborace/paths .
rm -rf /tmp/cdslst_roborace
```

> `f110_gym`(f1tenth_gym)은 `numpy<=1.22.0`, `numba` 구버전에 강하게 의존합니다. 나중에 다른 파이썬 패키지(특히 `scikit-image`, `scipy` 등)를 설치할 일이 있으면 `numpy`가 최신 버전으로 올라가면서 시뮬레이터가 깨질 수 있으니, 설치 후에는 항상 `python3 -c "import numpy; print(numpy.__version__)"`로 `1.22.0`인지 확인하세요.

### 2.3 Silverstone 맵을 시뮬레이터에 연결

```bash
cp ~/roboracer_ws/f1tenth_racetracks/Silverstone/Silverstone_map.png \
   ~/roboracer_ws/f1tenth_racetracks/Silverstone/Silverstone_map.yaml \
   ~/roboracer_ws/src/f1tenth_gym_ros/maps/
```

`~/roboracer_ws/src/f1tenth_gym_ros/config/sim.yaml`을 열어 아래처럼 수정하세요 (본인의 실제 홈 경로로):

```yaml
map_path: '/home/<본인계정>/roboracer_ws/src/f1tenth_gym_ros/maps/Silverstone_map'
...
sx: 0.0
sy: 0.0
stheta: 0.9444   # paths/silverstone_track.csv 첫 waypoint의 heading과 일치
...
kb_teleop: False   # MPPI가 /cmd_vel을 쓰므로 키보드 텔레옵과 충돌 방지를 위해 반드시 꺼야 함
```

> `kb_teleop: True`로 두면 `gym_bridge`가 `/cmd_vel`을 직접 가로채서 조향각을 ±0.3rad로 뭉개버리는 내부 로직과 MPPI 명령이 충돌합니다. 반드시 `False`로 두세요.

### 2.4 빌드

```bash
cd ~/roboracer_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --base-paths src --symlink-install
```

> `colcon build`를 워크스페이스 루트에서 그냥 실행하면 2.2의 2)번에서 클론한 `f1tenth_gym`(pip 라이브러리, ROS 패키지 아님)까지 ROS 패키지로 착각하고 빌드하려다 실패합니다. 항상 `--base-paths src` 옵션을 붙이세요.

편의를 위해 아래 alias를 `~/.bashrc`에 추가해두면 편합니다:
```bash
alias roshumble='source /opt/ros/humble/setup.bash && source ~/roboracer_ws/install/local_setup.bash'
```

---

## 3. 실행 방법 (터미널 2개)

**터미널 1 — 시뮬레이터 + RViz2**
```bash
roshumble
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
RViz2 창이 뜨고 Silverstone 트랙과 차량, 라이다 스캔이 보이면 정상입니다.

**터미널 2 — MPPI 경로 추종**
```bash
roshumble
ros2 launch f1tenth_mppi_nav mppi_path_follow_launch.py
```
`path_follower` 노드가 `paths/silverstone_track.csv`를 읽어서 `controller_server`(MPPI)에 바로 전송합니다. 몇 초 안에 **아무 조작 없이도** 차량이 트랙을 따라 스스로 주행을 시작합니다.

### MPPI 샘플 궤적 시각화

RViz2 Displays 패널에서 **Add → By topic**으로 아래 두 개를 추가하면 MPPI가 매 스텝 평가하는 후보 궤적 다발을 눈으로 볼 수 있습니다:
- `/trajectories` (`visualization_msgs/MarkerArray`) — 샘플링된 후보 궤적들
- `/transformed_global_plan` (`nav_msgs/Path`) — 현재 추종 중인 경로 구간

추가한 뒤 RViz2에서 `Ctrl+S`를 누르면 `f1tenth_gym_ros/launch/gym_bridge.rviz`에 저장되어 다음 실행부터 자동으로 뜹니다.

### 차량을 손으로 재배치하기

RViz2 상단 툴바의 **"2D Pose Estimate"** 툴로 맵 위 아무 곳이나 클릭+드래그하면 차량이 그 위치/방향으로 즉시 리셋됩니다 (`gym_bridge`가 `/initialpose`를 구독해서 처리). 코너에서 벽에 부딪혔을 때 트랙 위로 다시 옮겨놓는 용도로 씁니다.

### 대화형 모드 (경로 미리 지정 없이 매번 목표 지정)

미리 만든 경로 대신 RViz의 "2D Goal Pose"로 매번 목표를 클릭해서 그때그때 전역 경로를 계산하고 싶다면:
```bash
ros2 launch f1tenth_mppi_nav mppi_bringup_launch.py
```
이 launch는 `planner_server`(NavFn) + `bt_navigator`까지 같이 띄워서, "2D Goal Pose"로 클릭한 지점까지 매번 새로 전역 경로를 계산 후 MPPI로 추종합니다.

---

## 4. 전역 경로(global path)는 어떻게 만들었나

`paths/silverstone_track.csv`는 아래 절차로 만들어졌습니다 (한 번만 오프라인으로 실행하면 됨):

```bash
python3 src/f1tenth_mppi_nav/scripts/convert_racetrack_csv.py \
  f1tenth_racetracks/Silverstone/Silverstone_centerline.csv \
  paths/silverstone_track.csv
```

`f1tenth_racetracks`가 각 트랙마다 제공하는 `*_centerline.csv`(x_m, y_m, 좌우 트랙폭)를 읽어서, 우리 포맷(`x, y, yaw`)으로 변환합니다. `yaw`는 연속된 두 점 사이의 방향각으로 계산합니다. 이렇게 만든 경로가 실제 맵과 잘 정렬되는지는 트랙 링 픽셀 무게중심과 경로 무게중심을 비교해서 검증했습니다.

### 대안: 전역 경로 데이터가 없는 임의의 맵에서 자동 추출

공식 centerline 데이터가 없는 맵이라면, 맵 이미지에서 자유공간을 스켈레톤화(골격 추출)해서 폐루프 중심선을 자동으로 뽑아내는 도구도 포함되어 있습니다:

```bash
pip3 install --user scikit-image networkx   # numpy가 올라가지 않도록 버전 확인 필수 (2.1절 참고)
python3 src/f1tenth_mppi_nav/scripts/extract_centerline.py <map.yaml> <output.csv>
```

내부적으로:
1. occupancy 값으로 자유공간 마스크 생성 (`free_thresh` 기준, "unknown" 회색 영역 제외)
2. 4-connectivity로 연결영역을 분리해 "구멍이 있는" 링 형태(Euler number ≤ 0) 영역만 트랙으로 식별 — 순수 선으로만 그려진 맵(내부/외부가 채워지지 않은 라인아트)에서 얇은 벽선이 안팎을 제대로 못 갈라놓는 문제를 해결하기 위함
3. `skimage.morphology.skeletonize`로 중심선 추출 → 그래프화 → 짧은 곁가지 제거 → 최장 폐루프 사이클 탐색 → 순서대로 정렬 → 좌표 변환 + 스무딩

시케인(S자 구간)이 많은 복잡한 트랙에서는 완벽하지 않을 수 있어(경로가 지름길로 샐 수 있음), 가능하면 공식 centerline 데이터가 있는 맵에는 `convert_racetrack_csv.py` 사용을 권장합니다.

### 직접 운전해서 경로 기록하기 (또 다른 대안)

`record_path` / `path_follower` 조합으로, 텔레옵으로 직접 트랙을 한 바퀴 돌면서 `map → base_link` tf를 기록해 경로를 만들 수도 있습니다:
```bash
ros2 run f1tenth_mppi_nav record_path --ros-args -p output_file:=$(pwd)/paths/my_track.csv
```
주행 후 `Ctrl+C`하면 저장됩니다.

---

## 5. MPPI 제어는 어디서 어떻게 동작하나

- **컨트롤러 본체**: ROS2 Humble Nav2에 기본 포함된 `nav2_mppi_controller` 패키지를 그대로 사용합니다. 별도 알고리즘 구현 없음.
- **설정 파일**: `src/f1tenth_mppi_nav/config/nav2_params.yaml`의 `controller_server.FollowPath` 섹션
  - `motion_model: "Ackermann"` — 차량이 애커만 조향이므로 지정. `AckermannConstraints.min_turning_r`은 f1tenth_gym 기본 차량 제원(휠베이스 0.3302m, 최대 조향각 24°)에서 계산한 최소 회전반경(~0.75m)
  - `vx_max`, `wz_max` 등 속도/각속도 제한: 특히 `wz_max`는 목표 속도에서 타이어 마찰로 실제 낼 수 있는 최대 요레이트를 넘지 않도록 설정 — 이걸 너무 크게 잡으면 MPPI가 기구학적으로만 가능한(마찰 한계를 넘는) 급회전을 계획해서 코너에서 미끄러져 벽에 부딪힘
  - 각 critic(`CostCritic`, `PathAlignCritic`, `PathFollowCritic` 등)의 가중치로 "장애물 회피 vs 경로 추종 vs 목표 지향" 사이의 우선순위를 조정
- **로컬라이제이션**: AMCL/SLAM을 쓰지 않습니다. `f1tenth_gym_ros`의 `gym_bridge`가 시뮬레이션 ground-truth를 그대로 `map → ego_racecar/base_link` tf로 퍼블리시하기 때문에 별도 위치추정이 필요 없습니다.
- **경로 전달 경로 (goal-pose 없이)**: `path_follower` 노드가 CSV를 읽어 `nav_msgs/Path`로 만든 뒤, `controller_server`가 노출하는 `follow_path` 액션(`nav2_msgs/action/FollowPath`)에 **직접** 전송합니다. `bt_navigator`/`planner_server`를 거치지 않으므로 "2D Goal Pose" 클릭이 필요 없습니다.
- **명령 변환**: `nav2_mppi_controller`는 표준 Nav2 인터페이스인 `geometry_msgs/Twist`(`/cmd_vel`)를 출력합니다. 하지만 `f1tenth_gym_ros`는 `AckermannDriveStamped`(`/drive`)를 구독합니다. 이 둘을 잇는 게 `cmd_vel_to_ackermann` 노드로, 자전거 모델(`steering_angle = atan2(wheelbase * wz, vx)`)로 변환합니다.

---

## 6. 알려진 이슈 / 트러블슈팅 메모

- **`rosdep init` 에러 (`sources list already exists`)**: 이미 초기화된 것이므로 무시하고 `rosdep update`만 실행하면 됩니다.
- **ROS apt 저장소에서 특정 `.deb`가 404**: `packages.ros.org` 미러 동기화 지연 문제입니다. `sudo apt update` 재시도, 그래도 안 되면 몇 분 뒤 다시 시도하세요.
- **`lifecycle_manager`가 "Failed to change state" 에러**: 같은 이름(`map_server` 등)의 노드가 이미 다른 터미널에서 떠 있는 경우입니다. `ps aux | grep map_server` 등으로 좀비 프로세스를 찾아 종료하세요. (이 저장소의 launch 파일들은 애초에 `map_server`를 직접 띄우지 않고 `f1tenth_gym_ros`가 띄운 걸 재사용하도록 만들어서 이 문제를 피합니다.)
- **`CostCritic` 관련 "no robot footprint provided" 에러**: `consider_footprint: true`인데 costmap에 정확한 폴리곤 footprint가 없어서 발생. 이 저장소 설정은 `consider_footprint: false`로 두고 `robot_radius`(원형 근사)를 씁니다.
- **Gazebo Harmonic/Ionic 버전 충돌**: 현재 파이프라인은 Gazebo를 전혀 쓰지 않아 무관하지만, 나중에 실제 Gazebo 물리 시뮬레이션으로 확장할 계획이 있다면 `gz-ionic`과 `ros-humble-ros-gzharmonic-*`을 동시에 설치하지 않도록 주의하세요 (`gz sim --version` 실행 시 protobuf 중복 등록 에러가 뜨면 충돌 상태).
- **시뮬레이터 실행 시 `AttributeError: module 'coverage' has no attribute 'types'`로 `gym_bridge`가 죽는 경우**: 시스템에 이미 깔려있던 `python3-coverage`(apt) 버전이 `numba`(물리엔진이 씀)와 안 맞아서 생깁니다. `~/.local`에 최신 버전을 pip로 설치해서 덮어쓰면 해결됩니다:
  ```bash
  pip3 install --user --upgrade coverage
  ```
- **`import numpy` 버전이 안 맞거나, 이후 다른 pip 패키지 설치로 `numpy`가 최신 버전으로 올라가버린 경우**: `f110_gym`은 `numpy<=1.22.0`에 강하게 의존합니다 (`numba` 구버전과 ABI 호환 문제). 아래처럼 버전을 되돌리면 됩니다:
  ```bash
  pip3 install --user 'numpy==1.22.0' 'scipy==1.8.0'
  python3 -c "import f110_gym; print('OK')"   # 확인
  ```

---

## 7. 실제 하드웨어(F1TENTH 실차)로 이관하기

이 스택은 시뮬레이터와 실차가 **동일한 토픽 규약**(`/scan`, `/drive`, `map→base_link` tf)을 쓰도록 처음부터 설계되어 있어서, MPPI/경로추종 쪽 코드(`controller_server` 설정, `path_follower`, `cmd_vel_to_ackermann`)는 거의 그대로 재사용할 수 있습니다. 다만 시뮬레이터는 "정답 위치"를 그냥 tf로 흘려주지만 실차에는 그게 없으므로, **위치추정(localization)을 새로 추가**해야 합니다. 큰 흐름은 아래와 같습니다.

### 7.1 실차 드라이버 브링업

`f1tenth/f1tenth_system`(공식 실차 드라이버 저장소, 라이다 + VESC 모터 드라이버)을 실차의 온보드 컴퓨터(Jetson 등)에 설치해서 브링업합니다. 이 저장소도 `/scan`, `/drive`, `/odom`(휠 오도메트리)을 시뮬레이터와 같은 메시지 타입으로 발행/구독하도록 되어 있습니다.

### 7.2 맵 만들기 (SLAM)

시뮬레이터와 달리 이제 진짜로 맵이 없으므로, `slam_toolbox`로 실제 트랙을 한 바퀴 천천히 수동 주행(조이스틱/키보드 텔레옵)하면서 맵을 만듭니다:
```bash
sudo apt install ros-humble-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py   # 트랙 주행하며 맵 완성
ros2 run nav2_map_server map_saver_cli -f ~/my_track_map   # 완성되면 저장
```
저장된 `my_track_map.yaml`/`.png`가 지금 쓰는 `Silverstone_map.yaml`/`.png`와 **완전히 같은 포맷**입니다 — 이후 과정은 형식 걱정 없이 그대로 이어집니다.

### 7.3 로컬라이제이션 추가 (시뮬레이션과의 가장 큰 차이)

7.2에서 만든 맵에 대해 `slam_toolbox`를 localization 모드로 켜거나(추천) `nav2_amcl`을 씁니다. 둘 다 라이다 스캔을 맵과 매칭해서 `map→odom` tf를 계산해주고, 휠 오도메트리가 `odom→base_link`를 채워줘서 결과적으로 `controller_server`가 필요로 하는 `map→base_link`가 완성됩니다. `src/f1tenth_mppi_nav/config/nav2_params.yaml`에는 로컬라이제이션 관련 설정이 빠져있으니, 실차용으로는 `amcl` 섹션을 새로 추가하고 `lifecycle_manager`의 관리 노드 목록에도 넣어야 합니다.

### 7.4 전역 경로 만들기 — 지금과 완전히 같은 방법

7.2에서 만든 맵에 대해 아래 둘 중 하나를 오프라인으로 한 번만 실행하면 됩니다 (지금 Silverstone에서 한 것과 동일한 절차):
- `scripts/extract_centerline.py <my_track_map.yaml> <output.csv>` — 이미지 스켈레톤화로 자동 추출 (실제 SLAM 맵은 대개 안/밖이 채워진 형태라 Silverstone처럼 순수 선으로만 그려진 맵보다 오히려 더 안정적으로 동작할 가능성이 높습니다)
- `record_path.py`로 직접 한 바퀴 운전하며 기록 — 이제 7.3의 실제 로컬라이제이션이 `map→base_link`를 채워주므로 시뮬레이션 때와 동일한 방식으로 바로 동작합니다

### 7.5 파라미터 재조정 (반드시 필요)

- `robot_base_frame`을 실차 URDF의 프레임 이름(보통 `base_link`, 시뮬레이션의 `ego_racecar/base_link`가 아님)에 맞게 수정
- `odom_topic`을 실차 드라이버가 실제 발행하는 토픽명으로 수정
- `vx_max`, `wz_max`, `AckermannConstraints.min_turning_r`을 실차의 실제 휠베이스/최대 조향각/타이어 마찰 한계에 맞게 다시 계산 — 처음엔 지금보다 훨씬 보수적으로 낮게 잡고 점진적으로 올리는 걸 강력히 권장
- MPPI `batch_size`/`time_steps`는 온보드 컴퓨터(Jetson 등)의 실시간 연산 성능에 맞게 낮춰야 할 수 있음 (시뮬레이션은 PC에서 여유 있게 돌지만 임베디드 보드는 연산력이 부족할 수 있음)
- `cmd_vel_to_ackermann`의 `wheelbase`/`max_steering_angle` 파라미터를 실차 실측값으로 수정

### 7.6 안전

처음엔 반드시 저속으로, 사람이 즉시 개입할 수 있는 킬스위치/비상정지를 준비한 상태에서 테스트하세요.

---

## 8. TODO / 다음 단계

- [ ] 더 빠른 랩타임을 위한 MPPI critic 가중치 추가 튜닝
- [ ] `f1tenth_gym_ros`의 경량 물리엔진이 아니라 실제 Gazebo(gz-sim) 물리 시뮬레이션으로 이전 (Ackermann steering 플러그인 + `ros_gz` 브릿지)
- [ ] 실차(F1TENTH 하드웨어) 이관 — 7절 참고
