# CLAUDE.md — CDSLST_roborace 작업 컨텍스트

이 파일은 다음 세션에서 Claude가 빠르게 맥락을 파악하도록 남기는 메모입니다. 사용자 대상 사용법은 `README.md`를 참고하세요 (이 파일은 그걸 대체하지 않음).

## 이 워크스페이스가 뭔지

F1TENTH/RoboRacer 차량을 ROS2 Humble + `f1tenth_gym_ros` 시뮬레이터 위에서, 미리 만든 전역 경로(Silverstone 트랙)를 Nav2의 `nav2_mppi_controller`로 추종시키는 프로젝트. GitHub: https://github.com/lsw23101/CDSLST_roborace (branch: `main`).

환경: Windows 11 + WSL2 Ubuntu-22.04. Claude Code는 Windows 쪽에서 실행되고, 실제 작업은 전부 `wsl.exe -d Ubuntu-22.04 -- bash -lc "..."`로 WSL 안에 들어가서 함 (Bash 툴 자체는 Git Bash라 ROS 환경이 없음 — 반드시 `wsl.exe` 경유). 파일 편집은 `//wsl.localhost/Ubuntu-22.04/home/sangwon/...` UNC 경로로 직접 Read/Edit/Write 가능.

## 현재 상태 (지금까지 확인된 것)

- 시뮬레이터(`f1tenth_gym_ros`) + Nav2 MPPI + 저장된 전역 경로 추종 **정상 동작 확인됨** (RViz2에서 차량이 Silverstone 트랙을 스스로 완주 시도)
- git 저장소 초기화 + 첫 몇 커밋 push 완료 (사용자가 직접 push — 이 환경엔 GitHub 인증 수단이 없어서 push는 항상 사용자가 함)
- README.md에 설치법, 아키텍처, 트러블슈팅, 실차 이관 가이드까지 작성 완료

## 아키텍처 요약 (자세한 건 README 참고)

- **맵**: `f1tenth_racetracks`(외부 클론)의 Silverstone 맵 → `f1tenth_gym_ros/maps/`에 복사, `sim.yaml`에서 `map_path`/`sx,sy,stheta`/`kb_teleop:False` 수정해둠
- **전역 경로**: `f1tenth_racetracks/Silverstone/Silverstone_centerline.csv` → `scripts/convert_racetrack_csv.py`로 변환 → `paths/silverstone_track.csv` (git에 커밋됨, 1178 waypoints, x/y/yaw)
- **MPPI**: `nav2_mppi_controller`는 ROS2 Humble Nav2 표준 내장 컴포넌트, 직접 구현 안 함. 설정은 `src/f1tenth_mppi_nav/config/nav2_params.yaml`
- **연결 코드 (우리가 만든 것, `src/f1tenth_mppi_nav`)**:
  - `path_follower.py` — csv 경로를 읽어 `controller_server`의 `follow_path` 액션에 직접 전송 (goal-pose 불필요). 실패/abort 시 자동 재시도하도록 고쳐둠
  - `cmd_vel_to_ackermann.py` — MPPI의 `/cmd_vel`(Twist) → 시뮬레이터의 `/drive`(AckermannDriveStamped) 변환 (자전거 모델)
  - `goal_pose_relay.py`, `mppi_bringup_launch.py` — 대화형(2D Goal Pose) 모드, 병행 제공
  - `record_path.py` — teleop 주행 기록용 (현재 미사용, 실차용으로 남겨둠)
  - `scripts/extract_centerline.py` — 공식 centerline 데이터 없는 임의 맵에서 이미지 스켈레톤화로 폐루프 경로 자동 추출 (완벽하진 않음, 시케인에서 지름길로 샐 수 있음)
- 로컬라이제이션 없음 — `gym_bridge`가 ground-truth `map→ego_racecar/base_link` tf를 그냥 뿌려줌. 실차로 갈 땐 AMCL/slam_toolbox localization 필수로 추가해야 함 (README 7절에 상세)

## 실행 명령 (터미널 2개)

```bash
# 터미널 1
roshumble   # alias: source /opt/ros/humble/setup.bash && source ~/roboracer_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2
roshumble
ros2 launch f1tenth_mppi_nav mppi_path_follow_launch.py
```

## 진행 중 / 남은 일 (TaskList에도 등록되어 있음)

1. **MPPI 파라미터 튜닝 (진행중)** — `vx_max: 6.0`, `wz_max: 1.8`(타이어 마찰 한계 반영해서 낮춤), `CostCritic.cost_weight: 6.0`, `inflation_radius: 0.55`까지 조정한 상태. 코너에서 벽 충돌 이슈가 있었고 `wz_max` 조정 + `kb_teleop:False`(중요 버그였음, 아래 참고)로 개선함. 계속 실차/랩타임 기준으로 다듬는 중.
2. **실제 Gazebo(gz-sim) 물리 시뮬레이션 이전 (미착수)** — 지금은 Gazebo 전혀 안 씀. `gz-ionic`과 `ros-humble-ros-gzharmonic-*`가 이 시스템에 동시 설치되어 있어 충돌 상태였던 것도 발견해뒀음(`gz sim --version`에서 protobuf 중복등록 에러) — 나중에 Gazebo 작업 시작하면 먼저 정리 필요.
3. 실차 이관은 README 7절에 계획만 정리, 실행은 안 함.

## 겪었던 주요 버그 (재발 시 참고)

- **`kb_teleop: True`일 때 `gym_bridge`가 `/cmd_vel`도 직접 구독해서 조향각을 ±0.3rad로 뭉갬** — MPPI가 `/cmd_vel`에 명령을 내면 이 로직과 충돌해서 조향이 이상해짐. 반드시 `sim.yaml`에서 `kb_teleop: False`.
- **`numba`/`coverage` 패키지 버전 충돌**로 `gym_bridge`가 죽는 문제 — `pip3 install --user --upgrade coverage`로 해결.
- **`f110_gym`은 `numpy<=1.22.0`에 강하게 의존** — 다른 pip 패키지(`scikit-image` 등) 설치 시 numpy가 올라가면 깨짐. `pip3 install --user 'numpy==1.22.0' 'scipy==1.8.0'`로 되돌려야 함.
- **ROS apt 저장소 GPG 키 만료** — `ros2-apt-source_1.2.0.jammy_all.deb` 설치로 해결 (README 2.1절에 정확한 명령 있음).
- **동일 노드 이름(`map_server`) 중복 실행 충돌** — `lifecycle_manager`가 "Failed to change state" 에러. 우리 launch 파일들은 애초에 `map_server`를 안 띄우고 `f1tenth_gym_ros`가 띄운 걸 재사용하도록 설계해서 회피함.
- **matplotlib 프리뷰 이미지가 위아래 뒤집혀 그려졌던 버그** — `imshow(..., origin='lower')`가 ROS 맵 좌표계(row 0 = 맨 위 = 최고 y)와 반대라 생김. `origin='upper'`가 맞음. (이 프리뷰 그리는 코드는 파일로 저장 안 하고 매번 즉석 스크립트로 실행함 — 재사용하려면 스크립트로 뽑아둘 것을 사용자가 요청할 수도 있음, 아직 안 만듦)

## 소통 관련 메모

- 사용자는 한국어로 소통 선호. 영어로 물어볼 때도 있었는데(영어 연습 목적) 이젠 한국어로 전환함.
- git push는 이 환경에 GitHub 인증이 없어서 항상 사용자가 직접 실행 (`git push`). 커밋까지는 Claude가 해도 됨.
- `git commit --amend`는 auto-mode 정책상 막혀있음(이미 push된 걸로 오인해서) — 이미 push된 커밋을 수정해야 할 땐 그냥 새 커밋으로 추가할 것.
