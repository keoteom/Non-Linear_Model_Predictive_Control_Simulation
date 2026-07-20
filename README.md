# Non-Linear_Model_Predictive_Control_Simulation
Nonlinear MPC simulation for UAV trajectory tracking and obstacle avoidance - A* global path, Safe Flight Corridor constraints, ROS2 + Gazebo

# Nonlinear MPC UAV Simulation

ROS2 + PX4 SITL + Gazebo 기반 NMPC 드론 시뮬레이션.
전역 경로 주변에 Safe Flight Corridor(SFC)를 hard constraint로 걸고,
동적 장애물은 time-varying cutting plane으로 회피하며 경로를 추종한다.

## 주요 기능
- SFC linear 제약 기반 NMPC (PANOC / OpEn 솔버)
- 동적 장애물 대응 TV cutting plane 생성
- 예측 경로·SFC·장애물 RViz 시각화
- PX4 Offboard 제어 연동

## 환경
- Ubuntu 22.04 / ROS2 Humble (버전 확인)
- PX4-Autopilot (SITL) + Gazebo
- Micro XRCE-DDS Agent
- Python 3.10, opengen (솔버 생성용)

## 의존성 패키지
같은 워크스페이스 `src/`에 함께 clone 필요:
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)

## 빌드
```bash
mkdir -p ~/ros2ws/src && cd ~/ros2ws/src
git clone https://github.com/keoteom/Non-Linear_Model_Predictive_Control_Simulation.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ~/ros2ws
colcon build
source install/setup.bash
```

NMPC 솔버는 최초 1회 생성 필요:
```bash
python3 build_solver.py   # 경로 (확인)
```

## 실행
```bash
# 1. PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500     # 사용하는 모델/월드로 (확인)

# 2. XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 3. NMPC
ros2 launch nmpc_uav_avoidance (런치파일명).launch.py
```

## 노드 및 토픽
### nmpc_node
- 구독: `/fmu/out/vehicle_odometry` (확인), `/sfc_coefficients`, (장애물 토픽)
- 발행: (Offboard setpoint 토픽), `/nmpc/predicted_path`, (마커 토픽)

## 파라미터
| 이름 | 기본값 | 설명 |
|---|---|---|
| r_obs | 0.5 | 장애물 반경 |
| r_ego | 0.25 | 기체 반경 |
| (기타) | | |
