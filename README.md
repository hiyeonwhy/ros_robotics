<div align="center">

# 🤖 TurtleBot Maze Escape & ArUco Docking

**SLAM 매핑부터 미로 자율주행, ArUco 마커 정밀 도킹까지 — TurtleBot 자율주행 풀 파이프라인**

<!-- 로고 이미지가 준비되면 아래 주석을 해제하세요
<img src="docs/logo.png" width="180" alt="Project Logo">
-->

![ROS](https://img.shields.io/badge/ROS-1%20(Noetic)-22314E?logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-11-00599C?logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-2.7%20%7C%203-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-5C3EE8?logo=opencv&logoColor=white)
![Build](https://img.shields.io/badge/build-catkin__make-brightgreen)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

<sub>자율시스템설계 001분반 팀프로젝트 · **우희연 · 김가현 · 김수언**</sub>

</div>

---

## 📖 프로젝트 개요

미로 형태의 실내 환경에 놓인 TurtleBot이 **스스로 지도를 만들고 → 미로를 빠져나온 뒤 →
목표 지점의 마커에 정확히 접안(docking)** 하는 것을 목표로 한 프로젝트입니다.

일반적인 ROS Navigation Stack만으로는 **미로처럼 좁은 통로**에서 경로 생성에 실패하고,
목표 지점에 도달하더라도 **수 cm 단위의 정밀 접안**은 불가능합니다.
이 프로젝트는 두 가지 문제를 각각 다음과 같이 해결했습니다.

| 문제 | 접근 |
|---|---|
| 좁은 통로에서 경로 생성 실패 | `footprint`를 로봇 실치수로 축소하고 costmap `inflation_radius`를 1.0 → 0.06으로 조정 |
| 목표 지점 접안 시 수 cm 오차 | ArUco 마커 비전 피드백 + 근거리 구간의 **오도메트리 기반 개루프 보정(STRAIGHT-ONLY MODE)** |

정확한 마커 자세 추정을 위해 **카메라 내부 파라미터**와 **LiDAR–카메라 외부 파라미터**
캘리브레이션을 선행 작업으로 직접 수행했습니다.

---

## ✨ 주요 기능

### 1. 🗺️ SLAM 기반 매핑 & 미로 자율주행

SLAM으로 미로 환경의 2D 점유 격자 지도를 생성하고, costmap / DWA 파라미터를
3차에 걸쳐 튜닝하여 좁은 통로를 안정적으로 통과합니다.

### 2. 🎯 ArUco 마커 탐지 및 자세 추정

`DICT_4X4_50` 사전의 마커(ID 1, 한 변 4 cm)를 실시간 탐지하고,
`estimatePoseSingleMarkers`로 카메라 좌표계 기준 3D 위치(x, y, z)를 추정합니다.

### 3. 🔌 상태 머신 기반 정밀 도킹

`INITIAL_FORWARD → AUTO_MODE → STRAIGHT-ONLY MODE`의 3단계 상태 머신으로
마커 탐색 · 각도 정렬 · 접근 · 최종 보정을 자동 수행합니다.
마커가 시야에서 사라지는 근거리(7 cm 이내)에서는 오도메트리 기반 개루프 제어로 전환합니다.

### 4. 📷 카메라 / LiDAR 캘리브레이션 툴셋

체커보드 이미지 수집 → 내부 파라미터 계산 → 왜곡 보정 검증 → `solvePnP` 기반
LiDAR–카메라 외부 파라미터 산출 → 점군 투영 검증까지의 전 과정을 도구로 구현했습니다.

---

## 🛠️ 기술 스택

| 구분 | 사용 기술 |
|---|---|
| **Language** | C++ (11), Python |
| **Framework** | ROS 1 (catkin), `roscpp`, `rospy` |
| **Vision** | OpenCV, `opencv2/aruco`, `cv_bridge`, `image_transport` |
| **Sensor / Msg** | `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `laser_geometry`, `pcl_ros`, `pcl_conversions` |
| **Hardware** | TurtleBot (`/cmd_vel`, `/odom`, `/scan`), 단안 카메라 640 × 480, 2D LiDAR |
| **Build** | `catkin_make`, CMake ≥ 3.0.2 |

---

## 🚀 시작하기 (Getting Started)

### 사전 요구 사항

- Ubuntu + **ROS 1** 설치 및 catkin 워크스페이스 구성
- **OpenCV** (`aruco` 모듈 포함 — `opencv_contrib`)
- ROS 패키지: `cv_bridge`, `image_transport`, `laser_geometry`, `pcl_ros`, `pcl_conversions`
- TurtleBot 브링업 및 카메라 드라이버가 동작하는 상태
  (`/camera/image`, `/odom`, `/scan` 토픽이 퍼블리시되어야 합니다)

### 설치

```bash
# 1. catkin 워크스페이스의 src 로 이동
cd ~/catkin_ws/src

# 2. 저장소 클론
git clone <repository-url> ros_robotics

# 3. 두 패키지를 src 바로 아래에 배치
#    ※ camera_calibration_node 는 폴더가 한 단계 중첩되어 있으므로 안쪽 폴더를 옮깁니다
mv ros_robotics/aruco_docking .
mv ros_robotics/camera_calibration_node/camera_calibration_node .

# 4. 빌드
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> ⚠️ `camera_calibration_node/CMakeLists.txt`의 `add_executable`은 현재
> `/home/hiyeon/ros_ws/src/...` 절대 경로를 참조합니다.
> 빌드 전에 `src/파일명.cpp` 상대 경로로 수정해야 합니다. ([알려진 이슈](#-알려진-이슈) 참고)

### 빌드 결과물

| 패키지 | 실행 파일 | 소스 |
|---|---|---|
| `aruco_docking` | `aruco_tracker_node` | `src/aruco_tracker.cpp` |
| `aruco_docking` | `aruco_docking_node` | `src/aruco_docking.cpp` |
| `camera_calibration_node` | `calibrate_camera` | `src/calibrate_camera.cpp` |
| `camera_calibration_node` | `cv_image` | `src/cvimage.cpp` |
| `camera_calibration_node` | `scan_to_cloud_node` | `src/scan_to_cloud.cpp` |
| `camera_calibration_node` | `solve_pnp_node` | `src/rotatran.cpp` |
| `camera_calibration_node` | `project_lidar_to_image` | `src/image_projection.cpp` |

---

## 💡 사용법 (Usage)

### STEP 1. 카메라 캘리브레이션

```bash
# 체커보드 이미지 수집 (SPACE = 저장, ESC = 종료)
rosrun camera_calibration_node save_images.py

# 내부 파라미터 K, 왜곡 계수 D 계산 (내부 코너 8 × 6)
rosrun camera_calibration_node calibrate_camera

# 왜곡 보정 결과 육안 확인
rosrun camera_calibration_node cv_image
```

**산출된 파라미터** (본 프로젝트 카메라 기준)

```
K = [ 498.0449    0        317.8820
        0       498.7900   236.7252
        0         0          1      ]

D = [ -0.17349, -0.37953, 0.00084, 0.00023, 0.17296 ]
```

이 값은 각 노드 소스 상단에 하드코딩되어 있습니다. **카메라를 교체하면 반드시 갱신해야 합니다.**

```bash
# CameraInfo 토픽으로 퍼블리시하고 싶을 때 (1 Hz, plumb_bob)
rosrun camera_calibration_node camera_info_pub.py
```

### STEP 2. LiDAR–카메라 외부 파라미터 산출

```bash
# /scan → /cloud 변환 후 RViz 에서 LiDAR 점 좌표 확인
rosrun camera_calibration_node scan_to_cloud_node

# 이미지에서 대응 픽셀 좌표를 마우스 클릭으로 추출
rosrun camera_calibration_node point_image.py

# solvePnP → 회전행렬 R, 이동벡터 t 산출
rosrun camera_calibration_node solve_pnp_node

# X_cam = R·X_lidar + t 로 투영해 정합 검증
rosrun camera_calibration_node project_lidar_to_image
```

**산출된 외부 파라미터**

```
R = [  0.3083  -0.9499  -0.0509        t = [  0.0158
       0.0925   0.0832  -0.9922              -0.0748
       0.9468   0.3012   0.1135 ]            -0.0424 ]
```

### STEP 3. 마커 탐지 확인

```bash
rosrun aruco_docking aruco_tracker_node
```

`Aruco Tracker` 창에 마커 중심점(초록 원)과 좌표축이 그려지고, 터미널에 아래와 같이 출력됩니다.

```
[ INFO] 마커 ID 1 중심점 픽셀 위치: (322.41, 241.08)
[ INFO] 마커 ID 1 카메라 좌표계 위치: x=0.0102, y=-0.0035, z=0.3120
```

`z` 값이 마커까지의 거리(m)입니다. 이 값이 안정적으로 나오는지 먼저 확인하세요.

### STEP 4. 도킹 실행

```bash
rosrun aruco_docking aruco_docking_node
```

> ⚠️ 노드가 즉시 `/cmd_vel`로 주행 명령을 발행합니다.
> 로봇 주변을 정리하고 비상 정지가 가능한 상태에서 실행하세요.
> 도킹이 완료되면 노드가 스스로 종료(`ros::shutdown()`)됩니다.

#### 도킹 상태 머신

```
[ INITIAL_FORWARD ]
   odom 기준 0.6 m 직진 (linear.x = 0.08)
        │  거리 도달
        ▼
[ AUTO_MODE ] ───── 마커 미검출 ──▶ 시계방향 회전 탐색 (angular.z = -0.2)
        │                                    │
        │   ID 1 마커 검출                    └── 검출되면 복귀
        ▼
   z ≥ 0.07 m ?
     ├─ YES ─▶ θ = atan2(x, z)
     │           |θ| >  0.005 → 정밀 회전 (±0.08 rad/s)
     │           |θ| ≤ 0.005 → 전진 / 후진 (±0.045 m/s), 목표 z = 0.03 m
     │                          도달 시 finalOrientationCorrection() 후 종료
     │
     └─ NO  ─▶ [ STRAIGHT-ONLY MODE ]
                 진입 시점의 odom pose 저장
                 이동 거리 = z − 0.03 + 0.18 (m)
                 카메라를 무시하고 odom 거리만으로 직진 → 완료 후 종료
```

**튜닝 가능한 파라미터** — `aruco_docking/src/aruco_docking.cpp` 상단

```cpp
const double theta_tol   = 0.005;  // [rad] 각도 허용 오차
const double z_target    = 0.03;   // [m]   목표 도킹 거리
const double z_tol       = 0.008;  // [m]   거리 허용 오차
const double z_switch    = 0.07;   // [m]   STRAIGHT-ONLY MODE 전환 임계값
const double extra_straight = 0.18;         // [m]   근거리 구간 추가 직진 거리
const double initial_forward_distance = 0.6;// [m]   시작 직진 거리
const double rotate_speed       = 0.2;      // [rad/s] 마커 탐색 회전 속도
const double align_rotate_speed = 0.08;     // [rad/s] 정밀 정렬 회전 속도
const double approach_speed     = 0.045;    // [m/s]   정밀 전진 속도
```

> 💡 **STRAIGHT-ONLY MODE가 필요한 이유**
> 마커에 7 cm 이내로 접근하면 마커가 화면 밖으로 벗어나거나 자세 추정 오차가 급격히 커집니다.
> 이 시점에 비전 피드백을 끊고 **오도메트리 거리 기준 개루프 직진**으로 전환해
> 마지막 구간을 정밀하게 마무리합니다.

### 참고 : 미로 주행 파라미터

navigation stack 파라미터를 3차에 걸쳐 튜닝한 결과입니다.

| 항목 | 1차 | 2차 | 최종 | 의미 |
|---|---|---|---|---|
| `footprint` | `[[-0.105,-0.105],…]` | `[[-0.095,-0.09],…]` | 2차와 동일 | 로봇 외곽 치수 |
| `inflation_radius` | 1.0 | 1.0 | **local 0.06 / global 0.1** | 장애물 주변 안전 거리 |
| `cost_scaling_factor` | 3.0 | 3.0 | **local 4.8 / global 3.0** | 장애물 근처 비용 증가율 |
| `goal_distance_bias` | 20.0 | 20.0 | **18.0** | 직진하려는 정도 |
| `occdist_scale` | 0.02 | 0.02 | **0.2** | 장애물을 피하려는 정도 |
| `forward_point_distance` | 0.325 | 0.325 | **0.5** | 직진 및 회피 정도 |

> ⚠️ 위 값은 발표 자료에 기록된 내용이며, 실제 navigation `*.yaml` 파일은
> 이 저장소에 포함되어 있지 않습니다.

---

## 📂 폴더 구조 (Directory Structure)

```
ros_robotics/
├── aruco_docking/                        # 📦 ArUco 마커 탐지 및 도킹 패키지
│   ├── src/
│   │   ├── aruco_tracker.cpp             # 마커 탐지 + 자세 추정 시각화
│   │   └── aruco_docking.cpp             # 도킹 상태 머신 (핵심 노드)
│   ├── launch/
│   │   └── aruco.launch
│   ├── CMakeLists.txt
│   └── package.xml
│
├── camera_calibration_node/
│   └── camera_calibration_node/          # 📦 카메라 / LiDAR 캘리브레이션 패키지
│       ├── src/
│       │   ├── calibrate_camera.cpp      # 체커보드 기반 내부 파라미터 계산
│       │   ├── cvimage.cpp               # 왜곡 보정(undistort) 결과 확인
│       │   ├── rotatran.cpp              # solvePnP → 외부 파라미터 R, t
│       │   ├── image_projection.cpp      # LiDAR 점군 → 이미지 평면 투영
│       │   └── scan_to_cloud.cpp         # /scan → /cloud (PointCloud2)
│       ├── scripts/
│       │   ├── save_images.py            # 캘리브레이션 이미지 수집
│       │   ├── camera_info_pub.py        # CameraInfo 토픽 퍼블리시
│       │   └── point_image.py            # 클릭으로 2D 대응점 추출
│       ├── cali/                         # 체커보드 촬영 이미지
│       ├── CMakeLists.txt
│       └── package.xml
│
└── 우희연 김가현 김수언 텀프로젝트 PPT.pdf    # 📑 발표 자료
```

### 노드별 토픽 인터페이스

| 노드 | 구독 (Subscribe) | 발행 (Publish) |
|---|---|---|
| `aruco_tracker_node` | `/camera/image_raw` | — (OpenCV 창 출력) |
| `aruco_docking_node` | `/camera/image`, `/odom` | `/cmd_vel` |
| `scan_to_cloud_node` | `/scan` | `/cloud` |
| `camera_info_pub.py` | — | `/camera/image_info` |

---

## ⚠️ 알려진 이슈

현 저장소를 그대로 다른 환경에서 빌드 · 실행할 때 걸리는 부분들입니다.

- [ ] **CMake 절대 경로** — `camera_calibration_node/CMakeLists.txt`의 `add_executable`이
      `/home/hiyeon/ros_ws/src/...`를 참조합니다. `src/파일명.cpp` 상대 경로로 수정 필요.
- [ ] **소스 내 하드코딩 경로** — `calibrate_camera.cpp`, `cvimage.cpp`, `image_projection.cpp`,
      `point_image.py`, `save_images.py`가 `/home/hiyeon/...`를 사용합니다.
- [ ] **launch 파일 노드 타입 불일치** — `aruco.launch`의 `type="aruco_detector_node"`는
      실제 빌드되는 `aruco_tracker_node` / `aruco_docking_node`와 이름이 다릅니다.
- [ ] **패키지 디렉토리 중첩** — `camera_calibration_node/camera_calibration_node/`
      구조라 catkin 워크스페이스에 넣을 때 안쪽 폴더를 옮겨야 합니다.
- [ ] **캘리브레이션 이미지 부족** — `calibrate_camera.cpp`는 `calib0000~0025.jpg`(26장)를
      읽지만 `cali/`에는 3장만 있습니다. 위에 기재된 K·D를 재현하려면 이미지를 다시 수집해야 합니다.
- [ ] **이미지 토픽 불일치** — `aruco_tracker.cpp`는 `/camera/image_raw`,
      `aruco_docking.cpp`와 `save_images.py`는 `/camera/image`를 구독합니다.
- [ ] **왜곡 계수 D[0] 부호 불일치** — ArUco 노드들은 `-0.17349`,
      `cvimage.cpp` / `camera_info_pub.py`는 `+0.17434`를 사용합니다.
- [ ] **package.xml 의존성 누락** — `camera_calibration_node`에 `pcl_ros`, `pcl_conversions`가,
      `aruco_docking`에 `nav_msgs`가 선언되어 있지 않습니다.
- [ ] **체커보드 실치수 미반영** — `calibrate_camera.cpp`가 3D 점을 `Point3f(j, i, 0)`으로
      생성해 단위가 "칸"입니다. K에는 영향이 없으나 거리 단위 해석 시 실제 칸 크기(m)를 곱해야 합니다.
- [ ] **콜백 블로킹** — `aruco_docking.cpp`의 이미지 콜백 안에서
      `ros::topic::waitForMessage`(0.5 s)와 `finalOrientationCorrection()` 루프가 콜백을 블로킹합니다.

---

## 🤝 기여 방법 (Contributing)

기여는 언제나 환영합니다! 아래 절차를 따라주세요.

1. 이 저장소를 **Fork** 합니다.
2. 기능 브랜치를 생성합니다. — `git checkout -b feature/amazing-feature`
3. 변경 사항을 커밋합니다. — `git commit -m 'feat: Add amazing feature'`
4. 브랜치에 푸시합니다. — `git push origin feature/amazing-feature`
5. **Pull Request**를 생성합니다.

### 커밋 컨벤션

| 접두사 | 용도 |
|---|---|
| `feat:` | 새로운 기능 추가 |
| `fix:` | 버그 수정 |
| `docs:` | 문서 수정 |
| `refactor:` | 코드 리팩터링 |
| `chore:` | 빌드 설정 등 기타 변경 |

### 기여 우선순위

위 [알려진 이슈](#-알려진-이슈)의 체크리스트 항목부터 해결해 주시면 큰 도움이 됩니다.
특히 **하드코딩된 절대 경로 제거**와 **파라미터의 ROS param 서버 이전**이 가장 시급합니다.

---

## 📄 라이선스 (License)

현재 라이선스가 지정되어 있지 않습니다.
두 패키지의 `package.xml` 모두 `<license>TODO</license>` 상태이므로,
배포 전에 프로젝트에 맞는 라이선스(MIT, BSD-3-Clause, Apache-2.0 등)를 선택하여
`package.xml`과 `LICENSE` 파일에 명시해 주세요.

> ROS 생태계에서는 **BSD-3-Clause** 또는 **Apache-2.0**이 널리 사용됩니다.

---

## 👥 팀

| 이름 | 역할 |
|---|---|
| 우희연 | — |
| 김가현 | — |
| 김수언 | — |

<div align="center">
<sub>자율시스템설계 001분반 · 팀프로젝트</sub>
</div>
