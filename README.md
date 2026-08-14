# TurtleBot 자율주행 : 미로 탈출 및 도킹

자율시스템설계 001분반 팀프로젝트 — **우희연 · 김가현 · 김수언**

TurtleBot 기반으로 **SLAM 매핑 → 자율주행 미로 탈출 → ArUco 마커 정밀 도킹**까지의
전 과정을 구현한 ROS 1 (catkin) 프로젝트입니다.

---

## 1. 프로젝트 목표

| 단계 | 내용 |
|:---:|---|
| 1 | **자율주행을 위한 Mapping** — SLAM으로 미로 환경의 2D 맵 생성 |
| 2 | **자율 주행을 통한 미로 탈출** — 생성한 맵 + costmap / DWA 파라미터 튜닝으로 좁은 통로 주파 |
| 3 | **ArUco 마커 중심 도킹** — 카메라로 마커를 탐지해 목표 지점에 정렬 후 정밀 접안 |

이를 위해 카메라 내부 파라미터 캘리브레이션과 LiDAR–카메라 외부 파라미터(Extrinsic) 캘리브레이션을
선행 작업으로 수행했습니다.

---

## 2. 저장소 구조

```
ros_robotics/
├── aruco_docking/                       # [ROS 패키지] ArUco 마커 탐지 및 도킹
│   ├── src/
│   │   ├── aruco_tracker.cpp            # 마커 탐지 + 자세 추정 시각화
│   │   └── aruco_docking.cpp            # 도킹 상태 머신 (핵심 노드)
│   ├── launch/
│   │   └── aruco.launch
│   ├── CMakeLists.txt
│   └── package.xml
│
├── camera_calibration_node/
│   └── camera_calibration_node/         # [ROS 패키지] 카메라 / LiDAR 캘리브레이션
│       ├── src/
│       │   ├── calibrate_camera.cpp     # 체커보드 기반 내부 파라미터 계산
│       │   ├── cvimage.cpp              # 왜곡 보정(undistort) 결과 확인
│       │   ├── rotatran.cpp             # solvePnP → LiDAR-카메라 외부 파라미터 R, t
│       │   ├── image_projection.cpp     # LiDAR 점군을 이미지 평면에 투영
│       │   └── scan_to_cloud.cpp        # /scan → /cloud (PointCloud2) 변환
│       ├── scripts/
│       │   ├── save_images.py           # 캘리브레이션용 이미지 수집 (SPACE 저장)
│       │   ├── camera_info_pub.py       # CameraInfo 토픽 퍼블리시
│       │   └── point_image.py           # 이미지 클릭으로 2D 대응점 추출
│       ├── cali/                        # 체커보드 촬영 이미지 (calib0000~0002.jpg)
│       ├── CMakeLists.txt
│       └── package.xml
│
└── 우희연 김가현 김수언 텀프로젝트 PPT.pdf   # 발표 자료
```

---

## 3. 개발 환경

- **ROS 1** (catkin, `roscpp`)
- **TurtleBot** — `/cmd_vel`, `/odom`, `/scan`
- **OpenCV** + `opencv2/aruco` (`cv_bridge`, `image_transport`)
- `laser_geometry`, `pcl_ros`, `pcl_conversions`
- 카메라 해상도 **640 × 480**

---

## 4. 사전 작업 : 캘리브레이션

### 4-1. 카메라 내부 파라미터 (Intrinsic)

| 파일 | 역할 |
|---|---|
| `scripts/save_images.py` | `/camera/image` 구독, SPACE 키로 `cali/calib%04d.jpg` 저장 (ESC 종료) |
| `src/calibrate_camera.cpp` | `findChessboardCorners`(내부 코너 **8 × 6**) → `cornerSubPix` → `calibrateCamera` |
| `src/cvimage.cpp` | `getOptimalNewCameraMatrix` + `undistort` 후 ROI 크롭으로 보정 결과 육안 검증 |
| `scripts/camera_info_pub.py` | 산출한 K·D를 `/camera/image_info` (CameraInfo, `plumb_bob`)로 1 Hz 퍼블리시 |

**결과값**

```
K = [ 498.0449   0        317.8820
      0        498.7900   236.7252
      0          0          1      ]

D = [ -0.17349, -0.37953, 0.00084, 0.00023, 0.17296 ]
```

이 값은 `aruco_tracker.cpp`, `aruco_docking.cpp`, `camera_info_pub.py`, `cvimage.cpp`,
`image_projection.cpp`, `rotatran.cpp`에 하드코딩되어 있습니다.

### 4-2. LiDAR–카메라 외부 파라미터 (Extrinsic)

1. **`src/scan_to_cloud.cpp`** — `/scan`(LaserScan)을 `laser_geometry::LaserProjection`으로
   `/cloud`(PointCloud2)로 변환해 RViz에서 LiDAR 점 좌표를 확인
2. **`scripts/point_image.py`** — 같은 장면의 이미지에서 대응 픽셀 좌표를 마우스 클릭으로 추출
3. **`src/rotatran.cpp`** — 3D LiDAR 점 4개 ↔ 2D 픽셀 4개로 `solvePnP` 수행,
   `Rodrigues`로 회전행렬 R과 이동벡터 t 산출

   ```
   LiDAR 3D (m)                     Image 2D (px)
   (0.248768,  0.275237, 0)   ↔     (13, 186)
   (0.391173,  0.185669, 0)   ↔     (266, 204)
   (1.237631,  0.160540, 0)   ↔     (419, 258)
   (1.231193, -0.066738, 0)   ↔     (527, 254)
   ```

   ```
   R = [  0.3083  -0.9499  -0.0509        t = [  0.0158
          0.0925   0.0832  -0.9922              -0.0748
          0.9468   0.3012   0.1135 ]            -0.0424 ]
   ```

4. **`src/image_projection.cpp`** — `X_cam = R · X_lidar + t` 변환 후
   `u = fx·X/Z + cx`, `v = fy·Y/Z + cy` 로 투영해 이미지에 점을 찍어 정합 검증
   (Z ≤ 0 인 카메라 뒤쪽 점은 제외)

---

## 5. 자율주행 : 미로 탈출

SLAM으로 맵을 만든 뒤 navigation stack의 **footprint / costmap / DWA 파라미터**를
반복 튜닝하며 좁은 통로 통과 성능을 개선했습니다.

### 파라미터 변경 이력

| 항목 | 1차 | 2차 | 최종 |
|---|---|---|---|
| `footprint` | `[[-0.105,-0.105],[-0.105,0.105],[0.041,0.105],[0.041,-0.105]]` | `[[-0.095,-0.09],[-0.095,0.09],[0.045,0.09],[0.045,-0.09]]` | 2차와 동일 |
| `inflation_radius` | 1.0 (common) | 1.0 (common) | **local 0.06 / global 0.1** |
| `cost_scaling_factor` | 3.0 | 3.0 | **local 4.8 / global 3.0** |
| `goal_distance_bias` | 20.0 | 20.0 | **18.0** |
| `occdist_scale` | 0.02 | 0.02 | **0.2** |
| `forward_point_distance` | 0.325 | 0.325 | **0.5** |

- `inflation_radius` — 장애물 주변 안전 거리 범위
- `cost_scaling_factor` — 장애물 근처 비용 증가율
- `goal_distance_bias` — 목표를 향해 직진하려는 정도
- `occdist_scale` — 장애물을 피하려는 정도
- `forward_point_distance` — 직진 정도 및 회피 정도

`footprint`를 로봇 실치수에 맞게 줄이고 `local_costmap`의 `inflation_radius`를
1.0 → 0.06으로 크게 낮춘 것이 좁은 통로 진입 실패를 해결한 핵심 변경입니다.

> ⚠️ 위 값들은 발표 자료(PDF)에 기록된 내용이며, 실제 navigation `*.yaml` 파일은
> 이 저장소에 포함되어 있지 않습니다.

---

## 6. 도킹 : ArUco 마커 기반

두 노드 모두 `DICT_4X4_50` 사전의 **ID 1** 마커만 사용하며, 마커 한 변 길이는 **0.04 m**입니다.

### 6-1. `aruco_tracker_node` — 탐지 확인용

`/camera/image_raw` 구독 → 그레이스케일 변환 → `detectMarkers` →
`estimatePoseSingleMarkers`로 자세 추정 → 마커 중심점 픽셀 좌표와
카메라 좌표계 위치(x, y, z)를 로그로 출력하고, `drawAxis`로 좌표축을 그려 시각화합니다.

### 6-2. `aruco_docking_node` — 도킹 상태 머신

**구독** `/camera/image` (Image), `/odom` (Odometry) · **발행** `/cmd_vel` (Twist)

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
                 카메라를 무시하고 odom 거리만으로 직진 → 완료 후 ros::shutdown()
```

**주요 파라미터** (`aruco_docking.cpp` 상단)

| 상수 | 값 | 의미 |
|---|---|---|
| `initial_forward_distance` | 0.6 m | 시작 시 무조건 직진하는 거리 |
| `z_switch` | 0.07 m | 이보다 가까우면 STRAIGHT-ONLY MODE 전환 |
| `z_target` | 0.03 m | 목표 도킹 거리 |
| `z_tol` | 0.008 m | 거리 허용 오차 |
| `theta_tol` | 0.005 rad | 각도 허용 오차 |
| `extra_straight` | 0.18 m | STRAIGHT-ONLY MODE에서 추가로 더 가는 거리 |
| `rotate_speed` | 0.2 rad/s | 마커 탐색용 빠른 회전 |
| `align_rotate_speed` | 0.08 rad/s | 도킹 시 정밀 회전 |
| `approach_speed` | 0.045 m/s | 정밀 전진 속도 |
| `markerLength` | 0.04 m | ArUco 마커 한 변 길이 |

**STRAIGHT-ONLY MODE가 필요한 이유** — 마커에 7 cm 이내로 접근하면 마커가 화면 밖으로
벗어나거나 자세 추정 오차가 급격히 커집니다. 이 시점에 비전 피드백을 끊고
**오도메트리 거리 기준 개루프 직진**으로 전환해 마지막 구간을 정밀하게 마무리합니다.

---

## 7. 빌드 및 실행

### 빌드

```bash
cd ~/ros_ws/src
# aruco_docking, camera_calibration_node 를 이 디렉토리에 배치
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

빌드되는 실행 파일

| 패키지 | 타겟 | 소스 |
|---|---|---|
| `aruco_docking` | `aruco_tracker_node` | `src/aruco_tracker.cpp` |
| `aruco_docking` | `aruco_docking_node` | `src/aruco_docking.cpp` |
| `camera_calibration_node` | `calibrate_camera` | `src/calibrate_camera.cpp` |
| `camera_calibration_node` | `cv_image` | `src/cvimage.cpp` |
| `camera_calibration_node` | `scan_to_cloud_node` | `src/scan_to_cloud.cpp` |
| `camera_calibration_node` | `solve_pnp_node` | `src/rotatran.cpp` |
| `camera_calibration_node` | `project_lidar_to_image` | `src/image_projection.cpp` |

### 실행

```bash
# 1) TurtleBot 브링업 + 카메라 노드 실행 (환경에 맞게)

# 2) 마커 탐지 확인
rosrun aruco_docking aruco_tracker_node

# 3) 도킹 실행
rosrun aruco_docking aruco_docking_node

# 캘리브레이션 관련
rosrun camera_calibration_node calibrate_camera          # 내부 파라미터 계산
rosrun camera_calibration_node cv_image                  # 왜곡 보정 확인
rosrun camera_calibration_node solve_pnp_node            # 외부 파라미터 R, t
rosrun camera_calibration_node project_lidar_to_image    # LiDAR → 이미지 투영
rosrun camera_calibration_node scan_to_cloud_node        # /scan → /cloud
rosrun camera_calibration_node camera_info_pub.py        # CameraInfo 퍼블리시
rosrun camera_calibration_node save_images.py            # 캘리브레이션 이미지 수집
```

---

## 8. 문제점 및 개선 과정

| 문제점 | 개선 방법 |
|---|---|
| 초기 Mapping 오류로 인한 실제 미로와 지도 간의 오차 | 지도 생성 시 로봇 이동 속도를 조절하여 SLAM 안정성 개선 |
| 좁은 통로 진입 시 경로 생성 실패 | costmap 파라미터 수정을 통해 장애물 인식 크기 감소 |
| 도킹 거리 오차 발생 | **STRAIGHT-ONLY MODE**를 추가하여 마지막 거리 구간을 정밀하게 보정 |
| 초기 직진 이후 마커가 보이지 않아 도킹 중단 | 마커가 보일 때까지 시계 방향으로 회전하며 탐색 |

---

## 9. 알려진 이슈

현 저장소를 그대로 다른 환경에서 빌드 · 실행할 때 걸리는 부분들입니다.

- **`aruco.launch`의 노드 타입 불일치** — `type="aruco_detector_node"`로 되어 있으나
  실제 빌드되는 실행 파일은 `aruco_tracker_node` / `aruco_docking_node`입니다.
- **`camera_calibration_node/CMakeLists.txt`의 절대 경로** — `add_executable`이
  `/home/hiyeon/ros_ws/src/...` 절대 경로를 참조합니다. `src/xxx.cpp` 상대 경로로 바꿔야
  다른 워크스페이스에서 빌드됩니다.
- **소스 내 하드코딩된 절대 경로** — `calibrate_camera.cpp`, `cvimage.cpp`,
  `image_projection.cpp`, `point_image.py`, `save_images.py`가 `/home/hiyeon/...`를 사용합니다.
- **패키지 디렉토리 중첩** — `camera_calibration_node/camera_calibration_node/`로
  한 단계 더 들어가 있어, catkin 워크스페이스에 넣을 때 안쪽 폴더를 옮겨야 합니다.
- **캘리브레이션 이미지 부족** — `calibrate_camera.cpp`는 `calib0000~0025.jpg`(26장)를
  읽지만 `cali/`에는 3장(`0000`~`0002`)만 있습니다. 나머지는 "이미지를 불러올 수 없습니다"로
  건너뛰므로, README에 기재된 K·D를 재현하려면 이미지를 다시 수집해야 합니다.
- **이미지 토픽 불일치** — `aruco_tracker.cpp`는 `/camera/image_raw`를,
  `aruco_docking.cpp`와 `save_images.py`는 `/camera/image`를 구독합니다.
- **왜곡 계수 D[0]의 부호 불일치** — ArUco 노드들은 `-0.17349`,
  `cvimage.cpp` / `camera_info_pub.py`는 `+0.17434`를 사용합니다.
- **CMake 의존성 누락** — `camera_calibration_node`의 `CMakeLists.txt`는
  `pcl_ros`, `pcl_conversions`를 `find_package`하지만 `package.xml`에는 선언되어 있지 않습니다.
  `aruco_docking`의 `package.xml`에도 `nav_msgs` 의존성이 빠져 있습니다.
- **`calibrate_camera.cpp`의 체커보드 실치수 미반영** — 3D 점을 `Point3f(j, i, 0)`으로
  생성해 단위가 "칸"입니다. K는 영향을 받지 않지만, 거리 단위 해석이 필요하면
  실제 칸 크기(m)를 곱해야 합니다.
- **`aruco_docking.cpp`의 블로킹 호출** — 이미지 콜백 안에서
  `ros::topic::waitForMessage`(0.5 s)와 `finalOrientationCorrection()`의 회전 루프가
  콜백을 블로킹합니다. 동작에는 문제가 없었으나 구조적으로 개선 여지가 있습니다.
