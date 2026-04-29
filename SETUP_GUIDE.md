# 로보틱 암 급유 시스템 실행 매뉴얼

## 환경 정보

| 항목 | 값 |
|---|---|
| 워크스페이스 경로 | `~/robotarm-refueler/` |
| 소스 경로 | `~/robotarm-refueler/src/robotic-arm-refueler/` |
| 로봇 IP | `110.120.1.52` |
| 로봇 모델 | `e0509` |
| Flask 웹 서버 포트 | `5000` |
| UI Gateway (ROS2↔Flask) 포트 | `6000` |

### .bashrc 권장 alias

```bash
alias cb='cd ~/robotarm-refueler && colcon build'
alias sb='source ~/robotarm-refueler/install/setup.bash'
```

---

## 1단계: 빌드

> 처음 설치하거나 패키지를 수정했을 때 실행. 변경 없으면 생략 가능.

```bash
cd ~/robotarm-refueler
```

### 1-1. (최초 1회) setuptools 버전 고정

colcon 빌드 시 setuptools 버전 오류가 나면 아래 명령어 실행:

```bash
pip install setuptools==58.2.0
```

### 1-2. 인터페이스 패키지 먼저 빌드 (Action 타입 생성)

```bash
colcon build --packages-select fuel_robot_interfaces
```

### 1-3. 메인 패키지 빌드

```bash
colcon build --packages-select fuel_robot_pkg
```

### 1-4. 워크스페이스 소싱

```bash
source install/setup.bash
# 또는 alias 사용
sb
```

> **주의:** 새 터미널을 열 때마다 `sb` (또는 `source ~/robotarm-refueler/install/setup.bash`)를 실행해야 합니다.  
> `.bashrc`에 `source ~/robotarm-refueler/install/setup.bash`를 추가하면 자동으로 적용됩니다.

---

## 2단계: 시스템 실행 (터미널 3개)

### [터미널 1] RealSense 카메라 드라이버

카메라 USB 연결 확인 후 실행:

```bash
sb
ros2 launch realsense2_camera rs_launch.py
```

카메라 토픽이 정상 발행되는지 확인:

```bash
ros2 topic list | grep camera
# 아래 토픽들이 보여야 함:
# /camera/camera/color/image_raw
# /camera/camera/color/camera_info
```

---

### [터미널 2] ROS2 메인 런치 (로봇 + 모든 노드)

로봇 전원 및 네트워크 연결(`110.120.1.52`) 확인 후 실행:

```bash
sb
ros2 launch fuel_robot_pkg fuel_robot.launch.py
```

이 명령 하나로 아래가 모두 실행됩니다:
- Doosan 로봇 bringup (`dsr_bringup2`)
- Static TF 발행 (`robot_base → camera_link`)
- `ui_gateway_node` (Flask ↔ ROS2 브릿지, 포트 6000)
- `fueling_task_manager_node` (작업 상태 관리)
- `doosan_commander_node` (로봇 동작 Action Server)
- `safety_monitor_node` (안전 감시)
- `realsense_marker_tracker_node` (ArUco 마커 검출 → TCP 좌표 변환)
- `tcp_error_monitor_node` (TCP 오차 감시)
- `digital_twin_monitor_node` (디지털 트윈 로깅)

---

### [터미널 3] Flask 웹 서버

```bash
cd ~/robotarm-refueler/src/robotic-arm-refueler/web
python3 app.py
```

서버 실행 후 브라우저에서 접속:

```
http://localhost:5000
```

기본 관리자 계정:
- **아이디**: `admin`
- **비밀번호**: `admin`

키오스크 페이지 (고객용):
```
http://localhost:5000/kiosk/1/1
```

---

## 3단계: 정상 동작 확인

```bash
# 노드 목록 확인
ros2 node list

# 주요 토픽 확인
ros2 topic echo /fueling/vision_tcp_pose    # 카메라 → TCP 좌표 변환 결과
ros2 topic echo /fueling/status             # 작업 상태
```

---

## [선택] 핸드-아이 캘리브레이션

런치 파일의 TF 보정값(`x/y/z/roll/pitch/yaw`)을 새로 측정할 때 별도 터미널에서 실행:

```bash
sb
ros2 run fuel_robot_pkg hand_eye_calibration_node
```

캘리브레이션 결과는 `~/hand_eye_calib_result.yaml`에 저장됩니다.  
결과값을 `fuel_robot.launch.py`의 `static_tf_robot_to_camera` 인수에 반영해야 합니다.

```python
# fuel_robot.launch.py 수정 위치
arguments=[
    '--x',     '측정값',
    '--y',     '측정값',
    '--z',     '측정값',
    '--roll',  '측정값',
    '--pitch', '측정값',
    '--yaw',   '측정값',
    '--frame-id',       'robot_base',
    '--child-frame-id', 'camera_link',
],
```
