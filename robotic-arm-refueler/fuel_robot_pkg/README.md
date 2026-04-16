# 로봇 급유 시스템 (Robotic Arm Refueler)

자동화된 로봇 팔을 사용한 차량 급유 작업 시스템입니다. 
Doosan Robot을 ROS 2에서 제어하여 연료 포트를 인식하고 자동 급유를 수행합니다.

## 📋 목차
- [기능](#기능)
- [시스템 아키텍처](#시스템-아키텍처)
- [설치 및 설정](#설치-및-설정)
- [실행 방법](#실행-방법)
- [프로젝트 구조](#프로젝트-구조)
- [문서](#문서)

## ✨ 기능

- **로봇 제어**: Doosan Robot 자동 제어 (관절 이동, 직선 이동, 액추에이터 제어)
- **시각 인식**: YOLOv12를 활용한 연료 포트 위치 감지
- **자동 급유**: 차량 연료 포트 감지 → 충전 노즐 픽업 → 급유 → 노즐 반환
- **UI 게이트웨이**: 시스템 상태 모니터링 및 수동 제어 인터페이스
- **작업 관리**: 전체 급유 작업 흐름 조율

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│                    UI Gateway                            │
│           (상태 모니터링 & 수동 제어)                     │
└────────────────┬────────────────────────────────────────┘
                 │
        ┌────────┼────────┐
        │                  │
┌───────▼──────┐    ┌─────▼──────────┐
│ Task Manager │    │ Perception     │
│  (급유 작업   │    │ Node           │
│   조율)       │    │ (YOLOv12)      │
└───────┬──────┘    └─────────────────┘
        │
        └─────────────┬──────────────────┐
                      │                   │
            ┌─────────▼─────────┐    ┌────▼──────┐
            │                   │    │   Camera  │
            │  Doosan Commander │    │   Stream  │
            │  (로봇 제어)       │    │           │
            └─────────┬─────────┘    └───────────┘
                      │
            ┌─────────▼──────────┐
            │  Doosan Robot      │
            │  (로봇 팔)          │
            └────────────────────┘
```

## 🚀 설치 및 설정

### 필수 사항
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- Doosan Robot ROS 2 Packages

### 빠른 시작

1. **저장소 클론**
   ```bash
   git clone https://github.com/JaeyeonE/robotic-arm-refueler.git
   cd robotic-arm-refueler
   ```

2. **팀원 설정 (처음 1회만)**
   ```bash
   cp .env.example .env
   # .env 파일을 편집하여 로봇 IP와 SSH 정보 입력
   cat SETUP.md  # 상세 설정 가이드 읽기
   ```

3. **의존성 설치**
   ```bash
   pip install -r requirements.txt
   ```

4. **ROS 2 workspace 빌드**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select fuel_robot_pkg
   source install/setup.bash
   ```

자세한 설정은 [SETUP.md](SETUP.md)를 참고하세요.

## ▶️ 실행 방법

### 1. 환경 설정 로드
```bash
cd ~/ros2_ws/src/fuel_robot_pkg
set -a
source .env
set +a
```

### 2. 런치 파일 실행
```bash
ros2 launch fuel_robot_pkg fuel_robot.launch.py
```

### 3. 급유 작업 시작 (다른 터미널)
```bash
# UI 게이트웨이 서비스 호출
ros2 service call /fueling/start_button std_srvs/srv/Trigger
```

### 4. 상태 모니터링
```bash
# 현재 상태 구독
ros2 topic echo /fueling/status

# 작업 완료 신호
ros2 topic echo /fueling/done
```

## 📁 프로젝트 구조

```
fuel_robot_pkg/
├── launch/
│   └── fuel_robot.launch.py                # 메인 런치 파일
│
├── fuel_robot_pkg/
│   ├── config_loader.py                    # .env 로드 및 설정 관리
│   ├── doosan_commander_node.py            # 로봇 제어 노드
│   ├── fuel_port_perception_node.py        # 시각 인식 노드 (YOLOv12)
│   ├── fueling_task_manager_node.py        # 작업 관리 노드
│   └── ui_gateway_node.py                  # UI 게이트웨이 노드
│
├── .env.example                            # 환경설정 템플릿 (팀원과 공유용)
├── .gitignore                              # 민감한 파일 제외 규칙
├── requirements.txt                        # Python 의존성
├── SETUP.md                                # 팀원 온보딩 가이드
├── GITHUB_SETUP.md                         # GitHub 푸시 가이드
└── README.md                               # 이 파일
```

### 로컬에만 저장되는 파일 (.gitignore 적용)
```
.env                              # 로봇 IP, SSH 정보
~/.ssh/robot/robot_key.pem       # SSH 개인 키
weights/yolov12.pt               # ML 모델 (크기 > 100MB)
config/robot.local.yaml          # 로컬 로봇 설정
captures/, datasets/             # 학습 데이터, 테스트 영상
```

## 📚 문서

| 문서 | 설명 |
|------|------|
| [SETUP.md](SETUP.md) | 팀원 온보딩 및 환경 설정 가이드 |
| [GITHUB_SETUP.md](GITHUB_SETUP.md) | GitHub 푸시 및 협업 가이드 |
| [.env.example](.env.example) | 환경설정 템플릿 |

## 🔐 보안

### 공유되는 파일
✅ 코드 파일 (*.py, *.launch.py)
✅ 문서 (SETUP.md, GITHUB_SETUP.md)
✅ 설정 템플릿 (.env.example)

### 로컬에만 보관되는 파일 (절대 커밋 금지!)
❌ .env - 로봇 IP, SSH 정보
❌ *.pem, *.key - SSH 개인 키
❌ weights/*.pt - 대용량 모델
❌ config/robot.local.yaml - 로컬 설정

자세한 보안 정책은 [GITHUB_SETUP.md](GITHUB_SETUP.md#-보안-체크리스트)를 참고하세요.

## 🛠️ 기술 스택

- **ROS 2**: Humble
- **로봇 제어**: Doosan Robot ROS 2 인터페이스
- **비전**: YOLOv12, OpenCV
- **AI/ML**: PyTorch, Ultralytics
- **설정**: Python-dotenv
- **언어**: Python 3.10+

## 📊 토픽 및 서비스

### Publish 토픽
- `/fueling/start` (std_msgs/Bool) - 급유 시작 신호
- `/fueling/status` (std_msgs/String) - 현재 상태 메시지
- `/fueling/done` (std_msgs/Bool) - 작업 완료 신호

### Subscribe 토픽
- `/fueling/fuel_port_pose` (sensor_msgs/PointCloud2) - 감지된 포트 위치
- `/camera/color/image_raw` (sensor_msgs/Image) - 카메라 영상

### 서비스
- `/fueling/start_button` (std_srvs/srv/Trigger) - 급유 시작 버튼

## 🤝 기여 가이드

1. 저장소를 フォークします
2. feature 브랜치 생성: `git checkout -b feature/my-feature`
3. 코드 작성 및 테스트
4. 커밋: `git commit -am 'Add new feature'`
5. 브랜치 푸시: `git push origin feature/my-feature`
6. Pull Request 생성

**주의**: .env, SSH 키, 모델 파일은 커밋하지 말 것.

자세한 가이드는 [GITHUB_SETUP.md](GITHUB_SETUP.md)를 참고하세요.

## 📞 문제 해결

### 로봇 연결 안 됨
```bash
# 1. 로봇 IP 확인
echo $ROBOT_IP
ping $ROBOT_IP

# 2. .env 파일 로드 확인
set -a
source .env
set +a
echo $ROBOT_IP  # 값이 나와야 함
```

### YOLOv12 모델 누락
```bash
# 모델 파일 경로 확인
ls -la $YOLOv12_MODEL_PATH

# 없으면 다운로드 필요
mkdir -p weights/
```

### SSH 연결 문제
```bash
# SSH 키 권한 확인
ls -la ~/.ssh/robot/
chmod 600 ~/.ssh/robot/robot_key.pem

# 테스트
ssh -i ~/.ssh/robot/robot_key.pem root@$ROBOT_IP
```

더 많은 문제 해결은 [SETUP.md#-트러블슈팅](SETUP.md#-트러블슈팅)을 참고하세요.

## 📝 라이선스

이 프로젝트는 MIT 라이선스를 따릅니다.

## 👥 팀

- **팀장**: JaeyeonE
- **팀원들**: [팀원 추가]

## 📧 연락처

문제 또는 제안사항이 있으면 GitHub Issues를 통해 보고해주세요.

---

**마지막 업데이트**: 2026년 4월 15일
