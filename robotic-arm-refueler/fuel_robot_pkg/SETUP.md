# 로봇 급유 시스템 - 팀원 설정 가이드

## 📋 초기 설정 (처음 한 번만)

### 1. 저장소 클론
```bash
git clone https://github.com/JaeyeonE/robotic-arm-refueler.git
cd robotic-arm-refueler
```

### 2. 환경 설정 파일 생성
```bash
# .env.example을 기반으로 로컬 설정 파일 생성
cp .env.example .env
```

### 3. `.env` 파일 수정 (로컬에서만 관리)
```bash
# 편집: .env 파일을 열고 로봇 IP와 설정을 입력하세요
nano .env  # 또는 VSCode에서 열기
```

필수 항목:
- `ROBOT_IP`: 로봇 IP 주소 (192.168.x.x)
- `ROBOT_SSH_USER`: SSH 사용자명
- `ROBOT_SSH_KEY_PATH`: SSH 키 파일 경로 (**.env에는 절대 경로만 기록**)

### 4. SSH 키 파일 설정 (로컬 전용)
```bash
# SSH 키를 안전한 위치에 저장
mkdir -p ~/.ssh/robot
cp /path/to/robot_key.pem ~/.ssh/robot/
chmod 600 ~/.ssh/robot/robot_key.pem

# .env에서 ROBOT_SSH_KEY_PATH 설정:
# ROBOT_SSH_KEY_PATH=$HOME/.ssh/robot/robot_key.pem
```

## 📁 프로젝트 구조

공유되는 파일들:
```
fuel_robot_pkg/
├── launch/
│   └── fuel_robot.launch.py          # 메인 런치 파일
├── fuel_robot_pkg/
│   ├── doosan_commander_node.py       # 로봇 명령 제어
│   ├── fuel_port_perception_node.py   # 비전 인식
│   ├── fueling_task_manager_node.py   # 급유 작업 관리
│   └── ui_gateway_node.py             # UI 게이트웨이
├── .env.example                       # 환경설정 템플릿
└── SETUP.md                           # 이 파일
```

**로컬에서만 유지되는 파일들 (.gitignore):**
- `.env` - 로봇 IP, SSH 정보
- `~/.ssh/robot/` - SSH 개인 키
- `weights/` - ML 모델 파일 (크기 > 100MB)
- `config/robot.local.yaml` - 로컬 로봇 설정

## 🚀 실행 방법

### 환경 설정 로드
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 런치 파일 실행
```bash
# .env 파일 로드 후 실행
set -a
source .env
set +a

ros2 launch fuel_robot_pkg fuel_robot.launch.py
```

또는 Python에서:
```python
import os
from dotenv import load_dotenv

load_dotenv()  # .env 파일 로드
ROBOT_IP = os.getenv('ROBOT_IP')
ROBOT_PORT = int(os.getenv('ROBOT_PORT', 30015))
```

## 🔐 보안 규칙

### ✅ 이것들은 커밋해도 됨:
- 코드 파일 (*.py, *.launch.py)
- 설정 템플릿 (.env.example)
- 문서 (README.md, SETUP.md)
- 공개 리소스

### ❌ 절대 커밋하지 말 것:
```bash
# 이미 .gitignore에 설정되어 있음
.env                           # 로봇 IP, SSH 정보
*.pem, *.key                   # SSH 개인 키
id_rsa, id_ed25519            # SSH 키
config/robot.local.yaml       # 로컬 설정
weights/*.pt                  # 대용량 모델
captures/, datasets/          # 테스트 데이터
```

### 실수로 커밋한 경우:
```bash
# 1. 파일 언스테이징
git rm --cached .env config/robot.local.yaml

# 2. .gitignore에 추가될 파일 확인
git status

# 3. 커밋
git add .gitignore
git commit -m "Add local configs to gitignore"
```

## 🔄 팀 협업 워크플로우

### 1. 새로운 기능 추가
```bash
# feature 브랜치 생성
git checkout -b feature/my-feature

# 코드 작성 및 테스트
# ...

# 커밋 전 확인사항:
# ✓ .env 파일이 커밋되지 않았는가?
# ✓ SSH 키가 포함되지 않았는가?
# ✓ 모델 파일이 포함되지 않았는가?

git add fuel_robot_pkg/
git commit -m "Add feature: description"
git push origin feature/my-feature
```

### 2. Pull Request 검토
리뷰어는 다음을 확인:
- [ ] .env, *.pem, *.key 파일이 포함되지 않음
- [ ] secrets/ 폴더가 포함되지 않음
- [ ] weights/ 폴더가 포함되지 않음

### 3. 메인 브랜치에 병합
```bash
git checkout main
git pull origin main
git merge feature/my-feature
git push origin main
```

## 📦 첫 실행 시 필요한 것

### 1. .env 파일 생성
```bash
cp .env.example .env
# 로봇 정보 입력
```

### 2. SSH 키 설정
```bash
# 로봇 운영자에게서 SSH 키 받기
# ~/.ssh/robot/ 폴더에 저장
chmod 600 ~/.ssh/robot/robot_key.pem
```

### 3. 모델 파일 다운로드 (처음 1회)
```bash
# YOLOv12 모델 다운로드
mkdir -p weights/
wget -O weights/yolov12.pt https://[model-url]/yolov12.pt
```

### 4. ROS 2 의존성 설치
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. 빌드 및 실행
```bash
cd ~/ros2_ws
colcon build --packages-select fuel_robot_pkg
source install/setup.bash
ros2 launch fuel_robot_pkg fuel_robot.launch.py
```

## 🆘 트러블슈팅

### "Robot IP not found" 에러
```bash
# .env 파일이 로드되지 않음
echo $ROBOT_IP  # 값이 없으면 .env 미로드

# 해결: 아래 명령 실행
set -a
source .env
set +a
echo $ROBOT_IP  # 다시 확인
```

### SSH 연결 거부
```bash
# 1. 키 파일 권한 확인
ls -la ~/.ssh/robot/
chmod 600 ~/.ssh/robot/robot_key.pem

# 2. 로봇 IP 확인
ping $ROBOT_IP

# 3. 로봇 운영자에게 SSH 경로 재확인
```

### 모델 파일 누락
```bash
# .env의 YOLOv12_MODEL_PATH 확인
echo $YOLOv12_MODEL_PATH
ls -la $YOLOv12_MODEL_PATH

# 없으면 다운로드
mkdir -p weights/
# 모델 다운로드 스크립트 실행
```

## 📧 문의사항

로봇 설정이나 연결에 관련된 문제:
1. 팀장/로봇 운영자에게 연락
2. 로봇 IP, SSH 정보 확인 (공유 금지!)
3. SSH 키 권한 확인

## 참고

- [ROS 2 공식 문서](https://docs.ros.org/)
- [Doosan Robot ROS 2 패키지](https://github.com/doosan-robotics/doosan-robot2)
- [Python-dotenv 문서](https://github.com/theskumar/python-dotenv)
