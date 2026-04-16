# GitHub 저장소 설정 및 푸시 가이드

## 🎯 목표

GitHub 저장소에서 **다음 파일들만 관리**:
- ✅ `launch/fuel_robot.launch.py`
- ✅ `fuel_robot_pkg/doosan_commander_node.py`
- ✅ `fuel_robot_pkg/fuel_port_perception_node.py`
- ✅ `fuel_robot_pkg/fueling_task_manager_node.py`
- ✅ `fuel_robot_pkg/ui_gateway_node.py`
- ✅ `fuel_robot_pkg/config_loader.py` (설정 로드 유틸)
- ✅ 문서 및 설정 템플릿

**절대 포함시키지 말 것:**
- ❌ `.env` (로컬 전용)
- ❌ SSH 키 (`.pem`, `.key`)
- ❌ `weights/yolov12.pt` (대용량 모델)
- ❌ 로컬 설정 파일들

## 📦 초기 저장소 설정 (첫 번째만)

### 1. GitHub 저장소 생성
```bash
# GitHub에서 https://github.com/JaeyeonE/robotic-arm-refueler 생성

# 또는 GitHub CLI 사용:
gh repo create robotic-arm-refueler --public --source=.
```

### 2. 로컬 저장소 초기화
```bash
cd ~/ros2_ws/src/fuel_robot_pkg

# 기존 git 정보 제거 (필요시)
rm -rf .git

# 새로 초기화
git init

# 원본 저장소 추가
git remote add origin https://github.com/JaeyeonE/robotic-arm-refueler.git

# (또는 SSH)
git remote add origin git@github.com:JaeyeonE/robotic-arm-refueler.git
```

### 3. 첫 번째 커밋
```bash
# 상태 확인 (민감한 파일 확인)
git status

# .env, *.pem 파일이 보이지 않으면 OK!

# 파일 추가
git add .gitignore .env.example SETUP.md GITHUB_SETUP.md
git add launch/
git add fuel_robot_pkg/

# 커밋
git commit -m "Initial commit: robotic arm refueler control system"

# 메인 브랜치로 푸시
git branch -M main
git push -u origin main
```

## 🔄 정기적인 푸시 워크플로우

### 새로운 기능 추가
```bash
cd ~/ros2_ws/src/fuel_robot_pkg

# 1. 최신 코드 가져오기
git fetch origin
git pull origin main

# 2. feature 브랜치 생성
git checkout -b feature/perception-improvement

# 3. 코드 작성 및 테스트
# ... 파일 수정 ...

# 4. 커밋 전 체크리스트
echo "Checking for sensitive files..."
if git status | grep -E '\.env|\.pem|\.key|id_rsa|weights/.*\.pt'; then
    echo "❌ ERROR: Sensitive files detected!"
    echo "Run: git reset HEAD <file>"
    exit 1
fi
echo "✅ No sensitive files detected"

# 5. 파일 추가 및 커밋
git add fuel_robot_pkg/  # 또는 구체적인 파일: git add fuel_robot_pkg/fuel_port_perception_node.py
git commit -m "Improve perception accuracy with confidence threshold"

# 6. 원본으로 푸시
git push origin feature/perception-improvement
```

### GitHub에서 Pull Request 생성 및 병합
```bash
# 1. GitHub 웹사이트에서 Pull Request 생성
#    https://github.com/JaeyeonE/robotic-arm-refueler/pull/new/feature/perception-improvement

# 2. 코드 리뷰 대기

# 3. 리뷰 후 메인 브랜치로 병합

# 4. 로컬에서 메인 브랜치 동기화
git checkout main
git pull origin main

# 5. feature 브랜치 삭제
git branch -d feature/perception-improvement
git push origin --delete feature/perception-improvement
```

## 🛡️ 보안 체크리스트

### 커밋 전 확인사항
```bash
# 1. .env 파일 확인
git status | grep .env
# 출력이 없어야 함 (.gitignore에 의해 자동 제외)

# 2. SSH 키 파일 확인
git status | grep -E '\.pem|\.key|id_rsa'
# 출력이 없어야 함

# 3. 모델 파일 확인
git status | grep 'weights/'
git status | grep '\.pt'
# 출력이 없어야 함

# 4. 설정 파일 확인
git status | grep 'config/robot'
git status | grep 'secrets/'
# 출력이 없어야 함
```

### 혹시 모를 대비
```bash
# 1. 민감한 파일이 실수로 스테이징된 경우
git reset HEAD .env config/robot.local.yaml

# 2. 이미 커밋된 경우 (아직 푸시 안 함)
git reset --soft HEAD~1  # 커밋 취소, 파일 유지
git add .              # .gitignore 적용 재강제
git commit -m "Remove sensitive files"

# 3. 이미 푸시된 경우 (위험!)
# 팀에 알리고 JaeyeonE와 상담하기
git log --oneline  # 커밋 해시 확인
git revert <commit-hash>
git push origin main
```

## 📤 푸시 명령어 요약

### 간단한 한 줄 커맨드
```bash
# 1. 변경사항 확인
git status

# 2. 민감한 파일 있는지 확인
git diff --cached --name-only | grep -E '\.env|\.pem|\.key|id_rsa|\.pt$'

# 3. 모두 보이지 않으면 푸시
git push origin <branch-name>
```

### 자동 체크 스크립트
```bash
#!/bin/bash
# save as: check_before_push.sh

echo "🔍 Checking for sensitive files before push..."

# 위험한 파일 패턴
DANGEROUS_PATTERNS=(
    "\.env"
    "\.pem"
    "\.key"
    "id_rsa"
    "\.pt$"
    "config/robot"
    "secrets/"
)

found_dangerous=0
for pattern in "${DANGEROUS_PATTERNS[@]}"; do
    if git diff --cached --name-only | grep -E "$pattern"; then
        echo "❌ Found: $pattern"
        found_dangerous=1
    fi
done

if [ $found_dangerous -eq 0 ]; then
    echo "✅ Safe to push!"
else
    echo "⚠️  DO NOT PUSH! Remove dangerous files first."
    exit 1
fi
```

사용법:
```bash
chmod +x check_before_push.sh
./check_before_push.sh
```

## 🤝 팀 협업 시

### 팀원이 저장소를 클론했을 때
```bash
# 팀원이 실행:
git clone https://github.com/JaeyeonE/robotic-arm-refueler.git
cd robotic-arm-refueler

# SETUP.md 읽기
cat SETUP.md

# 로컬 설정 만들기
cp .env.example .env
# .env 파일 수정 (로컬 정보 입력)

# SSH 키 설정
mkdir -p ~/.ssh/robot
cp /path/to/robot_key.pem ~/.ssh/robot/
chmod 600 ~/.ssh/robot/robot_key.pem
```

### 코드리뷰 체크리스트 (리뷰어용)
PR 검토 시 다음을 확인:
```
[ ] .env 파일이 포함되지 않음
[ ] *.pem, *.key 파일이 포함되지 않음
[ ] id_rsa, id_ed25519 파일이 포함되지 않음
[ ] weights/ 폴더가 포함되지 않음
[ ] config/robot.local.yaml이 포함되지 않음
[ ] secrets/ 폴더가 포함되지 않음
[ ] 코드 변경사항이 의도한 바를 반영함
[ ] 테스트 완료됨
```

## 📋 저장소 구조 (GitHub)

```
robotic-arm-refueler/
├── .env.example                           # 팀원과 공유 (실제 .env는 로컬)
├── .gitignore                             # 민감한 파일 자동 제외
├── SETUP.md                               # 팀원 온보딩 가이드
├── GITHUB_SETUP.md                        # 이 파일
├── README.md                              # 프로젝트 개요
├── launch/
│   └── fuel_robot.launch.py
├── fuel_robot_pkg/
│   ├── config_loader.py                   # 환경 설정 로더
│   ├── doosan_commander_node.py
│   ├── fuel_port_perception_node.py
│   ├── fueling_task_manager_node.py
│   └── ui_gateway_node.py
└── requirements.txt                       # Python 의존성
```

## 🚀 첫 푸시 완료 후

1. GitHub에서 README.md 작성 (README.md가 없으면 생성)
2. 저장소 설명 추가: "Doosan robot arm fueling automation system"
3. Topics 추가: `ros2`, `robotics`, `doosan-robot`, `automation`
4. 팀원들에게 SETUP.md 문서 공유
5. 정기적인 백업 및 커밋 규칙 수립

## 🆘 도움말

### 원격 저장소 확인
```bash
git remote -v
```

### 커밋 로그 확인
```bash
git log --oneline -10
```

### 푸시된 파일 목록 확인
```bash
git ls-tree -r HEAD --name-only
```

### 특정 파일 이력 확인
```bash
git log --follow -- fuel_robot_pkg/doosan_commander_node.py
```

---

**모든 푸시 전에 한 번 더 확인하세요:**
1. ✅ `.env` 파일은 커밋되지 않음
2. ✅ SSH 키는 커밋되지 않음
3. ✅ 모델 파일은 커밋되지 않음
4. ✅ 의도한 코드만 커밋됨
