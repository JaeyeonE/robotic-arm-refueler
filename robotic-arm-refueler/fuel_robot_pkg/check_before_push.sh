#!/bin/bash
# 
# GitHub 푸시 전 민감한 파일 확인 스크립트
# 사용법: bash check_before_push.sh
#

set -e  # Exit on error

# 색상 설정
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

echo -e "${BLUE}🔍 GitHub 푸시 전 보안 검사 시작...${NC}\n"

# 1. Git 상태 확인
echo -e "${BLUE}1️⃣  Git 상태 확인${NC}"
git_status=$(git status --porcelain)
if [ -z "$git_status" ]; then
    echo -e "${GREEN}✅ 커밋할 변경사항 없음${NC}\n"
else
    echo -e "${YELLOW}ℹ️  커밋할 변경사항:${NC}"
    echo "$git_status" | head -10
    [ $(echo "$git_status" | wc -l) -gt 10 ] && echo "... and more"
    echo ""
fi

# 2. 민감한 파일 확인
echo -e "${BLUE}2️⃣  민감한 파일 검사${NC}"
DANGEROUS_PATTERNS=(
    "\.env$"
    "\.env\.local"
    "\.pem$"
    "\.key$"
    "id_rsa"
    "id_ed25519"
    "config/robot"
    "secrets/"
    "\.pt$"
    "weights/"
    "\.config$"
    "credentials"
    "password"
    "api_key"
    "secret"
)

found_dangerous=0
dangerous_files=()

# Staged 파일 확인
echo "  Staged 파일 검사 중..."
for pattern in "${DANGEROUS_PATTERNS[@]}"; do
    if git diff --cached --name-only 2>/dev/null | grep -i "$pattern" > /dev/null; then
        dangerous_files+=("$(git diff --cached --name-only | grep -i "$pattern")")
        found_dangerous=1
    fi
done

# 작업 디렉토리 파일 확인 (추적되지 않은 위험한 파일)
echo "  작업 디렉토리 파일 검사 중..."
for pattern in "${DANGEROUS_PATTERNS[@]}"; do
    if git ls-files -o --exclude-standard | grep -i "$pattern" > /dev/null; then
        untracked_dangerous=$(git ls-files -o --exclude-standard | grep -i "$pattern")
        if [ -n "$untracked_dangerous" ]; then
            echo -e "${YELLOW}⚠️  .gitignore 확인: $untracked_dangerous${NC}"
        fi
    fi
done

if [ $found_dangerous -eq 0 ]; then
    echo -e "${GREEN}✅ 위험한 파일 미감지${NC}\n"
else
    echo -e "${RED}❌ 위험한 파일 감지!${NC}"
    echo "감지된 파일:"
    for file in "${dangerous_files[@]}"; do
        echo -e "  ${RED}✘ $file${NC}"
    done
    echo ""
    echo -e "${YELLOW}해결 방법:${NC}"
    echo "  1. git reset HEAD <file>  # Unstage 파일"
    echo "  2. git add .gitignore     # .gitignore에 추가"
    echo "  3. 다시 커밋"
    echo ""
    exit 1
fi

# 3. .gitignore 확인
echo -e "${BLUE}3️⃣  .gitignore 설정 확인${NC}"
if [ ! -f ".gitignore" ]; then
    echo -e "${YELLOW}⚠️  .gitignore 파일이 없습니다!${NC}"
else
    echo -e "${GREEN}✅ .gitignore 파일 존재${NC}"
fi

# .env 파일 확인
if git check-ignore .env > /dev/null 2>&1; then
    echo -e "${GREEN}✅ .env는 .gitignore에 의해 보호됨${NC}"
else
    echo -e "${YELLOW}⚠️  .env 파일이 보호되지 않음!${NC}"
fi

echo ""

# 4. 커밋 로그 확인
echo -e "${BLUE}4️⃣  최근 커밋 확인${NC}"
echo "  최근 3개 커밋:"
git log --oneline -3
echo ""

# 5. 원본 저장소 상태
echo -e "${BLUE}5️⃣  원본 저장소 상태${NC}"
current_branch=$(git rev-parse --abbrev-ref HEAD)
echo "  현재 브랜치: $current_branch"

# 6. 푸시 대상 확인
echo -e "${BLUE}6️⃣  푸시 대상 확인${NC}"
if [ -z "$(git remote)" ]; then
    echo -e "${RED}❌ 원본 저장소가 설정되지 않음${NC}"
    exit 1
else
    default_remote=$(git remote)
    echo "  원본 저장소: $default_remote"
    git remote -v
fi

echo ""

# 최종 확인
echo -e "${GREEN}✅ 모든 검사 통과!${NC}"
echo -e "${YELLOW}푸시할 준비가 되었습니다.${NC}\n"

# 푸시 제안
echo -e "${BLUE}다음 명령으로 푸시할 수 있습니다:${NC}"
echo "  git push origin $current_branch"
echo ""

# 사용자 확인
read -p "지금 푸시하시겠습니까? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}📤 푸시 중...${NC}"
    git push origin "$current_branch"
    echo -e "${GREEN}✅ 푸시 완료!${NC}"
else
    echo -e "${YELLOW}푸시 취소됨${NC}"
fi
