# AutoJet 통합 노트 — `web/` ↔ `fuel_robot_pkg` (ROS2)

작성일: 2026-04-16
대상 브랜치/레포:
- 우리: `robotic-arm-refueler` (web, vision branches — orphan)
- 친구: `robotic-arm-refueler/main → fuel_robot_pkg/` (표준 ROS2 ament_python)

---

## 1. 두 계층의 현재 상태

### 1.1 `web/` (우리 쪽)
- Flask 앱 + SQLite 7테이블 + 세션 인증
- HTTP API 30+개 (branches/stations/robot/vision/task/logs)
- **키오스크/관리자 UI가 이미 완성됨** — admin.html, kiosk.html, login.html + JS 폴링
- `_fueling_loop()` 백그라운드 스레드 = **주유 시뮬레이터**. `vision_detections.label=='refusing'` 감지되면 50ms 틱으로 L/₩ 증가
- `post_vision.py` (vision branch) = **standalone** RealSense+YOLO → HTTP POST 클라이언트

### 1.2 `fuel_robot_pkg/` (친구 쪽)
4개 노드로 구성된 ROS2 Humble 패키지:

| 노드 | 역할 | 주요 토픽/서비스 |
|------|------|------------------|
| `camera.py` | RealSense 영상 퍼블리시 | (아직 확인 미필) |
| `fuel_port_perception_node` | YOLOv12 + PCA 핸들각 + 좌표변환 | sub `/fueling/request_detection`, pub `/fueling/fuel_port_pose` |
| `fueling_task_manager_node` | FSM: start→detect→move→done | pub `/fueling/status`, `/fueling/done`, `/fueling/robot_cmd` |
| `doosan_commander_node` | DART 서비스 클라이언트 (move_joint/move_line/tool_do) | sub `/fueling/robot_cmd` |
| `ui_gateway_node` | **UI 브리지 (스켈레톤)** — Trigger 서비스 ↔ `/fueling/start` Bool | 웹 연동 로직 **미구현** |

---

## 2. 발견된 중복 / 충돌

### 2.1 🚨 **비전 파이프라인 중복** (가장 큰 문제)
`post_vision.py`와 `fuel_port_perception_node.py`가 **같은 일을 두 번** 함:

| 항목 | `post_vision.py` | `fuel_port_perception_node.py` |
|------|-----------------|-------------------------------|
| RealSense pipeline | 640×480 / 30fps align | 동일 |
| YOLO 모델 | `20260415_1149.pt` | `yolov12.pt` (param) |
| 좌표변환 행렬 | `R_cam2robot`, `t=[420,65,80]` | **완전 동일** |
| cap_handle PCA 각도 | ✅ | ✅ |
| 출력 | HTTP `POST /api/vision/detect` | ROS2 `/fueling/fuel_port_pose` |

**해결**: `post_vision.py`는 폐기. ROS2 노드 쪽을 **정본**으로 삼고, 웹 연동은 `ui_gateway_node` 확장으로 처리.

### 2.2 FSM 모델 상이
- **웹**: 0=대기 / 1=탐색 / 2=접근 / 3=주유 / 4=복귀 + 리터 틱 시뮬
- **ROS2**: start → request_detection → xyz_received → move → done (단순 4단계, 실제 리터 계산 없음)

**해결**: 역할 분리
- ROS2 task_manager = **실제 로봇 동작 단계 관리** (0~2, 4)
- 웹 `_fueling_loop` = **주유 중(step=3) 리터/금액 누적** (실제 유량계 없으므로 시뮬 유지)
- ROS2가 step 전환할 때마다 `PATCH /api/task/<id>`로 `current_step`, `step_progress` 갱신

### 2.3 YOLO 가중치 파일명 불일치
- `post_vision.py`: `20260415_1149.pt`
- ROS2 param default: `yolov12.pt`
- **해결**: 팀 합의 후 하나로 통일. 파일은 `.gitignore`에 걸려있으므로 repo 외부 공유 (Google Drive 등)

### 2.4 좌표계 오프셋은 이미 동일 ✅
두 쪽 모두 `R_cam2robot`, `t_cam2robot=[420,65,80]mm`로 일치. 캘리브레이션 재검증 불필요.

---

## 3. `ui_gateway_node` — 핵심 통합 지점

현재 상태: **30줄짜리 스켈레톤**. `Trigger` 서비스 받아서 `/fueling/start` Bool publish만 함.

### 3.1 확장해야 할 기능 (우리 몫)

```
┌─────────────────────────────────────────────────────────────┐
│              ui_gateway_node (확장판)                         │
│                                                              │
│  HTTP → ROS2 방향:                                           │
│  ├─ Flask → 이 노드의 HTTP endpoint 또는                       │
│  │   이 노드가 주기적으로 GET /api/task/current 폴링 →        │
│  │   새 task 감지 시 /fueling/start 퍼블리시                   │
│  │                                                           │
│  ROS2 → HTTP 방향:                                           │
│  ├─ sub /fueling/status        → POST /api/logs              │
│  ├─ sub /fueling/done          → PATCH /api/task/<id>        │
│  ├─ sub /fueling/fuel_port_pose → POST /api/vision/detect    │
│  ├─ sub /joint_states          → POST /api/robot/snapshot    │
│  └─ sub /camera/color/image_raw → POST /api/vision/frame/<sid>│
│                                                              │
│  환경변수로 주입:                                              │
│  ├─ FLASK_URL=http://localhost:5000                          │
│  └─ STATION_ID=1                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 구현 선택지: **Polling** vs **Direct Trigger**

| 방식 | 장점 | 단점 |
|------|------|------|
| A. gateway가 `/api/task/current` 폴링 | Flask 단순, 기존 코드 무변경 | 최대 1초 지연 |
| B. Flask가 gateway의 HTTP 엔드포인트 호출 | 즉시 트리거 | gateway에 Flask/web server 추가 필요 |
| C. ROS2 service를 Flask가 rclpy로 직접 호출 | 가장 정석 | Flask에 rclpy import → 의존성 무거움 |

**추천: A안**. 이미 웹의 `_fueling_loop`가 1초 주기 폴링하고 있으니 동일 패턴. gateway를 0.5s 주기로 `/api/task/current` 폴링 → 새 task_id 감지 시 `/fueling/start` 트리거.

---

## 4. 통합 리포지토리 구조 — 권장안

### 4.1 현재 구조의 문제
- 우리: `web`, `vision` orphan branch (히스토리 분리됨)
- 친구: `main`에 `fuel_robot_pkg/`
- 머지 시 `--allow-unrelated-histories` 필요, 히스토리가 3갈래로 찢어짐

### 4.2 권장: **monorepo로 재정비**

`main` 브랜치 하나에 다음 구조로 통합:

```
robotic-arm-refueler/
├── README.md                      ← 통합 개요
├── fuel_robot_pkg/                ← 친구 ROS2 패키지 (그대로)
│   ├── fuel_robot_pkg/
│   ├── launch/
│   ├── package.xml
│   └── ...
├── web/                           ← 우리 Flask/UI
│   ├── app.py
│   ├── database.py
│   ├── logger.py
│   ├── templates/
│   ├── static/
│   └── ...
└── (vision/post_vision.py 폐기)
```

이후 작업은 `main`에서 feature branch를 떠서 PR 올리는 정석 흐름.

### 4.3 전환 절차 (한 사람이 한 번 정리)
```bash
# main 기준으로 web/, vision/ 브랜치 병합
git checkout main
git pull origin main
git merge web --allow-unrelated-histories -m "merge web layer"
# vision은 post_vision.py 통째 폐기 예정이므로 굳이 머지할 필요 없음
git push origin main

# 이후 orphan 브랜치는 archive 또는 삭제
git push origin --delete web
git push origin --delete vision
```

---

## 5. 단계별 통합 체크리스트

### Phase A. 리포지토리 정리 (즉시)
- [ ] `main` 브랜치에 `web/` 서브디렉토리로 머지 (위 4.3)
- [ ] `post_vision.py` 폐기 결정 팀 공지
- [ ] YOLO 가중치 파일 버전 통일 (팀 합의)
- [ ] `.env.example` 참고해서 `FLASK_URL`, `STATION_ID` 환경변수 규약 추가

### Phase B. `ui_gateway_node` 확장 (통합의 핵심)
- [ ] `requests` 라이브러리로 Flask HTTP 호출 로직 추가
- [ ] `/api/task/current` 0.5s 폴링 → 새 task 감지 시 `/fueling/start` 발행
- [ ] `/fueling/status` subscribe → `POST /api/logs` (source="control")
- [ ] `/fueling/done` subscribe → `PATCH /api/task/<id>` (status='success'|'failed', current_step=4)
- [ ] `/fueling/fuel_port_pose` subscribe → `POST /api/vision/detect`
- [ ] (선택) `/joint_states` subscribe → 1Hz throttle → `POST /api/robot/snapshot`
- [ ] (선택) `/camera/color/image_raw` subscribe → JPEG 인코딩 → `POST /api/vision/frame/<sid>`

### Phase C. 웹 쪽 소폭 수정
- [ ] `_fueling_loop()`는 **리터 누적만** 담당하도록 축소 (step 전환 코드 제거 — ROS2가 담당)
- [ ] `/api/task/start` 응답에 task_id 명확히 포함 (이미 됨)
- [ ] `/api/task/estop` 호출 시 gateway가 ROS2 쪽에도 중지 신호 전파하도록 훅 추가 (예: `/fueling/estop` 토픽 신설 — 친구와 협의 필요)

### Phase D. 통합 테스트
- [ ] 가상 모드(`mode:=virtual`)로 roundtrip 검증
- [ ] 키오스크 "시작" 버튼 → ROS2 FSM 진입 → 상태 로그가 admin 화면에 뜨는지
- [ ] E-STOP 경로 검증
- [ ] 실 로봇(`mode:=real`)은 DRCF access control 해결 후 최종 테스트

---

## 6. 협의 필요 사항 (친구와)

1. **E-STOP 토픽/서비스 신설**: 키오스크/관리자 화면에서 누른 E-STOP을 ROS2로 전파할 채널 (예: `std_srvs/Trigger` 서비스 `/fueling/estop`)
2. **task 입력값 전달**: 현재 `/fueling/start`는 `Bool`이라 유종/금액/리터 정보가 안 넘어감. `String` (JSON payload) 또는 custom msg로 확장 필요
3. **station_id 개념 도입**: ROS2 쪽은 단일 스테이션 전제. 다중 스테이션이면 namespace 규약 필요 (`/dsr01/fueling/...`)
4. **YOLO 가중치 관리**: `.gitignore`된 대용량 파일 공유 방법 (Google Drive / Git LFS 중 택)
5. **DRCF access control 이슈**: `MANAGE_ACCESS_CONTROL_FORCE_REQUEST` 실패 문제 미해결 (티칭 펜던트 없이 컨트롤러 상태 초기화 방법)

---

## 7. 빠른 의사결정 요약

| 쟁점 | 권장안 |
|------|--------|
| `post_vision.py` vs perception_node | **폐기**, ROS2 노드로 일원화 |
| FSM 담당 | **ROS2**가 step 전환, **Flask**는 리터/금액 누적만 |
| HTTP ↔ ROS2 브리지 | `ui_gateway_node` 확장, A안 (Flask API 폴링) |
| 레포 구조 | **monorepo** (`main`에 `web/` + `fuel_robot_pkg/`) |
| 트리거 방식 | gateway가 `/api/task/current` 0.5s 폴링 |
| 계정/권한 | 웹 세션 유지. ROS2 쪽은 로컬호스트 신뢰 (지금은 방화벽 없음) |

---

*이 문서는 통합 착수 전 한 번 더 친구와 공유/검토 권장.*
