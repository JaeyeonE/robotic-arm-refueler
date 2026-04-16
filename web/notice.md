# AutoJet 통합 노트 — `web/` ↔ `vision/` ↔ `fuel_robot_pkg/`

최종 갱신: 2026-04-16 (demo_integration 브랜치 기준)

---

## 0. 현재 상태 — 이미 동작하는 것

**3프로세스 구조로 로봇 팔 주유구 인식→이동 성공**.

```
┌──────────────────┐       HTTP :5000        ┌──────────────────┐
│  vision/         │ ──POST /api/vision/*──► │  web/            │
│  post_vision.py  │ ──POST /api/vision/     │  app.py (Flask)  │
│  (RealSense+YOLO)│    frame/<sid>          │  + SQLite 7테이블 │
└────────┬─────────┘                         │  + 키오스크/관리자 │
         │                                    └──────────────────┘
         │ (사람이 수동으로 xyz 확인 후)
         ▼
┌──────────────────────────────────────────┐
│  fuel_robot_pkg/ (ROS2 Humble)           │
│  ├─ ui_gateway_node     :6000 HTTP서버    │
│  │   POST /fueling/start {x,y,z}         │
│  │   POST /fueling/estop                 │
│  │   GET  /fueling/status                │
│  ├─ fueling_task_manager_node            │
│  │   start → xyz pub → move cmd → done   │
│  └─ doosan_commander_node                │
│      MoveJoint / MoveLine / ToolDO       │
└──────────────────────────────────────────┘
```

### 핵심 사실
- `demo_integration` 브랜치에 `web/`, `vision/`, `fuel_robot_pkg/` **모두 포함** (monorepo 이미 완성)
- `ui_gateway_node.py` = **134줄, HTTP 서버(:6000) 탑재 완료** (스켈레톤 아님)
  - `POST /fueling/start` → robot_x,y,z 받아서 `/fueling/start` Bool + `/fueling/fuel_port_xyz` Float64MultiArray 퍼블리시
  - `POST /fueling/estop` → 상태 퍼블리시 (로봇 정지 로직은 미구현)
  - `GET /fueling/status` → latest_status / latest_done 반환
- `post_vision.py` = **폐기 대상 아님**, 현재 유일한 비전→웹 파이프라인. Flask에 detection + frame POST하는 역할
- `fuel_port_perception_node.py`는 ROS2 내부용 (request_detection → xyz 퍼블리시). `post_vision.py`와 **역할 분리됨**:
  - `post_vision.py`: 웹 UI용 (프레임 스트리밍 + detection DB 기록)
  - `perception_node`: ROS2 FSM용 (요청 시 xyz 퍼블리시 → task_manager → commander)
- `t_cam2robot`은 `[400, 238, 110]mm`로 업데이트됨 (이전 `[420, 65, 80]`에서 변경)
- task_manager의 고정 자세값: `rx=90, ry=-90, rz=-90` (이전 `180,0,90`에서 변경)
- YOLO 가중치: `refueling.pt` 사용 중 (이전 `20260415_1149.pt`에서 변경)

---

## 1. 데이터 흐름 — 현재 vs 목표

### 1.1 현재 (수동 트리거)
```
post_vision.py → Flask DB에 detection 기록 + 프레임 전송 (자동)
사람이 xyz 확인 → 수동으로 gateway :6000에 POST → 로봇 이동 (수동)
```

### 1.2 목표 (자동 트리거)
```
post_vision.py → Flask DB에 detection 기록 + 프레임 전송 (자동)
키오스크 "시작" 버튼 → Flask /api/task/start
    → Flask가 최신 detection의 xyz를 가져와서
    → gateway :6000 POST /fueling/start {x,y,z} (자동)
    → ROS2 FSM 진행
    → gateway가 /fueling/status, /fueling/done 수신
    → Flask API로 상태 업데이트 (로그, step, 완료)
```

---

## 2. 남은 구현 항목

### ✅ 이미 완료
- [x] monorepo 구조 (`demo_integration` 브랜치)
- [x] `ui_gateway_node` HTTP 서버 (:6000) — start/estop/status
- [x] `post_vision.py` → Flask 비전 파이프라인
- [x] 좌표 변환 캘리브레이션 (`t_cam2robot`, RPY 업데이트)
- [x] 로봇 이동 roundtrip 성공 (수동 xyz 트리거)

### 🔨 Phase 1: 자동 트리거 연결 (웹 → ROS2)
- [ ] **`app.py` — `/api/task/start` 확장**: task 생성 후, 최신 `vision_detections`에서 `robot_x,y,z`를 가져와 gateway `:6000/fueling/start`에 POST
- [ ] **키오스크 JS** — 시작 버튼 클릭 시 `/api/task/start` 호출 (이미 있음, 동작 확인만)
- [ ] Flask에 `GATEWAY_URL = "http://localhost:6000"` 설정 추가

### 🔨 Phase 2: 상태 피드백 (ROS2 → 웹)
- [ ] **`app.py` — gateway 상태 폴링 스레드**: 0.5s 주기로 `GET :6000/fueling/status` → DB 업데이트
  - `status` 변경 시 → `POST /api/logs` (source="control")
  - `done=True` → `PATCH /api/task/<id>` (status='success', current_step=4, finished_at)
  - `done=False` → `PATCH /api/task/<id>` (status='failed')
- [ ] 또는 **`ui_gateway_node` 확장**: status/done 콜백에서 직접 `requests.post()` → Flask API (역방향 push)

### 🔨 Phase 3: E-STOP 완성
- [ ] **키오스크/관리자 E-STOP 버튼** → `POST /api/task/estop`
- [ ] **`app.py` estop 핸들러 확장**: task DB 업데이트 + gateway `:6000/fueling/estop` 호출
- [ ] **`ui_gateway_node` estop 확장**: 현재 `publish_status('estop_from_web')`만 함 → **실제 로봇 정지 로직 추가 필요** (commander에 stop 명령 또는 `SetRobotMode(0)` 서비스 콜)
- [ ] **`doosan_commander_node`에 estop 수신 경로 추가** (현재 없음)

### 🔨 Phase 4: 로그 / 모니터링 보강
- [ ] ROS2 `/fueling/status` 메시지 → Flask `logs` 테이블 연동 (Phase 2에서 함께 처리 가능)
- [ ] admin.js 로그 패널에 source/level 필터 UI
- [ ] (선택) `/joint_states` subscribe → `POST /api/robot/snapshot` 1Hz 브리지 — admin 관절/TCP 차트 활성화
- [ ] (선택) `_fueling_loop()` step 전환 로직 축소 → ROS2 FSM이 step 관리

### 🔨 Phase 5: 마무리
- [ ] 키오스크에서 end-to-end 테스트: 시작→인식→이동→(주유시뮬)→완료→금액표시
- [ ] admin에서 실시간 상태 확인 (로봇모드, 비전, 로그)
- [ ] E-STOP roundtrip 검증

---

## 3. 아키텍처 의사결정 기록

| 쟁점 | 결정 | 이유 |
|------|------|------|
| `post_vision.py` 폐기? | **유지** | 웹 UI 프레임 스트리밍 + DB 기록 역할. perception_node와 역할 분리됨 |
| HTTP↔ROS2 브리지 방식 | **B안 — gateway가 HTTP 서버 운영** | 이미 구현 완료 (:6000). Flask가 직접 호출 가능 |
| FSM step 관리 주체 | **ROS2** (0→1→2→4), **Flask** (3=주유 리터 시뮬) | 하드웨어 동작은 ROS2, 비즈니스 로직은 Flask |
| 레포 구조 | **monorepo** (`demo_integration` → main 머지 예정) | 이미 완성 |
| E-STOP 경로 | **웹 → Flask → gateway :6000 → ROS2** | 양방향 완성 필요 (commander 쪽 미구현) |
| 비전 파이프라인 중복 | **공존** (post_vision=웹용, perception_node=FSM용) | 카메라 1대인데 두 프로세스가 동시에 열 수 없음 → **장기적으로 통합 검토** |

---

## 4. 알려진 이슈

1. **카메라 점유 충돌**: `post_vision.py`와 `fuel_port_perception_node`가 동시에 RealSense를 열 수 없음. 현재는 `post_vision.py`만 카메라 사용, perception_node는 request 시에만 동작하는 것으로 보이지만 **동시 실행 시 충돌 가능**. 장기적으로 하나로 합쳐야 함.
2. **DRCF access control**: `MANAGE_ACCESS_CONTROL_FORCE_REQUEST` 실패 이슈 미해결 (이전 세션에서 발생). 가상모드에서는 문제없음.
3. **E-STOP 실제 효과**: `ui_gateway_node`가 `publish_status('estop_from_web')`만 하고 **로봇을 실제로 멈추지 않음**. commander에 정지 서비스 콜 필요.
4. **`/fueling/start` 메시지 제한**: Bool만 전달, 유종/금액/리터 정보 미포함. 당장은 Flask DB에서 관리하므로 문제없지만, ROS2 쪽에서 task 정보 필요 시 확장 필요.

---

## 5. 친구와 협의 필요

1. **E-STOP 실제 정지 로직**: commander에 `/fueling/estop` subscribe 또는 서비스 추가
2. **카메라 점유 전략**: post_vision.py + perception_node 동시 실행 방법 (또는 합칠지)
3. **`demo_integration` → `main` 머지 시점**: 자동 트리거(Phase 1) 완료 후?
4. **task 완료 후 복귀**: commander의 `home_j = [0,0,90,0,90,0]` 복귀 동작이 FSM에 포함되는지

---

*이전 버전 (2026-04-16 오전): orphan 브랜치 전제 분석. 현 문서로 대체됨.*
