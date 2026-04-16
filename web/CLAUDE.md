# AutoJet Refueling System — Web Layer

## 프로젝트 개요
ROS2 Humble 기반 Doosan E0509 협동로봇 + Intel RealSense D455 비전으로
가스통 주유구를 인식하고, 자동 주유 후 금액을 청구하는 시스템의 **웹 계층**.

세 개의 화면으로 분리 운영한다:
- **로그인** (`/login`) — 고객/관리자 공통 인증, role 기반 리다이렉트
- **고객 키오스크** (`/kiosk/<branch_id>/<station_no>`) — 스테이션 고정 배치, 주유 요청/진행/금액
- **관리자 모니터** (`/admin`) — 지점/스테이션 선택, 로봇 상태, 비전 인식, 시스템 로그

## 시스템 아키텍처 (4계층)
```
인식 계층 (Perception)
  RealSense D455 → YOLOv8/v11 → OpenCV 좌표변환
  ↕ ROS2 topic/service
제어 계층 (Control)
  Doosan DART SDK (E0509) → XYZ Gripper → 작업 FSM (Step 0~4)
  ↕ HTTP API
데이터 계층 (Data)                         ★ 본 프로젝트
  SQLite (작업/주유/비전/로봇 상태) + Python Logging
  ↕
서비스 계층 (Service)                      ★ 본 프로젝트
  Flask API → 고객 키오스크 + 관리자 모니터
```

## 기술 스택
- **Backend**: Python 3.10 + Flask
- **Database**: SQLite3 (WAL 모드)
- **Logging**: Python logging 모듈 + DB 핸들러
- **Frontend**: 순수 HTML/CSS/JS (customer_kiosk_light.html, admin_monitor_light.html)
- **Robot**: Doosan E0509 (6-DOF), DART SDK, ROS2 Humble
- **Vision**: Intel RealSense D455, YOLOv8, OpenCV

---

## 화면별 연동 API 명세

### 1. 고객 키오스크 (customer_kiosk_light.html)

| UI 영역 | 연동 API | Method | 설명 |
|---------|----------|--------|------|
| 시스템 상태 배지 | `GET /api/system/status` | GET | 로봇 온라인 여부, DART 연결 상태 |
| 카메라 피드 영역 | `GET /api/vision/current` | GET | 현재 인식 결과 (label, confidence, depth) |
| 안내 배너 | `GET /api/task/current` | GET | 현재 단계 안내 메시지 |
| 주유 정보 입력 → 시작 | `POST /api/task/start` | POST | 유종, 모드(금액/리터), 수량 전송 → task_id 반환 |
| 비상 정지 버튼 | `POST /api/task/estop` | POST | 즉시 E-STOP, 로봇 정지 명령 |
| 주유 진행 단계 (Step 1~4) | `GET /api/task/current` | GET | 현재 step, step별 완료 여부 |
| 실시간 주유 현황 (L, ₩, 시간) | `GET /api/task/metrics` | GET | 주유량, 금액, 경과/잔여 시간 |
| 시스템 로그 | `GET /api/logs?source=kiosk` | GET | 고객 화면용 로그 (제한적) |

### 2. 관리자 모니터 (admin_monitor_light.html)

| UI 영역 | 연동 API | Method | 설명 |
|---------|----------|--------|------|
| 상단 배지 (ONLINE/FUELING/DART) | `GET /api/system/status` | GET | 로봇 모드, 작업 상태, DART 연결 |
| 관절 상태 (J1~J6 angle + torque) | `GET /api/robot/joints` | GET | 6축 각도(°) + 토크(Nm) |
| TCP 좌표 (X,Y,Z,A,B,C) | `GET /api/robot/tcp` | GET | TCP 위치(mm) + 자세(°) |
| 관절 토크 차트 | `GET /api/robot/joints` | GET | 위와 동일 (토크 값으로 차트 렌더링) |
| 비전 인식 화면 | `GET /api/vision/current` | GET | 인식 label, confidence, depth, bbox |
| 인식 신뢰도 | `GET /api/vision/current` | GET | confidence 값 단독 표시 |
| 시스템 로그 | `GET /api/logs` | GET | 전체 로그 (필터: level, source) |

---

## 데이터 소스별 요구 파라미터

### Doosan DART SDK / ROS2 → Flask 전달 파라미터

| 카테고리 | 파라미터 | 타입 | 단위 | 출처 |
|----------|---------|------|------|------|
| **관절 상태** | joint_angles[6] | float[] | ° (degree) | `/joint_states` topic |
| | joint_torques[6] | float[] | Nm | `/joint_states` topic |
| **TCP 포즈** | tcp_x, tcp_y, tcp_z | float | mm | DART `get_current_tcp()` |
| | tcp_a, tcp_b, tcp_c | float | ° | DART `get_current_tcp()` |
| **로봇 상태** | robot_mode | str | - | IDLE / MOVING / FUELING / ERROR / ESTOP |
| | dart_connected | bool | - | DART SDK 연결 상태 |
| | safety_status | str | - | NORMAL / COLLISION / ESTOP |
| **작업 FSM** | current_step | int | 0~4 | 0:대기, 1:탐색, 2:접근, 3:주유, 4:복귀 |
| | step_progress | float | 0~1 | 현재 step 진행률 |

### Intel RealSense + YOLO → Flask 전달 파라미터

| 카테고리 | 파라미터 | 타입 | 단위 | 출처 |
|----------|---------|------|------|------|
| **인식 결과** | label | str | - | YOLO class name (주유구, fuel_cap 등) |
| | confidence | float | 0~1 | YOLO 인식 신뢰도 |
| | bbox (x1,y1,x2,y2) | int[] | px | 바운딩 박스 |
| | center_u, center_v | int | px | 바운딩 박스 중심 |
| **Depth** | depth | float | m | RealSense depth at (u,v) |
| **좌표 변환** | robot_x, robot_y, robot_z | float | mm | camera→robot base 변환 후 |

### 주유 세션 파라미터 (키오스크 입력)

| 파라미터 | 타입 | 설명 |
|---------|------|------|
| fuel_type | str | gasoline / diesel / lpg / electric |
| input_mode | str | amount (금액) / volume (리터) |
| target_value | float | 목표 금액(₩) 또는 목표 리터(L) |

### 주유 실시간 메트릭

| 파라미터 | 타입 | 단위 | 설명 |
|---------|------|------|------|
| liters | float | L | 현재까지 주유량 |
| cost | int | ₩ | 현재까지 금액 |
| elapsed_sec | int | s | 경과 시간 |
| remaining_sec | int | s | 예상 잔여 시간 |

---

## 로깅 체계

### 로그 레벨별 기록 대상

| Level | source | 기록 내용 |
|-------|--------|----------|
| **OK** | vision | 비전 인식 성공, 좌표 확정 |
| **INFO** | control | DART 연결, 이동 명령, TCP 업데이트 |
| **INFO** | system | 서버 시작/종료, 카메라 초기화 |
| **INFO** | kiosk | 주유 시작 요청, 유종 선택 |
| **WARN** | vision | 인식 신뢰도 하락 (<85%) |
| **WARN** | control | 충돌 민감도 알림, 미세 편차 |
| **ERROR** | control | 로봇 통신 타임아웃, E-STOP 발동 |
| **ERROR** | vision | 카메라 연결 실패, 프레임 드랍 |
| **ERROR** | system | DB 오류, API 장애 |

### 관리자 모니터에서 확인 가능한 항목
- 전체 로그 실시간 스트림 (source/level 필터)
- 인식 신뢰도 실시간 추이
- 관절 토크 실시간 차트
- 작업(주유) 이력 + 성공/실패 통계

---

## DB 스키마 (7테이블, 정규화)
```
users → branches → stations → fueling_tasks → robot_snapshots
                                             → vision_detections
                                             → logs
```
- `users`: 고객/관리자 계정 (role: customer/admin)
- `branches`: 지점 (name, address)
- `stations`: 주유 스테이션 = 로봇+카메라 1세트 (branch_id FK, station_no)
- `fueling_tasks`: 주유 작업 (station_id FK, user_id FK)
- `robot_snapshots`: 로봇 상태 스냅샷 (station_id FK)
- `vision_detections`: 비전 인식 기록 (station_id FK)
- `logs`: 시스템 로그 (station_id FK)

## 인증 체계
- Flask session 기반 (서버사이드 쿠키)
- `/login` → role='customer' → `/kiosk/<b>/<s>`, role='admin' → `/admin`
- 키오스크는 스테이션 고정 배치 (URL로 station 결정)
- 관리자는 모든 지점/스테이션 조회 가능 (좌측 사이드바 선택)
- 기본 시드: admin/admin 관리자 계정, 본점 1번 스테이션

## 디렉토리 구조
```
web/
├── CLAUDE.md
├── PLAN.md
├── app.py                          # Flask 메인 앱 (인증 + API + 라우트)
├── database.py                     # SQLite DB 스키마 7테이블 + 헬퍼
├── logger.py                       # Python logging + DB 핸들러
├── db_clear.py                     # DB 전체 초기화 스크립트
├── customer_kiosk_light.html       # 고객 키오스크 UI (원본 디자인)
├── admin_monitor_light.html        # 관리자 모니터 UI (원본 디자인)
├── templates/
│   ├── login.html                  # 공통 로그인 (회원가입 겸용)
│   ├── kiosk.html                  # 고객 키오스크 (station_id 주입)
│   └── admin.html                  # 관리자 모니터 (사이드바 + 스테이션 선택)
├── static/
│   ├── kiosk.js                    # 키오스크 API 연동 (station_id 기반)
│   └── admin.js                    # 관리자 API 연동 (지점/스테이션 선택)
├── captures/                       # 비전 캡처 이미지 저장
└── detections.db                   # SQLite DB (자동 생성)
```

## 실행 방법
```bash
cd web
pip install flask
python3 app.py
# 로그인: http://localhost:5000/login
# 관리자: admin / admin
# 고객 키오스크 (1번 지점 1번 스테이션): http://localhost:5000/kiosk/1/1
# 관리자 화면: http://localhost:5000/admin
```
