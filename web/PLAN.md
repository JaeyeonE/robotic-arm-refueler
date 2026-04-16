# 구현 계획 v2 — AutoJet Refueling Web Layer

## 개요

사용자/지점/스테이션 관리 체계 도입 + 정규화 DB 재설계.

- 고객: 로그인 → 스테이션 고정 키오스크에서 주유
- 관리자: 로그인 → 전체 지점/스테이션 모니터링

---

## Phase 1: 정규화 DB 스키마 재설계

### ERD 관계도

```
users ─────┐
           │ user_id (FK)
branches ──┤
  │        │
  │ branch_id (FK)
  ▼        │
stations ──┤
  │        │ station_id (FK)
  │ station_id (FK)
  ▼        ▼
fueling_tasks ◄── robot_snapshots
      ▲           vision_detections
      │           logs
      └── task_id (FK)
```

### 1-1. users — 사용자

```sql
CREATE TABLE users (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    username      TEXT NOT NULL UNIQUE,
    password_hash TEXT NOT NULL,
    name          TEXT NOT NULL,
    role          TEXT NOT NULL DEFAULT 'customer',  -- customer / admin
    phone         TEXT,
    created_at    DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### 1-2. branches — 지점

```sql
CREATE TABLE branches (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT NOT NULL,              -- '강남점', '판교점'
    address    TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### 1-3. stations — 주유 스테이션 (로봇 + 카메라 1세트)

```sql
CREATE TABLE stations (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    branch_id     INTEGER NOT NULL REFERENCES branches(id),
    station_no    INTEGER NOT NULL,         -- 지점 내 번호 (1, 2, 3...)
    robot_model   TEXT DEFAULT 'E0509',
    camera_model  TEXT DEFAULT 'D455',
    status        TEXT DEFAULT 'idle',      -- idle / busy / offline / error
    created_at    DATETIME DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(branch_id, station_no)
);
```

### 1-4. fueling_tasks — 주유 작업

```sql
CREATE TABLE fueling_tasks (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    station_id    INTEGER NOT NULL REFERENCES stations(id),
    user_id       INTEGER REFERENCES users(id),
    fuel_type     TEXT NOT NULL,
    input_mode    TEXT NOT NULL,
    target_value  REAL NOT NULL,
    current_step  INTEGER DEFAULT 0,
    step_progress REAL DEFAULT 0,
    status        TEXT DEFAULT 'pending',
    liters        REAL DEFAULT 0,
    cost          INTEGER DEFAULT 0,
    started_at    DATETIME,
    finished_at   DATETIME,
    created_at    DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### 1-5. robot_snapshots — 로봇 상태

```sql
CREATE TABLE robot_snapshots (
    id             INTEGER PRIMARY KEY AUTOINCREMENT,
    station_id     INTEGER NOT NULL REFERENCES stations(id),
    task_id        INTEGER REFERENCES fueling_tasks(id),
    j1_angle REAL, j2_angle REAL, j3_angle REAL,
    j4_angle REAL, j5_angle REAL, j6_angle REAL,
    j1_torque REAL, j2_torque REAL, j3_torque REAL,
    j4_torque REAL, j5_torque REAL, j6_torque REAL,
    tcp_x REAL, tcp_y REAL, tcp_z REAL,
    tcp_a REAL, tcp_b REAL, tcp_c REAL,
    robot_mode     TEXT DEFAULT 'IDLE',
    dart_connected INTEGER DEFAULT 0,
    timestamp      DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### 1-6. vision_detections — 비전 인식

```sql
CREATE TABLE vision_detections (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    station_id  INTEGER NOT NULL REFERENCES stations(id),
    task_id     INTEGER REFERENCES fueling_tasks(id),
    label       TEXT NOT NULL,
    confidence  REAL NOT NULL,
    bbox_x1 INTEGER, bbox_y1 INTEGER,
    bbox_x2 INTEGER, bbox_y2 INTEGER,
    center_u INTEGER, center_v INTEGER,
    depth       REAL,
    robot_x REAL, robot_y REAL, robot_z REAL,
    image_path  TEXT,
    timestamp   DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### 1-7. logs — 시스템 로그

```sql
CREATE TABLE logs (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    station_id INTEGER REFERENCES stations(id),
    task_id    INTEGER,
    level      TEXT NOT NULL DEFAULT 'INFO',
    source     TEXT DEFAULT 'system',
    message    TEXT NOT NULL,
    timestamp  DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

---

## Phase 2: 인증 + 페이지 라우팅

### 2-1. 로그인/세션

```
GET  /login                → login.html
POST /api/auth/login       → {username, password} → 세션 생성, role 반환
POST /api/auth/logout      → 세션 제거
GET  /api/auth/me          → 현재 로그인 사용자 정보
```

- Flask `session` 기반 (서버사이드 쿠키)
- role='customer' → /kiosk 로 리다이렉트
- role='admin' → /admin 으로 리다이렉트
- 미인증 접근 시 /login 으로 리다이렉트

### 2-2. 키오스크 스테이션 고정

- 키오스크 URL: `/kiosk?station_id=3` 또는 서버 config로 고정
- 고객은 스테이션 선택 불가 (물리적 배치 기준)
- 주유 시작 시 session의 user_id + URL의 station_id가 task에 자동 매핑

### 2-3. 관리자 지점/스테이션 선택

- admin.html 좌측에 지점 드롭다운 + 스테이션 목록
- 선택한 station_id 기준으로 모든 데이터 필터링
- 전체 지점 조회 가능

```
GET /api/branches                    → 지점 목록
GET /api/branches/<id>/stations      → 지점별 스테이션 목록
```

---

## Phase 3: API 계층 개편

### 기존 API 변경점

모든 데이터 API에 `station_id` 필터 추가:

```
GET  /api/system/status?station_id=3
GET  /api/robot/joints?station_id=3
GET  /api/robot/tcp?station_id=3
POST /api/robot/snapshot              → body에 station_id 필수
GET  /api/vision/current?station_id=3
POST /api/vision/detect               → body에 station_id 필수
POST /api/vision/frame/<station_id>   → 스테이션별 프레임
GET  /api/vision/frame/<station_id>
POST /api/vision/capture/<station_id>
GET  /api/task/current?station_id=3
GET  /api/task/metrics?station_id=3
POST /api/task/start                  → body에 station_id + user_id 자동 주입
GET  /api/logs?station_id=3
```

### 신규 API

```
GET  /api/branches                    → 전체 지점 목록
GET  /api/branches/<id>/stations      → 지점 내 스테이션 목록
POST /api/branches                    → 지점 추가 (admin)
POST /api/stations                    → 스테이션 추가 (admin)
POST /api/auth/register               → 회원가입
```

---

## Phase 4: 프론트엔드 개편

### 4-1. login.html (신규)
- 심플 로그인 폼 (username + password)
- 로그인 성공 시 role 기반 리다이렉트

### 4-2. kiosk.html 변경
- station_id를 URL 파라미터 또는 서버에서 주입
- 모든 API 호출에 station_id 포함
- 상단에 현재 지점명 + 스테이션 번호 표시

### 4-3. admin.html 변경
- 좌측 사이드바: 지점 선택 드롭다운 → 스테이션 카드 목록
- 스테이션 선택 시 우측 패널에 기존 모니터링 데이터 표시
- 전체 스테이션 상태 한눈에 보기 (idle/busy/offline 배지)

---

## Phase 5: 외부 노드 연동 변경

post_vision.py, 향후 post_robot.py 등에서:
- `station_id`를 설정값으로 보유
- POST 시 `station_id` 포함하여 전송

---

## 구현 순서

1. `database.py` 재설계 (7개 테이블 + 마이그레이션)
2. `app.py` 인증 로직 + 세션 관리
3. `login.html` + `login.js` 작성
4. 기존 API에 station_id 필터 적용
5. 신규 API (branches, stations, auth) 구현
6. `admin.html` 좌측 사이드바 + 스테이션 선택 UI
7. `kiosk.html` station_id 연동
8. `post_vision.py` station_id 추가
9. `db_clear.py` + seed 스크립트 갱신
10. 통합 테스트
