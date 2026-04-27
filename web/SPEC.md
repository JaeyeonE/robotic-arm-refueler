# AutoJet Refueling System — 함수/테이블 명세서

본 문서는 `web/` 디렉토리의 핵심 파일 4종(`app.py`, `database.py`, `logger.py`)과
`vision/post_vision.py`, 그리고 SQLite DB(`detections.db`)에 정의된
**모든 호출 가능한 함수, API 라우트, 테이블 스키마**에 대한 명세를 정리한다.

---

## 0. 시스템 토폴로지

```
┌─────────────────────┐  TCP 12345    ┌───────────────────────┐
│ Doosan E0509 컨트롤 │ ◄──────────►  │ ROS2 Humble (host PC) │
│   (110.120.1.52)    │                │  dsr_bringup2 launch  │
└─────────────────────┘                └──────────┬────────────┘
                                                  │ (예정: HTTP POST)
┌─────────────────────┐                           ▼
│ RealSense D455      │                ┌──────────────────────┐
│   + YOLOv8          │ ──HTTP POST──► │   Flask  app.py      │
│ post_vision.py      │  /api/vision/* │   :5000              │
└─────────────────────┘                │                      │
                                        │  ┌──────────────┐   │
              브라우저 ───── HTTP ─────►│  │ database.py  │   │
              (kiosk/admin)             │  │ logger.py    │   │
                                        │  └──────┬───────┘   │
                                        └─────────┼───────────┘
                                                  ▼
                                         ┌────────────────┐
                                         │ detections.db  │
                                         │ (SQLite, WAL)  │
                                         └────────────────┘
```

---

## 1. DB 테이블 스키마 (`detections.db`, 7테이블)

### 1.1 `users`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 사용자 ID |
| `username` | TEXT | NOT NULL, UNIQUE | 로그인 아이디 |
| `password_hash` | TEXT | NOT NULL | werkzeug.security 해시 |
| `name` | TEXT | NOT NULL | 표시용 이름 |
| `role` | TEXT | NOT NULL DEFAULT `'customer'` | `customer` / `admin` |
| `phone` | TEXT |  | 선택 |
| `created_at` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |

**관계**: `fueling_tasks.user_id → users.id` (1:N)

### 1.2 `branches`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 지점 ID |
| `name` | TEXT | NOT NULL | 지점명 (예: "본점") |
| `address` | TEXT |  | 주소 |
| `created_at` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |

**관계**: `stations.branch_id → branches.id` (1:N)

### 1.3 `stations`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 스테이션 ID |
| `branch_id` | INTEGER | NOT NULL FK → branches(id) | 소속 지점 |
| `station_no` | INTEGER | NOT NULL | 지점 내 번호 (1, 2, ...) |
| `robot_model` | TEXT | DEFAULT `'E0509'` | 로봇 모델명 |
| `camera_model` | TEXT | DEFAULT `'D455'` | 카메라 모델명 |
| `status` | TEXT | DEFAULT `'idle'` | `idle` / `busy` / `offline` |
| `created_at` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |
| **UNIQUE** | (`branch_id`, `station_no`) |  | 지점별 번호 중복 금지 |

**관계**: `fueling_tasks.station_id`, `robot_snapshots.station_id`, `vision_detections.station_id`, `logs.station_id` 모두 `stations.id`를 FK로 참조.

### 1.4 `fueling_tasks`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 작업 ID |
| `station_id` | INTEGER | NOT NULL FK → stations(id) | 작업 수행 스테이션 |
| `user_id` | INTEGER | FK → users(id) | 요청 고객 |
| `fuel_type` | TEXT | NOT NULL | `gasoline` / `diesel` / `lpg` / `electric` |
| `input_mode` | TEXT | NOT NULL | `amount` (₩) / `volume` (L) |
| `target_value` | REAL | NOT NULL | 목표 금액 또는 리터 |
| `current_step` | INTEGER | DEFAULT 0 | FSM 단계 0~4 |
| `step_progress` | REAL | DEFAULT 0 | 단계별 진행률 (0.0~1.0) |
| `status` | TEXT | DEFAULT `'pending'` | `pending` / `running` / `success` / `estop` / `failed` |
| `liters` | REAL | DEFAULT 0 | 누적 주유량(L) |
| `cost` | INTEGER | DEFAULT 0 | 누적 금액(원) |
| `started_at` | DATETIME |  | UTC ISO |
| `finished_at` | DATETIME |  | UTC ISO |
| `created_at` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |

**FSM Step 정의**: 0=대기, 1=탐색, 2=접근, 3=주유, 4=복귀

### 1.5 `robot_snapshots`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 스냅샷 ID |
| `station_id` | INTEGER | NOT NULL FK → stations(id) | 어느 스테이션 로봇인가 |
| `task_id` | INTEGER | FK → fueling_tasks(id) | 연관 작업 (없을 수도) |
| `j1_angle` ~ `j6_angle` | REAL × 6 |  | 관절 1~6 각도(°) |
| `j1_torque` ~ `j6_torque` | REAL × 6 |  | 관절 1~6 토크(Nm) |
| `tcp_x`, `tcp_y`, `tcp_z` | REAL × 3 |  | TCP 위치(mm) |
| `tcp_a`, `tcp_b`, `tcp_c` | REAL × 3 |  | TCP 자세(°) |
| `robot_mode` | TEXT | DEFAULT `'IDLE'` | `IDLE` / `MOVING` / `FUELING` / `ERROR` / `ESTOP` |
| `dart_connected` | INTEGER | DEFAULT 0 | bool (0/1) |
| `timestamp` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |

### 1.6 `vision_detections`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 인식 ID |
| `station_id` | INTEGER | NOT NULL FK → stations(id) | 어느 카메라 |
| `task_id` | INTEGER | FK → fueling_tasks(id) | 연관 작업 |
| `label` | TEXT | NOT NULL | YOLO 클래스 (`gas_cap`, `cap_handle`, `refusing`, ...) |
| `confidence` | REAL | NOT NULL | 0.0 ~ 1.0 |
| `bbox_x1`, `bbox_y1`, `bbox_x2`, `bbox_y2` | INTEGER × 4 |  | 바운딩 박스(px) |
| `center_u`, `center_v` | INTEGER × 2 |  | 박스 중심(px) |
| `depth` | REAL |  | RealSense depth(m) |
| `robot_x`, `robot_y`, `robot_z` | REAL × 3 |  | 로봇 베이스 좌표(mm) |
| `handle_angle` | REAL |  | cap_handle PCA 각도(-90~90°) |
| `image_path` | TEXT |  | 캡처 이미지 경로 (선택) |
| `timestamp` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |

### 1.7 `logs`
| 컬럼 | 타입 | 제약 | 설명 |
|------|------|------|------|
| `id` | INTEGER | PK, AUTOINCREMENT | 로그 ID |
| `station_id` | INTEGER | FK → stations(id) | 발생 스테이션 (없을 수도) |
| `task_id` | INTEGER |  | 연관 작업 (FK 미설정) |
| `level` | TEXT | NOT NULL DEFAULT `'INFO'` | `OK` / `INFO` / `WARN` / `WARNING` / `ERROR` |
| `source` | TEXT | DEFAULT `'system'` | `system` / `vision` / `control` / `kiosk` / `app` |
| `message` | TEXT | NOT NULL | 로그 본문 |
| `timestamp` | DATETIME | DEFAULT CURRENT_TIMESTAMP | UTC |

### 1.8 ER 다이어그램 요약

```
users 1 ─────< fueling_tasks >─────── 1 stations >────── 1 branches
                    │                       │
                    │                       ├─── 1:N robot_snapshots
                    │                       ├─── 1:N vision_detections
                    │                       └─── 1:N logs
                    │
                    └─── 1:N robot_snapshots / vision_detections / logs (task_id)
```

PRAGMA: `journal_mode=WAL`, `foreign_keys=ON`.

---

## 2. `database.py` 함수 명세

### 2.1 연결 / 초기화

#### `get_conn() → sqlite3.Connection`
SQLite 연결 반환. `row_factory = sqlite3.Row`, WAL 모드, FK 강제.
- **반환**: 매 호출마다 **새 connection** (호출자가 close 책임)

#### `init_db() → None`
7개 테이블을 `IF NOT EXISTS`로 생성. 진입점에서 1회만 호출.
- **부작용**: `detections.db` 파일 생성/스키마 보강
- **호출처**: `app.py:524` (`if __name__ == "__main__"` 블록)

### 2.2 users

#### `create_user(username, password, name, role="customer", phone=None) → int`
| 인자 | 타입 | 설명 |
|------|------|------|
| `username` | str | 로그인 ID (UNIQUE) |
| `password` | str | 평문, 내부에서 `generate_password_hash` 적용 |
| `name` | str | 표시명 |
| `role` | str | `'customer'` 또는 `'admin'` |
| `phone` | str/None | 선택 |
- **반환**: 신규 user_id (`INTEGER`)
- **예외**: `username` 중복 시 `sqlite3.IntegrityError`

#### `authenticate_user(username, password) → dict | None`
- 입력 username으로 조회 후 `check_password_hash`로 검증
- **반환**: 성공 시 `users` row dict (password_hash 포함 주의), 실패 시 `None`

#### `get_user(user_id) → dict | None`
- **반환**: `password_hash` 제외한 컬럼만 (`id, username, name, role, phone, created_at`)

### 2.3 branches

#### `create_branch(name, address=None) → int`
- **반환**: 신규 branch_id

#### `get_branches() → list[dict]`
- 모든 지점을 `id` 오름차순 반환

### 2.4 stations

#### `create_station(branch_id, station_no, robot_model="E0509", camera_model="D455") → int`
- **예외**: `(branch_id, station_no)` UNIQUE 위반 시 `IntegrityError`
- **반환**: 신규 station_id

#### `get_stations(branch_id) → list[dict]`
- 해당 지점의 모든 스테이션을 `station_no` 오름차순 반환

#### `get_station(station_id) → dict | None`
- 단일 스테이션 조회 (PK)

#### `get_station_by_branch(branch_id, station_no) → dict | None`
- 자연키 (`branch_id`, `station_no`)로 조회. 키오스크 라우트(`/kiosk/<b>/<s>`)에서 사용

#### `update_station_status(station_id, status) → None`
- `status` 컬럼만 UPDATE. 값: `'idle'` / `'busy'` / `'offline'`

### 2.5 fueling_tasks

#### `create_task(station_id, user_id, fuel_type, input_mode, target_value) → int`
- INSERT 시 `status='running'`, `started_at=CURRENT_TIMESTAMP` 자동 설정
- **반환**: 신규 task_id

#### `update_task(task_id, **fields) → None`
- 가변 키워드 인자로 임의 필드 UPDATE (동적 SQL 조립)
- **호출 예시**: `update_task(5, liters=2.3, cost=4600, current_step=3)`
- **주의**: 키 이름이 컬럼명과 일치해야 함 (검증 없음)

#### `get_current_task(station_id=None) → dict | None`
- `status IN ('pending','running')`인 가장 최근 작업
- `station_id` 미지정 시 전 스테이션 통틀어 1개

#### `get_task(task_id) → dict | None`
- PK 단건 조회

#### `get_task_history(station_id=None, limit=50, offset=0) → list[dict]`
- `created_at DESC` 정렬, 페이지네이션 지원

### 2.6 robot_snapshots

#### `insert_robot_snapshot(data: dict) → None`
**필수 키**: `station_id`
**선택 키**: `task_id`, `j{1..6}_angle`, `j{1..6}_torque`, `tcp_{x,y,z,a,b,c}`, `robot_mode`, `dart_connected`
- 키가 빠지면 `data.get(...)`로 `None` 입력
- `robot_mode` 기본 `'IDLE'`, `dart_connected` 기본 `0`

#### `get_latest_snapshot(station_id=None) → dict | None`
- `id DESC LIMIT 1`. 화면 폴링에 사용

### 2.7 vision_detections

#### `insert_detection(data: dict) → int`
**필수 키**: `station_id`, `label`, `confidence`
**선택 키**: `task_id`, `bbox_*`, `center_u/v`, `depth`, `robot_{x,y,z}`, `handle_angle`, `image_path`
- **반환**: 신규 detection id

#### `get_latest_detection(station_id=None) → dict | None`
- 최신 1건. `loadVision()` 폴링 + `_fueling_loop()` 트리거에 사용

#### `get_detection_history(station_id=None, task_id=None, limit=100) → list[dict]`
- 동적 WHERE 절 조립. `id DESC` 정렬

### 2.8 logs

#### `insert_log(level, message, source="system", station_id=None, task_id=None) → None`
- 위치 인자 순서 주의: `(level, message, source, station_id, task_id)`
- **호출 예시**: `insert_log("ERROR", "E-STOP", source="control", station_id=1, task_id=42)`

#### `get_logs(station_id=None, level=None, source=None, task_id=None, limit=50) → list[dict]`
- 모든 인자 None 가능 (전체 조회). `id DESC` 정렬
- 동적 WHERE 절: 지정된 인자만 필터

---

## 3. `logger.py` 함수 명세

### 3.1 `class DBLogHandler(logging.Handler)`

#### `emit(self, record: logging.LogRecord) → None`
표준 logging 호출(`logger.info(...)`)을 SQLite `logs` 테이블에 영속화.
- `record.levelname` → `level`
- `self.format(record)` → `message`
- `getattr(record, "source", record.name)` → `source` (기본은 logger 이름)
- `getattr(record, "station_id", None)`, `getattr(record, "task_id", None)` → 컬럼

**확장 필드 사용 예**:
```python
logger.info("주유 시작", extra={"source": "kiosk", "station_id": 1, "task_id": 42})
```
- 예외 발생 시 `self.handleError(record)` 호출 (logging 표준 패턴)

### 3.2 `setup_logger(name="app", level=logging.INFO) → logging.Logger`
- `StreamHandler` (콘솔, `[시각] LEVEL 메시지` 포맷)와 `DBLogHandler`를 동시 부착
- 멱등성: `logger.handlers`가 비어있을 때만 추가 (재호출해도 핸들러 중복 안 됨)
- **반환**: 설정된 logger 객체

---

## 4. `app.py` 라우트 / 함수 명세

### 4.1 전역 / 초기화

| 객체 | 값 | 설명 |
|------|------|------|
| `app` | `Flask(__name__)` | Flask 인스턴스 |
| `app.secret_key` | env `SECRET_KEY` 또는 `'autojet-dev-key-change-in-prod'` | 세션 서명용 |
| `logger` | `setup_logger("app")` | 모듈 로거 |
| `CAPTURES_DIR` | `web/captures/` | 캡처 이미지 저장 디렉토리 (자동 생성) |
| `_frame_lock` | `threading.Lock()` | `_latest_frames` 동시 접근 보호 |
| `_latest_frames` | `dict[int, bytes]` | `{station_id: JPEG bytes}` 메모리 버퍼 |
| `FUEL_PRICE` | `2000` | 1L당 원 |
| `TICK_INTERVAL` | `0.05` | 주유 시뮬레이션 틱 (초) |
| `TICK_VOLUME` | `0.05` | 틱당 증가 리터 (= 1L/sec) |

### 4.2 인증 데코레이터

#### `login_required(f)`
- 세션에 `user_id` 없으면:
  - JSON 요청 또는 `/api/`로 시작하는 path → `401 {"error": "로그인 필요"}`
  - 그 외 → `/login`으로 redirect

#### `admin_required(f)`
- `login_required` 검사 후 `session["role"] != "admin"`이면 `403 {"error": "관리자 권한 필요"}`

### 4.3 백그라운드 스레드: `_fueling_loop()`

데몬 스레드, 50ms 주기로 다음을 수행:

1. `get_current_task()`로 현재 진행 중(`running`) 작업 1건 조회
2. 해당 스테이션의 `get_latest_detection()`을 가져와 `label == "refusing"`인지 확인
3. 조건 충족 시:
   - `liters += 0.05`, `cost = liters × FUEL_PRICE`
   - 목표(`amount` / `volume`) 도달 시 `status='success'`, `current_step=4`, `finished_at=UTC now`로 종료 + `OK` 로그
   - 미도달 시 `current_step=3` 유지

**주의**: 어떤 예외도 무시(`except Exception: pass`) — 폴링 안정성 우선.

### 4.4 페이지 라우트

| 메서드 | 경로 | 인증 | 핸들러 | 동작 |
|--------|------|------|--------|------|
| GET | `/` | - | `index()` | 로그인 안 됨 → `/login`, admin → `/admin`, customer → `/login` (키오스크 직링크 유도) |
| GET | `/login` | - | `login_page()` | `templates/login.html` 렌더 |
| GET | `/kiosk/<int:branch_id>/<int:station_no>` | login | `kiosk_page()` | `get_station_by_branch()`로 station 조회 후 `kiosk.html`에 `station_id, branch_id, station_no` 주입. 없으면 404 |
| GET | `/admin` | admin | `admin_page()` | `templates/admin.html` 렌더 |
| GET | `/captures/<path:filename>` | - | `serve_capture()` | `CAPTURES_DIR`에서 정적 파일 서빙 |

### 4.5 인증 API (`/api/auth/*`)

#### `POST /api/auth/register`
**Body**: `{"username","password","name","role"?,"phone"?}`
- 누락 → `400`
- 중복 username → `409 {"error": "이미 존재하는 사용자명입니다"}`
- 성공 → `201 {"ok": true, "user_id": int}`

#### `POST /api/auth/login`
**Body**: `{"username","password"}`
- 실패 → `401`
- 성공 → 세션에 `user_id, role, name` 저장. `200 {"ok": true, "user_id", "role", "name"}`

#### `POST /api/auth/logout`
- `session.clear()` 후 `200 {"ok": true}`

#### `GET /api/auth/me` *(login 필요)*
- 현재 세션의 사용자 정보 반환. 사용자 사라졌으면 세션 클리어 후 `401`

### 4.6 지점/스테이션 API

| 메서드 | 경로 | 인증 | 동작 |
|--------|------|------|------|
| GET | `/api/branches` | login | `get_branches()` 결과 JSON |
| POST | `/api/branches` | admin | Body `{"name","address"?}` → 신규 지점 생성, `201 {"ok","branch_id"}` |
| GET | `/api/branches/<int:branch_id>/stations` | login | 해당 지점의 스테이션 목록 |
| POST | `/api/stations` | admin | Body `{"branch_id","station_no","robot_model"?,"camera_model"?}` → 신규 스테이션, `201` |

### 4.7 시스템 상태 API

#### `GET /api/system/status`
**Query**: `station_id` (선택)
**Response**:
```json
{
  "robot_mode": "IDLE|MOVING|...|OFFLINE",
  "dart_connected": true,
  "station_status": "idle|busy|offline|null",
  "current_task_id": 42,
  "current_step": 3,
  "task_status": "running"
}
```
- snapshot 없으면 `robot_mode="OFFLINE"`, `dart_connected=false`

### 4.8 로봇 상태 API

#### `GET /api/robot/joints`
**Query**: `station_id`
**Response**: `{"joints":[{"label":"J1","angle":...,"torque":...},...×6]}` (snapshot 없으면 `joints: []`)

#### `GET /api/robot/tcp`
**Query**: `station_id`
**Response**: `{"tcp":{"x","y","z","a","b","c"}}` (없으면 빈 dict)

#### `POST /api/robot/snapshot`
**Body**: `insert_robot_snapshot()` 입력 dict (필수 `station_id`)
- ROS2 측에서 주기적으로 호출 예정
- `201 {"ok": true}`

### 4.9 비전 API

#### `GET /api/vision/current`
**Query**: `station_id`
- `get_latest_detection()` 반환 그대로 JSON. 없으면 빈 객체

#### `POST /api/vision/detect`
**Body**: `insert_detection()` 입력 dict (필수 `station_id, label, confidence`)
- `201 {"ok": true, "id": int}`

#### `POST /api/vision/frame/<int:station_id>`
**Body**: raw JPEG bytes (`Content-Type: image/jpeg`)
- `_latest_frames[station_id] = request.data` (메모리만)
- `200 {"ok": true}`

#### `GET /api/vision/frame/<int:station_id>`
- 메모리에 프레임 있으면 `200 image/jpeg`, 없으면 `204 No Content`

#### `POST /api/vision/capture/<int:station_id>`
- 현재 메모리 프레임을 `captures/s{sid}_{YYYYMMDD_HHMMSS}.jpg`로 저장
- 프레임 없으면 `404`
- 성공: `201 {"ok","filename","path":"/captures/..."}`

#### `GET /api/vision/history`
**Query**: `station_id?`, `task_id?`, `limit=100`
- `get_detection_history()` 결과 배열

### 4.10 주유 작업 API

#### `POST /api/task/start` *(login 필요)*
**Body**: `{"station_id","fuel_type","input_mode","target_value"}` 모두 필수
- `create_task()` 호출, `INFO` 로그 기록
- `201 {"ok": true, "task_id": int}`

#### `POST /api/task/estop` *(login 필요)*
**Body**: `{"station_id"}`
- 진행 중 작업이 있으면 `status='estop'`, `finished_at=UTC now`로 마감
- `ERROR` 레벨 / `source=control` 로그 기록
- `200 {"ok": true, "stopped_task_id": int|null}`

#### `GET /api/task/current`
**Query**: `station_id`
- 현재 진행 중 작업 dict, 없으면 빈 객체

#### `GET /api/task/metrics`
**Query**: `station_id`
**Response**:
```json
{
  "task_id": 42,
  "liters": 1.25,
  "cost": 2500,
  "elapsed_sec": 7,
  "remaining_sec": 5,
  "fuel_type": "gasoline",
  "target_value": 5000
}
```
- `started_at`을 UTC로 파싱해 경과 초 계산 (timezone bug 방지)
- `remaining_sec`는 남은 리터 ≈ 남은 초 (1L/sec 시뮬레이션 가정)

#### `PATCH /api/task/<int:task_id>`
**Body**: `current_step`, `step_progress`, `status`, `liters`, `cost`, `finished_at` 중 일부
- 화이트리스트 외 키는 무시
- 빈 body → `400`. 정상 → `200 {"ok": true}`

#### `GET /api/task/history`
**Query**: `station_id?`, `limit=50`, `offset=0`
- `get_task_history()` 결과 배열

### 4.11 로그 API

#### `POST /api/logs`
**Body**: `{"level"?,"message"?,"source"?,"station_id"?,"task_id"?}`
- 모두 선택. 미지정 시 `level='INFO'`, `source='system'`, `message=''`
- `201 {"ok": true}`

#### `GET /api/logs`
**Query**: `station_id?`, `level?`, `source?`, `task_id?`, `limit=50`
- `get_logs()` 결과 배열 (`id DESC`)

### 4.12 시드 / 진입점

#### `seed_defaults() → None`
- branches 비어있으면 `'본점' / 서울시 강남구'` 추가
- stations 비어있으면 `(branch_id=1, station_no=1)` 추가
- admin role user 없으면 `admin / admin / 관리자` 생성

#### `if __name__ == "__main__":` 블록
1. `init_db()`
2. `seed_defaults()`
3. `insert_log("INFO", "Flask 서버 시작", source="system")`
4. `app.run(host="0.0.0.0", port=5000, debug=True)`

---

## 5. `vision/post_vision.py` 함수 / 흐름 명세

### 5.1 전역 설정

| 변수 | 값 | 설명 |
|------|------|------|
| `model` | `YOLO('20260415_1149.pt')` | 가중치 파일 |
| `API_URL` | `"http://localhost:5000"` | Flask 서버 |
| `STATION_ID` | `1` | 이 카메라가 연결된 스테이션 |
| `POST_INTERVAL` | `1.0` | detection POST 주기(초) |
| `CONF_THRESHOLD` | `0.80` | YOLO 신뢰도 컷 |
| `SAVE_DIR` | `"captures"` | (사용 안 함, 호환용) |
| `R_cam2robot` | `np.ndarray (3×3)` | 카메라 → 로봇 베이스 회전 행렬 |
| `t_cam2robot` | `np.array([420, 65, 80])` | 평행이동 (mm) |

RealSense pipeline은 640×480 / 30fps (depth z16, color bgr8) + `align(rs.stream.color)`.

### 5.2 함수

#### `get_handle_angle(roi: np.ndarray) → float | None`
ROI 내 검은 막대(주유 캡 손잡이)의 PCA 기반 각도 검출.

| 단계 | 처리 |
|------|------|
| 1 | `cv2.cvtColor(BGR→GRAY)` |
| 2 | `GaussianBlur(5×5)` |
| 3 | `threshold(80, INV)` — 어두운 영역만 1 |
| 4 | morphology OPEN+CLOSE (3×3 kernel) — 노이즈 제거 |
| 5 | `findContours(EXTERNAL, SIMPLE)` |
| 6 | 가장 큰 contour 선택 (면적 < 50이면 `None` 반환) |
| 7 | `cv2.PCACompute2`로 주성분 벡터 `(vx, vy)` 추출 |
| 8 | `math.atan2(vy, vx)` → 각도(°), `[-90, 90]` 범위로 정규화 |

**반환**: `round(angle, 1)` 또는 `None`

#### `pixel_to_robot(u, v, depth_m) → np.ndarray (3,)`
- `rs2_deproject_pixel_to_point(intrinsics, [u,v], depth)` → 카메라 좌표(m)
- `× 1000` → mm 단위
- `R_cam2robot @ point + t_cam2robot` → 로봇 베이스 좌표
- **반환**: `[rx, ry, rz]` (mm)

### 5.3 메인 루프 (while True)

매 프레임마다:

1. `pipeline.wait_for_frames()` → align → depth/color 프레임 추출
2. `model(color_image)` → YOLO 추론 (verbose=False)
3. 각 box에 대해:
   - `conf < 0.80` skip
   - `(u,v) = bbox 중심`, `depth_val = depth_frame.get_distance(u,v)`
   - `pixel_to_robot()`으로 `(rx, ry, rz)` 산출
   - `label == "cap_handle"`이면 8px 패딩 ROI에 `get_handle_angle()` 호출
   - `detected_objects` 리스트에 dict로 push (label, conf, bbox, pixel, depth, robot_xyz, handle_angle)
   - 화면에 bbox + 정보 텍스트 그림
4. **1초 주기 (POST_INTERVAL)**:
   - `detected_objects` 각 obj에 대해 `POST /api/vision/detect` (timeout 0.3s, 실패 silent)
5. **매 프레임**:
   - `cv2.imencode('.jpg', display, JPEG_QUALITY=80)` → bytes
   - `POST /api/vision/frame/{STATION_ID}` (Content-Type: image/jpeg, timeout 0.3s)
6. `time.sleep(0.03)` (~30fps 유지)

`finally`: `pipeline.stop()`

### 5.4 이 스크립트가 채우는 DB 컬럼 매핑

`POST /api/vision/detect` body → `vision_detections` row
- `station_id` → `station_id`
- `label` → `label`
- `confidence` → `confidence` (key 이름 `conf` → `confidence`)
- `bbox_x1..y2` → `bbox_x1..y2`
- `center_u, center_v` → `center_u, center_v`
- `depth` → `depth`
- `robot_x, robot_y, robot_z` → `robot_x, robot_y, robot_z`
- `handle_angle` → `handle_angle`

---

## 6. 데이터 흐름 요약

### 6.1 비전 → DB → 화면

```
RealSense + YOLO ─► POST /api/vision/detect ─► insert_detection() ─► vision_detections
                                                                          │
                                                                          ▼
admin.js loadVision()  ◄─ GET /api/vision/current ◄── get_latest_detection()
```

### 6.2 카메라 프레임 (DB 미경유)

```
post_vision.py ─► POST /api/vision/frame/<sid> ─► _latest_frames[sid] (RAM)
                                                       │
                                                       ▼
브라우저 <img>  ◄── GET /api/vision/frame/<sid>?t=...
```

### 6.3 주유 시뮬레이션 트리거

```
"refusing" detection 도착
        │
        ▼
_fueling_loop() (50ms 주기)
        │
        ├─► update_task(liters+=0.05, cost=liters×2000)
        │
        └─► 목표 도달 시 status='success', step=4, finished_at=UTC now
                            + insert_log("OK", "주유 완료...", source="kiosk")
```

### 6.4 로깅 경로 (이중 경로)

```
(A) 직접 호출:  insert_log(level, msg, source, station_id, task_id)
(B) logging:    logger.info(msg, extra={"source":..., "station_id":...})
                       └─► DBLogHandler.emit() ─► insert_log(...)
                       └─► StreamHandler ─► stdout

둘 다 종착지: logs 테이블
```

---

## 7. 인증 / 세션 흐름

```
브라우저          Flask                          DB
   │                │                              │
   │─POST /login──►│─authenticate_user(u,p)──────►│
   │                │◄────users row────────────────│
   │                │ session["user_id"|"role"|"name"] = ...
   │◄──Set-Cookie──│
   │                │
   │─GET /admin───►│ @admin_required → role=='admin' OK
   │◄─admin.html───│
   │
   │─GET /api/branches─► @login_required → 통과 → JSON
```

기본 시드 계정: **admin / admin**, role=admin
키오스크 URL: `/kiosk/1/1` (본점 1번 스테이션)

---

## 8. 폴링 주기 (브라우저 ↔ Flask)

| 클라이언트 | 호출 | 주기 |
|-----------|------|------|
| admin.js | `loadFrame()` | 1.0s |
| admin.js | `loadRobot()` | 1.5s |
| admin.js | `loadVision()` | 2.0s |
| admin.js | `loadStatus()` | 3.0s |
| admin.js | `loadLogs()` | 3.0s |
| post_vision.py | `POST /api/vision/frame` | 매 프레임 (~30fps) |
| post_vision.py | `POST /api/vision/detect` | 1.0s |
| `_fueling_loop()` (서버) | 작업 진행 | 50ms |

---

## 9. 향후 ROS2 연동 시 추가 호출 예정

- `POST /api/robot/snapshot`: ROS2 노드가 `/joint_states`, TCP 토픽 구독해서 100Hz → 서버 측 throttle 필요 시 1Hz로 다운샘플 후 전송
- `PATCH /api/task/<id>`: FSM step 진행마다 `current_step`, `step_progress` 갱신
- `POST /api/task/estop`: 컨트롤러 에러 / 충돌 감지 시 ROS 측에서도 호출 가능

---

*문서 생성 시각 기준 컬럼/엔드포인트 목록 일치 확인 완료.*
