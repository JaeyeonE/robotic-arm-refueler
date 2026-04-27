# web — AutoJet Refueling System (Web Layer)

Flask 기반 백엔드 + 키오스크/관리자 웹 UI.
ROS 2 ↔ Web 통신은 `ui_gateway_node` (HTTP bridge)를 거쳐 흐른다.

---

## 폴더 구조

```text
web/
├── app.py                          # Flask 메인 (인증/페이지/API)
├── database.py                     # SQLite 스키마 + 헬퍼
├── logger.py                       # logging → DB 핸들러
├── db_clear.py                     # DB 데이터 초기화
├── extract_session.py              # 세션별 CSV 덤프
├── extract_target.py               # 타겟/trajectory 덤프
├── customer_kiosk_light.html       # 키오스크 원본 디자인
├── admin_monitor_light.html        # 관리자 원본 디자인
├── templates/
│   ├── login.html
│   ├── kiosk.html
│   └── admin.html
├── static/
│   ├── kiosk.js
│   └── admin.js
├── captures/                       # 비전 캡처 이미지
├── extraction/                     # 추출 CSV 결과
├── README.md                       # 이 문서
└── SPEC.md                         # 함수/테이블/라우트 상세 명세
```

---

## 실행

```bash
cd web
pip install flask werkzeug requests
python3 app.py
```

서버는 `0.0.0.0:5000`에서 listen. `detections_v2.db`가 없으면 자동 생성 + 시드 (`admin / admin`, 본점 1번 스테이션).

---

## 화면

| URL | 권한 | 설명 |
|---|---|---|
| `/login` | - | 로그인 / 회원가입 |
| `/kiosk/<branch_id>/<station_no>` | login | 고객 키오스크 (URL로 스테이션 고정) |
| `/admin` | admin | 관리자 모니터 |

기본 시드 계정: **admin / admin** (관리자), 본점 1번 스테이션.

---

## DB 스키마 (10테이블)

| 테이블 | 용도 |
|---|---|
| `users` | 고객/관리자 계정 |
| `branches` | 지점 |
| `stations` | 주유 스테이션 = 로봇+카메라 1세트 |
| `fueling_tasks` | 주유 작업 (실시간 상태) |
| `task_sessions` | task당 1행 — denormalized 집계 (분석/추출용) |
| `robot_snapshots` | 1Hz 로봇 상태 (관절/TCP/외력/그리퍼) |
| `vision_detections` | 비전 인식 (bbox/depth/좌표/handle_angle) |
| `robot_actions` | 로봇 동작 step별 시작/종료/소요시간 |
| `impact_events` | 충격 임계값 초과 이벤트 |
| `logs` | 시스템 로그 (자유 텍스트) |

상세 컬럼 / 함수 / API 명세는 [SPEC.md](./SPEC.md) 참조.

### task lifecycle ↔ DB 흐름

```
[고객] /api/task/start
  → fueling_tasks 새 row INSERT (status='running')
  → task_sessions 새 row INSERT (메타만, target/통계는 NULL)
  → ROS 2 gateway POST /fueling/start

[ROS 2] step별 진행
  → POST /api/robot/action (event=start)  → robot_actions INSERT
  → POST /api/robot/action (event=end)    → robot_actions UPDATE
  → POST /api/robot/snapshot (1Hz)        → robot_snapshots INSERT
  → POST /api/robot/impact (임계값 초과 시) → impact_events INSERT

[종료] success / estop / failed
  → fueling_tasks status 갱신
  → finalize_task_session() 자동 호출
     → vision/snapshots/logs 시간 윈도우 집계
     → task_sessions UPDATE (target_x/y/z, max_torque, max_ext_force, ...)
```

---

## 주요 API

### 인증
| Method | Path | 설명 |
|---|---|---|
| POST | `/api/auth/register` | 회원가입 |
| POST | `/api/auth/login` | 로그인 |
| POST | `/api/auth/logout` | 로그아웃 |
| GET  | `/api/auth/me` | 현재 사용자 |

### 작업
| Method | Path | 설명 |
|---|---|---|
| POST | `/api/task/start` | 주유 시작 (task 생성 + 세션 생성 + ROS 2 호출) |
| POST | `/api/task/estop` | 비상 정지 + finalize |
| GET  | `/api/task/current` | 현재 진행 중 작업 |
| GET  | `/api/task/metrics` | 주유 진행 메트릭 (L / ₩ / 시간) |
| PATCH | `/api/task/<id>` | 작업 필드 갱신 (status 종료 시 자동 finalize) |
| GET  | `/api/task/history` | 작업 이력 |
| GET  | `/api/task/sessions` | 집계 세션 목록 |
| GET  | `/api/task/sessions/<id>` | 집계 세션 상세 |

### 로봇
| Method | Path | 설명 |
|---|---|---|
| POST | `/api/robot/snapshot` | 로봇 상태 스냅샷 수신 (ROS 2 → web) |
| GET  | `/api/robot/joints` | 관절 각도/토크 |
| GET  | `/api/robot/tcp` | TCP pose |
| POST | `/api/robot/action` | step 시작/종료 이벤트 (`event=start\|end`) |
| GET  | `/api/robot/actions?task_id=` | step 이력 |
| POST | `/api/robot/impact` | 충격 이벤트 수신 |
| GET  | `/api/robot/impacts?task_id=` | 충격 이력 |
| POST | `/api/robot/go_home` | 홈 위치 이동 명령 |

### 비전
| Method | Path | 설명 |
|---|---|---|
| POST | `/api/vision/detect` | YOLO 인식 결과 수신 |
| GET  | `/api/vision/current` | 최신 인식 결과 |
| POST | `/api/vision/frame/<sid>` | 프레임(JPEG) 업로드 |
| GET  | `/api/vision/frame/<sid>` | 프레임(JPEG) 조회 |
| POST | `/api/vision/capture/<sid>` | 캡처 이미지 저장 |
| GET  | `/api/vision/history` | 인식 이력 |

### 시스템
| Method | Path | 설명 |
|---|---|---|
| GET  | `/api/system/status` | 로봇/스테이션/작업 상태 통합 |
| GET  | `/api/branches` | 지점 목록 |
| GET  | `/api/branches/<id>/stations` | 지점별 스테이션 |
| POST | `/api/logs` | 로그 기록 |
| GET  | `/api/logs` | 로그 조회 (level/source 필터) |

전체 라우트 / 인자 / 응답 스키마는 [SPEC.md](./SPEC.md) 참조.

---

## DB 초기화

```bash
python3 db_clear.py
```

스키마는 유지되고 모든 row만 삭제 + ID seq 재시작. 시드는 서버 재시작 시 `seed_defaults()`가 다시 채움 (admin 계정 / 본점 / 1번 스테이션).

스키마 자체를 새로 만들고 싶으면 `detections_v2.db` 파일을 직접 삭제 후 `python3 app.py` 실행 → `init_db()`가 새로 생성.

---

## 의존성

- Python 3.10+
- Flask 2.x
- werkzeug
- requests

OS 패키지 추가 설치 불필요 (SQLite는 Python 표준).
