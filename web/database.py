import sqlite3
import os
import math
from datetime import datetime, timezone
from werkzeug.security import generate_password_hash, check_password_hash

DB_PATH = os.path.join(os.path.dirname(__file__), "detections_v2.db")


def get_conn():
    conn = sqlite3.connect(DB_PATH, timeout=10.0)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    conn.execute("PRAGMA busy_timeout=10000")
    return conn


def init_db():
    conn = get_conn()
    c = conn.cursor()

    # ── users ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            username      TEXT NOT NULL UNIQUE,
            password_hash TEXT NOT NULL,
            name          TEXT NOT NULL,
            role          TEXT NOT NULL DEFAULT 'customer',
            phone         TEXT,
            created_at    DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)

    # ── branches ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS branches (
            id         INTEGER PRIMARY KEY AUTOINCREMENT,
            name       TEXT NOT NULL,
            address    TEXT,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)

    # ── stations ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS stations (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            branch_id     INTEGER NOT NULL REFERENCES branches(id),
            station_no    INTEGER NOT NULL,
            robot_model   TEXT DEFAULT 'E0509',
            camera_model  TEXT DEFAULT 'D455',
            status        TEXT DEFAULT 'idle',
            created_at    DATETIME DEFAULT CURRENT_TIMESTAMP,
            UNIQUE(branch_id, station_no)
        )
    """)

    # ── fueling_tasks ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS fueling_tasks (
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
        )
    """)

    # ── robot_snapshots ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS robot_snapshots (
            id             INTEGER PRIMARY KEY AUTOINCREMENT,
            station_id     INTEGER NOT NULL REFERENCES stations(id),
            task_id        INTEGER REFERENCES fueling_tasks(id),
            j1_angle REAL, j2_angle REAL, j3_angle REAL,
            j4_angle REAL, j5_angle REAL, j6_angle REAL,
            j1_torque REAL, j2_torque REAL, j3_torque REAL,
            j4_torque REAL, j5_torque REAL, j6_torque REAL,
            tcp_x REAL, tcp_y REAL, tcp_z REAL,
            tcp_a REAL, tcp_b REAL, tcp_c REAL,
            ext_fx REAL, ext_fy REAL, ext_fz REAL,
            ext_mx REAL, ext_my REAL, ext_mz REAL,
            gripper_stroke INTEGER,
            robot_mode     TEXT DEFAULT 'IDLE',
            dart_connected INTEGER DEFAULT 0,
            timestamp      DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)
    _migrate_robot_snapshots(c)

    # ── vision_detections ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS vision_detections (
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
            handle_angle REAL,
            fuel_type   TEXT,
            image_path  TEXT,
            timestamp   DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)

    # ── logs ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS logs (
            id         INTEGER PRIMARY KEY AUTOINCREMENT,
            station_id INTEGER REFERENCES stations(id),
            task_id    INTEGER,
            level      TEXT NOT NULL DEFAULT 'INFO',
            source     TEXT DEFAULT 'system',
            message    TEXT NOT NULL,
            timestamp  DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)

    # ── task_sessions: task당 1행, 추출/분석 편의를 위한 denormalized 집계 ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS task_sessions (
            task_id              INTEGER PRIMARY KEY REFERENCES fueling_tasks(id),
            station_id           INTEGER,
            user_id              INTEGER,
            user_name            TEXT,
            branch_name          TEXT,
            station_no           INTEGER,
            fuel_type            TEXT,
            input_mode           TEXT,
            target_value         REAL,
            target_label         TEXT,
            target_x             REAL,
            target_y             REAL,
            target_z             REAL,
            target_handle_angle  REAL,
            target_confidence    REAL,
            target_captured_at   DATETIME,
            started_at           DATETIME,
            finished_at          DATETIME,
            duration_sec         REAL,
            final_status         TEXT,
            failed_step_index    INTEGER,
            failed_step_desc     TEXT,
            liters               REAL,
            cost                 INTEGER,
            detection_count      INTEGER,
            snapshot_count       INTEGER,
            warning_count        INTEGER,
            error_count          INTEGER,
            max_torque_j1        REAL, max_torque_j2 REAL, max_torque_j3 REAL,
            max_torque_j4        REAL, max_torque_j5 REAL, max_torque_j6 REAL,
            max_ext_force_x      REAL,
            max_ext_force_y      REAL,
            max_ext_force_z      REAL,
            max_ext_force_mag    REAL,
            impact_event_count   INTEGER,
            final_dist_to_target_mm REAL,
            created_at           DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)

    # ── robot_actions: step별 실행 이력 (시작/종료/소요/성공) ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS robot_actions (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            task_id         INTEGER REFERENCES fueling_tasks(id),
            station_id      INTEGER REFERENCES stations(id),
            step_index      INTEGER,
            total_steps     INTEGER,
            step_desc       TEXT,
            step_type       TEXT,
            target_data     TEXT,
            started_at      DATETIME DEFAULT CURRENT_TIMESTAMP,
            ended_at        DATETIME,
            duration_ms     INTEGER,
            success         INTEGER,
            error_message   TEXT,
            max_torque_during REAL,
            max_force_during  REAL
        )
    """)

    # ── impact_events: 충격 임계값 초과 이벤트만 기록 ──
    c.execute("""
        CREATE TABLE IF NOT EXISTS impact_events (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            task_id       INTEGER REFERENCES fueling_tasks(id),
            station_id    INTEGER REFERENCES stations(id),
            step_index    INTEGER,
            step_desc     TEXT,
            detected_at   DATETIME DEFAULT CURRENT_TIMESTAMP,
            source        TEXT,
            axis          TEXT,
            value         REAL,
            threshold     REAL,
            severity      TEXT
        )
    """)

    conn.commit()
    conn.close()


def _migrate_robot_snapshots(c):
    """기존 DB에 신규 컬럼이 없으면 ALTER TABLE로 추가 (idempotent)."""
    existing = {row[1] for row in c.execute("PRAGMA table_info(robot_snapshots)").fetchall()}
    new_cols = [
        ("ext_fx", "REAL"),
        ("ext_fy", "REAL"),
        ("ext_fz", "REAL"),
        ("ext_mx", "REAL"),
        ("ext_my", "REAL"),
        ("ext_mz", "REAL"),
        ("gripper_stroke", "INTEGER"),
    ]
    for name, type_ in new_cols:
        if name not in existing:
            c.execute(f"ALTER TABLE robot_snapshots ADD COLUMN {name} {type_}")


# ══════════════════════════════════════
#  users
# ══════════════════════════════════════

def create_user(username, password, name, role="customer", phone=None):
    conn = get_conn()
    c = conn.execute(
        """INSERT INTO users (username, password_hash, name, role, phone)
           VALUES (?, ?, ?, ?, ?)""",
        (username, generate_password_hash(password), name, role, phone),
    )
    uid = c.lastrowid
    conn.commit()
    conn.close()
    return uid


def authenticate_user(username, password):
    conn = get_conn()
    row = conn.execute(
        "SELECT * FROM users WHERE username=?", (username,)
    ).fetchone()
    conn.close()
    if row and check_password_hash(row["password_hash"], password):
        return dict(row)
    return None


def get_user(user_id):
    conn = get_conn()
    row = conn.execute("SELECT id, username, name, role, phone, created_at FROM users WHERE id=?", (user_id,)).fetchone()
    conn.close()
    return dict(row) if row else None


# ══════════════════════════════════════
#  branches
# ══════════════════════════════════════

def create_branch(name, address=None):
    conn = get_conn()
    c = conn.execute(
        "INSERT INTO branches (name, address) VALUES (?, ?)", (name, address)
    )
    bid = c.lastrowid
    conn.commit()
    conn.close()
    return bid


def get_branches():
    conn = get_conn()
    rows = conn.execute("SELECT * FROM branches ORDER BY id").fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ══════════════════════════════════════
#  stations
# ══════════════════════════════════════

def create_station(branch_id, station_no, robot_model="E0509", camera_model="D455"):
    conn = get_conn()
    c = conn.execute(
        """INSERT INTO stations (branch_id, station_no, robot_model, camera_model)
           VALUES (?, ?, ?, ?)""",
        (branch_id, station_no, robot_model, camera_model),
    )
    sid = c.lastrowid
    conn.commit()
    conn.close()
    return sid


def get_stations(branch_id):
    conn = get_conn()
    rows = conn.execute(
        "SELECT * FROM stations WHERE branch_id=? ORDER BY station_no", (branch_id,)
    ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


def get_station(station_id):
    conn = get_conn()
    row = conn.execute("SELECT * FROM stations WHERE id=?", (station_id,)).fetchone()
    conn.close()
    return dict(row) if row else None


def get_station_by_branch(branch_id, station_no):
    conn = get_conn()
    row = conn.execute(
        "SELECT * FROM stations WHERE branch_id=? AND station_no=?",
        (branch_id, station_no),
    ).fetchone()
    conn.close()
    return dict(row) if row else None


def update_station_status(station_id, status):
    conn = get_conn()
    conn.execute("UPDATE stations SET status=? WHERE id=?", (status, station_id))
    conn.commit()
    conn.close()


# ══════════════════════════════════════
#  fueling_tasks
# ══════════════════════════════════════

def create_task(station_id, user_id, fuel_type, input_mode, target_value):
    conn = get_conn()
    c = conn.execute(
        """INSERT INTO fueling_tasks
           (station_id, user_id, fuel_type, input_mode, target_value, status, started_at)
           VALUES (?, ?, ?, ?, ?, 'running', CURRENT_TIMESTAMP)""",
        (station_id, user_id, fuel_type, input_mode, target_value),
    )
    task_id = c.lastrowid
    conn.commit()
    conn.close()
    return task_id


def update_task(task_id, **fields):
    conn = get_conn()
    sets = ", ".join(f"{k}=?" for k in fields)
    vals = list(fields.values()) + [task_id]
    conn.execute(f"UPDATE fueling_tasks SET {sets} WHERE id=?", vals)
    conn.commit()
    conn.close()


def get_current_task(station_id=None):
    conn = get_conn()
    if station_id:
        row = conn.execute(
            "SELECT * FROM fueling_tasks WHERE station_id=? AND status IN ('pending','running') ORDER BY id DESC LIMIT 1",
            (station_id,),
        ).fetchone()
    else:
        row = conn.execute(
            "SELECT * FROM fueling_tasks WHERE status IN ('pending','running') ORDER BY id DESC LIMIT 1"
        ).fetchone()
    conn.close()
    return dict(row) if row else None


def get_task(task_id):
    conn = get_conn()
    row = conn.execute("SELECT * FROM fueling_tasks WHERE id=?", (task_id,)).fetchone()
    conn.close()
    return dict(row) if row else None


def get_task_history(station_id=None, limit=50, offset=0):
    conn = get_conn()
    if station_id:
        rows = conn.execute(
            "SELECT * FROM fueling_tasks WHERE station_id=? ORDER BY created_at DESC LIMIT ? OFFSET ?",
            (station_id, limit, offset),
        ).fetchall()
    else:
        rows = conn.execute(
            "SELECT * FROM fueling_tasks ORDER BY created_at DESC LIMIT ? OFFSET ?",
            (limit, offset),
        ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ══════════════════════════════════════
#  robot_snapshots
# ══════════════════════════════════════

def insert_robot_snapshot(data):
    conn = get_conn()
    conn.execute(
        """INSERT INTO robot_snapshots
           (station_id, task_id,
            j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle,
            j1_torque, j2_torque, j3_torque, j4_torque, j5_torque, j6_torque,
            tcp_x, tcp_y, tcp_z, tcp_a, tcp_b, tcp_c,
            ext_fx, ext_fy, ext_fz, ext_mx, ext_my, ext_mz,
            gripper_stroke,
            robot_mode, dart_connected)
           VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
        (
            data["station_id"],
            data.get("task_id"),
            data.get("j1_angle"), data.get("j2_angle"), data.get("j3_angle"),
            data.get("j4_angle"), data.get("j5_angle"), data.get("j6_angle"),
            data.get("j1_torque"), data.get("j2_torque"), data.get("j3_torque"),
            data.get("j4_torque"), data.get("j5_torque"), data.get("j6_torque"),
            data.get("tcp_x"), data.get("tcp_y"), data.get("tcp_z"),
            data.get("tcp_a"), data.get("tcp_b"), data.get("tcp_c"),
            data.get("ext_fx"), data.get("ext_fy"), data.get("ext_fz"),
            data.get("ext_mx"), data.get("ext_my"), data.get("ext_mz"),
            data.get("gripper_stroke"),
            data.get("robot_mode", "IDLE"),
            data.get("dart_connected", 0),
        ),
    )
    conn.commit()
    conn.close()


def get_latest_snapshot(station_id=None):
    conn = get_conn()
    if station_id:
        row = conn.execute(
            "SELECT * FROM robot_snapshots WHERE station_id=? ORDER BY id DESC LIMIT 1",
            (station_id,),
        ).fetchone()
    else:
        row = conn.execute(
            "SELECT * FROM robot_snapshots ORDER BY id DESC LIMIT 1"
        ).fetchone()
    conn.close()
    return dict(row) if row else None


# ══════════════════════════════════════
#  vision_detections
# ══════════════════════════════════════

def insert_detection(data):
    conn = get_conn()
    c = conn.execute(
        """INSERT INTO vision_detections
           (station_id, task_id, label, confidence,
            bbox_x1, bbox_y1, bbox_x2, bbox_y2,
            center_u, center_v, depth,
            robot_x, robot_y, robot_z, handle_angle, fuel_type, image_path)
           VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
        (
            data["station_id"],
            data.get("task_id"),
            data["label"], data["confidence"],
            data.get("bbox_x1"), data.get("bbox_y1"),
            data.get("bbox_x2"), data.get("bbox_y2"),
            data.get("center_u"), data.get("center_v"),
            data.get("depth"),
            data.get("robot_x"), data.get("robot_y"), data.get("robot_z"),
            data.get("handle_angle"),
            data.get("fuel_type"),
            data.get("image_path"),
        ),
    )
    det_id = c.lastrowid
    conn.commit()
    conn.close()
    return det_id


def get_latest_detection(station_id=None):
    conn = get_conn()
    if station_id:
        row = conn.execute(
            "SELECT * FROM vision_detections WHERE station_id=? ORDER BY id DESC LIMIT 1",
            (station_id,),
        ).fetchone()
    else:
        row = conn.execute(
            "SELECT * FROM vision_detections ORDER BY id DESC LIMIT 1"
        ).fetchone()
    conn.close()
    return dict(row) if row else None


def get_latest_handle_angle(station_id=None, lookback=50):
    """최근 lookback 행 중 handle_angle IS NOT NULL 첫 행의 angle 반환. 없으면 None."""
    conn = get_conn()
    if station_id:
        row = conn.execute(
            """SELECT handle_angle FROM vision_detections
               WHERE station_id=? AND handle_angle IS NOT NULL
               ORDER BY id DESC LIMIT 1""",
            (station_id,),
        ).fetchone()
    else:
        row = conn.execute(
            """SELECT handle_angle FROM vision_detections
               WHERE handle_angle IS NOT NULL
               ORDER BY id DESC LIMIT 1"""
        ).fetchone()
    conn.close()
    return row["handle_angle"] if row else None


def get_detection_history(station_id=None, task_id=None, limit=100):
    conn = get_conn()
    query = "SELECT * FROM vision_detections WHERE 1=1"
    params = []
    if station_id:
        query += " AND station_id=?"
        params.append(station_id)
    if task_id:
        query += " AND task_id=?"
        params.append(task_id)
    query += " ORDER BY id DESC LIMIT ?"
    params.append(limit)
    rows = conn.execute(query, params).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ══════════════════════════════════════
#  logs
# ══════════════════════════════════════

def insert_log(level, message, source="system", station_id=None, task_id=None):
    conn = get_conn()
    conn.execute(
        "INSERT INTO logs (station_id, task_id, level, source, message) VALUES (?,?,?,?,?)",
        (station_id, task_id, level, source, message),
    )
    conn.commit()
    conn.close()


def get_logs(station_id=None, level=None, source=None, task_id=None, limit=50):
    conn = get_conn()
    query = "SELECT * FROM logs WHERE 1=1"
    params = []

    if station_id:
        query += " AND station_id=?"
        params.append(station_id)
    if level:
        query += " AND level=?"
        params.append(level)
    if source:
        query += " AND source=?"
        params.append(source)
    if task_id:
        query += " AND task_id=?"
        params.append(task_id)

    query += " ORDER BY id DESC LIMIT ?"
    params.append(limit)

    rows = conn.execute(query, params).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ══════════════════════════════════════
#  task_sessions (집계 / 분석용)
# ══════════════════════════════════════

def create_task_session(task_id):
    """task 시작 시 호출. 메타정보(user_name, branch_name 등)를 미리 펼쳐둔다.
    target_*은 NULL로 시작 → finalize_task_session에서 채움."""
    conn = get_conn()
    row = conn.execute(
        """SELECT t.id, t.station_id, t.user_id,
                  t.fuel_type, t.input_mode, t.target_value, t.started_at,
                  u.name AS user_name,
                  s.station_no, b.name AS branch_name
           FROM fueling_tasks t
           LEFT JOIN users u    ON u.id = t.user_id
           LEFT JOIN stations s ON s.id = t.station_id
           LEFT JOIN branches b ON b.id = s.branch_id
           WHERE t.id = ?""",
        (task_id,),
    ).fetchone()
    if not row:
        conn.close()
        return

    conn.execute(
        """INSERT OR IGNORE INTO task_sessions
           (task_id, station_id, user_id, user_name, branch_name, station_no,
            fuel_type, input_mode, target_value, started_at)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
        (
            row["id"], row["station_id"], row["user_id"],
            row["user_name"], row["branch_name"], row["station_no"],
            row["fuel_type"], row["input_mode"], row["target_value"],
            row["started_at"],
        ),
    )
    conn.commit()
    conn.close()


def _parse_iso(ts):
    """sqlite/iso 두 포맷 모두 timezone-aware datetime으로 (naive는 UTC로 가정)."""
    if not ts:
        return None
    try:
        dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
    except Exception:
        try:
            dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
        except Exception:
            return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt


def finalize_task_session(task_id):
    """task 종료 시 호출. 시간 윈도우 기반으로 통계 집계 + UPDATE.
    여러 번 불러도 멱등 (마지막 호출 결과로 덮어씀)."""
    conn = get_conn()
    cur = conn.cursor()

    task = cur.execute("SELECT * FROM fueling_tasks WHERE id=?", (task_id,)).fetchone()
    if not task:
        conn.close()
        return
    task = dict(task)

    sid = task["station_id"]
    started = task["started_at"]
    finished = task["finished_at"] or datetime.now(timezone.utc).isoformat()

    if not started:
        conn.close()
        return

    # ── target: started_at 부근의 첫 gas_cap detection ──
    target_row = cur.execute(
        """SELECT label, robot_x, robot_y, robot_z,
                  handle_angle, confidence, timestamp
           FROM vision_detections
           WHERE station_id = ?
             AND label = 'gas_cap'
             AND timestamp >= datetime(?, '-2 seconds')
             AND timestamp <= datetime(?, '+30 seconds')
           ORDER BY timestamp ASC, id ASC LIMIT 1""",
        (sid, started, started),
    ).fetchone()
    target = dict(target_row) if target_row else {}

    # ── detection / snapshot count ──
    det_count = cur.execute(
        """SELECT COUNT(*) AS c FROM vision_detections
           WHERE station_id = ? AND timestamp BETWEEN ? AND ?""",
        (sid, started, finished),
    ).fetchone()["c"]

    snap_stats_row = cur.execute(
        """SELECT COUNT(*) AS c,
                  MAX(ABS(j1_torque)) AS mt1, MAX(ABS(j2_torque)) AS mt2,
                  MAX(ABS(j3_torque)) AS mt3, MAX(ABS(j4_torque)) AS mt4,
                  MAX(ABS(j5_torque)) AS mt5, MAX(ABS(j6_torque)) AS mt6,
                  MAX(ABS(ext_fx)) AS mefx,
                  MAX(ABS(ext_fy)) AS mefy,
                  MAX(ABS(ext_fz)) AS mefz
           FROM robot_snapshots
           WHERE station_id = ? AND timestamp BETWEEN ? AND ?""",
        (sid, started, finished),
    ).fetchone()
    snap_stats = dict(snap_stats_row) if snap_stats_row else {}

    # ext_force magnitude의 max — Python에서 계산 (snapshot 행 수 보통 <100)
    max_ef_mag = None
    snaps = cur.execute(
        """SELECT ext_fx, ext_fy, ext_fz FROM robot_snapshots
           WHERE station_id = ? AND timestamp BETWEEN ? AND ?
             AND (ext_fx IS NOT NULL OR ext_fy IS NOT NULL OR ext_fz IS NOT NULL)""",
        (sid, started, finished),
    ).fetchall()
    for s in snaps:
        fx = s["ext_fx"] or 0.0
        fy = s["ext_fy"] or 0.0
        fz = s["ext_fz"] or 0.0
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if max_ef_mag is None or mag > max_ef_mag:
            max_ef_mag = mag

    # ── log warn / error count (task_id 매칭 OR 시간 윈도우) ──
    log_stats = cur.execute(
        """SELECT
             SUM(CASE WHEN level IN ('WARN','WARNING') THEN 1 ELSE 0 END) AS warn_c,
             SUM(CASE WHEN level = 'ERROR' THEN 1 ELSE 0 END) AS err_c
           FROM logs
           WHERE task_id = ? OR (station_id = ? AND timestamp BETWEEN ? AND ?)""",
        (task_id, sid, started, finished),
    ).fetchone()
    log_stats = dict(log_stats) if log_stats else {}

    # ── impact event count ──
    impact_count = cur.execute(
        "SELECT COUNT(*) AS c FROM impact_events WHERE task_id = ?",
        (task_id,),
    ).fetchone()["c"]

    # ── failed step ──
    failed_action = cur.execute(
        """SELECT step_index, step_desc FROM robot_actions
           WHERE task_id = ? AND success = 0
           ORDER BY id DESC LIMIT 1""",
        (task_id,),
    ).fetchone()
    failed = dict(failed_action) if failed_action else {}

    # ── final dist (마지막 TCP - target) ──
    last_snap = cur.execute(
        """SELECT tcp_x, tcp_y, tcp_z FROM robot_snapshots
           WHERE station_id = ? AND timestamp BETWEEN ? AND ?
           ORDER BY id DESC LIMIT 1""",
        (sid, started, finished),
    ).fetchone()

    final_dist = None
    if (last_snap and target.get("robot_x") is not None
            and last_snap["tcp_x"] is not None):
        dx = last_snap["tcp_x"] - target["robot_x"]
        dy = last_snap["tcp_y"] - target["robot_y"]
        dz = last_snap["tcp_z"] - target["robot_z"]
        final_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    # ── duration ──
    duration = None
    s_dt = _parse_iso(started)
    f_dt = _parse_iso(task["finished_at"]) if task["finished_at"] else None
    if s_dt and f_dt:
        duration = (f_dt - s_dt).total_seconds()

    # ── ensure row exists, then UPDATE ──
    cur.execute("INSERT OR IGNORE INTO task_sessions (task_id) VALUES (?)", (task_id,))
    cur.execute(
        """UPDATE task_sessions SET
              target_label = ?,
              target_x = ?, target_y = ?, target_z = ?,
              target_handle_angle = ?, target_confidence = ?,
              target_captured_at = ?,
              finished_at = ?,
              duration_sec = ?,
              final_status = ?,
              failed_step_index = ?, failed_step_desc = ?,
              liters = ?, cost = ?,
              detection_count = ?,
              snapshot_count = ?,
              warning_count = ?, error_count = ?,
              max_torque_j1 = ?, max_torque_j2 = ?, max_torque_j3 = ?,
              max_torque_j4 = ?, max_torque_j5 = ?, max_torque_j6 = ?,
              max_ext_force_x = ?, max_ext_force_y = ?, max_ext_force_z = ?,
              max_ext_force_mag = ?,
              impact_event_count = ?,
              final_dist_to_target_mm = ?
           WHERE task_id = ?""",
        (
            target.get("label"),
            target.get("robot_x"), target.get("robot_y"), target.get("robot_z"),
            target.get("handle_angle"), target.get("confidence"),
            target.get("timestamp"),
            task["finished_at"],
            duration,
            task["status"],
            failed.get("step_index"), failed.get("step_desc"),
            task["liters"], task["cost"],
            det_count,
            snap_stats.get("c"),
            log_stats.get("warn_c"), log_stats.get("err_c"),
            snap_stats.get("mt1"), snap_stats.get("mt2"), snap_stats.get("mt3"),
            snap_stats.get("mt4"), snap_stats.get("mt5"), snap_stats.get("mt6"),
            snap_stats.get("mefx"), snap_stats.get("mefy"), snap_stats.get("mefz"),
            max_ef_mag,
            impact_count,
            final_dist,
            task_id,
        ),
    )
    conn.commit()
    conn.close()


def get_task_session(task_id):
    conn = get_conn()
    row = conn.execute("SELECT * FROM task_sessions WHERE task_id=?", (task_id,)).fetchone()
    conn.close()
    return dict(row) if row else None


def get_task_sessions(station_id=None, limit=50, offset=0):
    conn = get_conn()
    if station_id:
        rows = conn.execute(
            "SELECT * FROM task_sessions WHERE station_id=? ORDER BY task_id DESC LIMIT ? OFFSET ?",
            (station_id, limit, offset),
        ).fetchall()
    else:
        rows = conn.execute(
            "SELECT * FROM task_sessions ORDER BY task_id DESC LIMIT ? OFFSET ?",
            (limit, offset),
        ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ══════════════════════════════════════
#  robot_actions
# ══════════════════════════════════════

def insert_robot_action(data):
    """step 시작 시 호출. action_id 반환."""
    conn = get_conn()
    started = data.get("started_at") or datetime.now(timezone.utc).isoformat()
    c = conn.execute(
        """INSERT INTO robot_actions
           (task_id, station_id, step_index, total_steps, step_desc, step_type,
            target_data, started_at)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
        (
            data.get("task_id"), data.get("station_id"),
            data.get("step_index"), data.get("total_steps"),
            data.get("step_desc"), data.get("step_type"),
            data.get("target_data"),
            started,
        ),
    )
    aid = c.lastrowid
    conn.commit()
    conn.close()
    return aid


def update_robot_action_end(action_id, success, ended_at=None,
                            error_message=None,
                            max_torque_during=None, max_force_during=None):
    """step 종료 시 호출. duration_ms는 자동 계산."""
    conn = get_conn()
    if ended_at is None:
        ended_at = datetime.now(timezone.utc).isoformat()

    row = conn.execute(
        "SELECT started_at FROM robot_actions WHERE id=?", (action_id,)
    ).fetchone()
    duration_ms = None
    if row and row["started_at"]:
        s = _parse_iso(row["started_at"])
        e = _parse_iso(ended_at)
        if s and e:
            duration_ms = int((e - s).total_seconds() * 1000)

    conn.execute(
        """UPDATE robot_actions SET
              ended_at = ?, duration_ms = ?, success = ?, error_message = ?,
              max_torque_during = ?, max_force_during = ?
           WHERE id = ?""",
        (
            ended_at, duration_ms, 1 if success else 0, error_message,
            max_torque_during, max_force_during, action_id,
        ),
    )
    conn.commit()
    conn.close()


def get_robot_actions(task_id=None, station_id=None, limit=200):
    conn = get_conn()
    query = "SELECT * FROM robot_actions WHERE 1=1"
    params = []
    if task_id is not None:
        query += " AND task_id=?"
        params.append(task_id)
    if station_id is not None:
        query += " AND station_id=?"
        params.append(station_id)
    query += " ORDER BY id ASC LIMIT ?"
    params.append(limit)
    rows = conn.execute(query, params).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ══════════════════════════════════════
#  impact_events
# ══════════════════════════════════════

def insert_impact_event(data):
    conn = get_conn()
    conn.execute(
        """INSERT INTO impact_events
           (task_id, station_id, step_index, step_desc,
            source, axis, value, threshold, severity)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
        (
            data.get("task_id"), data.get("station_id"),
            data.get("step_index"), data.get("step_desc"),
            data.get("source"), data.get("axis"),
            data.get("value"), data.get("threshold"),
            data.get("severity", "warning"),
        ),
    )
    conn.commit()
    conn.close()


def get_impact_events(task_id=None, station_id=None, limit=100):
    conn = get_conn()
    query = "SELECT * FROM impact_events WHERE 1=1"
    params = []
    if task_id is not None:
        query += " AND task_id=?"
        params.append(task_id)
    if station_id is not None:
        query += " AND station_id=?"
        params.append(station_id)
    query += " ORDER BY id DESC LIMIT ?"
    params.append(limit)
    rows = conn.execute(query, params).fetchall()
    conn.close()
    return [dict(r) for r in rows]
