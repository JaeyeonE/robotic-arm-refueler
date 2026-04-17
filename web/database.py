import sqlite3
import os
from werkzeug.security import generate_password_hash, check_password_hash

DB_PATH = os.path.join(os.path.dirname(__file__), "detections_v2.db")


def get_conn():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
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
            robot_mode     TEXT DEFAULT 'IDLE',
            dart_connected INTEGER DEFAULT 0,
            timestamp      DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    """)

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

    conn.commit()
    conn.close()


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
            robot_mode, dart_connected)
           VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
        (
            data["station_id"],
            data.get("task_id"),
            data.get("j1_angle"), data.get("j2_angle"), data.get("j3_angle"),
            data.get("j4_angle"), data.get("j5_angle"), data.get("j6_angle"),
            data.get("j1_torque"), data.get("j2_torque"), data.get("j3_torque"),
            data.get("j4_torque"), data.get("j5_torque"), data.get("j6_torque"),
            data.get("tcp_x"), data.get("tcp_y"), data.get("tcp_z"),
            data.get("tcp_a"), data.get("tcp_b"), data.get("tcp_c"),
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
