import os
import time
import threading
import requests as http_requests
from datetime import datetime, timezone
from functools import wraps

from flask import (
    Flask, request, jsonify, session,
    render_template, send_from_directory, redirect, url_for, Response,
)
from database import (
    init_db,
    create_user, authenticate_user, get_user,
    create_branch, get_branches,
    create_station, get_stations, get_station, get_station_by_branch, update_station_status,
    create_task, update_task, get_current_task, get_task, get_task_history,
    insert_robot_snapshot, get_latest_snapshot,
    insert_detection, get_latest_detection, get_detection_history,
    get_latest_handle_angle,
    insert_log, get_logs,
)
from logger import setup_logger

app = Flask(__name__)
app.secret_key = os.environ.get("SECRET_KEY", "autojet-dev-key-change-in-prod")
logger = setup_logger("app")

CAPTURES_DIR = os.path.join(os.path.dirname(__file__), "captures")
os.makedirs(CAPTURES_DIR, exist_ok=True)

ROS2_GATEWAY_URL = os.environ.get("ROS2_GATEWAY_URL", "http://localhost:6000")

# 스테이션별 최신 프레임 (메모리 보관)
_frame_lock = threading.Lock()
_latest_frames = {}  # {station_id: JPEG bytes}


# ════════════════════════════════════════
#  인증 헬퍼
# ════════════════════════════════════════

def login_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        if "user_id" not in session:
            if request.is_json or request.path.startswith("/api/"):
                return jsonify({"error": "로그인 필요"}), 401
            return redirect(url_for("login_page"))
        return f(*args, **kwargs)
    return decorated


def admin_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        if "user_id" not in session:
            if request.is_json or request.path.startswith("/api/"):
                return jsonify({"error": "로그인 필요"}), 401
            return redirect(url_for("login_page"))
        if session.get("role") != "admin":
            return jsonify({"error": "관리자 권한 필요"}), 403
        return f(*args, **kwargs)
    return decorated


# ════════════════════════════════════════
#  주유 메트릭 백그라운드 스레드
# ════════════════════════════════════════

FUEL_PRICE = 2000
TICK_INTERVAL = 0.05
TICK_VOLUME = 0.05

def _fueling_loop():
    while True:
        time.sleep(TICK_INTERVAL)
        try:
            task = get_current_task()
            if not task or task["status"] != "running":
                continue

            station_id = task["station_id"]
            det = get_latest_detection(station_id)
            if not det or det.get("label") != "refusing":
                continue

            new_liters = round(task["liters"] + TICK_VOLUME, 3)
            new_cost = int(new_liters * FUEL_PRICE)

            done = False
            if task["input_mode"] == "amount" and new_cost >= task["target_value"]:
                new_cost = int(task["target_value"])
                new_liters = round(new_cost / FUEL_PRICE, 3)
                done = True
            elif task["input_mode"] == "volume" and new_liters >= task["target_value"]:
                new_liters = task["target_value"]
                new_cost = int(new_liters * FUEL_PRICE)
                done = True

            if done:
                update_task(task["id"],
                            liters=new_liters, cost=new_cost,
                            current_step=4, status="success",
                            finished_at=datetime.now(timezone.utc).isoformat())
                insert_log("OK", f"주유 완료: {new_liters}L / {new_cost:,}원",
                           source="kiosk", station_id=station_id, task_id=task["id"])
            else:
                update_task(task["id"],
                            liters=new_liters, cost=new_cost,
                            current_step=3)
        except Exception:
            pass

_fuel_thread = threading.Thread(target=_fueling_loop, daemon=True)
_fuel_thread.start()


# ════════════════════════════════════════
#  페이지 라우트
# ════════════════════════════════════════

@app.route("/")
def index():
    if "user_id" in session:
        if session.get("role") == "admin":
            return redirect(url_for("admin_page"))
        return redirect(url_for("login_page"))
    return redirect(url_for("login_page"))


@app.route("/login")
def login_page():
    return render_template("login.html")


@app.route("/kiosk/<int:branch_id>/<int:station_no>")
@login_required
def kiosk_page(branch_id, station_no):
    station = get_station_by_branch(branch_id, station_no)
    if not station:
        return "스테이션을 찾을 수 없습니다", 404
    return render_template("kiosk.html",
                           station_id=station["id"],
                           branch_id=branch_id,
                           station_no=station_no)


@app.route("/admin")
@admin_required
def admin_page():
    return render_template("admin.html")


@app.route("/captures/<path:filename>")
def serve_capture(filename):
    return send_from_directory(CAPTURES_DIR, filename)


# ════════════════════════════════════════
#  인증 API
# ════════════════════════════════════════

@app.route("/api/auth/register", methods=["POST"])
def api_register():
    data = request.get_json(force=True)
    username = data.get("username")
    password = data.get("password")
    name = data.get("name")
    role = data.get("role", "customer")
    phone = data.get("phone")

    if not all([username, password, name]):
        return jsonify({"error": "username, password, name 필수"}), 400

    import sqlite3
    try:
        uid = create_user(username, password, name, role, phone)
        return jsonify({"ok": True, "user_id": uid}), 201
    except sqlite3.IntegrityError:
        return jsonify({"error": "이미 존재하는 사용자명입니다"}), 409
    except Exception as e:
        logger.exception("register failed")
        return jsonify({"error": f"서버 오류: {type(e).__name__}"}), 500


@app.route("/api/auth/login", methods=["POST"])
def api_login():
    data = request.get_json(force=True)
    user = authenticate_user(data.get("username", ""), data.get("password", ""))
    if not user:
        return jsonify({"error": "아이디 또는 비밀번호가 틀렸습니다"}), 401

    session["user_id"] = user["id"]
    session["role"] = user["role"]
    session["name"] = user["name"]
    return jsonify({
        "ok": True,
        "user_id": user["id"],
        "role": user["role"],
        "name": user["name"],
    })


@app.route("/api/auth/logout", methods=["POST"])
def api_logout():
    session.clear()
    return jsonify({"ok": True})


@app.route("/api/auth/me", methods=["GET"])
@login_required
def api_me():
    user = get_user(session["user_id"])
    if not user:
        session.clear()
        return jsonify({"error": "사용자 없음"}), 401
    return jsonify(user)


# ════════════════════════════════════════
#  지점 / 스테이션 API
# ════════════════════════════════════════

@app.route("/api/branches", methods=["GET"])
@login_required
def api_get_branches():
    return jsonify(get_branches())


@app.route("/api/branches", methods=["POST"])
@admin_required
def api_create_branch():
    data = request.get_json(force=True)
    bid = create_branch(data["name"], data.get("address"))
    return jsonify({"ok": True, "branch_id": bid}), 201


@app.route("/api/branches/<int:branch_id>/stations", methods=["GET"])
@login_required
def api_get_stations(branch_id):
    return jsonify(get_stations(branch_id))


@app.route("/api/stations", methods=["POST"])
@admin_required
def api_create_station():
    data = request.get_json(force=True)
    sid = create_station(
        data["branch_id"], data["station_no"],
        data.get("robot_model", "E0509"), data.get("camera_model", "D455"),
    )
    return jsonify({"ok": True, "station_id": sid}), 201


# ════════════════════════════════════════
#  시스템 상태 API
# ════════════════════════════════════════

@app.route("/api/system/status", methods=["GET"])
def api_system_status():
    station_id = request.args.get("station_id", type=int)
    snap = get_latest_snapshot(station_id)
    task = get_current_task(station_id)
    station = get_station(station_id) if station_id else None
    return jsonify({
        "robot_mode": snap["robot_mode"] if snap else "OFFLINE",
        "dart_connected": bool(snap["dart_connected"]) if snap else False,
        "station_status": station["status"] if station else None,
        "current_task_id": task["id"] if task else None,
        "current_step": task["current_step"] if task else None,
        "task_status": task["status"] if task else None,
    })


# ════════════════════════════════════════
#  로봇 상태 API
# ════════════════════════════════════════

@app.route("/api/robot/joints", methods=["GET"])
def api_robot_joints():
    station_id = request.args.get("station_id", type=int)
    snap = get_latest_snapshot(station_id)
    if not snap:
        return jsonify({"joints": []}), 200
    joints = []
    for i in range(1, 7):
        joints.append({
            "label": f"J{i}",
            "angle": snap[f"j{i}_angle"],
            "torque": snap[f"j{i}_torque"],
        })
    return jsonify({"joints": joints})


@app.route("/api/robot/tcp", methods=["GET"])
def api_robot_tcp():
    station_id = request.args.get("station_id", type=int)
    snap = get_latest_snapshot(station_id)
    if not snap:
        return jsonify({"tcp": {}}), 200
    return jsonify({
        "tcp": {
            "x": snap["tcp_x"], "y": snap["tcp_y"], "z": snap["tcp_z"],
            "a": snap["tcp_a"], "b": snap["tcp_b"], "c": snap["tcp_c"],
        }
    })


@app.route("/api/robot/snapshot", methods=["POST"])
def api_post_snapshot():
    data = request.get_json(force=True)
    if "station_id" not in data:
        return jsonify({"error": "station_id 필수"}), 400
    insert_robot_snapshot(data)
    return jsonify({"ok": True}), 201


@app.route("/api/robot/go_home", methods=["POST"])
@login_required
def api_robot_go_home():
    data = request.get_json(force=True)
    station_id = data.get("station_id")
    try:
        http_requests.post(
            f"{ROS2_GATEWAY_URL}/fueling/go_home",
            json={"station_id": station_id},
            timeout=2.0,
        )
        insert_log("INFO", "홈 위치 이동 요청",
                   source="control", station_id=station_id)
    except Exception as e:
        insert_log("ERROR", f"ROS2 gateway 홈 이동 요청 실패: {e}",
                   source="control", station_id=station_id)
    return jsonify({"ok": True})


# ════════════════════════════════════════
#  비전 API
# ════════════════════════════════════════

@app.route("/api/vision/current", methods=["GET"])
def api_vision_current():
    station_id = request.args.get("station_id", type=int)
    det = get_latest_detection(station_id)
    if not det:
        return jsonify({}), 200
    return jsonify(det)


@app.route("/api/vision/detect", methods=["POST"])
def api_post_detection():
    data = request.get_json(force=True)
    if "station_id" not in data:
        return jsonify({"error": "station_id 필수"}), 400
    det_id = insert_detection(data)
    return jsonify({"ok": True, "id": det_id}), 201


@app.route("/api/vision/frame/<int:station_id>", methods=["POST"])
def api_post_frame(station_id):
    with _frame_lock:
        _latest_frames[station_id] = request.data
    return jsonify({"ok": True}), 200


@app.route("/api/vision/frame/<int:station_id>", methods=["GET"])
def api_get_frame(station_id):
    with _frame_lock:
        frame = _latest_frames.get(station_id)
    if not frame:
        return Response(status=204)
    return Response(frame, mimetype="image/jpeg")


@app.route("/api/vision/capture/<int:station_id>", methods=["POST"])
def api_vision_capture(station_id):
    with _frame_lock:
        frame = _latest_frames.get(station_id)
    if not frame:
        return jsonify({"error": "프레임 없음"}), 404
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"s{station_id}_{ts}.jpg"
    filepath = os.path.join(CAPTURES_DIR, filename)
    with open(filepath, "wb") as f:
        f.write(frame)
    return jsonify({"ok": True, "filename": filename, "path": f"/captures/{filename}"}), 201


@app.route("/api/vision/history", methods=["GET"])
def api_vision_history():
    rows = get_detection_history(
        station_id=request.args.get("station_id", type=int),
        task_id=request.args.get("task_id", type=int),
        limit=request.args.get("limit", 100, type=int),
    )
    return jsonify(rows)


# ════════════════════════════════════════
#  주유 작업 API
# ════════════════════════════════════════

@app.route("/api/task/start", methods=["POST"])
@login_required
def api_task_start():
    data = request.get_json(force=True)
    station_id = data.get("station_id")
    fuel_type = data.get("fuel_type")
    input_mode = data.get("input_mode")
    target_value = data.get("target_value")

    if not all([station_id, fuel_type, input_mode, target_value]):
        return jsonify({"error": "station_id, fuel_type, input_mode, target_value 필수"}), 400

    user_id = session["user_id"]
    task_id = create_task(station_id, user_id, fuel_type, input_mode, target_value)
    insert_log("INFO", f"주유 시작: {fuel_type} / {input_mode} / {target_value}",
               source="kiosk", station_id=station_id, task_id=task_id)

    det = get_latest_detection(station_id)
    if not det or det.get("robot_x") is None:
        insert_log("ERROR", "비전 인식 데이터 없음 — 로봇 제어 불가",
                   source="kiosk", station_id=station_id, task_id=task_id)
        return jsonify({"ok": True, "task_id": task_id,
                        "warn": "비전 데이터 없음, 로봇 대기 상태"}), 201

    handle_angle = get_latest_handle_angle(station_id)
    if handle_angle is None:
        insert_log("WARN", "최근 handle_angle 없음 — 기본 방향으로 진행",
                   source="kiosk", station_id=station_id, task_id=task_id)

    try:
        http_requests.post(
            f"{ROS2_GATEWAY_URL}/fueling/start",
            json={
                "robot_x": det["robot_x"],
                "robot_y": det["robot_y"],
                "robot_z": det["robot_z"],
                "handle_angle": handle_angle,
                "task_id": task_id,
            },
            timeout=2.0,
        )
        angle_txt = f"{handle_angle:.1f}" if handle_angle is not None else "default"
        insert_log("INFO",
                    f"로봇 제어 요청: xyz=({det['robot_x']:.1f}, {det['robot_y']:.1f}, {det['robot_z']:.1f}), angle={angle_txt}",
                    source="control", station_id=station_id, task_id=task_id)
    except Exception as e:
        insert_log("ERROR", f"ROS2 gateway 연결 실패: {e}",
                   source="control", station_id=station_id, task_id=task_id)

    return jsonify({"ok": True, "task_id": task_id}), 201


@app.route("/api/task/estop", methods=["POST"])
@login_required
def api_task_estop():
    station_id = request.get_json(force=True).get("station_id")
    task = get_current_task(station_id)
    if task:
        update_task(task["id"], status="estop",
                    finished_at=datetime.now(timezone.utc).isoformat())
        insert_log("ERROR", f"E-STOP 발동 (task_id={task['id']})",
                   source="control", station_id=station_id, task_id=task["id"])

    # ROS2 gateway에 E-STOP 전파
    try:
        http_requests.post(
            f"{ROS2_GATEWAY_URL}/fueling/estop",
            json={"station_id": station_id},
            timeout=1.0,
        )
    except Exception as e:
        insert_log("ERROR", f"ROS2 gateway E-STOP 전파 실패: {e}",
                   source="control", station_id=station_id)

    return jsonify({"ok": True, "stopped_task_id": task["id"] if task else None})


@app.route("/api/task/current", methods=["GET"])
def api_task_current():
    station_id = request.args.get("station_id", type=int)
    task = get_current_task(station_id)
    if not task:
        return jsonify({}), 200
    return jsonify(task)


@app.route("/api/task/metrics", methods=["GET"])
def api_task_metrics():
    station_id = request.args.get("station_id", type=int)
    task = get_current_task(station_id)
    if not task:
        return jsonify({}), 200

    elapsed = 0
    if task["started_at"]:
        started = datetime.fromisoformat(task["started_at"]).replace(tzinfo=timezone.utc)
        elapsed = int((datetime.now(timezone.utc) - started).total_seconds())

    if task["input_mode"] == "volume":
        remaining_l = max(0, task["target_value"] - task["liters"])
    else:
        remaining_l = max(0, (task["target_value"] - task["cost"]) / FUEL_PRICE)
    remaining_sec = int(remaining_l)

    return jsonify({
        "task_id": task["id"],
        "liters": task["liters"],
        "cost": task["cost"],
        "elapsed_sec": elapsed,
        "remaining_sec": remaining_sec,
        "fuel_type": task["fuel_type"],
        "target_value": task["target_value"],
    })


@app.route("/api/task/<int:task_id>", methods=["PATCH"])
def api_patch_task(task_id):
    data = request.get_json(force=True)
    allowed = {"current_step", "step_progress", "status", "liters", "cost", "finished_at"}
    fields = {k: v for k, v in data.items() if k in allowed}
    if not fields:
        return jsonify({"error": "업데이트할 필드 없음"}), 400
    update_task(task_id, **fields)
    return jsonify({"ok": True})


@app.route("/api/task/history", methods=["GET"])
def api_task_history():
    rows = get_task_history(
        station_id=request.args.get("station_id", type=int),
        limit=request.args.get("limit", 50, type=int),
        offset=request.args.get("offset", 0, type=int),
    )
    return jsonify(rows)


# ════════════════════════════════════════
#  로그 API
# ════════════════════════════════════════

@app.route("/api/logs", methods=["POST"])
def api_post_log():
    data = request.get_json(force=True)
    insert_log(
        level=data.get("level", "INFO"),
        message=data.get("message", ""),
        source=data.get("source", "system"),
        station_id=data.get("station_id"),
        task_id=data.get("task_id"),
    )
    return jsonify({"ok": True}), 201


@app.route("/api/logs", methods=["GET"])
def api_get_logs():
    rows = get_logs(
        station_id=request.args.get("station_id", type=int),
        level=request.args.get("level"),
        source=request.args.get("source"),
        task_id=request.args.get("task_id", type=int),
        limit=request.args.get("limit", 50, type=int),
    )
    return jsonify(rows)


# ════════════════════════════════════════
#  초기 데이터 시드
# ════════════════════════════════════════

def seed_defaults():
    """기본 지점 + 스테이션 + 관리자 계정 생성 (없으면)"""
    from database import get_conn
    conn = get_conn()

    if not conn.execute("SELECT 1 FROM branches LIMIT 1").fetchone():
        conn.execute("INSERT INTO branches (name, address) VALUES ('본점', '서울시 강남구')")
        conn.commit()

    if not conn.execute("SELECT 1 FROM stations LIMIT 1").fetchone():
        conn.execute("INSERT INTO stations (branch_id, station_no) VALUES (1, 1)")
        conn.commit()

    if not conn.execute("SELECT 1 FROM users WHERE role='admin' LIMIT 1").fetchone():
        conn.close()
        create_user("admin", "admin", "관리자", role="admin")
    else:
        conn.close()


# ════════════════════════════════════════

if __name__ == "__main__":
    init_db()
    seed_defaults()
    insert_log("INFO", "Flask 서버 시작", source="system")
    app.run(host="0.0.0.0", port=5000, debug=True)
