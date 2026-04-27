"""시연 세션별 로그/비전/스냅샷 덤프 스크립트.

각 fueling_tasks 행에 대해 시간 윈도우를 구해
 - task.csv      : 해당 task 한 행 + 사용자/스테이션 조인
 - logs.csv      : 윈도우 내 logs (task_id 일치 OR 시간 구간 포함)
 - vision.csv    : 윈도우 내 vision_detections
 - snapshots.csv : 윈도우 내 robot_snapshots
 - state.csv     : 스냅샷의 robot_mode 변동 + task step 변동 타임라인
을 extraction/session_<task_id>/ 아래 저장.

추가로 extraction/summary.csv 에 전체 세션 요약.
"""
import csv
import os
import sqlite3
from pathlib import Path

DB_PATH = Path(__file__).parent / "detections_v2.db"
OUT_DIR = Path(__file__).parent / "extraction"

WINDOW_PAD_SEC = 5  # started_at 이전 / finished_at 이후 여유


def dict_rows(cur):
    cols = [c[0] for c in cur.description]
    return cols, [dict(zip(cols, r)) for r in cur.fetchall()]


def write_csv(path: Path, rows, cols=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        # 헤더만이라도 남기고 싶으면 cols 필요
        with path.open("w", newline="", encoding="utf-8") as f:
            if cols:
                csv.writer(f).writerow(cols)
        return 0
    cols = cols or list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def main():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cur = conn.cursor()

    tasks = cur.execute(
        """SELECT t.*, u.username, u.name AS user_name, s.branch_id, s.station_no
           FROM fueling_tasks t
           LEFT JOIN users u ON u.id = t.user_id
           LEFT JOIN stations s ON s.id = t.station_id
           ORDER BY t.id ASC"""
    ).fetchall()
    tasks = [dict(r) for r in tasks]

    # 다음 task 시작 시각(윈도우 상한 보정용)
    next_started = {}
    for i, t in enumerate(tasks):
        next_started[t["id"]] = tasks[i + 1]["started_at"] if i + 1 < len(tasks) else None

    summary_rows = []

    for t in tasks:
        tid = t["id"]
        sid = t["station_id"]
        started = t["started_at"]
        finished = t["finished_at"]

        # 시간 윈도우 결정
        win_from = f"datetime('{started}', '-{WINDOW_PAD_SEC} seconds')"
        if finished:
            win_to = f"datetime('{finished}', '+{WINDOW_PAD_SEC} seconds')"
        elif next_started[tid]:
            win_to = f"datetime('{next_started[tid]}', '-1 seconds')"
        else:
            win_to = "datetime('now', '+1 day')"

        session_dir = OUT_DIR / f"session_{tid:03d}"

        # ── task.csv ──
        write_csv(session_dir / "task.csv", [t])

        # ── logs.csv : task_id 일치 OR 시간 윈도우 ──
        q_logs = f"""
            SELECT id, station_id, task_id, level, source, message, timestamp
            FROM logs
            WHERE station_id = ?
              AND (task_id = ? OR (timestamp BETWEEN {win_from} AND {win_to}))
            ORDER BY id ASC
        """
        cur.execute(q_logs, (sid, tid))
        _, logs = dict_rows(cur)
        n_logs = write_csv(session_dir / "logs.csv", logs)

        # ── vision.csv : task_id 일치 OR 시간 윈도우 ──
        q_vision = f"""
            SELECT id, station_id, task_id, label, confidence,
                   bbox_x1, bbox_y1, bbox_x2, bbox_y2,
                   center_u, center_v, depth,
                   robot_x, robot_y, robot_z,
                   handle_angle, fuel_type, image_path, timestamp
            FROM vision_detections
            WHERE station_id = ?
              AND (task_id = ? OR (timestamp BETWEEN {win_from} AND {win_to}))
            ORDER BY id ASC
        """
        cur.execute(q_vision, (sid, tid))
        _, vision = dict_rows(cur)
        n_vis = write_csv(session_dir / "vision.csv", vision)

        # ── snapshots.csv : task_id 일치 OR 시간 윈도우 ──
        q_snap = f"""
            SELECT id, station_id, task_id,
                   j1_angle,j2_angle,j3_angle,j4_angle,j5_angle,j6_angle,
                   j1_torque,j2_torque,j3_torque,j4_torque,j5_torque,j6_torque,
                   tcp_x,tcp_y,tcp_z,tcp_a,tcp_b,tcp_c,
                   robot_mode, dart_connected, timestamp
            FROM robot_snapshots
            WHERE station_id = ?
              AND (task_id = ? OR (timestamp BETWEEN {win_from} AND {win_to}))
            ORDER BY id ASC
        """
        cur.execute(q_snap, (sid, tid))
        _, snaps = dict_rows(cur)
        n_snap = write_csv(session_dir / "snapshots.csv", snaps)

        # ── state.csv : robot_mode 변동 타임라인 ──
        state_rows = []
        prev_mode = None
        for s in snaps:
            if s["robot_mode"] != prev_mode:
                state_rows.append({
                    "timestamp": s["timestamp"],
                    "kind": "robot_mode",
                    "value": s["robot_mode"],
                    "dart_connected": s["dart_connected"],
                })
                prev_mode = s["robot_mode"]
        # step 진행 이벤트는 logs message에서 추출 (간이)
        for lg in logs:
            msg = lg["message"] or ""
            if "주유 진행" in msg or "move_completed" in msg or "estop" in msg.lower() or "fueling_complete" in msg:
                state_rows.append({
                    "timestamp": lg["timestamp"],
                    "kind": "event",
                    "value": msg,
                    "dart_connected": None,
                })
        state_rows.sort(key=lambda r: r["timestamp"])
        write_csv(
            session_dir / "state.csv",
            state_rows,
            cols=["timestamp", "kind", "value", "dart_connected"],
        )

        summary_rows.append({
            "task_id": tid,
            "station_id": sid,
            "user": t.get("username"),
            "fuel_type": t["fuel_type"],
            "input_mode": t["input_mode"],
            "target_value": t["target_value"],
            "status": t["status"],
            "current_step": t["current_step"],
            "started_at": started,
            "finished_at": finished,
            "logs": n_logs,
            "vision_rows": n_vis,
            "snapshots": n_snap,
            "state_events": len(state_rows),
        })

    # 전체 summary
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_csv(
        OUT_DIR / "summary.csv",
        summary_rows,
        cols=[
            "task_id","station_id","user","fuel_type","input_mode","target_value",
            "status","current_step","started_at","finished_at",
            "logs","vision_rows","snapshots","state_events",
        ],
    )

    # 전역 덤프 (task에 묶이지 않은 고아 행 포함)
    for name, query, cols in [
        ("all_logs.csv",
         "SELECT * FROM logs ORDER BY id ASC", None),
        ("all_vision.csv",
         "SELECT * FROM vision_detections ORDER BY id ASC", None),
        ("all_snapshots.csv",
         "SELECT * FROM robot_snapshots ORDER BY id ASC", None),
        ("all_tasks.csv",
         "SELECT * FROM fueling_tasks ORDER BY id ASC", None),
    ]:
        cur.execute(query)
        c, rs = dict_rows(cur)
        write_csv(OUT_DIR / name, rs, cols=c)

    conn.close()

    print(f"Wrote {len(tasks)} sessions under {OUT_DIR}")
    for r in summary_rows:
        print(
            f"  task {r['task_id']:>2} [{r['status']:<7}] "
            f"logs={r['logs']:>3} vision={r['vision_rows']:>4} "
            f"snap={r['snapshots']:>4} events={r['state_events']:>3}"
        )


if __name__ == "__main__":
    main()
