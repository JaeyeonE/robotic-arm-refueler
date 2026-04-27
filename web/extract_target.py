"""주유 시작 버튼 → target(x,y,z) → robot trajectory 추출.

각 fueling_tasks (= 주유 시작 버튼 1회 클릭) 마다:
  1) started_at 부근에서 vision_detections(label별 gas_cap / cap_handle / gas_hole)
     의 첫 robot_x/y/z → '버튼 눌림 시점 target'
  2) started_at ~ finished_at(없으면 다음 task 직전) 동안의 robot_snapshots
     전 행을 끌어와 TCP 위치 변화 기록
  3) 두 결과를 JOIN → trajectory.csv (시간·TCP·target·delta·distance)

산출물: web/extraction/target/
  - targets.csv                           (task × label 집계)
  - trajectories.csv                      (전 세션 trajectory 단일 덤프)
  - session_<tid>/target.csv              (해당 task의 label별 target)
  - session_<tid>/trajectory.csv          (snapshot + primary target + delta)
"""
import csv
import math
import sqlite3
from pathlib import Path

DB_PATH = Path(__file__).parent / "detections_v2.db"
OUT_DIR = Path(__file__).parent / "extraction" / "target"

LABELS = ("gas_cap", "cap_handle", "gas_hole")
PRIMARY_LABEL = "gas_cap"  # 버튼 눌렀을 때 최초로 잡히는 주유구 타겟
PRE_PAD_SEC = 2             # started_at 이전 여유
POST_PAD_SEC = 30           # target 포착용 started_at 이후 여유


def dict_rows(cur):
    cols = [c[0] for c in cur.description]
    return [dict(zip(cols, r)) for r in cur.fetchall()]


def write_csv(path: Path, rows, cols=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        if not rows:
            if cols:
                csv.writer(f).writerow(cols)
            return 0
        cols = cols or list(rows[0].keys())
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def main():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    # ── task 목록 + 다음 task 시작 시각(윈도우 상한) ──
    cur.execute("SELECT * FROM fueling_tasks ORDER BY id ASC")
    tasks = dict_rows(cur)
    next_started = {}
    for i, t in enumerate(tasks):
        next_started[t["id"]] = tasks[i + 1]["started_at"] if i + 1 < len(tasks) else None

    # ── 핵심 JOIN 쿼리:
    #    task × label 별로 started_at 부근의 첫 detection(=target) 찾기
    q_target_per_label = """
        SELECT
            t.id               AS task_id,
            t.station_id,
            t.started_at,
            t.finished_at,
            t.status,
            t.fuel_type,
            t.input_mode,
            t.target_value,
            ?                  AS label,
            v.id               AS detection_id,
            v.confidence,
            v.depth,
            v.robot_x          AS target_x,
            v.robot_y          AS target_y,
            v.robot_z          AS target_z,
            v.handle_angle,
            v.timestamp        AS target_ts
        FROM fueling_tasks t
        LEFT JOIN vision_detections v
            ON v.id = (
                SELECT id FROM vision_detections
                WHERE station_id = t.station_id
                  AND label = ?
                  AND timestamp >= datetime(t.started_at, ?)
                  AND timestamp <= datetime(t.started_at, ?)
                ORDER BY timestamp ASC, id ASC
                LIMIT 1
            )
        WHERE t.id = ?
    """

    # ── trajectory JOIN:
    #    snapshots × target(=primary label gas_cap) 조인해 delta/거리 계산
    q_trajectory = """
        WITH tgt AS (
            SELECT id, robot_x AS tx, robot_y AS ty, robot_z AS tz, timestamp AS t_ts
            FROM vision_detections
            WHERE station_id = ?
              AND label = ?
              AND timestamp >= datetime(?, ?)
              AND timestamp <= datetime(?, ?)
            ORDER BY timestamp ASC, id ASC LIMIT 1
        )
        SELECT
            s.id           AS snap_id,
            s.task_id      AS snap_task_id,
            s.timestamp    AS snap_ts,
            s.robot_mode,
            s.dart_connected,
            s.tcp_x, s.tcp_y, s.tcp_z,
            s.tcp_a, s.tcp_b, s.tcp_c,
            s.j1_angle, s.j2_angle, s.j3_angle, s.j4_angle, s.j5_angle, s.j6_angle,
            s.j1_torque, s.j2_torque, s.j3_torque, s.j4_torque, s.j5_torque, s.j6_torque,
            tgt.tx AS target_x,
            tgt.ty AS target_y,
            tgt.tz AS target_z,
            tgt.t_ts AS target_ts
        FROM robot_snapshots s
        LEFT JOIN tgt ON 1=1
        WHERE s.station_id = ?
          AND (s.task_id = ?
               OR (datetime(s.timestamp) BETWEEN datetime(?, ?) AND datetime(?)))
        ORDER BY s.timestamp ASC, s.id ASC
    """

    all_targets = []
    all_traj = []

    for t in tasks:
        tid, sid, started, finished = t["id"], t["station_id"], t["started_at"], t["finished_at"]
        pre_mod = f"-{PRE_PAD_SEC} seconds"
        post_mod = f"+{POST_PAD_SEC} seconds"

        # target end 바운드: finished_at 있으면 그거, 없으면 다음 task 시작 직전
        traj_end = finished if finished else (next_started[tid] or "2099-01-01")

        # ── 1) label별 target ──
        targets_this_task = []
        for lbl in LABELS:
            cur.execute(q_target_per_label, (lbl, lbl, pre_mod, post_mod, tid))
            row = dict_rows(cur)[0]
            targets_this_task.append(row)
            all_targets.append(row)

        # ── 2) trajectory (primary label = gas_cap) ──
        cur.execute(q_trajectory, (
            sid, PRIMARY_LABEL, started, pre_mod, started, post_mod,   # tgt CTE
            sid, tid, started, pre_mod, traj_end,                      # snapshot 범위
        ))
        traj = dict_rows(cur)
        # delta·distance 추가 계산 (파이썬)
        for r in traj:
            tx, ty, tz = r["target_x"], r["target_y"], r["target_z"]
            if tx is not None and r["tcp_x"] is not None:
                dx = r["tcp_x"] - tx
                dy = r["tcp_y"] - ty
                dz = r["tcp_z"] - tz
                r["dx"] = dx
                r["dy"] = dy
                r["dz"] = dz
                r["dist_mm"] = math.sqrt(dx * dx + dy * dy + dz * dz)
            else:
                r["dx"] = r["dy"] = r["dz"] = r["dist_mm"] = None
            r["task_id"] = tid  # 세션 식별용
            all_traj.append(r)

        # ── 세션별 CSV ──
        sess_dir = OUT_DIR / f"session_{tid:03d}"
        write_csv(sess_dir / "target.csv", targets_this_task)
        write_csv(sess_dir / "trajectory.csv", traj)

    # ── 전체 합본 ──
    write_csv(OUT_DIR / "targets.csv", all_targets)
    write_csv(OUT_DIR / "trajectories.csv", all_traj)

    conn.close()

    print(f"Wrote {len(tasks)} sessions to {OUT_DIR}")
    print(f"  targets.csv       : {len(all_targets)} rows ({len(tasks)}×{len(LABELS)})")
    print(f"  trajectories.csv  : {len(all_traj)} rows")
    # per-task 요약
    for t in tasks:
        tid = t["id"]
        tgt_primary = next(
            (r for r in all_targets
             if r["task_id"] == tid and r["label"] == PRIMARY_LABEL),
            None,
        )
        hit = "HIT" if tgt_primary and tgt_primary["detection_id"] else "MISS"
        tx = tgt_primary["target_x"] if tgt_primary and tgt_primary["target_x"] else None
        n_snap = sum(1 for r in all_traj if r["task_id"] == tid)
        print(
            f"  task {tid:>2} [{t['status']:<7}] {hit}  "
            f"target_x={tx if tx is None else f'{tx:7.2f}'}  snaps={n_snap:>4}"
        )


if __name__ == "__main__":
    main()
