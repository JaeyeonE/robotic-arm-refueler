"""detections.db 전체 테이블 데이터 초기화 (스키마 유지)"""

from database import get_conn

# FK 의존성 순서: 자식 → 부모
TABLES = [
    "impact_events",
    "robot_actions",
    "task_sessions",
    "logs",
    "vision_detections",
    "robot_snapshots",
    "fueling_tasks",
    "stations",
    "branches",
    "users",
]

conn = get_conn()
for t in TABLES:
    conn.execute(f"DELETE FROM {t}")
conn.execute("DELETE FROM sqlite_sequence")
conn.commit()
conn.execute("VACUUM")
conn.close()

print(f"초기화 완료: {', '.join(TABLES)} (ID 1부터 재시작)")
