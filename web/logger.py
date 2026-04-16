import logging
from database import insert_log


class DBLogHandler(logging.Handler):
    """로그를 SQLite logs 테이블에 기록"""

    def emit(self, record):
        try:
            insert_log(
                level=record.levelname,
                message=self.format(record),
                source=getattr(record, "source", record.name),
                station_id=getattr(record, "station_id", None),
                task_id=getattr(record, "task_id", None),
            )
        except Exception:
            self.handleError(record)


def setup_logger(name="app", level=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level)

    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s"))
        logger.addHandler(ch)

        dh = DBLogHandler()
        dh.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(dh)

    return logger
