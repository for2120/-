import os
import sys
from pathlib import Path

import pymysql


DB_HOST = os.environ.get("SMARTCART_DB_HOST", "localhost")
DB_PORT = int(os.environ.get("SMARTCART_DB_PORT", "3306"))
DB_USER = os.environ.get("SMARTCART_DB_USER", "root")
DB_PASSWORD = os.environ.get("SMARTCART_DB_PASSWORD", "1234")
DB_NAME = os.environ.get("SMARTCART_DB_NAME", "smartcart")

PROJECT_DIR = Path(__file__).resolve().parent.parent
SCHEMA_PATH = PROJECT_DIR / "queries" / "middle_4-1.sql"


def iter_sql_statements(sql_text: str):
    buffer = []
    for line in sql_text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("--"):
            continue
        buffer.append(line)
        if stripped.endswith(";"):
            statement = "\n".join(buffer).strip()
            if statement:
                yield statement[:-1].strip()
            buffer = []

    trailing = "\n".join(buffer).strip()
    if trailing:
        yield trailing


def database_exists(connection, database_name: str) -> bool:
    with connection.cursor() as cur:
        cur.execute("SHOW DATABASES LIKE %s", (database_name,))
        return cur.fetchone() is not None


def bootstrap_database():
    if not SCHEMA_PATH.exists():
        print(f"[DB] 스키마 파일이 없습니다: {SCHEMA_PATH}")
        return 1

    connection = pymysql.connect(
        host=DB_HOST,
        port=DB_PORT,
        user=DB_USER,
        password=DB_PASSWORD,
        charset="utf8mb4",
        autocommit=True,
        cursorclass=pymysql.cursors.Cursor,
    )

    try:
        if database_exists(connection, DB_NAME):
            print(f"[DB] 데이터베이스 '{DB_NAME}' 이미 존재함")
            return 0

        print(f"[DB] 데이터베이스 '{DB_NAME}' 이(가) 없어 자동 초기화를 시작합니다.")
        sql_text = SCHEMA_PATH.read_text(encoding="utf-8")
        with connection.cursor() as cur:
            for statement in iter_sql_statements(sql_text):
                cur.execute(statement)
        print(f"[DB] 데이터베이스 '{DB_NAME}' 초기화 완료")
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    try:
        raise SystemExit(bootstrap_database())
    except pymysql.MySQLError as exc:
        print(f"[DB] 자동 초기화 실패: {exc}")
        raise SystemExit(1)
