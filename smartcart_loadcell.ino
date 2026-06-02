# SmartCart bridge.
#
# - Arduino/HX711 sends: WEIGHT|TOTAL_G:<grams>
# - Two UHF RFID readers are read directly from Python.
# - Reader 1 OR Reader 2 detecting a tag is enough to start product handling.
# - A product is added only after its RFID tag and loadcell weight both match.
# - If an item already in the cart is removed and not put back within the
#   removal timeout, the cart UI is updated by decrementing/removing that item.

# SmartCart bridge.
#
# - Arduino/HX711 sends: WEIGHT|TOTAL_G:<grams>
# - Two UHF RFID readers are read directly from Python.
# - Reader 1 OR Reader 2 detecting a tag is enough to start product handling.
# - A product is added only after its RFID tag and loadcell weight both match.
# - If an item already in the cart is removed and not put back within the
#   removal timeout, the cart UI is updated by decrementing/removing that item.

import os
import time
from collections import Counter

import pymysql
import serial
from serial.tools import list_ports


DEFAULT_LOADCELL_PORT = "COM6" if os.name == "nt" else "/dev/ttyACM0"
LOADCELL_BAUDRATE = 9600

RFID_PORT1 = os.environ.get("SMARTCART_RFID_PORT1", "/dev/ttyUSB0")
RFID_PORT2 = os.environ.get("SMARTCART_RFID_PORT2", "/dev/ttyUSB1")
RFID_BAUDRATE = int(os.environ.get("SMARTCART_RFID_BAUDRATE", "115200"))
RFID_CMD = bytes([0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E])
RFID_NO_TAG = bytes([0xBB, 0x01, 0xFF, 0x00, 0x01, 0x15, 0x16, 0x7E])
RFID_READ_WINDOW_S = float(os.environ.get("SMARTCART_RFID_READ_WINDOW_S", "0.15"))
RFID_POLL_INTERVAL_S = float(os.environ.get("SMARTCART_RFID_POLL_INTERVAL_S", "0.05"))
RFID_TAG_LOST_TIMEOUT_S = float(os.environ.get("SMARTCART_RFID_TAG_LOST_TIMEOUT_S", "0.25"))
RFID_DUPLICATE_WINDOW_S = float(os.environ.get("SMARTCART_RFID_DUPLICATE_WINDOW_S", "0.8"))

WEIGHT_TOLERANCE_G = float(os.environ.get("SMARTCART_WEIGHT_TOLERANCE_G", "10.0"))
WEIGHT_TOLERANCE_PERCENT = float(os.environ.get("SMARTCART_WEIGHT_TOLERANCE_PERCENT", "0.15"))
PENDING_SCAN_TIMEOUT_S = float(os.environ.get("SMARTCART_PENDING_SCAN_TIMEOUT_S", "12.0"))
REMOVAL_CONFIRM_TIMEOUT_S = float(os.environ.get("SMARTCART_REMOVAL_CONFIRM_TIMEOUT_S", "5.0"))
MIN_WEIGHT_CHANGE_G = float(os.environ.get("SMARTCART_MIN_WEIGHT_CHANGE_G", "5.0"))

DB_CONFIG = {
    "host": os.environ.get("SMARTCART_DB_HOST", "localhost"),
    "port": int(os.environ.get("SMARTCART_DB_PORT", "3306")),
    "user": os.environ.get("SMARTCART_DB_USER", "root"),
    "password": os.environ.get("SMARTCART_DB_PASSWORD", "1234"),
    "database": os.environ.get("SMARTCART_DB_NAME", "smartcart"),
}

DEFAULT_ACTIVE_USER_ID = int(os.environ.get("SMARTCART_DEFAULT_USER_ID", "11"))


def normalize_tag_uid(value):
    return " ".join(str(value).replace("-", " ").upper().split())


def to_hex(data):
    return " ".join(f"{byte:02X}" for byte in data)


def split_frames(buffer):
    frames = []
    index = 0
    while index < len(buffer):
        if buffer[index] != 0xBB:
            index += 1
            continue
        if index + 6 > len(buffer):
            break

        payload_length = (buffer[index + 3] << 8) | buffer[index + 4]
        frame_length = 1 + 1 + 1 + 2 + payload_length + 1 + 1
        end = index + frame_length
        if end > len(buffer):
            break

        if buffer[end - 1] == 0x7E:
            frames.append(buffer[index:end])
            index = end
        else:
            index += 1
    return frames


def extract_epc(frame):
    if frame == RFID_NO_TAG or len(frame) < 12:
        return None
    if frame[0] == 0xBB and frame[1] == 0x02 and frame[2] == 0x22:
        payload_length = (frame[3] << 8) | frame[4]
        payload = frame[5:5 + payload_length]
        if len(payload) < 5:
            return None
        epc = payload[3:-2]
        if epc:
            return to_hex(epc)
    return None


class DualUhfRfidReader:
    def __init__(self, port1, port2):
        self.reader1 = serial.Serial(port1, RFID_BAUDRATE, timeout=0.02)
        self.reader2 = serial.Serial(port2, RFID_BAUDRATE, timeout=0.02)
        time.sleep(0.5)
        self.reader1.reset_input_buffer()
        self.reader2.reset_input_buffer()
        self.current_epc1 = None
        self.current_epc2 = None
        self.last_seen1 = 0.0
        self.last_seen2 = 0.0

    def close(self):
        self.reader1.close()
        self.reader2.close()

    def read_reader(self, reader):
        epcs = []
        buffer = bytearray()
        reader.write(RFID_CMD)
        start_time = time.time()

        while time.time() - start_time < RFID_READ_WINDOW_S:
            waiting = reader.in_waiting
            if waiting > 0:
                buffer.extend(reader.read(waiting))
            else:
                time.sleep(0.01)

        for frame in split_frames(buffer):
            epc = extract_epc(frame)
            if epc:
                epcs.append(epc)
        if not epcs:
            return None
        return Counter(epcs).most_common(1)[0][0]

    def poll(self):
        now = time.time()
        epc1 = self.read_reader(self.reader1)
        epc2 = self.read_reader(self.reader2)

        if epc1:
            self.current_epc1 = epc1
            self.last_seen1 = now
        if epc2:
            self.current_epc2 = epc2
            self.last_seen2 = now

        if now - self.last_seen1 > RFID_TAG_LOST_TIMEOUT_S:
            self.current_epc1 = None
        if now - self.last_seen2 > RFID_TAG_LOST_TIMEOUT_S:
            self.current_epc2 = None

        if self.current_epc1:
            if self.current_epc2 and self.current_epc2 != self.current_epc1:
                message = f"Reader 1 tagged {self.current_epc1}; Reader 2 also sees {self.current_epc2}. Using Reader 1."
            else:
                message = f"Reader 1 tagged {self.current_epc1}"
            return {"status": "detected", "uid": self.current_epc1, "reader": "Reader 1", "message": message}

        if self.current_epc2:
            return {
                "status": "detected",
                "uid": self.current_epc2,
                "reader": "Reader 2",
                "message": f"Reader 2 tagged {self.current_epc2}",
            }

        return {"status": "idle", "uid": None, "reader": None, "message": "waiting"}


def list_available_ports():
    return list(list_ports.comports())


def choose_loadcell_port():
    env_port = os.environ.get("SMARTCART_LOADCELL_PORT", "").strip() or os.environ.get("SMARTCART_SERIAL_PORT", "").strip()
    default_port = os.environ.get("SMARTCART_DEFAULT_LOADCELL_PORT", DEFAULT_LOADCELL_PORT).strip()
    available_ports = list_available_ports()
    available_names = {port.device.upper() for port in available_ports}
    rfid_ports = {RFID_PORT1.upper(), RFID_PORT2.upper()}

    if env_port:
        return env_port
    if default_port and default_port.upper() in available_names:
        return default_port

    for port in available_ports:
        if port.device.upper() in rfid_ports:
            continue
        description = f"{port.description} {port.manufacturer or ''}".upper()
        if "BLUETOOTH" in description:
            continue
        if any(keyword in description for keyword in ("USB", "ARDUINO", "CH340", "CP210", "UART")):
            return port.device
    return None


def print_available_ports():
    ports = list_available_ports()
    if not ports:
        print("[SERIAL] No serial ports detected.")
        return
    print("[SERIAL] Available ports:")
    for port in ports:
        print(f"  - {port.device}: {port.description}")


def connect_loadcell_serial():
    while True:
        try:
            resolved_port = choose_loadcell_port()
            if not resolved_port:
                print("[LOADCELL] Could not find Arduino/HX711 serial port.")
                print_available_ports()
                time.sleep(3)
                continue
            ser = serial.Serial(resolved_port, LOADCELL_BAUDRATE, timeout=0.05)
            print(f"[LOADCELL] Connected: {resolved_port} ({LOADCELL_BAUDRATE} bps)")
            return ser
        except Exception as exc:
            print(f"[LOADCELL] Serial connection failed: {exc}")
            print_available_ports()
            time.sleep(3)


def connect_rfid_readers():
    while True:
        try:
            reader = DualUhfRfidReader(RFID_PORT1, RFID_PORT2)
            print(f"[RFID] Reader 1 connected: {RFID_PORT1}")
            print(f"[RFID] Reader 2 connected: {RFID_PORT2}")
            return reader
        except Exception as exc:
            print(f"[RFID] Reader connection failed: {exc}")
            print_available_ports()
            time.sleep(3)


def connect_db():
    while True:
        try:
            conn = pymysql.connect(
                host=DB_CONFIG["host"],
                port=DB_CONFIG["port"],
                user=DB_CONFIG["user"],
                password=DB_CONFIG["password"],
                database=DB_CONFIG["database"],
                charset="utf8mb4",
                autocommit=False,
            )
            print("[DB] Connected to MariaDB smartcart.")
            return conn
        except pymysql.MySQLError as exc:
            print(f"[DB] Connection failed: {exc}")
            time.sleep(3)


def ensure_runtime_schema(conn):
    with conn.cursor() as cur:
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS cart_event_log (
                event_id INT NOT NULL AUTO_INCREMENT,
                user_id INT NOT NULL,
                event_type VARCHAR(30) NOT NULL,
                tag_uid VARCHAR(64) DEFAULT NULL,
                product_id INT DEFAULT NULL,
                product_name VARCHAR(100) DEFAULT NULL,
                message VARCHAR(255) NOT NULL,
                item_weight_g FLOAT DEFAULT NULL,
                total_weight_g FLOAT DEFAULT NULL,
                allowed_tolerance_g FLOAT DEFAULT NULL,
                created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
                PRIMARY KEY (event_id),
                KEY idx_cart_event_log_user_created (user_id, created_at),
                KEY idx_cart_event_log_product (product_id)
            ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci
            """
        )
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS app_runtime_state (
                state_id TINYINT NOT NULL,
                active_user_id INT DEFAULT NULL,
                updated_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
                PRIMARY KEY (state_id)
            ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci
            """
        )
        for sql in (
            "ALTER TABLE products ADD COLUMN IF NOT EXISTS weight_tolerance_g FLOAT DEFAULT NULL",
            "ALTER TABLE products ADD COLUMN IF NOT EXISTS weight_tolerance_percent FLOAT DEFAULT NULL",
            "ALTER TABLE cart_items ADD COLUMN IF NOT EXISTS allowed_tolerance_g FLOAT DEFAULT NULL",
            "ALTER TABLE cart_event_log ADD COLUMN IF NOT EXISTS item_weight_g FLOAT DEFAULT NULL",
            "ALTER TABLE cart_event_log ADD COLUMN IF NOT EXISTS total_weight_g FLOAT DEFAULT NULL",
            "ALTER TABLE cart_event_log ADD COLUMN IF NOT EXISTS allowed_tolerance_g FLOAT DEFAULT NULL",
        ):
            cur.execute(sql)

        cur.execute(
            """
            INSERT INTO app_runtime_state (state_id, active_user_id)
            VALUES (1, %s)
            ON DUPLICATE KEY UPDATE active_user_id = COALESCE(active_user_id, VALUES(active_user_id))
            """,
            (DEFAULT_ACTIVE_USER_ID,),
        )
    conn.commit()


def parse_loadcell_line(line):
    raw_line = line.strip()
    if not raw_line.startswith("WEIGHT|"):
        return None
    for chunk in raw_line.split("|")[1:]:
        if chunk.startswith("TOTAL_G:"):
            value = chunk.replace("TOTAL_G:", "", 1).strip()
            if not value or value.upper() == "NA":
                return None
            return float(value)
    return None


def get_active_user_context(cursor):
    cursor.execute(
        """
        SELECT ars.active_user_id, u.user_name
        FROM app_runtime_state AS ars
        LEFT JOIN users AS u ON ars.active_user_id = u.user_id
        WHERE ars.state_id = 1
        """
    )
    row = cursor.fetchone()
    if not row or row[0] is None:
        return None, None
    return row[0], row[1]


def get_product_info_by_uid(cursor, uid):
    cursor.execute(
        """
        SELECT p.product_id, p.product_name, p.standard_weight_g,
               p.weight_tolerance_g, p.weight_tolerance_percent
        FROM rfid_tags AS rt
        JOIN products AS p ON rt.product_id = p.product_id
        WHERE rt.tag_uid = %s
        """,
        (normalize_tag_uid(uid),),
    )
    return cursor.fetchone()


def calculate_weight_tolerance_g(expected_weight_g, quantity, tolerance_g=None, tolerance_percent=None):
    quantity = max(int(quantity or 1), 1)
    if expected_weight_g is None:
        return float(tolerance_g or WEIGHT_TOLERANCE_G) * quantity
    base_tolerance = float(tolerance_g) if tolerance_g is not None else WEIGHT_TOLERANCE_G
    percent_value = float(tolerance_percent) if tolerance_percent is not None else WEIGHT_TOLERANCE_PERCENT
    return max(base_tolerance * quantity, abs(float(expected_weight_g)) * percent_value)


def calculate_weight_match(expected_weight_g, measured_weight_g, quantity, tolerance_g=None, tolerance_percent=None):
    if expected_weight_g is None or measured_weight_g is None:
        return None
    tolerance = calculate_weight_tolerance_g(expected_weight_g, quantity, tolerance_g, tolerance_percent)
    return int(abs(float(expected_weight_g) - float(measured_weight_g)) <= tolerance)


def log_cart_event(
    cursor,
    user_id,
    event_type,
    message,
    tag_uid=None,
    product_id=None,
    product_name=None,
    item_weight_g=None,
    total_weight_g=None,
    allowed_tolerance_g=None,
):
    cursor.execute(
        """
        INSERT INTO cart_event_log (
            user_id, event_type, tag_uid, product_id, product_name, message,
            item_weight_g, total_weight_g, allowed_tolerance_g
        )
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
        """,
        (
            user_id,
            event_type,
            tag_uid,
            product_id,
            product_name,
            message[:255],
            item_weight_g,
            total_weight_g,
            allowed_tolerance_g,
        ),
    )


def add_to_cart(cursor, user_id, product_id, standard_weight_g, measured_weight_g, qty, tolerance_g, tolerance_percent):
    cursor.execute(
        """
        SELECT cart_item_id, quantity, expected_weight_g, measured_weight_g
        FROM cart_items
        WHERE user_id = %s AND product_id = %s
        """,
        (user_id, product_id),
    )
    row = cursor.fetchone()
    added_expected = float(standard_weight_g) * qty if standard_weight_g is not None else None

    if row:
        cart_item_id, current_qty, current_expected, current_measured = row
        new_qty = int(current_qty) + qty
        new_expected = (float(current_expected or 0.0) + added_expected) if added_expected is not None else current_expected
        new_measured = float(current_measured or 0.0) + float(measured_weight_g or 0.0)
        allowed = calculate_weight_tolerance_g(new_expected, new_qty, tolerance_g, tolerance_percent)
        matched = calculate_weight_match(new_expected, new_measured, new_qty, tolerance_g, tolerance_percent)
        cursor.execute(
            """
            UPDATE cart_items
            SET quantity = %s, expected_weight_g = %s, measured_weight_g = %s,
                allowed_tolerance_g = %s, weight_matched = %s
            WHERE cart_item_id = %s
            """,
            (new_qty, new_expected, new_measured, allowed, matched, cart_item_id),
        )
        return new_qty, new_expected, new_measured, allowed, matched

    allowed = calculate_weight_tolerance_g(added_expected, qty, tolerance_g, tolerance_percent)
    matched = calculate_weight_match(added_expected, measured_weight_g, qty, tolerance_g, tolerance_percent)
    cursor.execute(
        """
        INSERT INTO cart_items (
            user_id, product_id, quantity, expected_weight_g, measured_weight_g,
            allowed_tolerance_g, weight_matched
        )
        VALUES (%s, %s, %s, %s, %s, %s, %s)
        """,
        (user_id, product_id, qty, added_expected, measured_weight_g, allowed, matched),
    )
    return qty, added_expected, measured_weight_g, allowed, matched


def cart_contains_product(cursor, user_id, product_id):
    cursor.execute(
        """
        SELECT quantity
        FROM cart_items
        WHERE user_id = %s AND product_id = %s
        """,
        (user_id, product_id),
    )
    row = cursor.fetchone()
    return bool(row and int(row[0] or 0) > 0)


def decrement_cart_item(cursor, user_id, product_id, removed_weight_g):
    cursor.execute(
        """
        SELECT cart_item_id, quantity, expected_weight_g, measured_weight_g
        FROM cart_items
        WHERE user_id = %s AND product_id = %s
        """,
        (user_id, product_id),
    )
    row = cursor.fetchone()
    if not row:
        return False, 0

    cart_item_id, quantity, expected_weight_g, measured_weight_g = row
    quantity = int(quantity)
    if quantity <= 1:
        cursor.execute("DELETE FROM cart_items WHERE cart_item_id = %s", (cart_item_id,))
        return True, 0

    new_qty = quantity - 1
    per_expected = float(expected_weight_g or 0.0) / quantity if expected_weight_g is not None else None
    per_measured = float(measured_weight_g or 0.0) / quantity if measured_weight_g is not None else float(removed_weight_g)
    new_expected = float(expected_weight_g or 0.0) - per_expected if per_expected is not None else None
    new_measured = float(measured_weight_g or 0.0) - per_measured
    allowed = calculate_weight_tolerance_g(new_expected, new_qty)
    matched = calculate_weight_match(new_expected, new_measured, new_qty)
    cursor.execute(
        """
        UPDATE cart_items
        SET quantity = %s, expected_weight_g = %s, measured_weight_g = %s,
            allowed_tolerance_g = %s, weight_matched = %s
        WHERE cart_item_id = %s
        """,
        (new_qty, new_expected, new_measured, allowed, matched, cart_item_id),
    )
    return True, new_qty


def find_removed_cart_item(cursor, user_id, removed_weight_g):
    cursor.execute(
        """
        SELECT ci.product_id, p.product_name, ci.quantity, ci.expected_weight_g,
               ci.measured_weight_g, p.standard_weight_g,
               p.weight_tolerance_g, p.weight_tolerance_percent
        FROM cart_items AS ci
        JOIN products AS p ON p.product_id = ci.product_id
        WHERE ci.user_id = %s
        """,
        (user_id,),
    )
    best = None
    for row in cursor.fetchall():
        product_id, product_name, quantity, expected_total, measured_total, standard_weight, tolerance_g, tolerance_percent = row
        quantity = max(int(quantity or 1), 1)
        candidates = []
        if standard_weight is not None:
            candidates.append(float(standard_weight))
        if expected_total is not None:
            candidates.append(float(expected_total) / quantity)
        if measured_total is not None:
            candidates.append(float(measured_total) / quantity)
        if not candidates:
            continue
        expected_one = sum(candidates) / len(candidates)
        allowed = calculate_weight_tolerance_g(expected_one, 1, tolerance_g, tolerance_percent)
        diff = abs(float(removed_weight_g) - expected_one)
        if diff <= allowed and (best is None or diff < best["diff"]):
            best = {
                "product_id": product_id,
                "product_name": product_name,
                "quantity": quantity,
                "expected_one": expected_one,
                "removed_weight_g": float(removed_weight_g),
                "allowed_tolerance_g": allowed,
                "diff": diff,
            }
    return best


def expire_pending_scan(pending_scan, now):
    if pending_scan is None:
        return None
    if now - pending_scan["created_at"] <= PENDING_SCAN_TIMEOUT_S:
        return pending_scan
    print(f"[WEIGHT] Add timeout for {pending_scan['product_name']}. Scan the tag again.")
    return None


def handle_rfid_detected(cursor, db, uid, reader_name, last_stable_weight_g, pending_scan, now, last_uid, last_uid_time):
    uid = normalize_tag_uid(uid)
    if uid == last_uid and (now - last_uid_time) < RFID_DUPLICATE_WINDOW_S:
        return pending_scan, last_uid, last_uid_time

    if pending_scan is not None:
        print(f"[RFID] Ignored {uid}; waiting for weight confirmation for {pending_scan['product_name']}.")
        return pending_scan, uid, now

    active_user_id, active_user_name = get_active_user_context(cursor)
    if active_user_id is None:
        print("[RFID] No active user. Select/login a user in the UI first.")
        return pending_scan, uid, now

    product_info = get_product_info_by_uid(cursor, uid)
    if product_info is None:
        print(f"[RFID] Unknown EPC: {uid}. Register it with measure_products.py first.")
        log_cart_event(cursor, active_user_id, "RFID_UNKNOWN", f"{reader_name} unknown tag: {uid}", tag_uid=uid)
        db.commit()
        return pending_scan, uid, now

    if last_stable_weight_g is None:
        print("[RFID] Waiting for first stable loadcell weight.")
        return pending_scan, uid, now

    product_id, product_name, standard_weight_g, tolerance_g, tolerance_percent = product_info
    if cart_contains_product(cursor, active_user_id, product_id):
        print(f"[RFID] {product_name} is already in cart. Keeping current item; repeated tag ignored.")
        return pending_scan, uid, now

    standard_weight_g = float(standard_weight_g) if standard_weight_g is not None else None
    tolerance_g = float(tolerance_g) if tolerance_g is not None else None
    tolerance_percent = float(tolerance_percent) if tolerance_percent is not None else None
    allowed = calculate_weight_tolerance_g(standard_weight_g, 1, tolerance_g, tolerance_percent)
    pending_scan = {
        "created_at": now,
        "uid": uid,
        "reader_name": reader_name,
        "user_id": active_user_id,
        "user_name": active_user_name,
        "product_id": product_id,
        "product_name": product_name,
        "standard_weight_g": standard_weight_g,
        "weight_tolerance_g": tolerance_g,
        "weight_tolerance_percent": tolerance_percent,
        "baseline_weight_g": float(last_stable_weight_g),
    }
    message = f"{reader_name} tagged {product_name}. Put item in cart."
    print(f"[RFID] {message} EPC={uid}")
    log_cart_event(
        cursor,
        active_user_id,
        "RFID_SCANNED",
        message,
        tag_uid=uid,
        product_id=product_id,
        product_name=product_name,
        total_weight_g=last_stable_weight_g,
        allowed_tolerance_g=allowed,
    )
    db.commit()
    return pending_scan, uid, now


def handle_add_weight(cursor, db, pending_scan, total_weight_g):
    weight_delta_g = float(total_weight_g) - pending_scan["baseline_weight_g"]
    if weight_delta_g < -MIN_WEIGHT_CHANGE_G:
        return pending_scan
    if abs(weight_delta_g) < MIN_WEIGHT_CHANGE_G:
        return pending_scan

    expected = pending_scan["standard_weight_g"]
    tolerance_g = pending_scan["weight_tolerance_g"]
    tolerance_percent = pending_scan["weight_tolerance_percent"]
    allowed = calculate_weight_tolerance_g(expected, 1, tolerance_g, tolerance_percent)
    matched = calculate_weight_match(expected, weight_delta_g, 1, tolerance_g, tolerance_percent)
    if expected is not None and matched != 1:
        print(f"[WEIGHT] Not matched yet for {pending_scan['product_name']}: expected={expected:.1f}g current={weight_delta_g:.1f}g allowed=+/-{allowed:.1f}g")
        return pending_scan

    try:
        qty, expected_total, measured_total, allowed, matched = add_to_cart(
            cursor,
            pending_scan["user_id"],
            pending_scan["product_id"],
            expected,
            weight_delta_g,
            1,
            tolerance_g,
            tolerance_percent,
        )
        message = f"{pending_scan['product_name']} added ({weight_delta_g:.1f}g via {pending_scan['reader_name']})"
        log_cart_event(
            cursor,
            pending_scan["user_id"],
            "CART_CONFIRMED",
            message,
            tag_uid=pending_scan["uid"],
            product_id=pending_scan["product_id"],
            product_name=pending_scan["product_name"],
            item_weight_g=weight_delta_g,
            total_weight_g=measured_total,
            allowed_tolerance_g=allowed,
        )
        db.commit()
        print(f"[CART] {message}; quantity={qty}; matched={matched}")
        return None
    except pymysql.MySQLError as exc:
        db.rollback()
        print(f"[DB] Cart add failed: {exc}")
        return pending_scan


def maybe_start_removal(cursor, db, last_stable_weight_g, total_weight_g, pending_removal):
    if last_stable_weight_g is None or pending_removal is not None:
        return pending_removal
    delta = float(total_weight_g) - float(last_stable_weight_g)
    if delta >= -MIN_WEIGHT_CHANGE_G:
        return pending_removal

    active_user_id, _ = get_active_user_context(cursor)
    if active_user_id is None:
        return pending_removal
    removed_weight = abs(delta)
    match = find_removed_cart_item(cursor, active_user_id, removed_weight)
    if match is None:
        print(f"[REMOVE] Weight decreased by {removed_weight:.1f}g, but no cart item matched.")
        return None

    pending_removal = {
        "created_at": time.time(),
        "deadline": time.time() + REMOVAL_CONFIRM_TIMEOUT_S,
        "baseline_weight_g": float(last_stable_weight_g),
        "current_weight_g": float(total_weight_g),
        "user_id": active_user_id,
        **match,
    }
    message = f"{match['product_name']} removed from cart? Waiting {REMOVAL_CONFIRM_TIMEOUT_S:.0f}s before UI delete."
    print(f"[REMOVE] {message}")
    log_cart_event(
        cursor,
        active_user_id,
        "REMOVAL_PENDING",
        message,
        product_id=match["product_id"],
        product_name=match["product_name"],
        item_weight_g=removed_weight,
        total_weight_g=total_weight_g,
        allowed_tolerance_g=match["allowed_tolerance_g"],
    )
    db.commit()
    return pending_removal


def handle_pending_removal(cursor, db, pending_removal, total_weight_g, now):
    if pending_removal is None:
        return None

    restored_delta = abs(float(total_weight_g) - pending_removal["baseline_weight_g"])
    if restored_delta <= max(MIN_WEIGHT_CHANGE_G, pending_removal["allowed_tolerance_g"]):
        print(f"[REMOVE] {pending_removal['product_name']} was put back before timeout. Keeping cart item.")
        log_cart_event(
            cursor,
            pending_removal["user_id"],
            "REMOVAL_CANCELLED",
            f"{pending_removal['product_name']} put back before timeout",
            product_id=pending_removal["product_id"],
            product_name=pending_removal["product_name"],
            total_weight_g=total_weight_g,
        )
        db.commit()
        return None

    if now < pending_removal["deadline"]:
        return pending_removal

    try:
        changed, new_qty = decrement_cart_item(
            cursor,
            pending_removal["user_id"],
            pending_removal["product_id"],
            pending_removal["removed_weight_g"],
        )
        if changed:
            message = f"{pending_removal['product_name']} auto-deleted from UI after removal timeout"
            log_cart_event(
                cursor,
                pending_removal["user_id"],
                "CART_AUTO_REMOVED",
                message,
                product_id=pending_removal["product_id"],
                product_name=pending_removal["product_name"],
                item_weight_g=pending_removal["removed_weight_g"],
                total_weight_g=total_weight_g,
            )
            db.commit()
            print(f"[CART] {message}; remaining quantity={new_qty}")
        return None
    except pymysql.MySQLError as exc:
        db.rollback()
        print(f"[DB] Cart remove failed: {exc}")
        pending_removal["deadline"] = now + REMOVAL_CONFIRM_TIMEOUT_S
        return pending_removal


def main():
    loadcell = connect_loadcell_serial()
    rfid = connect_rfid_readers()
    db = connect_db()
    ensure_runtime_schema(db)

    cursor = db.cursor()
    last_stable_weight_g = None
    pending_scan = None
    pending_removal = None
    last_uid = None
    last_uid_time = 0.0
    next_rfid_poll_at = 0.0

    print("[SMARTCART] Running. Ctrl+C to stop.")
    print(f"[SMARTCART] RFID rule: Reader 1 OR Reader 2 detection is accepted.")
    print(f"[SMARTCART] Removal timeout: {REMOVAL_CONFIRM_TIMEOUT_S:.1f}s.")

    try:
        while True:
            now = time.time()
            pending_scan = expire_pending_scan(pending_scan, now)

            raw = loadcell.readline()
            if raw:
                try:
                    weight_g = parse_loadcell_line(raw.decode(errors="ignore").strip())
                except ValueError:
                    print(f"[LOADCELL] Could not parse weight: {raw!r}")
                    weight_g = None

                if weight_g is not None:
                    if pending_scan is not None:
                        pending_scan = handle_add_weight(cursor, db, pending_scan, weight_g)
                    else:
                        pending_removal = maybe_start_removal(cursor, db, last_stable_weight_g, weight_g, pending_removal)
                        pending_removal = handle_pending_removal(cursor, db, pending_removal, weight_g, time.time())
                    last_stable_weight_g = weight_g

            if pending_removal is not None:
                pending_removal = handle_pending_removal(cursor, db, pending_removal, last_stable_weight_g, time.time())

            if now >= next_rfid_poll_at:
                next_rfid_poll_at = now + RFID_POLL_INTERVAL_S
                state = rfid.poll()
                if state["status"] == "detected":
                    pending_scan, last_uid, last_uid_time = handle_rfid_detected(
                        cursor,
                        db,
                        state["uid"],
                        state["reader"],
                        last_stable_weight_g,
                        pending_scan,
                        now,
                        last_uid,
                        last_uid_time,
                    )

    except KeyboardInterrupt:
        print("\n[SMARTCART] Stopped.")
    finally:
        for resource in (cursor, db, loadcell, rfid):
            try:
                resource.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
