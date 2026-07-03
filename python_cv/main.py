import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
import time
import csv
import json

# =========================
# OPTIONAL SERIAL TO ESP32
# =========================
try:
    import serial
except ImportError:
    serial = None

# =========================
# OPTIONAL WIFI TO ESP32
# =========================
try:
    import requests
except ImportError:
    requests = None


# =========================
# CONFIG
# =========================

CAMERA_INDEX = 0

# Untuk WiFi mode
WIFI_ENABLED = True
ESP32_IP = "192.168.1.37"   # GANTI dengan IP ESP32 dari Serial Monitor
ESP32_BASE_URL = f"http://{ESP32_IP}"

# Serial optional, boleh dimatikan karena kita pakai WiFi
SERIAL_ENABLED = False
SERIAL_PORT = "COM5"
SERIAL_BAUD = 115200

BASELINE_DIR = Path("baselines")
CAPTURE_DIR = Path("captures")
LOG_DIR = Path("logs")
ITEM_DB_PATH = Path("items.csv")
CONFIG_PATH = Path("config.json")

# Log ringkas sesuai kebutuhan laporan/pengujian.
# Kolom: timestamp, item_id, side, visual_status, change_pct, shock_g, tilt_deg, final_status
COMPACT_LOG_PATH = LOG_DIR / "inspection_compact_log.csv"

BASELINE_DIR.mkdir(exist_ok=True)
CAPTURE_DIR.mkdir(exist_ok=True)
LOG_DIR.mkdir(exist_ok=True)

STANDARD_WIDTH = 640
STANDARD_HEIGHT = 360

# Threshold default. Nilai ini akan dioverride oleh config.json kalau file tersedia.
# visual_check_pct  : batas PASS ke CHECK
# visual_reject_pct : batas CHECK ke REJECT
PASS_THRESHOLD = 2.0
CHECK_THRESHOLD = 8.0

DIFF_THRESHOLD = 35
MIN_CONTOUR_AREA = 250
EDGE_IGNORE_RATIO = 0.04

# Threshold handling untuk dokumentasi/kalibrasi.
# Keputusan handling real-time tetap berasal dari ESP32, sedangkan nilai ini dicatat agar konsisten di sisi Python.
SHOCK_CHECK_G = 1.7
SHOCK_REJECT_G = 3.0
TILT_CHECK_DEG = 20.0
TILT_REJECT_DEG = 60.0

SHOW_DIFF_WINDOW = True

# Refresh data final status dari ESP32 untuk dashboard OpenCV
ESP32_STATUS_REFRESH_SEC = 1.0

SIDES = ["TOP", "FRONT", "BACK", "LEFT", "RIGHT", "BOTTOM"]


# =========================
# UTILITY
# =========================

def safe_filename(text: str) -> str:
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
    return "".join(c if c in allowed else "_" for c in text)


def get_timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def get_log_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def ensure_config_file():
    if CONFIG_PATH.exists():
        return

    default_config = {
        "visual_check_pct": PASS_THRESHOLD,
        "visual_reject_pct": CHECK_THRESHOLD,
        "shock_check_g": SHOCK_CHECK_G,
        "shock_reject_g": SHOCK_REJECT_G,
        "tilt_check_deg": TILT_CHECK_DEG,
        "tilt_reject_deg": TILT_REJECT_DEG,
        "diff_threshold": DIFF_THRESHOLD,
        "min_contour_area": MIN_CONTOUR_AREA,
        "edge_ignore_ratio": EDGE_IGNORE_RATIO
    }

    with open(CONFIG_PATH, "w", encoding="utf-8") as file:
        json.dump(default_config, file, indent=2)

    print(f"Config created: {CONFIG_PATH}")


def load_runtime_config():
    global PASS_THRESHOLD, CHECK_THRESHOLD
    global SHOCK_CHECK_G, SHOCK_REJECT_G, TILT_CHECK_DEG, TILT_REJECT_DEG
    global DIFF_THRESHOLD, MIN_CONTOUR_AREA, EDGE_IGNORE_RATIO

    ensure_config_file()

    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as file:
            config = json.load(file)
    except Exception as e:
        print("WARNING: config.json gagal dibaca. Pakai threshold default.")
        print("Detail:", e)
        return

    PASS_THRESHOLD = float(config.get("visual_check_pct", PASS_THRESHOLD))
    CHECK_THRESHOLD = float(config.get("visual_reject_pct", CHECK_THRESHOLD))
    SHOCK_CHECK_G = float(config.get("shock_check_g", SHOCK_CHECK_G))
    SHOCK_REJECT_G = float(config.get("shock_reject_g", SHOCK_REJECT_G))
    TILT_CHECK_DEG = float(config.get("tilt_check_deg", TILT_CHECK_DEG))
    TILT_REJECT_DEG = float(config.get("tilt_reject_deg", TILT_REJECT_DEG))
    DIFF_THRESHOLD = int(config.get("diff_threshold", DIFF_THRESHOLD))
    MIN_CONTOUR_AREA = int(config.get("min_contour_area", MIN_CONTOUR_AREA))
    EDGE_IGNORE_RATIO = float(config.get("edge_ignore_ratio", EDGE_IGNORE_RATIO))

    print("Runtime threshold loaded from config.json")
    print(f"Visual PASS < {PASS_THRESHOLD}% | CHECK < {CHECK_THRESHOLD}% | REJECT >= {CHECK_THRESHOLD}%")
    print(f"Shock check/reject: {SHOCK_CHECK_G}G / {SHOCK_REJECT_G}G")
    print(f"Tilt check/reject : {TILT_CHECK_DEG} deg / {TILT_REJECT_DEG} deg")


# =========================
# ITEM DATABASE / QR VALIDATION
# =========================

def ensure_item_database():
    if ITEM_DB_PATH.exists():
        return

    sample_rows = [
        {"item_id": f"FRAGILE-PT.JAYAGANESHA-{number:03d}",
         "item_name": f"Barang Fragile Jayaganesha {number:03d}",
         "category": "Fragile Goods",
         "expected_sides": "6"}
        for number in range(1, 11)
    ]

    with open(ITEM_DB_PATH, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["item_id", "item_name", "category", "expected_sides"]
        )
        writer.writeheader()
        writer.writerows(sample_rows)

    print(f"Item database created: {ITEM_DB_PATH}")


def load_item_database():
    ensure_item_database()

    item_db = {}
    with open(ITEM_DB_PATH, "r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            item_id = row.get("item_id", "").strip()
            if item_id:
                item_db[item_id] = {
                    "item_id": item_id,
                    "item_name": row.get("item_name", "").strip(),
                    "category": row.get("category", "").strip(),
                    "expected_sides": row.get("expected_sides", str(len(SIDES))).strip(),
                }

    print(f"Loaded {len(item_db)} registered item(s) from {ITEM_DB_PATH}")
    return item_db


def validate_item_id(item_id: str, item_db: dict):
    if not item_id:
        return False, None

    item_info = item_db.get(item_id.strip())
    return item_info is not None, item_info


def get_overall_visual_status(results: dict) -> str:
    if not results:
        return "WAIT"

    statuses = [value["status"] for value in results.values()]

    if "REJECT" in statuses:
        return "REJECT"

    if "CHECK" in statuses:
        return "CHECK"

    if len(results) < len(SIDES):
        return "INCOMPLETE"

    return "PASS"


def status_short(status: str) -> str:
    mapping = {
        "PASS": "P",
        "CHECK": "C",
        "REJECT": "R",
        "REGISTERED": "B",
        "REGISTER_REQUIRED": "REG",
        "READY_TO_INSPECT": "RDY",
        "WAIT_QR": "-",
        "WAIT": "-",
        "INCOMPLETE": "INC",
        "QR_INVALID": "INV",
        "TAMPER": "TMP",
        "ERROR": "ERR",
    }
    return mapping.get(status, "-")


# =========================
# SERIAL ESP32
# =========================

def init_serial():
    if not SERIAL_ENABLED:
        print("Serial ESP32: DISABLED")
        return None

    if serial is None:
        print("ERROR: pyserial belum terinstall.")
        print("Install dengan: pip install pyserial")
        return None

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        time.sleep(2)
        print(f"Serial ESP32 connected: {SERIAL_PORT} @ {SERIAL_BAUD}")
        return ser
    except Exception as e:
        print("ERROR: ESP32 serial tidak tersambung.")
        print("Detail:", e)
        return None


def send_to_esp32_serial(ser, item_id: str, side: str, visual_status: str, qr_status: str = "VALID"):
    if ser is None:
        return

    commands = [
        f"ID:{item_id}",
        f"QR:{qr_status}",
        f"SIDE:{side}",
        f"VIS:{visual_status}",
    ]

    try:
        for cmd in commands:
            ser.write((cmd + "\n").encode("utf-8"))
            time.sleep(0.03)

        print("Sent to ESP32 Serial:", " | ".join(commands))

    except Exception as e:
        print("ERROR sending to ESP32 Serial:", e)


def send_reset_to_esp32_serial(ser):
    if ser is None:
        return

    try:
        ser.write(b"RESET\n")
        print("Sent to ESP32 Serial: RESET")
    except Exception as e:
        print("ERROR sending RESET to ESP32 Serial:", e)


# =========================
# WIFI ESP32
# =========================

def send_to_esp32_wifi(item_id: str, side: str, visual_status: str, qr_status: str = "VALID"):
    if not WIFI_ENABLED:
        return

    if requests is None:
        print("ERROR: requests belum terinstall. Install: pip install requests")
        return

    url = f"{ESP32_BASE_URL}/update"

    params = {
        "id": item_id,
        "qr": qr_status,
        "side": side,
        "vis": visual_status,
    }

    try:
        response = requests.get(url, params=params, timeout=1.5)

        if response.status_code == 200:
            print("Sent to ESP32 WiFi:", params)
        else:
            print("ESP32 WiFi error:", response.status_code, response.text)

    except Exception as e:
        print("ERROR sending to ESP32 WiFi:", e)


def send_reset_to_esp32_wifi():
    if not WIFI_ENABLED:
        return

    if requests is None:
        print("ERROR: requests belum terinstall. Install: pip install requests")
        return

    try:
        response = requests.get(f"{ESP32_BASE_URL}/reset", timeout=1.5)

        if response.status_code == 200:
            print("Sent to ESP32 WiFi: RESET")
        else:
            print("ESP32 reset error:", response.status_code, response.text)

    except Exception as e:
        print("ERROR sending RESET to ESP32 WiFi:", e)


def send_status_to_esp32(ser, item_id: str, side: str, visual_status: str, qr_status: str = "VALID"):
    send_to_esp32_serial(ser, item_id, side, visual_status, qr_status)
    send_to_esp32_wifi(item_id, side, visual_status, qr_status)


def send_reset_to_esp32(ser):
    send_reset_to_esp32_serial(ser)
    send_reset_to_esp32_wifi()


def get_esp32_status():
    if not WIFI_ENABLED:
        return {}

    if requests is None:
        return {}

    try:
        response = requests.get(f"{ESP32_BASE_URL}/status", timeout=1.5)

        if response.status_code == 200:
            return response.json()

        print("ESP32 status error:", response.status_code, response.text)
        return {}

    except Exception as e:
        print("ERROR reading ESP32 status:", e)
        return {}


# =========================
# QR DETECTION
# =========================

def detect_qr(frame, qr_detector):
    qr_text, qr_points, _ = qr_detector.detectAndDecode(frame)

    if qr_text:
        qr_text = qr_text.strip()

    return qr_text, qr_points


def draw_qr_box(frame, qr_points, qr_text):
    if qr_points is None:
        return

    points = qr_points.astype(int).reshape(-1, 2)

    for i in range(len(points)):
        pt1 = tuple(points[i])
        pt2 = tuple(points[(i + 1) % len(points)])
        cv2.line(frame, pt1, pt2, (255, 255, 255), 2)

    x, y = points[0]
    cv2.putText(
        frame,
        f"QR: {qr_text}",
        (x, max(30, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )


# =========================
# INSPECTION AREA
# =========================

def get_fixed_inspection_roi(frame):
    h, w, _ = frame.shape

    roi_w = int(w * 0.67)
    roi_h = int(h * 0.60)

    x1 = (w - roi_w) // 2
    y1 = int(h * 0.25)

    x2 = x1 + roi_w
    y2 = y1 + roi_h

    roi = frame[y1:y2, x1:x2]

    return roi, (x1, y1, x2, y2)


def normalize_roi(roi):
    resized = cv2.resize(roi, (STANDARD_WIDTH, STANDARD_HEIGHT))
    return resized


# =========================
# BASELINE MANAGEMENT
# =========================

def get_baseline_path(item_id, side):
    filename = f"{safe_filename(item_id)}_{safe_filename(side)}.jpg"
    return BASELINE_DIR / filename


def save_baseline(item_id, side, normalized_roi):
    path = get_baseline_path(item_id, side)
    cv2.imwrite(str(path), normalized_roi)
    return path


def load_baseline(item_id, side):
    path = get_baseline_path(item_id, side)

    if not path.exists():
        return None, path

    baseline = cv2.imread(str(path))

    if baseline is None:
        return None, path

    baseline = cv2.resize(baseline, (STANDARD_WIDTH, STANDARD_HEIGHT))
    return baseline, path


# =========================
# IMAGE COMPARISON
# =========================

def preprocess_for_compare(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    equalized = cv2.equalizeHist(blur)
    return equalized


def compare_with_baseline(baseline, current):
    baseline_prep = preprocess_for_compare(baseline)
    current_prep = preprocess_for_compare(current)

    diff = cv2.absdiff(baseline_prep, current_prep)

    _, diff_mask = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)

    kernel = np.ones((5, 5), np.uint8)
    diff_mask = cv2.morphologyEx(diff_mask, cv2.MORPH_OPEN, kernel)
    diff_mask = cv2.morphologyEx(diff_mask, cv2.MORPH_CLOSE, kernel)

    # Abaikan noise di pinggir ROI
    h, w = diff_mask.shape
    margin_x = int(w * EDGE_IGNORE_RATIO)
    margin_y = int(h * EDGE_IGNORE_RATIO)

    diff_mask[:margin_y, :] = 0
    diff_mask[h - margin_y:, :] = 0
    diff_mask[:, :margin_x] = 0
    diff_mask[:, w - margin_x:] = 0

    contours, _ = cv2.findContours(
        diff_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    changed_area = 0
    filtered_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area > MIN_CONTOUR_AREA:
            changed_area += area
            filtered_contours.append(contour)

    total_area = STANDARD_WIDTH * STANDARD_HEIGHT
    visual_change_percent = (changed_area / total_area) * 100

    return visual_change_percent, diff_mask, filtered_contours


def create_change_overlay(current, contours):
    overlay = current.copy()

    if contours:
        cv2.drawContours(overlay, contours, -1, (0, 0, 255), 2)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), 2)

    return overlay


# =========================
# DECISION ENGINE
# =========================

def calculate_status(qr_text, qr_status, baseline_exists, visual_change_percent):
    if not qr_text:
        return "WAIT_QR", "QR not detected"

    if qr_status != "VALID":
        return "QR_INVALID", "QR ID is not registered in item database"

    if not baseline_exists:
        return "REGISTER_REQUIRED", "Baseline not found. Press S to save baseline"

    if visual_change_percent < PASS_THRESHOLD:
        return "PASS", "Visual condition matched baseline"

    if visual_change_percent < CHECK_THRESHOLD:
        return "CHECK", "Moderate visual difference detected"

    return "REJECT", "Major visual integrity change detected"


def get_status_color(status):
    if status == "PASS":
        return (0, 180, 0)

    if status in [
        "CHECK",
        "WAIT_QR",
        "REGISTER_REQUIRED",
        "READY_TO_INSPECT",
        "REGISTERED",
        "INCOMPLETE"
    ]:
        return (0, 180, 255)

    return (0, 0, 255)


# =========================
# LOGGING
# =========================

def get_esp32_value(esp32_status, *keys, default=""):
    esp32_status = esp32_status or {}
    for key in keys:
        value = esp32_status.get(key)
        if value not in [None, ""]:
            return value
    return default


def get_compact_final_status(visual_status, esp32_status=None):
    esp32_status = esp32_status or {}
    esp_final = str(esp32_status.get("final", "")).strip()
    if esp_final:
        return esp_final
    return visual_status


def log_compact_result(item_id, side, visual_status, change_pct, esp32_status=None):
    esp32_status = esp32_status or {}
    is_new = not COMPACT_LOG_PATH.exists()

    shock_g = get_esp32_value(esp32_status, "maxDynamicG", "max_g", "g", default="")
    tilt_deg = get_esp32_value(esp32_status, "tilt", "tilt_deg", default="")
    final_status = get_compact_final_status(visual_status, esp32_status)

    fieldnames = [
        "timestamp",
        "item_id",
        "side",
        "visual_status",
        "change_pct",
        "shock_g",
        "tilt_deg",
        "final_status",
    ]

    row = {
        "timestamp": get_log_timestamp(),
        "item_id": item_id,
        "side": side,
        "visual_status": visual_status,
        "change_pct": f"{change_pct:.2f}",
        "shock_g": shock_g,
        "tilt_deg": tilt_deg,
        "final_status": final_status,
    }

    with open(COMPACT_LOG_PATH, "a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        if is_new:
            writer.writeheader()
        writer.writerow(row)


def log_result(
    item_id,
    side,
    visual_status,
    visual_change,
    reason,
    qr_status="WAIT",
    item_info=None,
    esp32_status=None
):
    log_path = LOG_DIR / "inspection_log.csv"
    is_new = not log_path.exists()

    item_info = item_info or {}
    esp32_status = esp32_status or {}

    fieldnames = [
        "timestamp",
        "item_id",
        "item_name",
        "category",
        "side",
        "qr_status",
        "visual_status",
        "visual_change",
        "reason",
        "shock_count",
        "max_dynamic_g",
        "tilt_angle",
        "handling_status",
        "final_status",
    ]

    row = {
        "timestamp": get_timestamp(),
        "item_id": item_id,
        "item_name": item_info.get("item_name", ""),
        "category": item_info.get("category", ""),
        "side": side,
        "qr_status": qr_status,
        "visual_status": visual_status,
        "visual_change": f"{visual_change:.2f}",
        "reason": reason,
        "shock_count": esp32_status.get("shock", ""),
        "max_dynamic_g": esp32_status.get("maxDynamicG", ""),
        "tilt_angle": esp32_status.get("tilt", ""),
        "handling_status": esp32_status.get("handling", ""),
        "final_status": esp32_status.get("final", ""),
    }

    with open(log_path, "a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        if is_new:
            writer.writeheader()
        writer.writerow(row)


def print_final_summary(item_id, results):
    print("==============================================")
    print("FINAL VISUAL INSPECTION SUMMARY")
    print("Item ID:", item_id)
    print("----------------------------------------------")

    for side in SIDES:
        if side in results:
            data = results[side]
            print(
                f"{side:<8} | {data['status']:<6} | "
                f"{data['change']:.2f}% | {data['reason']}"
            )
        else:
            print(f"{side:<8} | NOT INSPECTED")

    overall = get_overall_visual_status(results)
    print("----------------------------------------------")
    print("Overall Visual Status:", overall)
    print("==============================================")


# =========================
# UI DRAWING
# =========================

def draw_dashboard(
    frame,
    roi_box,
    item_id,
    current_side,
    baseline_found,
    visual_change,
    status,
    reason,
    mode,
    results,
    esp32_status=None
):
    h, w, _ = frame.shape
    esp32_status = esp32_status or {}

    # Final status hanya ditampilkan sebagai FINAL setelah operator menekan F.
    # Sebelum F, dashboard menampilkan status visual/operasional supaya tidak membingungkan.
    is_final_mode = mode == "FINAL"

    final_status = esp32_status.get("final", "") or status
    handling_status = esp32_status.get("handling", "-")
    shock_count = esp32_status.get("shock", "-")
    max_dynamic_g = esp32_status.get("maxDynamicG", "-")
    tilt_angle = esp32_status.get("tilt", "-")

    dashboard_status = final_status if is_final_mode else status
    dashboard_title = "FINAL STATUS" if is_final_mode else "VISUAL STATUS"
    top_final_text = final_status if is_final_mode else "PRESS F"

    x1, y1, x2, y2 = roi_box

    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
    cv2.putText(
        frame,
        "INSPECTION AREA",
        (x1, max(25, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.rectangle(frame, (0, 0), (w, 210), (20, 20, 20), -1)

    cv2.putText(
        frame,
        "SMART FRAGILE GOODS MULTI-SIDE RE-INSPECTION",
        (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2
    )

    cv2.putText(frame, f"Mode       : {mode}", (15, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Item ID    : {item_id if item_id else 'WAITING QR'}", (15, 73),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Side       : {current_side}", (15, 96),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Baseline   : {'FOUND' if baseline_found else 'NOT FOUND'}", (15, 119),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Change     : {visual_change:.2f}%", (15, 142),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    summary_1 = ""
    summary_2 = ""

    for idx, side in enumerate(SIDES):
        value = status_short(results[side]["status"]) if side in results else "-"
        text = f"{side}:{value} "

        if idx < 3:
            summary_1 += text
        else:
            summary_2 += text

    cv2.putText(frame, summary_1, (15, 164),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    cv2.putText(frame, summary_2, (330, 164),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    cv2.putText(frame, f"Handling  : {handling_status} | Shock:{shock_count} | MaxG:{max_dynamic_g} | Tilt:{tilt_angle}",
                (15, 188), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

    cv2.putText(frame, f"Final     : {top_final_text}",
                (15, 207), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    status_color = get_status_color(dashboard_status)

    cv2.rectangle(frame, (0, h - 140), (w, h), status_color, -1)

    cv2.putText(frame, f"{dashboard_title}: {dashboard_status}", (15, h - 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 3)

    cv2.putText(frame, f"Visual: {status} | Handling: {handling_status} | Shock: {shock_count} | MaxG: {max_dynamic_g} | Tilt: {tilt_angle}",
                (15, h - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Reason: {reason}", (15, h - 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(
        frame,
        "1-6: Side | S: Save | I: Inspect | F: Final | C: Cam Setting | R: Reset | Q: Quit",
        (15, h - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1
    )


# =========================
# CAMERA INIT
# =========================

def init_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(CAMERA_INDEX)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_ZOOM, 0)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

    return cap


# =========================
# MAIN PROGRAM
# =========================

def main():
    load_runtime_config()

    ser = init_serial()

    cap = init_camera()

    if not cap.isOpened():
        print("ERROR: Webcam tidak terbaca.")
        print("Coba ubah CAMERA_INDEX menjadi 1 atau 2.")
        return

    print("Camera width :", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Camera height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("Camera zoom  :", cap.get(cv2.CAP_PROP_ZOOM))

    qr_detector = cv2.QRCodeDetector()
    item_db = load_item_database()

    current_side_index = 0
    current_side = SIDES[current_side_index]

    last_item_id = ""
    last_qr_status = "WAIT"
    last_item_info = None
    last_status = "WAIT_QR"
    last_reason = "QR not detected"
    last_visual_change = 0.0
    last_baseline_found = False
    last_esp32_status = {}
    last_esp32_status_read_time = 0.0

    mode = "LIVE"
    inspection_results = {}

    print("==============================================")
    print("SMART FRAGILE GOODS MULTI-SIDE RE-INSPECTION")
    print("OpenCV Baseline Comparison Started")
    print("==============================================")
    print("Controls:")
    print("1 = TOP")
    print("2 = FRONT")
    print("3 = BACK")
    print("4 = LEFT")
    print("5 = RIGHT")
    print("6 = BOTTOM")
    print("S = Save baseline for selected side")
    print("I = Inspect selected side")
    print("F = Print final summary")
    print("C = Open camera setting")
    print("R = Reset")
    print("Q = Quit")
    print("==============================================")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("ERROR: Frame gagal dibaca.")
            break

        display_frame = frame.copy()

        qr_text, qr_points = detect_qr(frame, qr_detector)

        if qr_text:
            if qr_text != last_item_id:
                last_item_id = qr_text
                qr_valid, last_item_info = validate_item_id(last_item_id, item_db)
                last_qr_status = "VALID" if qr_valid else "INVALID"

                inspection_results = {}
                last_visual_change = 0.0
                mode = "LIVE"

                if qr_valid:
                    item_name = last_item_info.get("item_name", "registered item")
                    last_status = "READY_TO_INSPECT"
                    last_reason = f"QR valid: {item_name}. Select side and inspect"
                    send_status_to_esp32(ser, last_item_id, current_side, "WAIT", "VALID")
                else:
                    last_status = "QR_INVALID"
                    last_reason = "QR ID is not registered in item database"
                    send_status_to_esp32(ser, last_item_id, current_side, "WAIT", "INVALID")

        draw_qr_box(display_frame, qr_points, qr_text)

        roi, roi_box = get_fixed_inspection_roi(frame)
        normalized_roi = normalize_roi(roi)

        baseline = None
        baseline_path = None

        if last_item_id and last_qr_status == "VALID":
            baseline, baseline_path = load_baseline(last_item_id, current_side)

        last_baseline_found = baseline is not None

        key = cv2.waitKey(1) & 0xFF

        if key in [ord("1"), ord("2"), ord("3"), ord("4"), ord("5"), ord("6")]:
            selected_index = key - ord("1")

            if selected_index < len(SIDES):
                current_side_index = selected_index
                current_side = SIDES[current_side_index]
                mode = "LIVE"
                last_visual_change = 0.0

                if last_item_id:
                    if last_qr_status != "VALID":
                        last_status = "QR_INVALID"
                        last_reason = "QR ID is not registered in item database"
                    else:
                        baseline, _ = load_baseline(last_item_id, current_side)

                        if baseline is None:
                            last_status = "REGISTER_REQUIRED"
                            last_reason = f"{current_side} baseline not found. Press S"
                        else:
                            last_status = "READY_TO_INSPECT"
                            last_reason = f"{current_side} baseline found. Press I"
                else:
                    last_status = "WAIT_QR"
                    last_reason = "QR not detected"

                print("Selected side:", current_side)

        elif key == ord("s") or key == ord("S"):
            if not last_item_id:
                last_status = "WAIT_QR"
                last_reason = "Cannot save baseline. QR not detected"
                print(last_reason)
            elif last_qr_status != "VALID":
                last_status = "QR_INVALID"
                last_reason = "Cannot save baseline. QR ID is not registered"
                print(last_reason)
            else:
                saved_path = save_baseline(last_item_id, current_side, normalized_roi)

                last_status = "REGISTERED"
                last_reason = f"{current_side} baseline saved"
                last_visual_change = 0.0
                last_baseline_found = True
                mode = "REGISTER"

                capture_path = CAPTURE_DIR / (
                    f"{safe_filename(last_item_id)}_"
                    f"{current_side}_baseline_{get_timestamp()}.jpg"
                )
                cv2.imwrite(str(capture_path), normalized_roi)

                print("----------------------------------------------")
                print("Baseline saved")
                print("Item ID :", last_item_id)
                print("Side    :", current_side)
                print("Path    :", saved_path)
                print("----------------------------------------------")

        elif key == ord("i") or key == ord("I"):
            mode = "INSPECT"

            if not last_item_id:
                last_status = "WAIT_QR"
                last_reason = "QR not detected"
                last_visual_change = 0.0

            elif last_qr_status != "VALID":
                last_status = "QR_INVALID"
                last_reason = "Inspection blocked. QR ID is not registered"
                last_visual_change = 0.0
                last_baseline_found = False

            else:
                baseline, baseline_path = load_baseline(last_item_id, current_side)

                if baseline is None:
                    last_status = "REGISTER_REQUIRED"
                    last_reason = f"{current_side} baseline not found. Press S"
                    last_visual_change = 0.0
                    last_baseline_found = False

                else:
                    last_baseline_found = True

                    visual_change, diff_mask, contours = compare_with_baseline(
                        baseline,
                        normalized_roi
                    )

                    last_visual_change = visual_change

                    last_status, last_reason = calculate_status(
                        last_item_id,
                        last_qr_status,
                        True,
                        last_visual_change
                    )

                    overlay = create_change_overlay(normalized_roi, contours)

                    capture_path = CAPTURE_DIR / (
                        f"{safe_filename(last_item_id)}_"
                        f"{current_side}_current_{get_timestamp()}.jpg"
                    )
                    overlay_path = CAPTURE_DIR / (
                        f"{safe_filename(last_item_id)}_"
                        f"{current_side}_overlay_{get_timestamp()}.jpg"
                    )
                    diff_path = CAPTURE_DIR / (
                        f"{safe_filename(last_item_id)}_"
                        f"{current_side}_diff_{get_timestamp()}.jpg"
                    )

                    cv2.imwrite(str(capture_path), normalized_roi)
                    cv2.imwrite(str(overlay_path), overlay)
                    cv2.imwrite(str(diff_path), diff_mask)

                    inspection_results[current_side] = {
                        "status": last_status,
                        "change": last_visual_change,
                        "reason": last_reason,
                        "capture": str(capture_path),
                        "overlay": str(overlay_path),
                        "diff": str(diff_path),
                    }

                    send_status_to_esp32(
                        ser,
                        last_item_id,
                        current_side,
                        last_status,
                        last_qr_status
                    )

                    esp32_status = get_esp32_status()
                    if esp32_status:
                        last_esp32_status = esp32_status

                    log_result(
                        last_item_id,
                        current_side,
                        last_status,
                        last_visual_change,
                        last_reason,
                        last_qr_status,
                        last_item_info,
                        esp32_status
                    )
                    log_compact_result(
                        last_item_id,
                        current_side,
                        last_status,
                        last_visual_change,
                        esp32_status
                    )

                    print("----------------------------------------------")
                    print("Inspection Result")
                    print("Item ID       :", last_item_id)
                    print("Side          :", current_side)
                    print("Baseline      :", baseline_path)
                    print("Visual Change :", f"{last_visual_change:.2f}%")
                    print("Status        :", last_status)
                    print("Reason        :", last_reason)
                    print("Capture       :", capture_path)
                    print("Overlay       :", overlay_path)
                    print("Diff          :", diff_path)
                    print("----------------------------------------------")

                    if SHOW_DIFF_WINDOW:
                        cv2.imshow("Current ROI with Change Overlay", overlay)
                        cv2.imshow("Difference Mask", diff_mask)

        elif key == ord("f") or key == ord("F"):
            if not last_item_id:
                print("Cannot print summary. QR not detected.")
                last_status = "WAIT_QR"
                last_reason = "QR not detected"
            elif last_qr_status != "VALID":
                last_status = "QR_INVALID"
                last_reason = "Cannot finalize. QR ID is not registered"
                send_status_to_esp32(ser, last_item_id, "ALL", "WAIT", "INVALID")
            else:
                overall_status = get_overall_visual_status(inspection_results)
                print_final_summary(last_item_id, inspection_results)

                last_status = overall_status
                last_reason = f"Overall visual status: {overall_status}"
                mode = "FINAL"

                esp_status = overall_status
                if overall_status == "INCOMPLETE":
                    esp_status = "CHECK"

                send_status_to_esp32(ser, last_item_id, "ALL", esp_status, last_qr_status)
                esp32_status = get_esp32_status()
                if esp32_status:
                    last_esp32_status = esp32_status

                if inspection_results:
                    avg_change = sum(data["change"] for data in inspection_results.values()) / len(inspection_results)
                else:
                    avg_change = 0.0

                log_result(
                    last_item_id,
                    "ALL",
                    overall_status,
                    avg_change,
                    last_reason,
                    last_qr_status,
                    last_item_info,
                    esp32_status
                )
                log_compact_result(
                    last_item_id,
                    "ALL",
                    overall_status,
                    avg_change,
                    esp32_status
                )

        elif key == ord("c") or key == ord("C"):
            cap.set(cv2.CAP_PROP_SETTINGS, 1)
            print("Camera settings opened.")

        elif key == ord("r") or key == ord("R"):
            mode = "LIVE"
            last_status = "WAIT_QR"
            last_reason = "Reset complete"
            last_visual_change = 0.0
            last_item_id = ""
            last_qr_status = "WAIT"
            last_item_info = None
            last_baseline_found = False
            last_esp32_status = {}
            last_esp32_status_read_time = 0.0
            inspection_results = {}

            send_reset_to_esp32(ser)

            print("System reset.")

        elif key == ord("q") or key == ord("Q"):
            break

        if mode == "LIVE":
            if not last_item_id:
                last_status = "WAIT_QR"
                last_reason = "QR not detected"
                last_visual_change = 0.0
                last_baseline_found = False
            else:
                if last_qr_status != "VALID":
                    last_status = "QR_INVALID"
                    last_reason = "QR ID is not registered in item database"
                    last_baseline_found = False
                else:
                    baseline, _ = load_baseline(last_item_id, current_side)
                    baseline_found = baseline is not None
                    last_baseline_found = baseline_found

                    if not baseline_found:
                        last_status = "REGISTER_REQUIRED"
                        last_reason = f"{current_side} baseline not found. Press S"
                    else:
                        last_status = "READY_TO_INSPECT"
                        last_reason = f"{current_side} baseline found. Press I"

        # Update status gabungan dari ESP32 supaya final decision juga tampil di OpenCV.
        # Data ini berisi handlingStatus dan finalStatus hasil gabungan visual + MPU6050.
        now_time = time.time()
        if WIFI_ENABLED and last_item_id and (now_time - last_esp32_status_read_time >= ESP32_STATUS_REFRESH_SEC):
            esp32_snapshot = get_esp32_status()
            if esp32_snapshot:
                last_esp32_status = esp32_snapshot
            last_esp32_status_read_time = now_time

        draw_dashboard(
            display_frame,
            roi_box,
            last_item_id,
            current_side,
            last_baseline_found,
            last_visual_change,
            last_status,
            last_reason,
            mode,
            inspection_results,
            last_esp32_status
        )

        cv2.imshow("Smart Fragile Goods Multi-Side Re-Inspection", display_frame)
        cv2.imshow("Normalized Inspection ROI", normalized_roi)

    cap.release()

    if ser is not None:
        ser.close()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()