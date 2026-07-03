import csv
from pathlib import Path

try:
    import qrcode
except ImportError:
    raise SystemExit("Install dulu: pip install qrcode[pil]")

ITEM_DB_PATH = Path("items.csv")
OUTPUT_DIR = Path("qrcodes")
OUTPUT_DIR.mkdir(exist_ok=True)

if not ITEM_DB_PATH.exists():
    raise SystemExit("items.csv tidak ditemukan. Jalankan dari folder python_cv.")

with open(ITEM_DB_PATH, "r", newline="", encoding="utf-8") as file:
    reader = csv.DictReader(file)
    for row in reader:
        item_id = row.get("item_id", "").strip()
        if not item_id:
            continue

        img = qrcode.make(item_id)
        out_path = OUTPUT_DIR / f"{item_id}.png"
        img.save(out_path)
        print(f"QR saved: {out_path}")
