#!/usr/bin/env python3
"""Uploads a firmware image to the ESP32-S3 over its web OTA endpoint and waits for it
to come back, the same as the page's "Tải lên & khởi động lại" button.

  python tools/ota_upload.py <ip> [build/lds_motor_idf.bin]
"""
import sys
import time
import urllib.request
from pathlib import Path

host = sys.argv[1]
image = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent.parent / "build/lds_motor_idf.bin"
data = image.read_bytes()

t0 = time.time()
req = urllib.request.Request(f"http://{host}/ota", data=data, method="POST",
                             headers={"Content-Type": "application/octet-stream"})
print(f"uploading {len(data)} bytes: {urllib.request.urlopen(req, timeout=180).read().decode()} "
      f"({time.time() - t0:.1f} s)")

time.sleep(4)
for _ in range(60):
    try:
        urllib.request.urlopen(f"http://{host}/", timeout=3).read()
        print(f"web page back after {time.time() - t0:.0f} s")
        break
    except OSError:
        time.sleep(1)
else:
    sys.exit("device did not come back within 60 s (the bootloader rolls back a new app that never confirms itself)")
