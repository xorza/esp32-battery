#!/usr/bin/env python3
"""Poll battery.lan API endpoints, stream to local files with reconnect.

/api/log    -> ring buffer of ESP_LOG output. Dedupe overlap, append new bytes.
/api/errors -> JSON snapshot of INA/XY error counters. Append on change.

Writes:
  logs/http_stream.log    - append-only deduped log content + status markers
  logs/http_snapshot.txt  - last full /api/log ring (overwritten each poll)
  logs/errors_stream.log  - append-only /api/errors changes (timestamped)
  logs/errors_snapshot.json - last /api/errors body (overwritten each poll)
"""
import os
import ssl
import time
import urllib.request
import urllib.error
from datetime import datetime

HOST = "https://battery.lan"
INTERVAL = 30.0
TIMEOUT = 8.0
DIR = os.path.dirname(os.path.abspath(__file__))
LOG_STREAM = os.path.join(DIR, "http_stream.log")
LOG_SNAP = os.path.join(DIR, "http_snapshot.txt")
ERR_STREAM = os.path.join(DIR, "errors_stream.log")
ERR_SNAP = os.path.join(DIR, "errors_snapshot.json")

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

def stamp():
    return datetime.now().strftime("%H:%M:%S")

def fetch(path: str) -> str:
    req = urllib.request.Request(HOST + path)
    with urllib.request.urlopen(req, timeout=TIMEOUT, context=ctx) as r:
        return r.read().decode("utf-8", errors="replace")

def find_new_suffix(prev: str, cur: str) -> str:
    if not prev:
        return cur
    max_len = min(len(prev), 4096)
    for n in range(max_len, 64, -64):
        anchor = prev[-n:]
        idx = cur.rfind(anchor)
        if idx != -1:
            return cur[idx + n:]
    return cur

def append(path: str, line: str):
    with open(path, "a") as f:
        f.write(line)
        if not line.endswith("\n"):
            f.write("\n")

prev_log = ""
prev_err = ""
log_connected = True
err_connected = True

append(LOG_STREAM, f"=== monitor started {datetime.now().isoformat()} ===")
append(ERR_STREAM, f"=== monitor started {datetime.now().isoformat()} ===")

while True:
    # /api/log
    try:
        data = fetch("/api/log")
        if not log_connected:
            append(LOG_STREAM, f"--- [{stamp()}] /api/log reconnected ---")
            log_connected = True
        with open(LOG_SNAP, "w") as f:
            f.write(data)
        new = find_new_suffix(prev_log, data)
        if new:
            with open(LOG_STREAM, "a") as f:
                f.write(new)
        prev_log = data
    except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as e:
        if log_connected:
            append(LOG_STREAM, f"--- [{stamp()}] /api/log disconnected: {e} ---")
            log_connected = False
    except Exception as e:
        append(LOG_STREAM, f"--- [{stamp()}] /api/log error: {type(e).__name__}: {e} ---")

    # /api/errors
    try:
        data = fetch("/api/errors")
        if not err_connected:
            append(ERR_STREAM, f"--- [{stamp()}] /api/errors reconnected ---")
            err_connected = True
        with open(ERR_SNAP, "w") as f:
            f.write(data)
        if data != prev_err:
            append(ERR_STREAM, f"[{stamp()}] {data}")
        prev_err = data
    except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as e:
        if err_connected:
            append(ERR_STREAM, f"--- [{stamp()}] /api/errors disconnected: {e} ---")
            err_connected = False
    except Exception as e:
        append(ERR_STREAM, f"--- [{stamp()}] /api/errors error: {type(e).__name__}: {e} ---")

    time.sleep(INTERVAL)
