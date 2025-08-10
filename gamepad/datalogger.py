#!/usr/bin/env python3
# DataLogger: čeká na NOVÁ data (msg_index) a zapisuje JSONL log
import os, time, threading
from datetime import datetime
import gamepad_core as core

LOG_DIR = "/data/robot/gamepad"
WRITE_PAUSE_SEC = 0.0  # není nutné brzdit

dataloger_thread_started = False
_log_fp = None

def _open_new_file():
    global _log_fp
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"gamepad_{stamp}.log")
    _log_fp = open(path, "a", buffering=1, encoding="utf-8")
    print(f"[DATALOGER] Log file: {path}")

def dataloger_loop():
    global dataloger_thread_started
    print("[DATALOGER] Vlákno START")
    try:
        _open_new_file()
        last_idx = 0
        while not core.stop_event.is_set():
            with core.cond:
                core.cond.wait_for(lambda: core.stop_event.is_set() or core.msg_index > last_idx, timeout=0.5)
                if core.stop_event.is_set():
                    break
                if core.msg_index <= last_idx or core.latest_payload is None:
                    continue
                last_idx = core.msg_index
                payload = core.latest_payload  # str
            if payload is not None:
                _log_fp.write(payload + "\n")
            if WRITE_PAUSE_SEC:
                time.sleep(WRITE_PAUSE_SEC)
    finally:
        if _log_fp:
            _log_fp.close()
        dataloger_thread_started = False
        print("[DATALOGER] Vlákno STOP")

def start_dataloger_once():
    global dataloger_thread_started
    if dataloger_thread_started:
        return False
    dataloger_thread_started = True
    t = threading.Thread(target=dataloger_loop, daemon=True)
    t.start()
    return True
