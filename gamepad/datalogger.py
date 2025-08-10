#!/usr/bin/env python3
# DataLoger: čeká na data_ready_event, čte poslední payload a zapisuje do JSONL logu
import os, time, json, threading
from datetime import datetime
from gamepad_core import cond, stop_event
from gamepad_core import latest_payload, msg_index   # čteme pod zámkem

LOG_DIR = "/data/robot/gamepad"
WRITE_PAUSE_SEC = 0.008


dataloger_thread_started = False
_log_fp = None


def _open_new_file():
    global _log_fp
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"gamepad_{stamp}.log")
    _log_fp = open(path, "a", buffering=1)
    print(f"[DATALOGER] Log file: {path}")

def dataloger_loop():
    global dataloger_thread_started, cond, stop_event, latest_payload
    print("[DATALOGER] Vlákno START")
    try:
        _open_new_file()
        while not stop_event.is_set():
            with cond:
                cond.wait_for(lambda: stop_event.is_set())
                if stop_event.is_set():
                    break
                payload = latest_payload      # kopie pod zámkem
            time.sleep(WRITE_PAUSE_SEC)       # neblokuj DATA odpovědi
            _log_fp.write(payload + "\n")
    finally:
        if _log_fp: _log_fp.close()
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
