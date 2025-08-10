#!/usr/bin/env python3
# DataLoger: čeká na data_ready_event, čte poslední payload a zapisuje do JSONL logu
import os, time, json, threading
from datetime import datetime
from gamepad_core import data_ready_event, get_latest_payload, stop_event

LOG_DIR = "/data/robot/gamepad"
WRITE_PAUSE_SEC = 0.008  # krátká pauza, aby zápis nekonkuroval odpovědi DATA

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
    print("[DATALOGER] Vlákno START")
    try:
        _open_new_file()
        while not stop_event.is_set():
            if not data_ready_event.wait(timeout=1.0):
                continue
            data_ready_event.clear()
            payload = get_latest_payload()
            time.sleep(WRITE_PAUSE_SEC)
            try:
                _log_fp.write(json.dumps(payload, ensure_ascii=False) + "\n")
            except Exception as e:
                print(f"[DATALOGER] Chyba zápisu: {e}")
    except Exception as e:
        print(f"[DATALOGER] Chyba ve vlákně: {e}")
    finally:
        try:
            if _log_fp: _log_fp.close()
        except Exception:
            pass
        print("[DATALOGER] Vlákno STOP")

def start_dataloger_once():
    global dataloger_thread_started
    if dataloger_thread_started:
        return False
    dataloger_thread_started = True
    t = threading.Thread(target=dataloger_loop, daemon=True)
    t.start()
    return True
