import time
import logging
import logging.handlers
import queue
import threading
from collections import deque
import sys

ITER = 1000

# ---- Simulace reálné INTENT zprávy ----
vals = {
    "dist_eq": 3.789, "dist_hav": 3.789, "dist_ema": 3.775,
    "brg_eq": 11.03, "brg_hav": 11.03, "heading_deg": 13.2,
    "err_eq": -2.2, "err_hav": -2.2, "speed_mps": 0.012,
    "hacc_m": 0.027, "dx": 0.72, "dy": 3.73, "radius": 1.0,
    "rotate_only": True, "left": 0, "right": 0
}

def build_intent_str(v):
    return (
        "INTENT"
        f" dist_eq={v['dist_eq']:.2f}m dist_hav={v['dist_hav']:.2f}m dist_ema={v['dist_ema']:.2f}m"
        f" brg_eq={v['brg_eq']:.1f}° brg_hav={v['brg_hav']:.1f}°"
        f" head={v['heading_deg']:.1f}° err_eq={v['err_eq']:.1f}° err_hav={v['err_hav']:.1f}°"
        f" speed={v['speed_mps']:.2f}m/s hAcc={v['hacc_m']:.3f}m"
        f" dx={v['dx']:.2f} dy={v['dy']:.2f} radius={v['radius']:.2f}m"
        f" rotate_only={v['rotate_only']} PWM=({v['left']},{v['right']})"
    )

def bench_build_only():
    start = time.perf_counter()
    for _ in range(ITER):
        _ = build_intent_str(vals)
    return time.perf_counter() - start

def bench_print_to_file(path, do_flush=False):
    old_stdout = sys.stdout
    try:
        with open(path, "w") as f:
            sys.stdout = f
            start = time.perf_counter()
            for _ in range(ITER):
                print(build_intent_str(vals), flush=do_flush)
            elapsed = time.perf_counter() - start
            f.flush()
        return elapsed
    finally:
        sys.stdout = old_stdout

def bench_logging_sync_to_file(path):
    logger = logging.getLogger("sync_file_bench")
    logger.handlers.clear()
    logger.setLevel(logging.INFO)
    fh = logging.FileHandler(path, mode="w")
    fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.addHandler(fh)
    start = time.perf_counter()
    for _ in range(ITER):
        logger.info(build_intent_str(vals))
    elapsed = time.perf_counter() - start
    for h in list(logger.handlers):
        h.flush()
        h.close()
        logger.removeHandler(h)
    return elapsed

def bench_logging_async_to_file(path):
    logger = logging.getLogger("async_file_bench")
    logger.handlers.clear()
    logger.setLevel(logging.INFO)

    q = queue.Queue()
    qh = logging.handlers.QueueHandler(q)
    logger.addHandler(qh)

    fh = logging.FileHandler(path, mode="w")
    fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    listener = logging.handlers.QueueListener(q, fh)
    listener.start()

    start = time.perf_counter()
    for _ in range(ITER):
        logger.info(build_intent_str(vals))
    elapsed = time.perf_counter() - start

    listener.stop()
    fh.flush()
    fh.close()
    logger.removeHandler(qh)
    return elapsed

def bench_queue_put_only():
    q = queue.Queue()
    start = time.perf_counter()
    for _ in range(ITER):
        q.put(build_intent_str(vals))
    return time.perf_counter() - start

def bench_ring_buffer_locked():
    ring = deque(maxlen=100)
    lock = threading.Lock()
    start = time.perf_counter()
    for _ in range(ITER):
        s = build_intent_str(vals)
        with lock:
            ring.append(s)
    return time.perf_counter() - start

if __name__ == "__main__":
    res_build = bench_build_only()
    res_print_file = bench_print_to_file("/tmp/bench_print_file.log", do_flush=False)
    res_print_file_flush = bench_print_to_file("/tmp/bench_print_file_flush.log", do_flush=True)
    res_log_sync_file = bench_logging_sync_to_file("/tmp/bench_logging_sync.log")
    res_log_async_file = bench_logging_async_to_file("/tmp/bench_logging_async.log")
    res_queue_put = bench_queue_put_only()
    res_ring = bench_ring_buffer_locked()

    print(f"=== Výsledky (INTENT zpráva, {ITER} iterací; I/O = soubor) ===")
    print(f"build only:               {res_build:.4f} s  ({res_build/ITER*1000:.3f} ms/zpráva)")
    print(f"print → file:             {res_print_file:.4f} s  ({res_print_file/ITER*1000:.3f} ms/zpráva)")
    print(f"print → file (flush):     {res_print_file_flush:.4f} s  ({res_print_file_flush/ITER*1000:.3f} ms/zpráva)")
    print(f"logging sync → file:      {res_log_sync_file:.4f} s  ({res_log_sync_file/ITER*1000:.3f} ms/zpráva)")
    print(f"logging async → file:     {res_log_async_file:.4f} s  ({res_log_async_file/ITER*1000:.3f} ms/zpráva)")
    print(f"queue.Queue.put() only:   {res_queue_put:.4f} s  ({res_queue_put/ITER*1000:.3f} ms/zpráva)")
    print(f"ring buffer (deque+lock): {res_ring:.4f} s  ({res_ring/ITER*1000:.3f} ms/zpráva)")
