#!/usr/bin/env python3
import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/gnss1"
BAUD = 921600
MEASURE_WINDOW_S = 0.010
LOG_FILE = "usb_uart_bursts.csv"

def main():
    ser = serial.Serial(
        PORT, BAUD,
        timeout=0,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    print(f"Listening on {PORT} @ {BAUD}")
    print(f"Logging to {LOG_FILE}")

    state = 0
    t0 = None
    last_empty = time.perf_counter_ns()
    cnt = 0

    # poslední zalogovaná hodnota in_waiting v rámci okna
    last_logged_n = None

    with open(LOG_FILE, "w", buffering=1) as f:
        f.write("phase,ts_ns,dt_us,in_waiting\n")

        try:
            while True:
                now = time.perf_counter_ns()
                n = ser.in_waiting

                if state == 0:
                    # čekáme, až se z prázdného bufferu objeví data
                    if n == 0:
                        last_empty = now
                        last_logged_n = 0
                    else:
                        # začátek nového měřicího okna
                        dt_us = (now - last_empty) / 1000.0
                        f.write(f"start,{now},{dt_us:.1f},{n}\n")
                        t0 = now
                        state = 1
                        last_logged_n = n   # tady jsme zalogovali
                else:
                    # stav 1 – měřicí okno po detekci prvních dat
                    elapsed_s = (now - t0) / 1e9
                    if elapsed_s < MEASURE_WINDOW_S:
                        # loguj jen při změně počtu bytů
                        if n != last_logged_n:
                            dt_us = (now - t0) / 1000.0
                            f.write(f"window,{now},{dt_us:.1f},{n}\n")
                            last_logged_n = n
                    else:
                        # konec okna – buffer vyčteme a zaznamenáme
                        if n > 0:
                            dt_us = (now - t0) / 1000.0
                            f.write(f"drain_begin,{now},{dt_us:.1f},{n}\n")
                            data = ser.read(n)
                            now2 = time.perf_counter_ns()
                            n2 = ser.in_waiting
                            dt2_us = (now2 - t0) / 1000.0
                            f.write(f"drain_end,{now2},{dt2_us:.1f},{n2}\n")
                        else:
                            dt_us = (now - t0) / 1000.0
                            f.write(f"drain_empty,{now},{dt_us:.1f},0\n")

                        # další cyklus
                        cnt += 1
                        if cnt >= 10:      # tvoje omezení na 5 měření
                            break

                        state = 0
                        last_empty = time.perf_counter_ns()
                        last_logged_n = None

                #if state == 0:
                #    time.sleep(0.0005)  # 0.5 ms

        except KeyboardInterrupt:
            print("\nStopping...")

if __name__ == "__main__":
    main()
