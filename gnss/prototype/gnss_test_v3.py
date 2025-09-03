#!/usr/bin/env python3
import serial
import pynmea2

SERIAL_PORT = "/dev/gnss"
BAUDRATE = 115200

def run():
    lat, lon, alt = None, None, None
    speed, heading = None, None
    fix_quality = 0

    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print(f"📡 Čtu GNSS z {SERIAL_PORT} ...")
        while True:
            try:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line.startswith("$"):
                    continue

                msg = pynmea2.parse(line)

                if isinstance(msg, pynmea2.types.talker.GGA):
                    lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
                    fix_quality = msg.gps_qual if msg.gps_qual is not None else 0

                elif isinstance(msg, pynmea2.types.talker.RMC):
                    if msg.spd_over_grnd:
                        speed = float(msg.spd_over_grnd) * 1.852  # uzly → km/h
                    if msg.true_course:
                        heading = float(msg.true_course)

                if lat is not None and lon is not None:
                    alt_str = f"{alt:.1f}" if alt is not None else "---"
                    speed_str = f"{speed:.2f}" if speed is not None else "---"
                    heading_str = f"{heading:.2f}" if heading is not None else "---"

                    # textový popis kvality fixu
                    fix_map = {
                        0: "NOFIX",
                        1: "GPS",
                        2: "DGPS",
                        4: "RTK_FIX",
                        5: "RTK_FLOAT",
                        6: "DEAD_RECKONING",
                    }
                    fix_desc = fix_map.get(fix_quality, f"Q{fix_quality}")

                    print(
                        f"🌍 lat={lat:.6f}, lon={lon:.6f}, alt={alt_str} m, "
                        f"🚗 speed={speed_str} km/h, heading={heading_str}°, fix={fix_desc}"
                    )

            except pynmea2.ParseError:
                continue
            except KeyboardInterrupt:
                print("🛑 Ukončeno uživatelem.")
                break

if __name__ == "__main__":
    run()
