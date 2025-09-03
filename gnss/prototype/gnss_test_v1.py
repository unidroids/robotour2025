#!/usr/bin/env python3
import serial
import pynmea2

# Nastav zařízení a rychlost portu podle GNSS modulu
# (u-blox moduly obvykle 9600 nebo 38400 baud, C102-F9R často 115200)
SERIAL_PORT = "/dev/gnss"
BAUDRATE = 115200

def run():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print("📡 GNSS prototyp běží...")
        while True:
            try:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line.startswith("$"):
                    continue

                msg = pynmea2.parse(line)

                # Pozice a výška (GGA zpráva)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    lat = msg.latitude
                    lon = msg.longitude
                    alt = msg.altitude
                    print(f"🌍 Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt} m")

                # Rychlost a směr (RMC zpráva obsahuje obojí)
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    speed_knots = float(msg.spd_over_grnd) if msg.spd_over_grnd else 0.0
                    speed_kmh = speed_knots * 1.852
                    heading = float(msg.true_course) if msg.true_course else 0.0
                    print(f"🚗 Speed: {speed_kmh:.2f} km/h, Heading: {heading:.2f}°")

            except pynmea2.ParseError:
                continue
            except KeyboardInterrupt:
                print("🛑 Ukončeno uživatelem.")
                break

if __name__ == "__main__":
    run()
