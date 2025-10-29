# GNSS – socket služba (Robotour 2025)

Port: 9006  
Zařízení: u-blox F9R / D9S (rozpoznání přes UBX-MON-VER)

## Příkazy

- `PING` → `PONG`
- `START` → inicializace GNSS, detekce typu
- `STATE` → běžný stav (satellites, fix, IMU stav)
- `CALIBRATE` → spustí kalibraci IMU (F9R)
- `DATA` → 10Hz výpis fixu (lat, lon, alt, heading, speed, time)
- `EXIT` → ukončí spojení

## Testování
```bash
nc 127.0.0.1 9006
PING
START
STATE
DATA
EXIT
