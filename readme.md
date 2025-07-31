# Unidroids – Robotour 2025

Tým **Unidroids** je novým účastníkem soutěže **Robotour 2025**. Náš robot je navržen s důrazem na modularitu, jednoduchost údržby a přehlednou softwarovou architekturu.

## 🚗 Hardwarová výbava

| Komponenta         | Popis                                                            |
| ------------------ | ---------------------------------------------------------------- |
| Výpočetní jednotka | NVIDIA Jetson Orin Nano 8GB, JetPack 6.2                         |
| Kamery             | 2× Waveshare IMX219, 200°, CSI (stereo pohled dolů)              |
| LiDAR              | Unitree L2 (Ethernet)                                            |
| GNSS               | C102-F9R GNSS + IMU (USB)                                        |
| Mobilní základna   | Hoverboard s upraveným firmware (řízení přes USB sériovou linku) |
| Ovládání           | Gamepad a Android telefon (Infinix Smart 8)                      |
| Úložiště           | SSD Lexar NM620 2TB (Jetson root + data)                         |

## 🧠 Softwarová architektura

* **Operační systém:** Ubuntu (JetPack 6)
* **Programovací jazyk:** Python 3.10
* **Vývojové prostředí:** VSCode Remote – SSH (headless přes USB-C gadget mód)
* **Struktura:** oddělené služby pro kamery, LiDAR, řízení, GNSS a centrální FastAPI server
* **Komunikace:** sockety (mezi službami), HTTP pouze pro externí API (FastAPI)
* **Záznamy:** logování obrazu, lidarových dat, pohybu a GNSS do /robot/data/logs

## 🎥 Registrační video

YouTube video: [ROBOTOUR 2025 REGISTRATION – Unidroids](https://www.youtube.com/watch?v=jIPX0ZO7tB0)

Obsahuje:

* Detekci překážky pomocí LiDARu
* Ukázku funkčního Emergency STOP tlačítka

## 📁 Struktura projektu

```
/robot/opt/projects/robotour
├── server/          # socket + HTTP servery
├── journey/         # plánování a workflow
├── camera/          # čtení, segmentace a logování kamer
├── lidar/           # TCP server pro lidar, transformace a analýza bodů
├── gnns/            # čtení pozice a rychlosti
└── install/         # systemd skripty, udev, konfigurace
```

Repozitář: [https://github.com/unidroids/robotour2025](https://github.com/unidroids/robotour2025)
