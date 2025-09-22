# GNSS služba pro ZED‑F9R — README

Tento projekt je Python služba pro u‑blox ZED‑F9R (HPS 1.40), která:

* komunikuje v protokolu UBX přes sériový port (PySerial),
* spouští TCP server pro příjem jednoduchých příkazů (např. `START`, `STOP`, `PRIO ON|OFF`, `ODO …`),
* parsuje širokou sadu UBX zpráv a vypisuje je jako přehledné jednorázkové logy (stdout),
* periodicky „polluje“ vybrané systémové zprávy,
* umí posílat odometrická data (ESF‑MEAS typy 8 a 9) ze softwaru do F9R,
* drží konfigurační profily (CFG‑\*) pro fyzickou instalaci a provoz.

> **Pozn.** Všechny příklady a názvy tříd/handlerů odpovídají poslední dodané verzi. Pokud se struktura v budoucnu změní, berte README jako vodítko „co a proč“.

---

## Struktura projektu

```
<repo-root>/
├─ main.py                # Entrypoint – spustí službu a vlákna
├─ service.py             # Třída GnssService (I/O, dispatcher, poller, TCP server)
├─ client_handler.py      # Obsluha TCP klientů a příkazů
├─ ubx_dispacher.py       # Dispatcher pro UBX zprávy
├─ poller.py              # Rotující poller na UBX zprávy
├─ gnss_serial.py         # Serial reader/IO, framing UBX zpráv
├─ handlers/              # Handlery pro jednotlivé UBX zprávy
│  ├─ ack_handler.py      # UBX-ACK-ACK / UBX-ACK-NAK
│  ├─ mon_sys_handler.py  # UBX-MON-SYS
│  ├─ mon_comms_handler.py# UBX-MON-COMMS
│  ├─ esf_status_handler.py# UBX-ESF-STATUS
│  ├─ esf_meas_handler.py # UBX-ESF-MEAS (I/O)
│  ├─ esf_raw_handler.py  # UBX-ESF-RAW (IMU raw)
│  ├─ esf_ins_handler.py  # UBX-ESF-INS (dynamika vozidla)
│  ├─ nav_att_handler.py  # UBX-NAV-ATT (roll/pitch/heading)
│  ├─ nav_velned_handler.py# UBX-NAV-VELNED (rychlosti v NED)
│  ├─ nav_pvat_handler.py # UBX-NAV-PVAT (kombinace PVA+čas)
│  └─ …                   # Další handlery, lze snadno přidat
├─ builders/              # Buidery pro jednotlivé UBX zprávy (odesílání/kompletní generování)
│  ├─ __init__.py         #
│  ├─ odo.py              # Odometrická data (UBX-ESF-MEAS typy 8/9)
│  ├─ perfect.py          # PointPerfect zprávy
│  ├─ mon_sys.py          # Systémové UBX zprávy
│  └─ …                   # Další generátory zpráv
└─ README.md              # Tento soubor (popis projektu, instrukce)
```

> Složky `ubx/` a `handlers/` mohou být v projektu pojmenované jinak; důležité je, že dispatch mapa (viz níže) ví, jaký handler zavolat pro daný `(msg_class, msg_id)`.

---

## Vlákna a fronty

Služba je záměrně jednoduchá a čitelná – používá několik jasně oddělených vláken:

1. **SerialReader (GnssSerialIO)**

   * Čte byty ze sériového portu a předává je UBX parseru.
   * V témže vlákně (nebo v parser callbacku) se po sestavení celé UBX zprávy volá **Dispatcher**.

2. **Dispatcher**

   * Slabě spřažená mapa `(msg_class, msg_id) → handler`.
   * Každý handler implementuje `handle(msg_class, msg_id, payload)` a řeší zodpovědně kontroly délek a `struct.unpack`.

3. **RotatingPoller**

   * Periodicky posílá dotazy/příkazy na zařízení – typicky v kruhu:

     * `UBX-MON-SYS`, `UBX-MON-COMMS`, `UBX-ESF-STATUS` (+ případně další polovatelné zprávy),
     * **Pozor:** `UBX-ESF-RAW` není polovatelná; je čistě „Output“ a řídí se konfigurací výstupní frekvence (CFG‑MSGOUT‑UBX\_ESF\_RAW‑\*).

4. **TCP Server**

   * Naslouchá na daném portu (např. `9006`) a pro každý příchozí socket spouští **ClientHandler**.

5. **ClientHandler (per connection)**

   * Parsuje příkazy po řádcích a volá metody služby (START/STOP, PRIO ON|OFF, ODO …, apod.).

6. **(Volitelně) Binární FIFO streaming**

   * Některé handlery (např. `NavVelNedHandler`) mohou posílat malý binární záznam do fronty `bin_stream_fifo` pro další konsumenty (viz příklad níže). Fronta je typicky malá a „dropuje“ nejstarší záznamy při zaplnění.

### Fronty

* `bin_stream_fifo`: krátká, lockovaná FIFO pro rychlé telemetrické výstupy (rychlosti/heading/nejistoty…),
* (dle potřeby) další jednosměrné fronty mezi vlákny.

---

## Příkazy TCP služby (nc/netcat)

Server poslouchá na konfigurovatelném portu (např. `9006`). Příkazy jsou **řádkové** a **bez parametrických uvozovek**.

* `START` – spustí poller, aktivuje zpracování.
* `STOP` – zastaví poller.
* `PRIO ON` / `PRIO OFF` – nastaví `CFG-RATE-NAV_PRIO` (UBX‑CFG‑VALSET), očekává `ACK`/`NAK`.
* `ODO <timeTagHex> <rlTicksHex> <rlDir> <rrTicksHex> <rrDir>` – pošle ESF‑MEAS s typy 8 a 9 (rear‑left/right wheel ticks); adresováno pro RLM dynamický model. `timeTag` v ms (TTAG, musí být kompatibilní s `CFG‑SFCORE‑SEN_TTAG_FACT`), směr 0/1.

**Příklad:**

```bash
echo "START" | nc -q0 127.0.0.1 9006
```

**Sekvenční start dvou služeb (např. 9003 → 9007) s kontrolou odpovědi):**

```bash
#!/usr/bin/env bash

send_start(){
  local PORT="$1"
  local OUT
  OUT=$(printf "START\n" | nc -w 2 127.0.0.1 "$PORT") || return 1
  # úspěch bereme, pokud služba něco rozumného vrátí / neukončí se s chybou
  [[ -n "$OUT" ]] && return 0 || return 1
}

if send_start 9003; then
  echo "9003 OK → startuji 9007"
  send_start 9007 && echo "9007 OK" || echo "9007 SELHAL"
else
  echo "9003 SELHAL"
fi
```

---

## Handlery UBX zpráv (výběr)

Každý handler dělá tři věci: **kontrola délky**, **bezpečný `struct.unpack`**, **jednořádkový, čitelný log**. Níže přehled těch klíčových:

* **ACK/NAK** – `UBX-ACK-ACK` (0x05 0x01), `UBX-ACK-NAK` (0x05 0x00). Po změně konfigurace se zvyšuje čítač ACK a vypíše se potvrzení včetně adresovaného `clsID/msgID`.

* **MON‑SYS** – CPU/mem/IO využití, teplota, uptime, agreguje do jednoho řádku typu:

  `ver=1 boot=4 cpu=57%/61% mem=32%/36% io=99%/99% t=31°C errors=889 warnings=0 notices=14 up=893s`

* **MON‑COMMS** – informace o portech a TX/RX chybách.

* **ESF‑STATUS** – kompletní hlavička i per‑sensor řádky. Hlavní řádek obsahuje:

  `iTOW=… ver=… fusionMode=… (Fusion|Init|Suspended|Disabled) wtInit=… mntAlg=… insInit=… imuInit=… numSens=N`

  a následně N krát senzory s `type, used, ready, calibStatus, timeStatus, freq, faults`.

* **ESF‑MEAS (I/O)** – vstup/výstup měření (vstup: odometrie typy 8/9; výstup: IMU a další měření). Přehledně vypisuje typy a dekódované hodnoty (ticks + směr, nebo surové hodnoty).

* **ESF‑RAW (Output)** – surové IMU vzorky (accX/Y/Z v m/s², gyroX/Y/Z v deg/s, teplota). **Nelze pollovat**; frekvence se řídí přes `CFG-MSGOUT-UBX_ESF_RAW-*`.

* **ESF‑INS (Output)** – kompenzované úhlové rychlosti a zrychlení (vozidlový rám), včetně `bitfield0` validit.

* **NAV‑ATT (P/P/R)** – roll, pitch, heading + jejich přesnosti, jednou za sekundu (podle `iTOW`):

  `[NAV-ATT] iTOW=… roll=…° pitch=…° heading=…° accRoll=… accPitch=… accHeading=…`

* **NAV‑VELNED** – rychlosti v NED, ground speed, course/heading; umí také posílat binární záznam do FIFO.

* **NAV‑PVAT** – kombinované **P**osition/**V**elocity/**A**ttitude/**T**ime + přesnosti, **116 B** payload. Výstup je jednorázkový (viz níže příklad formátu).

* **NAV‑HPPOSLLH** – přesná poloha s hAcc/vAcc v mm.

> Při přidávání nového handleru začněte vždy kontrolou `len(payload)` oproti specifikaci a až pak dělejte `unpack`.

---

## Příklad formátu logu (NAV‑PVAT)

Jednořádkový výstup shrnuje klíčová pole:

```
[PVAT] iTOW=103926000 fix=4(g+dr) carr=2(fixed) sv=21 UTC=2025-02-12T09:32:06.123Z
       pos: 50.0615682,14.5997083 h=245.123 m hMSL=210.456 m hAcc=0.089 m vAcc=0.700 m
       vel: NED(0.000,-0.001,-0.002) m/s gSpeed=0.003 m/s sAcc=0.006 m/s
       att: roll=+2.908° (±0.28°) pitch=-1.785° (±0.28°) vehHead=76.376° (±3.75°) motHead=76.370°
       err-ellipse: major=0.120 m minor=0.045 m orient=15.2°
```

> Skutečný výstup je na jeden řádek (bez zalomení); výše je rozděleno jen pro čitelnost README.

---

## Konfigurace (CFG‑\*)

Konfigurace je rozdělena na dva profily:

* **`PROFILE_PHYSICAL`** – statické vlastnosti instalace:

  * `CFG-NAVSPG-DYNMODEL = 11` (RLM / robotic lawn mower),
  * páky **IMU→CRP/VRP/ANT** (cm) – měřit s přesností v cm,
  * **IMU mount** – buď automatika (`CFG-SFIMU-AUTO_MNTALG_ENA=1`), nebo ručně zadané úhly YAW/PITCH/ROLL (deg ×100),
  * odometrie: `CFG-SFODO-FACTOR` (m/tick), `CFG-SFODO-QUANT_ERROR` (m), `CFG-SFODO-COUNT_MAX` (pro absolutní čítače).

* **`PROFILE_OPERATIONAL`** – provozní nastavení:

  * `CFG-SFODO-FREQUENCY` (Hz) a `CFG-SFODO-LATENCY` (ms),
  * zákaz automatických detekcí dle integr. manuálu (např. `DIS_AUTODIRPINPOL=1`, `DIS_AUTOSPEED=1`),
  * `CFG-SFCORE-USE_SF=1` pro povolení fusion filtru,
  * časování: `CFG-SFCORE-SEN_TTAG_FACT = 1000` (1 ms) – **musí** souhlasit pro všechny ESF‑MEAS.

Změny posíláme přes **UBX‑CFG‑VALSET**; služba očekává `ACK` a loguje případné `NAK` s důvodem.

---

## Spuštění

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt   # pyserial, (případně) dataclasses_json, atd.

# Nastavte port/baud v service.py (např. /dev/ttyACM0, 921600) a TCP port (např. 9006)
python main.py
# [SERVER] GNSS Service listening on port 9006
```

**Test:**

```bash
echo "START" | nc -q0 127.0.0.1 9006
```

---

## Rozšiřování

1. Vytvořte nový handler v `handlers/xyz_handler.py` s metodou `handle(self, msg_class, msg_id, payload)`.
2. V `service.py` (nebo modulu s registry) přidejte registraci do dispatch mapy, např.:

   ```python
   dispatcher[(0x01, 0x17)] = NavPvatHandler()
   ```
3. Používejte striktní kontroly délek a odolné parsování (nespoléhat na dokonale formátovaná data).
4. Logujte **jedním řádkem**; u periodických zpráv používejte „throttling“ podle `iTOW//1000`, aby neplnil log více než 1×/s.

---

## Tipy a řešení potíží

* **Vysoké `vAcc`/zhoršení přesnosti po aktivaci DR**

  * Zkontrolujte RTCM/RTK proud (bez fixu RLM kalibrace nepoběží dle manuálu),
  * ověřte páky (IMU→VRP/ANT) a IMU mount úhly,
  * ESF‑STATUS: všechny použité senzory by měly mít `calibStatus=2` a `used=1`.

* **ESF‑RAW zahlcuje I/O**

  * Nelze pollovat; snižte/zakážte výstupní rychlost přes `CFG-MSGOUT-UBX_ESF_RAW-*`.

* **`ACK-NAK` při `PRIO ON|OFF` nebo CFG‑VALSET**

  * Špatná keyID/vrstva (RAM/BBR/FLASH) nebo nevalidní kombinace. Posílejte do RAM a uložte dle potřeby.

* **Sériová chyba `NoneType` ve čtečce**

  * Indikuje zavření portu nebo neošetřený návrat `read`; přidejte retry/guard a čisté ukončení vláken.

* **Heading rate / rychlost otáčení**

  * Lze vzít z `ESF-INS` (kompenzované gyro v deg/s) nebo numericky derivovat `heading` z `NAV-VELNED`/`NAV-PVAT`. `NAV-ATT` neposílá `headingRate` (není v 1.40).

---

## Ukázka FIFO z `NavVelNedHandler`

Handler agreguje 1×/s čitelný log a zároveň sype binární rekord do FIFO (pro grafy/telemetrii):

```python
# zápis do FIFO (malý binární rámec)
data = struct.pack('<I I I i I I', iTOW, speed, gSpeed, heading, sAcc, cAcc)
bin_stream_fifo.put_nowait(data)
```

---

## Licenční a kredit

Interní nástroj pro Robotour projekt. Specifikace u‑blox ZED‑F9R HPS 1.40 (UBX) – viz dokumentace v příloze projektu.
