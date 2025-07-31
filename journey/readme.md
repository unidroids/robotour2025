# Journey – workflow orchestrátor robota (Robotour 2025)

**Journey** je socket server, který řídí hlavní workflow robota přes jednotné rozhraní.
Služba zajišťuje přímou komunikaci s ostatními komponentami robota (DRIVE, LIDAR, CAMERA)
a umožňuje spouštění, zastavování a monitorování workflow přes síťový socket.

---

## Architektura

```
   [Client]          [Journey]          [CAMERA]      [LIDAR]      [DRIVE]
 nc/telnet/   <--->  socket:9004  <--> 9001/tcp   <--> 9002/tcp <--> 9003/tcp
```

* **Journey** běží na `127.0.0.1:9004` a přijímá příkazy:

  * `PING` ... odpoví `PONG`
  * `DEMO` ... spustí automatický workflow
  * `STOP` ... bezpečně zastaví všechny služby i workflow
  * `LOG` ... zobrazí posledních 40 záznamů z interního logu

* **Průběh DEMO** (viz workflow\.py) odpovídá sekvenci kroků popsaných v dokumentaci projektu.

---

## Instalace

1. **Kopíruj celý adresář `journey` na Jetson:**

   ```
   /opt/projects/robotour/journey
   ```

2. **Spusť server:**

   ```bash
   cd /opt/projects/robotour/journey
   python3 main.py
   ```

3. **Testuj klientem:**

   ```bash
   nc 127.0.0.1 9004
   ```

---

## Systémová služba (systemd)

Použij přiložený skript `register_journey_service.sh` pro vytvoření a aktivaci služby
(jako uživatel `user`, pracovní adresář `/opt/projects/robotour/journey`).

---

## Logování

* Veškeré události jsou ukládány do paměťového logu a vypisovány do konzole.
* Pro dlouhodobé logování nastav standardní výstup v systemd na soubor v `/data/logs/journey/`.

---

## Ukončení služby

* Bezpečné ukončení přes `Ctrl+C` (`SIGINT`) – všechny služby dostanou příkaz STOP.
* Zůstává funkční i při běžném restartu serveru.

---

## Požadavky

* Python 3.7+ (odzkoušeno na Jetson Orin Nano s JetPack 6)
* Všechny závislosti jsou standardní součástí Pythonu (není třeba instalovat třetí strany).

---

## Struktura projektu

```
journey/
├── main.py            # Hlavní server a signály
├── workflow.py        # DEMO workflow logika
├── services.py        # Komunikace se službami robota (sockets)
├── util.py            # Logování, parsování, utility
├── README.md          # Tento soubor
└── register_journey_service.sh  # Registrační skript pro systemd
```

---

## Autor

Robotour 2025, tým unidroids
[https://github.com/unidroids/robotour2025](https://github.com/unidroids/robotour2025)
