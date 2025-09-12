# PointPerfect – socket služba (Robotour 2025)

Služba pro čtení korekčních dat PointPerfect (NTRIP) a jejich přeposílání do GNSS služby.
Běží na portu **9007**.

## Architektura
[Client] <-> 127.0.0.1:9007 <-> [PointPerfect worker] -> (PERFECT <payload>) -> GNSS (127.0.0.1:9006)

## Spuštění

1. Vytvořte v adresáři `pointperfect/` soubor `.env`:

```ini
POINTPERFECT_USER="tvuj_username"
POINTPERFECT_PASS="tvoje_heslo!$#"
```

> `.env` je v `.gitignore`, nikdy se nedostane do gitu.

2. Spusťte službu:

```bash
python3 main.py
```

Logy běží na stdout.

## Příkazy

- `PING`   -> `PONG`
- `START`  -> `OK` (spustí vlákno, začne číst PointPerfect data)
- `STOP`   -> `OK` (zastaví vlákno)
- `STATUS` -> `RUNNING <count>` nebo `IDLE <count>` (počet přijatých zpráv)
- `EXIT`   -> `BYE` (ukončí spojení s klientem)

## Testování (nc)

```bash
nc 127.0.0.1 9007
```

Ukázka:
```
PING
# -> PONG

STATUS
# -> IDLE 0

START
# -> OK

STATUS
# -> RUNNING 5
```

## Poznámky

- Každá přijatá zpráva se loguje s časem a velikostí.
- Zprávy se base64 zakódují a odešlou do GNSS služby (port 9006) jako `PERFECT <payload>`.
- Bez platných údajů (username/password) v `.env` nebude stream fungovat.
- Graceful shutdown: Ctrl+C (SIGINT) ukončí server i vlákno.
