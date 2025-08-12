import threading
import time
import socket
from typing import Optional, Tuple

from services import send_command
from util import log_event, parse_lidar_distance

# Používané služby pro MANUAL:
PORT_GAMEPAD = 9005
PORT_LIDAR   = 9002
PORT_DRIVE   = 9003

manual_running = threading.Event()
_stop_requested = threading.Event()

# Interní reference na klienta, který startoval workflow – výsledky mu zkusíme posílat.
_client_conn_lock = threading.Lock()
_client_conn: Optional[socket.socket] = None


def _safe_send_to_client(text: str) -> None:
    """Nepřeruší workflow ani při odpojeném klientovi."""
    with _client_conn_lock:
        conn = _client_conn
    if not conn:
        return
    try:
        conn.sendall(text.encode())
    except Exception:
        # Klient už nemusí být připojen – nevadí, workflow běží dál.
        pass


def _send_and_report(port: int, cmd: str, expect_response: bool = True) -> str:
    """Pošle příkaz službě + zaloguje + vrátí odpověď + pošle klientovi řádek."""
    resp = send_command(port, cmd, expect_response=expect_response)
    line = f"SERVICE[{port}] {cmd} -> {resp}"
    #log_event(line)
    _safe_send_to_client(line + "\n")
    return resp


def _loop_until_distance_valid() -> Tuple[Optional[int], Optional[float]]:
    """Opakovaně čte LIDAR,DISTANCE dokud nevrátí validní vzdálenost (ne -1) nebo STOP."""
    while not _stop_requested.is_set():
        resp = _send_and_report(PORT_LIDAR, "DISTANCE")
        idx, dist = parse_lidar_distance(resp)
        # Specifikace: dokud je odpověď -1, pokračovat
        if idx is not None and idx != -1:
            return idx, dist
        time.sleep(0.05)  # malý odstup, ať zbytečně netočíme CPU
    return None, None


def _control_loop() -> None:
    """
    Hlavní smyčka MANUAL:
    - čte LIDAR,DISTANCE
    - pokud < 50 → DRIVE,BREAK
    - jinak → PWM = GAMEPAD,DATA a pošli na DRIVE přímo jako příkaz
    Končí pouze po STOP.
    """
    while not _stop_requested.is_set():
        resp = _send_and_report(PORT_LIDAR, "DISTANCE")
        idx, dist = parse_lidar_distance(resp)

        if dist is not None and dist < 50.0:
            _send_and_report(PORT_DRIVE, "BREAK")
        else:
            # Získej PWM z gamepadu a pošli ho na DRIVE "tak jak je"
            data = _send_and_report(PORT_GAMEPAD, "DATA")
            # Upraví zprávu z gamepadu
            pwm = data.split('#', 1)[0].rstrip()
            # Pokud DATA vrátí chybu, i tak ji pošleme dál (služba DRIVE si s tím poradí/zaloguje)
            _send_and_report(PORT_DRIVE, pwm)

        time.sleep(0.03)  # cca ~30 Hz


def _manual_workflow():
    try:
        log_event("MANUAL workflow: START")
        _safe_send_to_client("WORKFLOW MANUAL START\n")

        # 1) GAMEPAD PING/START
        _send_and_report(PORT_GAMEPAD, "PING")
        _send_and_report(PORT_GAMEPAD, "START")

        # 2) LIDAR PING/START
        _send_and_report(PORT_LIDAR, "PING")
        _send_and_report(PORT_LIDAR, "START")

        # 3) DRIVE PING/START
        _send_and_report(PORT_DRIVE, "PING")
        _send_and_report(PORT_DRIVE, "START")

        # 4) Smyčka dokud LIDAR,DISTANCE nevrátí validní vzdálenost (není -1)
        _loop_until_distance_valid()

        # 5) Hlavní řízení do STOP
        _control_loop()

    except Exception as e:
        log_event(f"WORKFLOW ERROR: {e}")
        _safe_send_to_client(f"WORKFLOW ERROR: {e}\n")

    finally:
        # Při ukončení workflow (normálně i při STOP) odeslat STOP příkazy:
        try:
            _send_and_report(PORT_GAMEPAD, "STOP")
            _send_and_report(PORT_DRIVE,  "STOP")
            _send_and_report(PORT_LIDAR,  "STOP")
        except Exception as e:
            log_event(f"MANUAL stop cleanup error: {e}")

        manual_running.clear()
        _stop_requested.clear()
        _safe_send_to_client("WORKFLOW MANUAL END\n")
        log_event("MANUAL workflow: END")


def start_manual_workflow(client_conn: Optional[socket.socket]) -> None:
    """Spustí MANUAL workflow (pokud už neběží)."""
    if manual_running.is_set():
        raise RuntimeError("MANUAL already running")
    manual_running.set()
    _stop_requested.clear()
    with _client_conn_lock:
        # uchováme si (neexkluzivně) referenci – workflow poběží i po odpojení
        _client_conn = client_conn
    t = threading.Thread(target=_manual_workflow, daemon=True)
    t.start()


def stop_manual_workflow() -> None:
    """Požádá o STOP a rovnou pošle STOP do všech relevantních služeb (idempotentní)."""
    _stop_requested.set()
    # poslat STOP i při ručním STOP požadavku; když už neběží, služby si poradí
    try:
        _send_and_report(PORT_GAMEPAD, "STOP")
        _send_and_report(PORT_DRIVE,  "STOP")
        _send_and_report(PORT_LIDAR,  "STOP")
    except Exception:
        pass
