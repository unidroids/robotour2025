"""client_handler.py – vlákno obsluhy klienta pro službu DRIVE (port 9003)

Textový příkazový protokol (1 příkaz na řádek):
  PING                       -> "PONG DRIVE"
  START                      -> spustí službu (UART + cmd=2), odpoví "OK RUNNING"
  STOP                       -> zastaví službu (cmd=1 + zavře UART), "OK STOPPED"
  STATE                      -> JSON na jednom řádku
  EXIT                       -> ukončí spojení s klientem ("BYE")
  POWER_OFF                  -> cmd=3, "OK"/"ERROR"
  HALT                       -> cmd=0, "OK"/"ERROR"
  BREAK                      -> cmd=5, "OK"/"ERROR"
  DRIVE <max_pwm> <vL> <vR>  -> cmd=4 (encode), "OK"/"ERROR"
  PWM <pwmL> <pwmR>          -> cmd=101 (encode), "OK"/"ERROR"

Pozn.: Parsování je case‑insensitive, argumenty musí být celá čísla.
"""
from __future__ import annotations

import json
import socket
from typing import Tuple

from service import DriveService

__all__ = ["client_thread"]


def client_thread(conn: socket.socket, addr: Tuple[str, int], svc: DriveService) -> None:
    """Obslouží jedno klientské TCP spojení. Běží v samostatném vlákně.

    - čte textové příkazy ukončené \n (CR je volitelný)
    - posílá odpovědi ukončené \n
    Latence: socket timeout je krátký, smyčka je lehká.
    """
    conn.settimeout(0.5)
    peer = f"{addr[0]}:{addr[1]}"
    #_send_line(conn, "HELLO DRIVE 9003")

    buf = bytearray()
    try:
        while True:
            line = _recv_line(conn, buf)
            if line is None:
                # timeout: pokračuj
                continue
            if line == "":
                # protistrana ukončila spojení
                break

            cmdline = line.strip()
            if not cmdline:
                continue

            tokens = cmdline.split()
            cmd = tokens[0].upper()
            args = tokens[1:]

            try:
                if cmd == "PING":
                    _send_line(conn, svc.ping())

                elif cmd == "START":
                    state = svc.start()
                    _send_line(conn, f"OK {state}")

                elif cmd == "STOP":
                    state = svc.stop()
                    _send_line(conn, f"OK {state}")

                elif cmd == "STATE":
                    st = svc.get_state()
                    _send_line(conn, json.dumps(st, separators=(",",":")))

                elif cmd == "EXIT":
                    _send_line(conn, "BYE")
                    break

                elif cmd == "POWER_OFF":
                    ok = svc.power_off()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "HALT":
                    ok = svc.halt()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "BREAK":
                    ok = svc.brake()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "DRIVE":
                    if len(args) != 3:
                        _send_line(conn, "ERROR BAD_ARGS use: DRIVE <max_pwm> <vL> <vR>")
                    else:
                        max_pwm = int(args[0])
                        vL = int(args[1])
                        vR = int(args[2])
                        ok = svc.drive(max_pwm, vL, vR)
                        _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "PWM":
                    if len(args) != 2:
                        _send_line(conn, "ERROR BAD_ARGS use: PWM <pwmL> <pwmR>")
                    else:
                        pwmL = int(args[0])
                        pwmR = int(args[1])
                        ok = svc.pwm(pwmL, pwmR)
                        _send_line(conn, "OK" if ok else "ERROR")

                else:
                    _send_line(conn, "ERROR UNKNOWN_CMD")

            except ValueError as e:
                _send_line(conn, f"ERROR {e}")
            except Exception as e:
                _send_line(conn, f"ERROR {type(e).__name__}: {e}")
    except Exception:
        # Tlumení chyb spojení; hlavní smyčka serveru žije dál
        pass
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass


# ---------------- pomocné I/O ----------------

def _send_line(conn: socket.socket, s: str) -> None:
    try:
        conn.sendall((s + "\n").encode("utf-8", errors="replace"))
    except Exception:
        raise


def _recv_line(conn: socket.socket, buf: bytearray) -> str | None:
    """Vrátí jednu textovou řádku bez CR/LF.

    - None: timeout (žádná data teď)
    - "":   protistrana zavřela spojení
    - str:  kompletní řádka
    """
    try:
        chunk = conn.recv(4096)
        if not chunk:
            return ""  # EOF
        buf.extend(chunk)
    except socket.timeout:
        return None

    # najdi LF
    nl = buf.find(b"\n")
    if nl < 0:
        return None
    line = bytes(buf[:nl])
    del buf[: nl + 1]
    # odstraň volitelný CR
    if line.endswith(b"\r"):
        line = line[:-1]
    try:
        return line.decode("utf-8", errors="replace")
    except Exception:
        return line.decode("latin1", errors="replace")
