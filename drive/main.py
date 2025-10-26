# main.py
from __future__ import annotations

import os
import socket
import signal
import sys
import threading
import time
from typing import List, Tuple

from service import DriveService
from client_handler import client_thread


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9003


class DriveServer:
    """
    Jednoduchý TCP server pro službu DRIVE (port 9003).

    - Singleton DriveService sdílený napříč klienty
    - Každý klient obsloužen ve vlákně client_thread()
    - Graceful shutdown na SIGINT/SIGTERM
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        self.host = host
        self.port = port
        self._stop = threading.Event()
        self._srv_sock: socket.socket | None = None
        self._client_threads: List[threading.Thread] = []
        self._svc = DriveService()

    # ---------- lifecycle ----------
    def start(self) -> None:
        self._srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv_sock.bind((self.host, self.port))
        self._srv_sock.listen(8)
        self._srv_sock.settimeout(0.5)  # krátký timeout kvůli _stop Eventu

        print(f"[DRIVE] Listening on {self.host}:{self.port}")

        # Hlavní accept smyčka
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = self._srv_sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    # socket byl zavřen během shutdownu
                    break

                t = threading.Thread(
                    target=client_thread, args=(conn, addr, self._svc), daemon=True
                )
                t.start()
                self._client_threads.append(t)
        finally:
            self._cleanup()

    def stop(self) -> None:
        """Vyžádá ukončení hlavní smyčky a zavření server socketu."""
        self._stop.set()
        if self._srv_sock:
            try:
                self._srv_sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self._srv_sock.close()
            except Exception:
                pass
            self._srv_sock = None

    # ---------- internals ----------
    def _cleanup(self) -> None:
        # Požádej klientská vlákna, ať doběhnou
        for t in self._client_threads:
            try:
                if t.is_alive():
                    t.join(timeout=0.5)
            except Exception:
                pass

        # Zastav DriveService – pošli STOP motorům a zavři UART/dispatcher
        try:
            self._svc.stop()
        except Exception:
            pass

        print("[DRIVE] Server stopped.")


# ---------- signal handling ----------
def _install_signal_handlers(server: DriveServer):
    def _handler(signum, frame):
        print(f"[DRIVE] Caught signal {signum}, shutting down ...")
        server.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except Exception:
            # Některá prostředí (např. Windows/threads) nemusí vše podporovat
            pass


def main() -> int:
    # Volitelná CLI: main.py [host] [port]
    host = DEFAULT_HOST
    port = DEFAULT_PORT

    server = DriveServer(host, port)
    _install_signal_handlers(server)
    try:
        server.start()
        return 0
    except Exception as e:
        print(f"[DRIVE] Fatal error: {e}")
        try:
            server.stop()
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    sys.exit(main())
