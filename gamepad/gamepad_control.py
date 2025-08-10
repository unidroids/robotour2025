#!/usr/bin/env python3
# Sdílené řízení ukončení a registrace klientů/soketu (bez tříd)
import threading, socket

shutdown_event = threading.Event()
server_sock = None

_clients = set()
_clients_lock = threading.Lock()

def set_server_socket(sock: socket.socket):
    global server_sock
    server_sock = sock

def register_client(conn: socket.socket):
    with _clients_lock:
        _clients.add(conn)

def unregister_client(conn: socket.socket):
    with _clients_lock:
        _clients.discard(conn)

def close_all_clients():
    with _clients_lock:
        for c in list(_clients):
            try:
                c.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                c.close()
            except Exception:
                pass
        _clients.clear()

def request_shutdown():
    """Vyžádá ukončení: nastaví event a zavře server socket."""
    shutdown_event.set()
    try:
        if server_sock:
            server_sock.close()
    except Exception:
        pass
