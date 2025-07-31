import socket
from util import log_event

HOST = '127.0.0.1'

def send_command(port, cmd, expect_response=True):
    try:
        with socket.create_connection((HOST, port), timeout=150) as s:
            s.sendall((cmd+'\n').encode())
            if expect_response:
                resp = s.recv(1024).decode().strip()
                log_event(f"SERVICE[{port}] {cmd} → {resp}")
                return resp
            else:
                log_event(f"SERVICE[{port}] {cmd} → (no wait)")
                return ""
    except Exception as e:
        log_event(f"ERROR[{port}] {cmd}: {e}")
        return f"ERROR {e}"
