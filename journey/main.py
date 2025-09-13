import socket
import threading
import signal
import sys
from typing import Optional

from util import log_event
# Původní DEMO workflow:
from workflow_demo import start_demo_workflow, stop_demo_workflow, demo_running
# Nový MANUAL workflow:
from workflow_manual import start_manual_workflow, stop_manual_workflow, manual_running
# Nové workflow POINT a AUTO:
from workflow_point import start_point_workflow, stop_point_workflow, point_running
from workflow_auto  import start_auto_workflow,  stop_auto_workflow,  auto_running

HOST = "127.0.0.1"
PORT = 9004

shutdown_flag = threading.Event()


def safe_send(conn: Optional[socket.socket], text: str) -> None:
    """Bezpečné odeslání zprávy klientovi, neblokuje běh workflow při chybě."""
    if not conn:
        return
    try:
        conn.sendall(text.encode())
    except Exception:
        pass


def status_text() -> str:
    if manual_running.is_set():
        return "RUNNING MANUAL"
    if demo_running.is_set():
        return "RUNNING DEMO"
    if point_running.is_set():
        return "RUNNING POINT"
    if auto_running.is_set():
        return "RUNNING AUTO"
    return "IDLE"


def _any_workflow_running() -> Optional[str]:
    if manual_running.is_set(): return "MANUAL"
    if demo_running.is_set():   return "DEMO"
    if point_running.is_set():  return "POINT"
    if auto_running.is_set():   return "AUTO"
    return None


def handle_client(conn: socket.socket, addr):
    log_event(f"Client connected: {addr}")
    buf = b""
    try:
        with conn:
            while not shutdown_flag.is_set():
                data = conn.recv(1024)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode(errors="ignore").strip()
                    if not cmd:
                        continue

                    # ---- ZÁKLADNÍ PŘÍKAZY -----------------------------------
                    if cmd == "PING":
                        safe_send(conn, "PONG\n")

                    elif cmd == "STATUS":
                        safe_send(conn, status_text() + "\n")

                    elif cmd == "LOG":
                        try:
                            last = log_event.get_log()[-60:]
                            safe_send(conn, "\n".join(last) + "\nEND\n")
                        except Exception as e:
                            safe_send(conn, f"ERROR {e}\n")

                    elif cmd == "STOP":
                        # Zastaví libovolné běžící workflow (MANUAL/DEMO/POINT/AUTO)
                        stop_manual_workflow()
                        stop_demo_workflow()
                        stop_point_workflow()
                        stop_auto_workflow()
                        safe_send(conn, "OK STOP SENT\n")

                    elif cmd == "EXIT":
                        safe_send(conn, "BYE\n")
                        return

                    # ---- WORKFLOW – MANUAL ----------------------------------
                    elif cmd == "MANUAL":
                        running = _any_workflow_running()
                        if running:
                            safe_send(conn, f"ERR: workflow {running} je právě aktivní\n")
                        else:
                            start_manual_workflow(client_conn=conn)
                            safe_send(conn, "OK MANUAL WORKFLOW STARTED\n")

                    # ---- WORKFLOW – DEMO ------------------------------------
                    elif cmd == "DEMO":
                        running = _any_workflow_running()
                        if running:
                            safe_send(conn, f"ERR: workflow {running} je právě aktivní\n")
                        else:
                            start_demo_workflow(client_conn=conn)
                            safe_send(conn, "OK DEMO WORKFLOW STARTED\n")

                    # ---- WORKFLOW – POINT -----------------------------------
                    elif cmd == "POINT":
                        running = _any_workflow_running()
                        if running:
                            safe_send(conn, f"ERR: workflow {running} je právě aktivní\n")
                        else:
                            start_point_workflow(client_conn=conn)
                            safe_send(conn, "OK POINT WORKFLOW STARTED\n")

                    # ---- WORKFLOW – AUTO ------------------------------------
                    elif cmd == "AUTO":
                        running = _any_workflow_running()
                        if running:
                            safe_send(conn, f"ERR: workflow {running} je právě aktivní\n")
                        else:
                            start_auto_workflow(client_conn=conn)
                            safe_send(conn, "OK AUTO WORKFLOW STARTED\n")

                    else:
                        safe_send(conn, "ERR Unknown cmd\n")

    except Exception as e:
        log_event(f"Client handler error {addr}: {e}")
    finally:
        log_event(f"Client disconnected: {addr}")


def server_main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen()
    log_event(f"Journey listening on {HOST}:{PORT}")

    try:
        while not shutdown_flag.is_set():
            srv.settimeout(1.0)
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    except Exception as e:
        log_event(f"Journey server exception: {e}")
    finally:
        try:
            srv.close()
        except Exception:
            pass
        log_event("Journey server stopped.")


def sigint_handler(signum, frame):
    print("\nSIGINT caught, shutting down Journey ...")
    shutdown_flag.set()
    # korektně ukončí běžící workflow a pošle STOP do služeb
    stop_manual_workflow()
    stop_demo_workflow()
    stop_point_workflow()
    stop_auto_workflow()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    try:
        server_main()
    except Exception as e:
        print(f"Journey main exception: {e}")
    print("Journey service stopped.")
