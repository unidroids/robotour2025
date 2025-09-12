# autopilot/client.py
import traceback
from typing import Tuple
from control import start_controller, stop_controller, haversine_distance, bearing

def handle_client(conn, addr, ctx):
    try:
        with conn:
            buf = b""
            while not ctx.shutdown_flag.is_set():
                data = conn.recv(1024)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode(errors="ignore").strip()
                    if not cmd:
                        continue

                    # --- základní příkazy ---
                    if cmd == "PING":
                        conn.sendall(b"PONG\n")

                    elif cmd == "START":
                        start_controller(ctx)
                        conn.sendall(b"OK\n")

                    elif cmd == "STOP":
                        stop_controller(ctx)
                        conn.sendall(b"OK\n")

                    elif cmd == "STATUS":
                        with ctx.lock:
                            conn.sendall((ctx.status + "\n").encode())

                    elif cmd == "EXIT":
                        conn.sendall(b"BYE\n")
                        return

                    # --- doménové příkazy ---
                    elif cmd.startswith("WAYPOINT "):
                        parts = cmd.split()
                        if len(parts) != 4:
                            conn.sendall(b"ERR Usage: WAYPOINT <lat> <lon> <radius_m>\n")
                            continue
                        try:
                            lat = float(parts[1])
                            lon = float(parts[2])
                            radius = float(parts[3])
                            with ctx.lock:
                                ctx.waypoint = (lat, lon, radius)
                            conn.sendall(b"OK\n")
                        except ValueError:
                            conn.sendall(b"ERR Invalid numbers\n")

                    elif cmd == "CLEAR":
                        with ctx.lock:
                            ctx.waypoint = None
                        conn.sendall(b"OK\n")

                    elif cmd == "GET":
                        with ctx.lock:
                            status = ctx.status
                            lat, lon = ctx.last_pose if ctx.last_pose else (0.0, 0.0)
                            if ctx.waypoint:
                                wlat, wlon, wrad = ctx.waypoint
                                dist = haversine_distance(lat, lon, wlat, wlon)
                                brg = bearing(lat, lon, wlat, wlon)
                                reply = f"{status} {lat:.6f} {lon:.6f} {dist:.1f} {brg:.1f} {wlat:.6f} {wlon:.6f} {wrad:.1f}\n"
                            else:
                                reply = f"{status} {lat:.6f} {lon:.6f}\n"
                            conn.sendall(reply.encode())

                    else:
                        conn.sendall(b"ERR Unknown cmd\n")

    except Exception as e:
        print(f"Chyba klienta {addr}: {e}")
        traceback.print_exc()
    finally:
        print(f"🔌 Odpojeno: {addr}")
