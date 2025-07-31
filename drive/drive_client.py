from drive_serial import open_serial, close_serial, send_serial
import threading

shutdown_flag = False

def handle_client(conn, addr):
    print(f"📡 Klient připojen: {addr}")
    global shutdown_flag

    def send(msg):
        try:
            conn.sendall((msg + '\n').encode())
        except:
            pass

    try:
        with conn:
            while not shutdown_flag:
                data = conn.recv(1024)
                if not data:
                    break
                cmd_line = data.decode().strip()
                parts = cmd_line.split()
                if not parts:
                    continue
                cmd = parts[0].upper()

                if cmd == "PING":
                    send("PONG")

                elif cmd == "START":
                    open_serial()
                    send("OK")

                elif cmd == "STOP":
                    close_serial()
                    send("OK")

                elif cmd == "PWM":
                    if len(parts) == 3:
                        try:
                            pwm_left = int(parts[1])
                            pwm_right = int(parts[2])
                            data_bytes = pwm_left.to_bytes(2, 'little', signed=True) + pwm_right.to_bytes(2, 'little', signed=True)
                            send_serial(data_bytes)
                            send("OK")
                        except Exception as e:
                            send(f"ERR {e}")
                    else:
                        send("ERR Param count")

                elif cmd == "BREAK":
                    data_bytes = (0).to_bytes(2, 'little', signed=True) + (0).to_bytes(2, 'little', signed=True)
                    send_serial(data_bytes)
                    send("OK")

                elif cmd == "EXIT":
                    send("BYE")
                    break

                else:
                    send("ERR Unknown cmd")
    except Exception as e:
        print(f"Chyba: {e}")
    finally:
        try:
            conn.close()
        except:
            pass
        print(f"🔌 Odpojeno: {addr}")
