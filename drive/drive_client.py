from drive_serial import open_serial, close_serial, send_serial
import threading

shutdown_flag = False

def clamp(val, minval, maxval):
    return max(minval, min(maxval, val))

def pwm_left_to_serial_byte(axis_1):
    # Negace osy podle vzoru!
    val = clamp(int(round(axis_1 * 30)), -30, 30)
    return val + 158

def pwm_right_to_serial_byte(axis_3):
    val = clamp(int(round(axis_3 * 30)), -30, 30)
    return val + 219

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
                    # Očekáváme hodnoty v rozsahu -100..100 (jako z původního zadání)
                    if len(parts) == 3:
                        try:
                            axis_1 = float(parts[1]) / 100.0  # převod na -1..1
                            axis_3 = float(parts[2]) / 100.0
                            left_motor = pwm_left_to_serial_byte(axis_1)
                            right_motor = pwm_right_to_serial_byte(axis_3)
                            send_serial(bytes([left_motor]))
                            send_serial(bytes([right_motor]))
                            print(f"PWM {axis_1:.2f} {axis_3:.2f} => {left_motor} {right_motor}")
                            send("OK")
                        except Exception as e:
                            send(f"ERR {e}")
                    else:
                        send("ERR Param count")

                elif cmd == "BREAK" or cmd == "B":
                    # Posílá "střed" (zastavení) pro obě kola
                    send_serial(bytes([158]))
                    send_serial(bytes([219]))
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
