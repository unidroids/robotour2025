import socket

class DriveClient:
    def __init__(self, host='127.0.0.1', port=9003):
        self.host = host
        self.port = port
        self.sock = None

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        

    def _close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                pass
            finally:
                self.sock = None

    def start(self):
        self._connect()

    def stop(self):
        self.send_pwm(0, 0)  # Stop the robot before closing
        self._close()

    def send_pwm(self, left, right):
        msg = f"PWM {left} {right}\n"
        if self.sock:
            self.sock.sendall(msg.encode("ascii"))
        else:
            print("[DRIVE CLIENT] Socket not connected. Cannot send PWM command.")
