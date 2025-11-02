import socket
import time
from time import monotonic

class DriveClient:
    def __init__(self, host='127.0.0.1', port=9003):
        self.host = host
        self.port = port
        self.sock = None

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
    def _reconnect(self):
        retries = 0
        while not self.sock:
            self._connect()
            time.sleep(0.5)
            retries += 1
            if retries >= 5:
                print("[DRIVE CLIENT] Failed to reconnect after 5 attempts.")
                raise ConnectionError("Unable to reconnect to drive server.")
                break

    def _send_and_read(self, msg):
        self._reconnect() # Ensure connection is active
        st = time.monotonic()
        self.sock.sendall(msg.encode("ascii"))
        resp = self.sock.recv(1024).decode().strip()
        et = time.monotonic()
        #print(f"[DRIVE CLIENT] Sent: {msg.strip()} Received: {resp} (in {(et-st)*1000:.2f}ms)")
        return resp


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
        self.send_start() # Start the drive system

    def stop(self):
        self.send_stop()  # Stop the robot before closing
        self._close()

    def send_start(self):
        resp = self._send_and_read(f"START\n")

    def send_stop(self):
        resp = self._send_and_read(f"STOP\n")

    def send_break(self):
        resp = self._send_and_read(f"HALT\n")

    def send_break(self):
        resp = self._send_and_read(f"BREAK\n")

    def send_drive(self, pwm, left_speed, right_speed):
        resp = self._send_and_read(f"DRIVE {pwm} {left_speed} {right_speed}\n")

    def send_pwm(self, left, right):
        resp = self._send_and_read(f"PWM {left} {right}\n")

if __name__ == "__main__":
    client = DriveClient()
    client.start()
    time.sleep(1)
    client.send_drive(100, 50, 50)
    for speed in [50, 60, 70, 80, 90, 100]:
        client.send_drive(100, speed, speed)
        time.sleep(1)
    client.send_break()
    time.sleep(1)
    client.stop()