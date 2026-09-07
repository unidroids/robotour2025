# acc_handler.py
import struct
import zmq
import time
import json

class AccHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._latest_ax = 0.0
        self._latest_ay = 0.0
        self._latest_az = 0.0
        self._latest_temp = 0.0
        self._latest_ts = 0.0
        self._last_print_time = 0.0
        self._last_temp_pub_time = 0.0

    def handle(self, message_bytes: bytes):
        if len(message_bytes) != 11 or message_bytes[1] != 0x51:
            return

        # Data format: 0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM
        ax_raw, ay_raw, az_raw, temp_raw = struct.unpack('<hhhh', message_bytes[2:10])

        self._latest_ax = ax_raw / 32768.0 * 16.0
        self._latest_ay = ay_raw / 32768.0 * 16.0
        self._latest_az = az_raw / 32768.0 * 16.0
        self._latest_temp = temp_raw / 100.0

        current_time = time.monotonic()
        self._latest_ts = current_time
        
        json_data = json.dumps({
            "ts": current_time,
            "ax": self._latest_ax,
            "ay": self._latest_ay,
            "az": self._latest_az
        })
        msg_payload = json_data.encode('utf-8')
        
        if current_time - self._last_print_time > 1.0:
            print(f"[AccHandler] ACC {json_data} | Temp: {self._latest_temp:.2f}°C")
            self._last_print_time = current_time
            
        try:
            self._zmq_pub.send_multipart([b"ACC", msg_payload])
        except Exception:
            pass

        # Publikovat téma TEMP pouze 1× za sekundu
        if current_time - self._last_temp_pub_time >= 1.0:
            temp_json = json.dumps({
                "ts": current_time,
                "temp": self._latest_temp
            })
            temp_payload = temp_json.encode('utf-8')
            try:
                self._zmq_pub.send_multipart([b"TEMP", temp_payload])
                self._last_temp_pub_time = current_time
            except Exception:
                pass

    def get_latest(self) -> dict:
        return {
            "ts": self._latest_ts,
            "ax": self._latest_ax,
            "ay": self._latest_ay,
            "az": self._latest_az,
            "temp": self._latest_temp
        }
