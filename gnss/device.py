# device.py – GNSS zařízení pro Robotour
import serial, threading, time
from ubx import build_msg, parse_stream, parse_nav_pvt

# CFG-MSGOUT klíče (USB)
CFG_NAV_PVT_USB = 0x20910009
CFG_NAV_SAT_USB = 0x20910018

class GNSSDevice:
    def __init__(self):
        self.port = None
        self.running = False
        self.device_type = None
        self.lock = threading.Lock()
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.last_fix = None
        self.last_state = None

    def _wait_for_ack(self, cls_id, msg_id, timeout=1.0):
        """Čeká na UBX-ACK pro danou zprávu"""
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            data = self.port.read(256)
            if not data:
                continue
            buf += data
            while True:
                mc, mi, payload, buf = parse_stream(buf)
                if mc is None:
                    break
                if mc == 0x05:  # ACK
                    if mi == 0x01:  # ACK-ACK
                        print("✅ ACK-ACK přijato")
                        return True
                    elif mi == 0x00:  # ACK-NAK
                        print("❌ ACK-NAK přijato")
                        return False
        print("⚠️ ACK timeout")
        return False

    def start(self):
        if self.running:
            return True
        for dev in ["/dev/gnss1", "/dev/gnss2"]:
            try:
                ser = serial.Serial(dev, baudrate=38400, timeout=0.2)
                ser.write(build_msg(0x0A, 0x04))  # MON-VER
                data = ser.read(200)
                if b"F9R" in data:
                    self.device_type = "F9R"
                    self.port = ser
                    break
                elif b"D9S" in data:
                    self.device_type = "D9S"
                    self.port = ser
                    break
                ser.close()
            except Exception:
                continue
        if not self.port:
            return False

        # --- nastavení výstupu ---
        cfg_payload = bytearray()
        cfg_payload += b"\x00"      # version
        cfg_payload += b"\x01"      # RAM only
        cfg_payload += b"\x00\x00"  # reserved

        # NAV-PVT USB = 10 Hz
        cfg_payload += CFG_NAV_PVT_USB.to_bytes(4, "little")
        cfg_payload += (10).to_bytes(1, "little")

        # NAV-SAT USB = 1 Hz
        cfg_payload += CFG_NAV_SAT_USB.to_bytes(4, "little")
        cfg_payload += (1).to_bytes(1, "little")

        msg = build_msg(0x06, 0x8A, cfg_payload)
        self.port.write(msg)
        if not self._wait_for_ack(0x06, 0x8A):
            print("⚠️ Konfigurace nebyla potvrzena")

        # spustit reader thread
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.running = True
        return True

    def stop(self):
        if not self.running:
            return
        self.stop_event.set()
        if self.reader_thread:
            self.reader_thread.join(2.0)
        try: self.port.close()
        except: pass
        self.running = False
        self.device_type = None
        self.last_fix = None
        self.last_state = None
        print("🛑 GNSS stop")

    def _reader_loop(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = self.port.read(1024)
                if not data:
                    continue
                buf += data
                while True:
                    msg_class, msg_id, payload, buf = parse_stream(buf)
                    if msg_class is None:
                        break
                    print(f"UBX {msg_class:02X}/{msg_id:02X} len={len(payload)}")
                    if msg_class == 0x01 and msg_id == 0x07:  # NAV-PVT
                        fix = parse_nav_pvt(payload)
                        if fix:
                            with self.lock:
                                self.last_fix = fix
                                self.last_state = f"sat={fix['numSV']} fix={fix['fixType']}"
            except Exception as e:
                print(f"Reader error: {e}")
                time.sleep(0.5)

    def get_state(self):
        if not self.running:
            return "IDLE"
        with self.lock:
            return self.last_state or f"RUNNING {self.device_type} (no data)"

    def get_fix(self):
        if not self.running:
            return {"error": "not running"}
        with self.lock:
            return self.last_fix if self.last_fix else {"error": "NOFIX"}

gnss_device = GNSSDevice()
