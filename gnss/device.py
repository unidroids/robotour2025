# device.py – GNSS zařízení pro Robotour
import serial
import struct
import threading
import time
from ubx import build_msg, parse_stream, parse_nav_pvt

# CFG-MSGOUT klíče (USB) – UBX-CFG-VALSET
CFG_USBOUT_UBX = 0x10780001          # CFG-USBOUTPROT-UBX (U1)
CFG_USBOUT_NMEA = 0x10780002         # CFG-USBOUTPROT-NMEA (U1)
CFG_MSGOUT_NAV_PVT_USB = 0x20910009  # U1: 1 = každou epochu
CFG_MSGOUT_NAV_SAT_USB = 0x20910018  # U1: 10 = každá 10. epocha
# --- CFG-RATE klíče (VALSET) ---
CFG_RATE_MEAS     = 0x30210001  # U2, jednotky 1 ms
CFG_RATE_NAV      = 0x30210002  # U2, "solutions per measurement"
CFG_RATE_TIMEREF  = 0x20210003  # U1, 0=UTC, 1=GPS, ...
CFG_RATE_NAV_PRIO = 0x20210004  # U1, Hz (priority navigation rate)

CFG_NAVSPG_DYNMODEL = 0x20110021  # key ID podle u-blox interface description


class GNSSDevice:
    def __init__(self):
        self.port = None
        self.running = False
        self.device_type = None  # "F9R" / "D9S" / None
        self.lock = threading.Lock()
        self.reader_thread = None
        self.stop_event = threading.Event()

        self.last_fix = None        # dict z NAV-PVT
        self.last_state = None      # textový quick-stav
        self.msg_counter = 0
        self.bad_counter = 0        # CRC fail
        self.ignored_counter = 0    # jiné chyby/bufferové odřezky

    # ---------- Konfigurace ----------

    def _build_valset_payload_data(self):
        """
        Nastaví do RAM:
        - USB protokoly: UBX=1, NMEA=0
        - MSGOUT: NAV-PVT 1x/epochu, NAV-SAT 1x/10 epoch
        - NAV priority mode (CFG-RATE-NAV_PRIO) = 30 Hz
        - Měřící/navigační periodu (CFG-RATE-MEAS = 33 ms, CFG-RATE-NAV = 1)
        + timeRef = GPS
        """
        p = bytearray()
        p += b"\x00"      # version
        p += b"\x01"      # layers = RAM
        p += b"\x00\x00"  # reserved

        # --- Protokoly na USB ---
        p += (CFG_USBOUT_UBX).to_bytes(4, "little");   p += (1).to_bytes(1, "little")
        p += (CFG_USBOUT_NMEA).to_bytes(4, "little");  p += (0).to_bytes(1, "little")

        # --- Výstupní zprávy (počet na navigační epochu) ---
        p += (CFG_MSGOUT_NAV_PVT_USB).to_bytes(4, "little"); p += (1).to_bytes(1, "little")
        # p += (CFG_MSGOUT_NAV_SAT_USB).to_bytes(4, "little"); p += (10).to_bytes(1, "little")

        p += (CFG_NAVSPG_DYNMODEL).to_bytes(4, "little"); p += (11).to_bytes(1, "little")   # 11 = E-scooter

        # --- Priority navigation mode & rychlost 30 Hz ---
        # p += (CFG_RATE_NAV_PRIO).to_bytes(4, "little"); p += (30).to_bytes(1, "little")

        # # --- Měřicí/navigační perioda (≈30 Hz) & timeRef ---
        # # MEAS = 33 ms, NAV = 1, TIMEREF = GPS(1)
        # p += (CFG_RATE_MEAS).to_bytes(4, "little");    p += (33).to_bytes(2, "little")  # U2
        # p += (CFG_RATE_NAV).to_bytes(4, "little");     p += (1).to_bytes(2, "little")   # U2
        # p += (CFG_RATE_TIMEREF).to_bytes(4, "little"); p += (1).to_bytes(1, "little")   # U1

        return p

    def _build_cfg_hnr_payload_30hz(self):
        """
        UBX-CFG-HNR (class 0x06, id 0x5C) – nastavení High Navigation Rate.
        highNavRate = 30 (Hz)
        Zbytek rezervovaný (0).
        """
        highNavRate = 30
        # Podle F9R: payload má 4 bajty (U1 rate + 3×reserved).
        return struct.pack("<BBBB", highNavRate, 0, 0, 0)

    def _build_valset_payload_speed(self):
        """
        Nastaví do RAM:
        - USB protokoly: UBX=1, NMEA=0
        - MSGOUT: NAV-PVT 1x/epochu, NAV-SAT 1x/10 epoch
        - NAV priority mode (CFG-RATE-NAV_PRIO) = 30 Hz
        - Měřící/navigační periodu (CFG-RATE-MEAS = 33 ms, CFG-RATE-NAV = 1)
        + timeRef = GPS
        """
        p = bytearray()
        p += b"\x00"      # version
        p += b"\x01"      # layers = RAM
        p += b"\x00\x00"  # reserved


        # # --- Měřicí/navigační perioda (≈10 Hz) & timeRef ---
        # # MEAS = 100 ms, NAV = 1, TIMEREF = GPS(1)
        p += (CFG_RATE_MEAS).to_bytes(4, "little");    p += (100).to_bytes(2, "little")  # U2
        p += (CFG_RATE_NAV).to_bytes(4, "little");     p += (1).to_bytes(2, "little")   # U2
        p += (CFG_RATE_TIMEREF).to_bytes(4, "little"); p += (0).to_bytes(1, "little")   # U1
        # --- Priority navigation mode & rychlost 10 Hz ---
        #p += (CFG_RATE_NAV_PRIO).to_bytes(4, "little"); p += (0).to_bytes(1, "little")

        return p

    def _build_cfg_rate_payload_10hz(self):
        """
        POZOR: Historický název, ale nastavujeme ~30 Hz:
        measRate = 33 ms (≈30 Hz)
        navRate  = 1
        timeRef  = 1 (GPS time)
        """
        measRate_ms = 33
        navRate = 1
        timeRef = 1
        return struct.pack("<HHH", measRate_ms, navRate, timeRef)

    def _wait_for_ack(self, expect_cls, expect_id, timeout=1.0):
        """
        Čeká na ACK-ACK/ACK-NAK k dané zprávě (expect_cls/expect_id).
        UBX-ACK-* payload = [clsID(1), msgID(1)]
        """
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            chunk = self.port.read(256)
            if not chunk:
                continue
            buf += chunk
            while True:
                mc, mi, payload, buf = parse_stream(buf)
                if mc is None:
                    break
                # ACK
                if mc == 0x05 and len(payload) >= 2:
                    ack_cls = payload[0]
                    ack_id  = payload[1]
                    if ack_cls == expect_cls and ack_id == expect_id:
                        if mi == 0x01:
                            print("✅ ACK-ACK přijato")
                            return True
                        elif mi == 0x00:
                            print("❌ ACK-NAK přijato")
                            return False
        print("⚠️ ACK timeout")
        return False

    # ---------- Životní cyklus ----------

    def start(self):
        if self.running:
            return True

        # 1) Vyber port podle MON-VER
        for dev in ["/dev/gnss1", "/dev/gnss2"]:
            try:
                #ser = serial.Serial(dev, baudrate=38400, timeout=0.2)
                ser = serial.Serial(dev, baudrate=115200, timeout=0.2)
                ser.write(build_msg(0x0A, 0x04))  # MON-VER
                data = ser.read(300)
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

        # 2) VALSET: USB protokoly + MSGOUT (RAM)
        valset = build_msg(0x06, 0x8A, self._build_valset_payload_data())
        self.port.write(valset)
        if not self._wait_for_ack(0x06, 0x8A):
            print("⚠️ Konfigurace data nebyla potvrzena")

        # # 3) CFG-RATE: 10 Hz (legacy, bývá spolehlivější – řeší „Speed“ NAK)
        # rate = build_msg(0x06, 0x08, self._build_valset_payload_speed())
        # self.port.write(rate)
        # if not self._wait_for_ack(0x06, 0x08):
        #     print("⚠️ Nastavení rychlosti nebyla potvrzena")

        # 4) Spusť reader thread
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
        try:
            self.port.close()
        except Exception:
            pass
        self.running = False
        self.device_type = None
        with self.lock:
            self.last_fix = None
            self.last_state = None
        print("🛑 GNSS stop")

    # ---------- Reader & stav ----------

    def _reader_loop(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = self.port.read(1024)
                if not data:
                    continue
                buf_before = len(buf)
                buf += data

                while True:
                    msg_class, msg_id, payload, new_buf = parse_stream(buf)
                    if msg_class is None:
                        # CRC fail rozpoznáme porovnáním délky (parse_stream posouvá o 2 na CRC fail)
                        if len(new_buf) < len(buf):
                            self.bad_counter += 1
                            ts = time.strftime("%H:%M:%S")
                            print(f"[{ts}] ⚠️ CRC fail, bad={self.bad_counter}")
                        buf = new_buf
                        break

                    # posun
                    buf = new_buf

                    # log
                    self.msg_counter += 1
                    ts = time.strftime("%H:%M:%S")
                    #print(f"[{ts}] UBX #{self.msg_counter} {msg_class:02X}/{msg_id:02X} len={len(payload)}")

                    # NAV-PVT
                    if msg_class == 0x01 and msg_id == 0x07:
                        fix = parse_nav_pvt(payload)
                        if fix:
                            with self.lock:
                                self.last_fix = fix
                                self.last_state = f"sat={fix['numSV']} fixType={fix['fixType']}"

            except Exception as e:
                self.ignored_counter += 1
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] ⚠️ Reader error: {e}, ignored={self.ignored_counter}")
                time.sleep(0.3)

    def get_state(self):
        if not self.running:
            return {"status": "IDLE"}
        with self.lock:
            return {
                "status": "RUNNING",
                "device": self.device_type,
                "state": self.last_state or "no data",
                "messages": self.msg_counter,
                "crc_fail": self.bad_counter,
                "ignored": self.ignored_counter,
            }

    def get_fix(self):
        if not self.running:
            return {"error": "not running"}
        with self.lock:
            return self.last_fix if self.last_fix else {"error": "NOFIX"}


gnss_device = GNSSDevice()
