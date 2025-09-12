# device.py – GNSS zařízení pro Robotour
import serial
import struct
import threading
import time
import queue
from ubx import build_msg, parse_stream
from ubx import parse_nav_pvt, parse_esf_status
from ubx import build_esf_meas_ticks
from esf_dicts import fusion_modes, sensor_types

# CFG-MSGOUT klíče (USB) – UBX-CFG-VALSET
CFG_USBOUT_UBX = 0x10780001          # CFG-USBOUTPROT-UBX (U1)
CFG_USBOUT_NMEA = 0x10780002         # CFG-USBOUTPROT-NMEA (U1)
CFG_MSGOUT_NAV_PVT_USB = 0x20910009  # U1: 1 = každou epochu
CFG_MSGOUT_NAV_SAT_USB = 0x20910018  # U1: 10 = každá 10. epocha
CFG_MSGOUT_ESF_STATUS_USB = 0x20910145 

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
        self.last_esf_status = None
        self.msg_counter = 0
        self.bad_counter = 0        # CRC fail
        self.ignored_counter = 0    # jiné chyby/bufferové odřezky

        self.tx_queue = queue.Queue()
        self.writer_thread = None        

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
        p += (CFG_MSGOUT_ESF_STATUS_USB).to_bytes(4, "little");p += (1).to_bytes(1, "little")        
        # p += (CFG_MSGOUT_NAV_SAT_USB).to_bytes(4, "little"); p += (10).to_bytes(1, "little")

        p += (CFG_NAVSPG_DYNMODEL).to_bytes(4, "little"); p += (11).to_bytes(1, "little")   # 11 = mower

        return p

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


    def _start_writer(self):
        if self.writer_thread and self.writer_thread.is_alive():
            return
        def _writer():
            while not self.stop_event.is_set():
                try:
                    data = self.tx_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    # jediný bod, kde se opravdu zapisuje do sériového portu
                    self.port.write(data)
                    print(f"Writer to device done.")
                except Exception as e:
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] ⚠️ Writer error: {e}")
        self.writer_thread = threading.Thread(target=_writer, daemon=True)
        self.writer_thread.start()

    def enqueue_raw(self, data: bytes):
        """Vloží syrová data (UBX/RTCM/SPARTN) do odesílací fronty."""
        if self.running:
            self.tx_queue.put(data)

    def send_esf_ticks(self, time_tag: int, l_ticks: int, l_dir: int, r_ticks: int, r_dir: int):
        """
        Přijme odometrické informace (tick počítadla a směry 0=+,1=-),
        sestaví UBX-ESF-MEAS (wheel ticks) a vloží do odesílací fronty.
        """
        # směry přeložíme do znaménka čítačů
        lt = -l_ticks if l_dir == 1 else l_ticks
        rt = -r_ticks if r_dir == 1 else r_ticks
        from ubx import build_esf_meas_ticks
        msg = build_esf_meas_ticks(lt, rt, time_tag=time_tag)
        self.enqueue_raw(msg)

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
            return False

        # 4) Spusť reader and writer thread
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        self._start_writer()  #writer

        self.running = True
        return True

    def stop(self):
        if not self.running:
            return
        self.stop_event.set()
        if self.reader_thread:
            self.reader_thread.join(2.0)
        self.stop_event.set()
        if self.writer_thread:
            self.writer_thread.join(2.0)
        # ukonči klienty
        try:
            self.port.close()
        except Exception:
            pass
        # vyprázdni frontu
        try:
            while True:
                self.tx_queue.get_nowait()
        except queue.Empty:
            pass            
        self.running = False
        self.device_type = None
        with self.lock:
            self.last_fix = None
            self.last_state = None
            self.last_esf_status = None
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
                    
                    print(f"[{ts}] UBX #{self.msg_counter} {msg_class:02X}/{msg_id:02X} len={len(payload)}")

                    # NAV-PVT
                    if msg_class == 0x01 and msg_id == 0x07:
                        fix = parse_nav_pvt(payload)
                        if fix:
                            with self.lock:
                                self.last_fix = fix
                                self.last_state = f"sat={fix['numSV']} fixType={fix['fixType']}"

                    # ESF-STATUS
                    if msg_class == 0x10 and msg_id == 0x10:
                        st = parse_esf_status(payload)
                        if st:
                            with self.lock:
                                self.last_esf_status = st
                                # stručný status string
                                #self.last_state = f"ESF fusion={st['fusionMode']} sensors={len(st['sensors'])}"
                    

            except Exception as e:
                self.ignored_counter += 1
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] ⚠️ Reader error: {e}, ignored={self.ignored_counter}")
                time.sleep(0.3)

    def get_state(self):
        if not self.running:
            return {"status": "IDLE"}
        with self.lock:
            state = {
                "status": "RUNNING",
                "device": self.device_type,
                "state": self.last_state or "no data",
                "messages": self.msg_counter,
                "crc_fail": self.bad_counter,
                "ignored": self.ignored_counter,
            }

            if hasattr(self, "last_esf_status") and self.last_esf_status:
                esf = self.last_esf_status
                esf_info = {
                    "fusionMode": fusion_modes.get(esf["fusionMode"], esf["fusionMode"]),
                    "sensors": []
                }
                for s in esf["sensors"]:
                    esf_info["sensors"].append({
                        "type": sensor_types.get(s["sensorType"], s["sensorType"]),
                        "used": s["used"],
                        "ready": s["ready"],
                        "qual": s["qual"],
                        "freq": s["freq"],
                        "faults": s["faults"],
                    })
                state["esf_status"] = esf_info

            return state

    def get_fix(self):
        if not self.running:
            return {"error": "not running"}
        with self.lock:
            return self.last_fix if self.last_fix else {"error": "NOFIX"}


gnss_device = GNSSDevice()
