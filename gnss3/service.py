# service.py
import queue
import threading

from gnss_serial import GnssSerialIO
from ubx_dispatcher import UbxDispatcher
from poller import RotatingPollerThread

# Importuj všechny handlery/buildery co potřebuješ!
from handlers.dummy import DummyHandler
from handlers.nav_hpposllh import NavHpposllhHandler
from handlers.nav_velned import NavVelNedHandler
from handlers.esf_ins import EsfInsHandler
from handlers.mon_sys import MonSysHandler
from handlers.mon_comms import MonCommsHandler
from handlers.esf_status import EsfStatusHandler
from handlers.ack import AckHandler

from builders import build_mon_sys_poll, build_mon_comms_poll, build_esf_status_poll

# Konfigurace polleru – přidej/ubírej podle potřeby
POLL_TABLE = [
    {"name": "MON-SYS",   "builder": build_mon_sys_poll},
    {"name": "MON-COMMS", "builder": build_mon_comms_poll},
    {"name": "ESF-STATUS","builder": build_esf_status_poll},
]

SERIAL_DEVICE = '/dev/gnss1'

class GnssService:
    def __init__(self):
        self.gnss = None
        self.dispatcher = None
        self.bin_stream_fifo = queue.Queue(maxsize=60)
        self.fifo_lock = threading.Lock()
        self.hpposllh_handler = NavHpposllhHandler(self.bin_stream_fifo, self.fifo_lock)
        self.running = False
        self.poller = None

    def start(self):
        if self.running:
            return "ALREADY_RUNNING"
        self.gnss = GnssSerialIO(SERIAL_DEVICE)
        self.gnss.open()
        self.poller = RotatingPollerThread(self.gnss.send_ubx, POLL_TABLE, period=2.0)
        self.poller.start()
        self.dispatcher = UbxDispatcher(self.gnss)
        self.dispatcher.register_handler(0x01, 0x05, DummyHandler())
        self.dispatcher.register_handler(0x01, 0x14, self.hpposllh_handler)
        self.dispatcher.register_handler(0x01, 0x12, NavVelNedHandler(self.bin_stream_fifo, self.fifo_lock))
        self.dispatcher.register_handler(0x10, 0x15, EsfInsHandler(self.bin_stream_fifo, self.fifo_lock))
        self.dispatcher.register_handler(0x0a, 0x39, MonSysHandler())
        self.dispatcher.register_handler(0x0a, 0x36, MonCommsHandler())
        self.dispatcher.register_handler(0x10, 0x10, EsfStatusHandler())
        self.dispatcher.register_handler(0x05, 0x00, AckHandler()) # NAK
        self.dispatcher.register_handler(0x05, 0x01, AckHandler()) # ACK
        self.dispatcher.start()
        self.running = True
        print("[SERVICE] STARTED")
        return "OK"

    def stop(self):
        if self.poller:
            self.poller.stop()
        if self.dispatcher:
            self.dispatcher.stop()
        if self.gnss:
            self.gnss.close()
        self.dispatcher = None
        self.gnss = None
        self.poller = None
        self.running = False
        print("[SERVICE] STOPPED")
        return "OK"

    def get_data_json(self):
        ctx = self.hpposllh_handler.get_last_context()
        if ctx:
            data = dict(ctx)
            if 'timestamp' in data:
                data['timestamp'] = float(data['timestamp'])
            return json.dumps(data)
        else:
            return "{}"

    def send_binary_stream(self, sock):
        try:
            while True:
                data = self.bin_stream_fifo.get(timeout=5.0)
                sock.sendall(data)
        except queue.Empty:
            pass
        except Exception as e:
            print(f"[SERVICE] Binary stream error: {e}")
