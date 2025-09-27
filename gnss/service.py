# service.py
import queue
import threading

from gnss_serial import GnssSerialIO
from ubx_dispatcher import UbxDispatcher
from nmea_dispatcher import NmeaDispatcher
from poller import RotatingPollerThread

from gnss.nav_fusion import NavFusion


# Importuj všechny handlery/buildery co potřebuješ!
from handlers.dummy import DummyHandler
from handlers.nav_hpposllh import NavHpposllhHandler
from handlers.nav_velned import NavVelNedHandler
from handlers.esf_ins import EsfInsHandler
from handlers.mon_sys import MonSysHandler
from handlers.mon_comms import MonCommsHandler
from handlers.esf_status import EsfStatusHandler
from handlers.ack import AckHandler
from handlers.esf_meas import EsfMeasHandler
from handlers.nav_att import NavAttHandler
from handlers.esf_raw import EsfRawHandler
from handlers.nav_pvat import NavPvatHandler

from handlers.nmea_gga_handler import NmeaGgaHandler

SERIAL_DEVICE = '/dev/gnss1'

class GnssService:
    def __init__(self):
        self.gnss = None
        self.ubx_dispatcher = None
        self.nmea_dispatcher = None
        self.bin_stream_fifo = queue.Queue(maxsize=60)
        self.fifo_lock = threading.Lock()
        
        self.running = False
        self.poller = None

        # Handlery        
        self.hpposllh_handler = NavHpposllhHandler(self.bin_stream_fifo, self.fifo_lock)
        self.nav_pvat_handler = NavPvatHandler(self.bin_stream_fifo, self.fifo_lock)
        self.nmea_gga_handler = NmeaGgaHandler()

        self._nav_fusion = NavFusion(raw_window_sec=0.15, max_samples=256)

    def start(self):
        if self.running:
            return "ALREADY_RUNNING"
        self.gnss = GnssSerialIO(SERIAL_DEVICE)
        self.poller = RotatingPollerThread(self.gnss.send_ubx)

        # UBX dispatcher
        self.ubx_dispatcher = UbxDispatcher(self.gnss)
        self.ubx_dispatcher.register_handler(0x0a, 0x39, MonSysHandler())
        self.ubx_dispatcher.register_handler(0x01, 0x17, self.nav_pvat_handler)
        self.ubx_dispatcher.register_handler(0x10, 0x03, EsfRawHandler())
        #self.dispatcher.register_handler(0x01, 0x05, DummyHandler())
        #self.ubx_dispatcher.register_handler(0x01, 0x14, self.hpposllh_handler)
        #self.ubx_dispatcher.register_handler(0x01, 0x12, NavVelNedHandler(self.bin_stream_fifo, self.fifo_lock))
        #self.ubx_dispatcher.register_handler(0x10, 0x15, EsfInsHandler(self.bin_stream_fifo, self.fifo_lock))
        #self.ubx_dispatcher.register_handler(0x0a, 0x36, MonCommsHandler())
        #self.ubx_dispatcher.register_handler(0x10, 0x10, EsfStatusHandler())
        #self.ubx_dispatcher.register_handler(0x05, 0x00, AckHandler()) # NAK
        #self.ubx_dispatcher.register_handler(0x05, 0x01, AckHandler()) # ACK
        #self.ubx_dispatcher.register_handler(0x10, 0x02, EsfMeasHandler())
        #self.ubx_dispatcher.register_handler(0x01, 0x05, NavAttHandler())
        #self.ubx_dispatcher.register_handler(0x10, 0x03, EsfRawHandler())

        # NMEA dispatcher
        self.nmea_dispatcher = NmeaDispatcher(self.gnss)
        self.nmea_dispatcher.register_3('GGA', self.nmea_gga_handler)

        # Start všeho
        self.nmea_dispatcher.start()
        self.ubx_dispatcher.start()
        self.gnss.open()
        self.poller.start()

        self.running = True
        print("[SERVICE] STARTED")
        return "OK"

    def stop(self):
        if self.poller:
            self.poller.stop()
        if self.gnss:
            self.gnss.close()
        if self.ubx_dispatcher:
            self.ubx_dispatcher.stop()
        if self.nmea_dispatcher:
            self.nmea_dispatcher.stop()

        self.nmea_dispatcher = None
        self.ubx_dispatcher = None
        self.gnss = None
        self.poller = None

        self.running = False
        print("[SERVICE] STOPPED")
        return "OK"

    def get_gga(self):
        return self.nmea_gga_handler.get_last_gga()

    def get_data_json(self):
        import json
        res = self._nav_fusion.get_latest()
        if not res:
            return "{}"
        obj = {
            "iTOW": res.iTOW_ms,
            "heading_deg": res.heading_deg,
            "speed_mps": res.speed_mps,
            "quality": res.quality,
            "t_mono": res.t_mono,
        }
        return json.dumps(obj)

    # === NavFusion API (pro klienty/handler integraci) =======================
    def get_nav_fusion(self) -> NavFusion:
        """Umožní klientům/handlerům sáhnout na NavFusion instanci přes service."""
        return self._nav_fusion

    def get_latest_data(self) -> str:
        """
        Vrací JSON snapshot posledního výsledku NavFusion (pro příkaz DATA).
        Pokud zatím nic není, vrátí "{}".
        """
        import json
        res = self._nav_fusion.get_latest()
        if not res:
            return "{}"
        obj = {
            "iTOW": res.iTOW_ms,
            "heading_deg": res.heading_deg,
            "speed_mps": res.speed_mps,
            "quality": res.quality,
            "t_mono": res.t_mono,
        }
        return json.dumps(obj)

