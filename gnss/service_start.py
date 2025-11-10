# service_start.py
import threading
from types import SimpleNamespace

from gnss_serial import GnssSerialIO
from ubx_dispatcher import UbxDispatcher
from nmea_dispatcher import NmeaDispatcher
from poller import RotatingPollerThread
from nav_fusion import NavFusion


from handlers.nav_pvat import NavPvatHandler
from handlers.esf_raw import EsfRawHandler
from handlers.nmea_gga_handler import NmeaGgaHandler
from handlers.mon_sys import MonSysHandler

SERIAL_DEVICE = '/dev/gnss1'

# Konfigurace polleru – přidej/ubírej podle potřeby
from builders import build_mon_sys_poll
POLL_TABLE = [
    {"name": "MON-SYS",   "builder": build_mon_sys_poll},
]

from nav_fusion import NavFusion


def init_gnss_service():
    # === Nav Fusion ===
    nav_fusion = NavFusion()
    #nav_fusion = NavFusion(raw_window_sec=0.15, max_samples=256)

    # === Handlery s vazbami na sdílené fronty/locky ===
    nav_pvat_handler = NavPvatHandler(on_data=nav_fusion.on_nav_pvat)
    #esf_raw_handler = EsfRawHandler(on_data=nav_fusion.on_esf_raw)
    nmea_gga_handler = NmeaGgaHandler()
    mon_sys_handler = MonSysHandler()

    # === GNSS + Dispatchery ===
    gnss = GnssSerialIO(SERIAL_DEVICE)

    # === Poller ===
    poller = RotatingPollerThread(gnss.send_ubx, poll_table=POLL_TABLE, period=5.0)

    # ===  UBX Dispatchery + registrace handlerů ===
    ubx_dispatcher = UbxDispatcher(gnss)
    ubx_dispatcher.register_handler(0x01, 0x17, nav_pvat_handler)
    #ubx_dispatcher.register_handler(0x10, 0x03, esf_raw_handler)
    ubx_dispatcher.register_handler(0x0a, 0x39, mon_sys_handler)

    # === NMEA Dispatchery + registrace handlerů ===
    nmea_dispatcher = NmeaDispatcher(gnss)
    nmea_dispatcher.register_3('GGA', nmea_gga_handler)

    # === Inicializované jádra gnss služby ===
    return {
        "nav_fusion": nav_fusion,
        "gnss": gnss,
        "poller": poller,
        "ubx_dispatcher": ubx_dispatcher,
        "nmea_dispatcher": nmea_dispatcher,
        "handlers": {
            "nav_pvat": nav_pvat_handler,
            #"esf_raw": esf_raw_handler,
            "nmea_gga": nmea_gga_handler,
            "mon_sys": mon_sys_handler
        },        
    }


# Dostupné handlery podle potřeby
# from handlers.dummy import DummyHandler
# from handlers.nav_hpposllh import NavHpposllhHandler
# from handlers.nav_velned import NavVelNedHandler
# from handlers.esf_ins import EsfInsHandler
# from handlers.mon_sys import MonSysHandler
# from handlers.mon_comms import MonCommsHandler
# from handlers.esf_status import EsfStatusHandler
# from handlers.ack import AckHandler
# from handlers.esf_meas import EsfMeasHandler
# from handlers.nav_att import NavAttHandler
# from handlers.esf_raw import EsfRawHandler
# from handlers.nav_pvat import NavPvatHandler
# from handlers.nmea_gga_handler import NmeaGgaHandler

# Další handlery podle potřeby
# ubx_dispatcher.register_handler(0x01, 0x14, hpposllh_handler)
# ubx_dispatcher.register_handler(0x01, 0x12, NavVelNedHandler(bin_stream_fifo, fifo_lock))
# ubx_dispatcher.register_handler(0x10, 0x15, EsfInsHandler(bin_stream_fifo, fifo_lock))
# ubx_dispatcher.register_handler(0x0a, 0x36, MonCommsHandler())
# ubx_dispatcher.register_handler(0x10, 0x10, EsfStatusHandler())
# ubx_dispatcher.register_handler(0x05, 0x00, AckHandler())
# ubx_dispatcher.register_handler(0x05, 0x01, AckHandler())
# ubx_dispatcher.register_handler(0x10, 0x02, EsfMeasHandler())
# ubx_dispatcher.register_handler(0x01, 0x05, NavAttHandler())
# ubx_dispatcher.register_handler(0x10, 0x03, EsfRawHandler())

# Importy builderů podle potřeby
#from builders import build_mon_sys_poll
#from builders import build_mon_comms_poll
#from builders import build_esf_status_poll
#from builders import build_esf_raw_poll
#from builders import build_gga_poll

# Konfigurace polleru – přidej/ubírej podle potřeby
#POLL_TABLE = [
#    {"name": "MON-SYS",   "builder": build_mon_sys_poll},
#    {"name": "MON-COMMS", "builder": build_mon_comms_poll},
#    {"name": "ESF-STATUS","builder": build_esf_status_poll},
#    {"name": "GGA",      "builder": build_gnq_gga_poll},
#    {"name": "ESF-RAW","builder": build_poll_esf_raw},
#]
