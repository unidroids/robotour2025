# gnss_config.py
import time, queue, logging
from ubx_proto import build_cfg_valset
from ubx_keys import K

def make_initial_config(prio_hz=10, enable_nav_eoe=True):
    items = []

    # --- Výstupy/protokoly: jen UBX out na USB; USB in: UBX+RTCM3X(+SPARTN); ostatní OUT vypnout ---
    items += [
        (K.USBOUTPROT_UBX,   1, 'L'),
        (K.USBOUTPROT_NMEA,  0, 'L'),
        (K.USBINPROT_UBX,    1, 'L'),
        (K.USBINPROT_NMEA,   0, 'L'),
        (K.USBINPROT_RTCM3X, 1, 'L'),
        (K.USBINPROT_SPARTN, 0, 'L'),  # dej 1 pokud budeš používat SPARTN
    ]
    for key in (
        K.UART1OUTPROT_UBX, K.UART1OUTPROT_NMEA,
        K.UART2OUTPROT_UBX, K.UART2OUTPROT_NMEA,
        K.I2COUTPROT_UBX,   K.I2COUTPROT_NMEA,
        K.SPIOUTPROT_UBX,   K.SPIOUTPROT_NMEA,
    ):
        items.append((key, 0, 'L'))  # OUT vypnout

    # --- Rate / PRIO ---
    #meas_ms = 33 if prio_hz >= 30 else 500   # 33ms≈30Hz, 100ms=10Hz
    meas_ms = 500
    items += [
        (K.CFG_RATE_MEAS,     meas_ms, 'U2'),
        (K.CFG_RATE_NAV,      1,       'U2'),
        (K.CFG_RATE_TIMEREF,  1,       'U1'),    # GPS
        (K.CFG_RATE_NAV_PRIO, prio_hz, 'U1'),    # 0..30 Hz
    ]
    # (PRIO funguje pro vyjmenované zprávy do 30 Hz) 

    # --- PRIO zprávy na USB (rate=1 => každá epocha) ---
    items += [
        (K.CFG_MSGOUT_NAV_HPPOSLLH_USB, 1, 'U1'),
        (K.CFG_MSGOUT_NAV_VELNED_USB,   1, 'U1'),
        (K.CFG_MSGOUT_NAV_ATT_USB,      1, 'U1'),
        (K.CFG_MSGOUT_ESF_INS_USB,      1, 'U1'),
    ]
    if enable_nav_eoe:
        items.append((K.CFG_MSGOUT_NAV_EOE_USB, 1, 'U1'))

    print ("Config items:",items)
    # Pošli pouze do RAM (rychlé ověření), ne do BBR/Flash:
    return build_cfg_valset(items, layer_ram=True, layer_bbr=False, layer_flash=False)


def apply_config_with_ack(ser, recv_queue, frame_bytes, timeout=1.0):
    """Pošle VALSET a čeká na UBX-ACK-ACK/NAK k (0x06,0x8A). Vrací True/False."""
    while True:
        try:
            cls, mid, payload, ts = recv_queue.get(timeout=0.05)
        except queue.Empty:
            break

    ser.write(frame_bytes)
    ser.flush()
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            cls, mid, payload, ts = recv_queue.get(timeout=1)
        except queue.Empty:
            continue
        # ACK má payload: clsID, msgID
        if cls == 0x05 and mid in (0x00,0x01):
            if len(payload) >= 2 and payload[0]==0x06 and payload[1]==0x8A:
                return (mid == 0x01)
        # jiné zprávy vrátíme zpět do fronty pro hlavní loop
        recv_queue.put((cls,mid,payload,ts))
    return False
