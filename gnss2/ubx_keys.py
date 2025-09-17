# ubx_keys.py (výběr)
class K:
    # --- CFG-RATE ---
    CFG_RATE_MEAS      = 0x30210001  # U2, ms; 100 => 10 Hz
    CFG_RATE_NAV       = 0x30210002  # U2, 1 = každé měření
    CFG_RATE_TIMEREF   = 0x20210003  # E1, 1 = GPS
    CFG_RATE_NAV_PRIO  = 0x20210004  # U1, Hz (0..30)  ← PRIO výstup
    # (viz tabulka CFG-RATE) :contentReference[oaicite:1]{index=1}

    # --- USB protokoly ---
    USBOUTPROT_UBX     = 0x10780001  # L
    USBOUTPROT_NMEA    = 0x10780002  # L
    USBINPROT_UBX      = 0x10770001  # L
    USBINPROT_NMEA     = 0x10770002  # L
    USBINPROT_RTCM3X   = 0x10770004  # L
    USBINPROT_SPARTN   = 0x10770005  # L
    # mapování CFG-PRT -> CFG-USB* / UART* / I2C* / SPI* viz dokumentace. :contentReference[oaicite:2]{index=2}

    # --- Vypnutí OUT na jiných rozhraních ---
    UART1OUTPROT_UBX   = 0x10740001
    UART1OUTPROT_NMEA  = 0x10740002
    UART2OUTPROT_UBX   = 0x10760001
    UART2OUTPROT_NMEA  = 0x10760002
    I2COUTPROT_UBX     = 0x10720001
    I2COUTPROT_NMEA    = 0x10720002
    SPIOUTPROT_UBX     = 0x107a0001
    SPIOUTPROT_NMEA    = 0x107a0002
    # (I2C/SPI klíče a defaulty) 

    # --- PRIO/NAV zprávy (USB) ---
    CFG_MSGOUT_NAV_HPPOSLLH_USB = 0x20910036  # :contentReference[oaicite:4]{index=4}
    CFG_MSGOUT_NAV_VELNED_USB   = 0x20910045  # :contentReference[oaicite:5]{index=5}
    CFG_MSGOUT_NAV_ATT_USB      = 0x20910022  # :contentReference[oaicite:6]{index=6}
    CFG_MSGOUT_ESF_INS_USB      = 0x20910117  # :contentReference[oaicite:7]{index=7}
    CFG_MSGOUT_NAV_EOE_USB      = 0x20910162  # :contentReference[oaicite:8]{index=8}

    CLASS_NAV = 0x01
    ID_NAV_HPPOSLLH = 0x14
    ID_NAV_VELNED   = 0x12
    ID_NAV_ATT      = 0x01  # (ATT je ve třídě NAV s ID 0x01? Pozor: ATT má ID 0x01 *v NAV?* Správně je NAV-ATT (0x01,0x01) dle tabulek.)
    ID_NAV_EOE      = 0x61

    CLASS_ESF = 0x10
    ID_ESF_INS  = 0x15

    CLASS_MON = 0x0A
    ID_MON_TXBUF = 0x08
    ID_MON_SYS   = 0x39

    CLASS_ACK = 0x05
    ID_ACK_NAK = 0x00
    ID_ACK_ACK = 0x01

    NAV_ATT      = (0x01, 0x05)
    NAV_VELNED   = (0x01, 0x12)
    NAV_HPPOSLLH = (0x01, 0x14)
    NAV_EOE      = (0x01, 0x61)
    ESF_INS      = (0x10, 0x15)

    NEEDED = {NAV_ATT, NAV_VELNED, NAV_HPPOSLLH, ESF_INS}