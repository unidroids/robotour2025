#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
u-blox ZED-F9R configurator – VALSET sender for Robotour
- Supports Serial (USB CDC ACM) and TCP sockets
- Profiles: 'physical', 'operational', 'all'
- Layers: RAM, BBR, FLASH
- Full, self-contained implementation (no extra libs beyond pyserial for serial mode)

Author: Robotour 2025
"""

import argparse
import socket
import struct
import sys
import time
from typing import Dict, List, Tuple, Optional

try:
    import serial  # pyserial (optional if you use --tcp)
except ImportError:
    serial = None


# =========================
# UBX utilities
# =========================

SYNC1 = 0xB5
SYNC2 = 0x62

# UBX classes/ids
UBX_ACK_CLASS = 0x05
UBX_ACK_ACK   = 0x01
UBX_ACK_NAK   = 0x00

UBX_CFG_CLASS = 0x06
UBX_CFG_VALSET = 0x8A
# (VALGET = 0x8B)  # not used here, can be added if needed

def ubx_checksum(payload: bytes) -> Tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def build_ubx(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    header = bytes([SYNC1, SYNC2, msg_class, msg_id])
    length = struct.pack('<H', len(payload))
    ck_a, ck_b = ubx_checksum(bytes([msg_class, msg_id]) + length + payload)
    return header + length + payload + bytes([ck_a, ck_b])


# =========================
# CFG-VALSET payload builder
# =========================
#
# VALSET payload:
#  U1 version (0)
#  U1 layers (bitmask: 0=RAM, 1=BBR, 2=FLASH -> e.g. 0b0000_0111 for all)
#  U2 reserved (0)
#  Repeated: U4 keyId + value (size by type)
#
# Key type codes we actually use: L, E1, U1, U2, U4, I2
# (Values must be already scaled to the integer the key expects.)

# Map of keys we set: key_id, type_code, scale_info
# type_code => how many bytes + signedness
# scale: None => raw integer; "1e-6" => multiply value[float] by 1e6 and round; "1e-2_deg" => deg*100
KEYS: Dict[str, Dict] = {
    # ===== SFCORE – lever arm IMU->CRP (cm), I2
    "CFG-SFCORE-IMU2CRP_LA_X": {"id": 0x30080002, "type": "I2", "scale": "cm"},
    "CFG-SFCORE-IMU2CRP_LA_Y": {"id": 0x30080003, "type": "I2", "scale": "cm"},
    "CFG-SFCORE-IMU2CRP_LA_Z": {"id": 0x30080004, "type": "I2", "scale": "cm"},

    # ===== SFIMU – lever arm IMU->ANT (cm), I2
    "CFG-SFIMU-IMU2ANT_LA_X": {"id": 0x30060020, "type": "I2", "scale": "cm"},
    "CFG-SFIMU-IMU2ANT_LA_Y": {"id": 0x30060021, "type": "I2", "scale": "cm"},
    "CFG-SFIMU-IMU2ANT_LA_Z": {"id": 0x30060022, "type": "I2", "scale": "cm"},

    # ===== SFIMU – user mount alignment
    "CFG-SFIMU-AUTO_MNTALG_ENA":      {"id": 0x10060027, "type": "L",  "scale": None},
    "CFG-SFIMU-IMU_MNTALG_YAW":       {"id": 0x4006002D, "type": "U4", "scale": "1e-2_deg"},
    "CFG-SFIMU-IMU_MNTALG_PITCH":     {"id": 0x3006002E, "type": "I2", "scale": "1e-2_deg"},
    "CFG-SFIMU-IMU_MNTALG_ROLL":      {"id": 0x3006002F, "type": "I2", "scale": "1e-2_deg"},
    "CFG-SFIMU-IMU_MNTALG_TOLERANCE": {"id": 0x20060030, "type": "E1", "scale": None},  # 0=LOW,1=HIGH

    # ===== SFODO – lever arm IMU->VRP (cm), I2
    "CFG-SFODO-IMU2VRP_LA_X": {"id": 0x30070012, "type": "I2", "scale": "cm"},
    "CFG-SFODO-IMU2VRP_LA_Y": {"id": 0x30070013, "type": "I2", "scale": "cm"},
    "CFG-SFODO-IMU2VRP_LA_Z": {"id": 0x30070014, "type": "I2", "scale": "cm"},

    # ===== SFODO – wheel tick model
    "CFG-SFODO-FACTOR":        {"id": 0x40070007, "type": "U4", "scale": "1e-6"},  # m/tick -> *1e6
    "CFG-SFODO-QUANT_ERROR":   {"id": 0x40070008, "type": "U4", "scale": "1e-6"},  # m -> *1e6
    "CFG-SFODO-COUNT_MAX":     {"id": 0x40070009, "type": "U4", "scale": None},
    "CFG-SFODO-LATENCY":       {"id": 0x3007000A, "type": "U2", "scale": None},    # ms
    "CFG-SFODO-FREQUENCY":     {"id": 0x2007000B, "type": "U1", "scale": None},    # Hz
    "CFG-SFODO-COMBINE_TICKS": {"id": 0x10070001, "type": "L",  "scale": None},
    "CFG-SFODO-USE_SPEED":     {"id": 0x10070003, "type": "L",  "scale": None},
    "CFG-SFODO-DIS_AUTOCOUNTMAX":   {"id": 0x10070004, "type": "L", "scale": None},
    "CFG-SFODO-DIS_AUTODIRPINPOL":  {"id": 0x10070005, "type": "L", "scale": None},
    "CFG-SFODO-DIS_AUTOSPEED":      {"id": 0x10070006, "type": "L", "scale": None},
    "CFG-SFODO-CNT_BOTH_EDGES":     {"id": 0x1007000D, "type": "L", "scale": None},
    "CFG-SFODO-SPEED_BAND":         {"id": 0x3007000E, "type": "U2", "scale": None}, # cm/s
    "CFG-SFODO-USE_WT_PIN":         {"id": 0x1007000F, "type": "L", "scale": None},
    "CFG-SFODO-DIR_PINPOL":         {"id": 0x10070010, "type": "L", "scale": None},
    "CFG-SFODO-DIS_AUTOSW":         {"id": 0x10070011, "type": "L", "scale": None},
    "CFG-SFODO-DIS_DIR_INFO":       {"id": 0x1007001C, "type": "L", "scale": None},

    # ===== SFCORE – USE_SF (ne-fyzické, ale často chceme zapnout)
    "CFG-SFCORE-USE_SF": {"id": 0x10080001, "type": "L", "scale": None},

    # (Pozn.: CFG-NAVSPG-DYNMODEL = 11 (RLM) lze přidat, až budeš chtít – stačí doplnit jeho keyId.)
    "CFG-NAVSPG-DYNMODEL": {"id": 0x20110021, "type": "E1", "scale": None}
    
}

def scale_value(key: str, human_value) -> int:
    spec = KEYS[key]
    scale = spec["scale"]
    if scale is None:
        # raw integer/boolean/enumeration already
        return int(human_value)
    if scale == "cm":
        # values are already in centimeters (signed)
        return int(human_value)
    if scale == "1e-6":
        # float -> integer micro-units
        return int(round(float(human_value) * 1_000_000))
    if scale == "1e-2_deg":
        # degrees -> centi-degrees
        return int(round(float(human_value) * 100))
    raise ValueError(f"Unknown scale spec: {scale}")

def pack_typed(type_code: str, value_int: int) -> bytes:
    if type_code in ("L", "E1", "U1"):
        return struct.pack('<B', value_int & 0xFF)
    if type_code == "I1":
        return struct.pack('<b', int(value_int))
    if type_code == "U2":
        return struct.pack('<H', value_int & 0xFFFF)
    if type_code == "I2":
        return struct.pack('<h', int(value_int))
    if type_code == "U4":
        return struct.pack('<I', value_int & 0xFFFFFFFF)
    if type_code == "I4":
        return struct.pack('<i', int(value_int))
    # We don't use U8/I8 here; add if needed
    raise ValueError(f"Unsupported type_code: {type_code}")

def build_valset(cfg_items: List[Tuple[str, int]], layers_mask: int) -> bytes:
    """
    cfg_items: list of (key_name, scaled_int_value)
    layers_mask: bitmask (1=RAM, 2=BBR, 4=FLASH) -> combine (e.g., 0x07 for all)
    """
    payload = bytearray()
    payload += struct.pack('<BBH', 0x00, layers_mask & 0xFF, 0x0000)  # version=0, layers, reserved
    for key_name, val_int in cfg_items:
        spec = KEYS[key_name]
        key_id = spec["id"]
        type_code = spec["type"]
        payload += struct.pack('<I', key_id)
        payload += pack_typed(type_code, val_int)
    return build_ubx(UBX_CFG_CLASS, UBX_CFG_VALSET, bytes(payload))


# =========================
# IO transport
# =========================

class Transport:
    def write(self, data: bytes) -> None:
        raise NotImplementedError
    def read(self, timeout_s: float = 0.5) -> bytes:
        raise NotImplementedError
    def close(self) -> None:
        pass

class SerialTransport(Transport):
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        if serial is None:
            raise RuntimeError("pyserial not installed. pip install pyserial")
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    def write(self, data: bytes) -> None:
        self.ser.write(data)
        self.ser.flush()
    def read(self, timeout_s: float = 0.5) -> bytes:
        end = time.time() + timeout_s
        buf = bytearray()
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
            else:
                time.sleep(0.01)
        return bytes(buf)
    def close(self) -> None:
        self.ser.close()

class TcpTransport(Transport):
    def __init__(self, host: str, port: int, timeout: float = 2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
    def write(self, data: bytes) -> None:
        self.sock.sendall(data)
    def read(self, timeout_s: float = 0.5) -> bytes:
        self.sock.settimeout(timeout_s)
        try:
            return self.sock.recv(4096)
        except socket.timeout:
            return b""
    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass


# =========================
# Profiles (values in human units; scaler will convert)
# =========================

PROFILE_PHYSICAL: Dict[str, float] = {
    "CFG-NAVSPG-DYNMODEL": 11, #11,

    # lever-arms (cm)
    "CFG-SFCORE-IMU2CRP_LA_X": 0, #-28,
    "CFG-SFCORE-IMU2CRP_LA_Y": 0, #-1,
    "CFG-SFCORE-IMU2CRP_LA_Z": 0, #-93,

    "CFG-SFODO-IMU2VRP_LA_X": 0, #-28,
    "CFG-SFODO-IMU2VRP_LA_Y": 0, #-1,
    "CFG-SFODO-IMU2VRP_LA_Z": 0, #-93,

    "CFG-SFIMU-IMU2ANT_LA_X": 0, #4,
    "CFG-SFIMU-IMU2ANT_LA_Y": 0, #-1,
    "CFG-SFIMU-IMU2ANT_LA_Z": 0, #3,

    # IMU mount (user-defined)
    "CFG-SFIMU-AUTO_MNTALG_ENA": 1, #0,    # manual alignment
    "CFG-SFIMU-IMU_MNTALG_YAW": 0, #90.0,  # deg -> *100
    "CFG-SFIMU-IMU_MNTALG_PITCH": 0, #2.0, # deg -> *100
    "CFG-SFIMU-IMU_MNTALG_ROLL": 0, #0.0,  # deg -> *100
    "CFG-SFIMU-IMU_MNTALG_TOLERANCE": 0,  # LOW

    # Wheel model (geometry)
    "CFG-SFODO-FACTOR": 0, #0.008866,      # m/tick (10" + 90 ticks/rot)
    "CFG-SFODO-QUANT_ERROR": 0, #0.008866, # m (quantization step)
    "CFG-SFODO-COUNT_MAX": 1, #8388607,    # 2^23 - 1 (absolute ticks rollover-1)
}

PROFILE_OPERATIONAL: Dict[str, int] = {
    # ESF/Odo behavior
    "CFG-SFODO-LATENCY": 0, #1,             # ms
    "CFG-SFODO-FREQUENCY": 0, #10,          # Hz
    "CFG-SFODO-COMBINE_TICKS": 1,
    "CFG-SFODO-USE_SPEED": 0,
    "CFG-SFODO-DIS_AUTOCOUNTMAX": 0,
    "CFG-SFODO-DIS_AUTODIRPINPOL": 1,   # per RLM recommendation
    "CFG-SFODO-DIS_AUTOSPEED": 1,
    "CFG-SFODO-CNT_BOTH_EDGES": 0,
    "CFG-SFODO-SPEED_BAND": 0,          # not used in our setup
    "CFG-SFODO-USE_WT_PIN": 0,
    "CFG-SFODO-DIR_PINPOL": 0,
    "CFG-SFODO-DIS_AUTOSW": 0,
    "CFG-SFODO-DIS_DIR_INFO": 0,

    # Sensor fusion enable (non-physical, ale prakticky nutné)
    "CFG-SFCORE-USE_SF": 1,
}

def build_items_from_profile(profile: Dict[str, float]) -> List[Tuple[str, int]]:
    items = []
    for k, human_v in profile.items():
        if k not in KEYS:
            raise KeyError(f"Unknown key in profile: {k}")
        v_int = scale_value(k, human_v)
        items.append((k, v_int))
    return items


# =========================
# ACK sniffer (best-effort)
# =========================

def find_ack(buf: bytes) -> Optional[Tuple[bool, int, int]]:
    """
    Tries to find UBX-ACK-(ACK/NAK) for the last packet in buf.
    Returns tuple (is_ack, cls, id) if found, else None.
    """
    i = 0
    while i + 8 <= len(buf):
        if buf[i] == SYNC1 and buf[i+1] == SYNC2:
            cls = buf[i+2]
            mid = buf[i+3]
            length = struct.unpack_from('<H', buf, i+4)[0]
            end = i + 6 + length + 2
            if end <= len(buf):
                payload = buf[i+6:i+6+length]
                if cls == UBX_ACK_CLASS and mid in (UBX_ACK_ACK, UBX_ACK_NAK) and len(payload) == 2:
                    # payload: class,id of acknowledged message
                    acked_cls = payload[0]
                    acked_id  = payload[1]
                    return (mid == UBX_ACK_ACK, acked_cls, acked_id)
                i = end
            else:
                break
        else:
            i += 1
    return None


# =========================
# Main CLI
# =========================

def parse_layers(s: str) -> int:
    s = s.upper()
    mask = 0
    for part in s.split(','):
        part = part.strip()
        if part == 'RAM':
            mask |= 0x01
        elif part == 'BBR':
            mask |= 0x02
        elif part == 'FLASH':
            mask |= 0x04
        elif part == '':
            continue
        else:
            raise ValueError(f"Unknown layer: {part}")
    if mask == 0:
        raise ValueError("At least one layer required (RAM,BBR,FLASH)")
    return mask

def main():
    ap = argparse.ArgumentParser(description="u-blox ZED-F9R configurator (VALSET)")
    g1 = ap.add_mutually_exclusive_group(required=True)
    g1.add_argument("--serial", help="Serial device (e.g., /dev/ttyACM0)", default="/dev/gnss1")
    g1.add_argument("--tcp", help="TCP host:port (e.g., 192.168.1.10:2000)", default=None)
    ap.add_argument("--baud", type=int, default=115200, help="Serial baudrate (ignored on USB CDC ACM)")
    ap.add_argument("--set", choices=["physical", "operational", "all"], default="physical",
                    help="Which profile to apply")
    ap.add_argument("--layers", default="RAM,BBR,FLASH", help="Layers to write (comma-separated)")
    ap.add_argument("--dry-run", action="store_true", help="Only print what would be sent")
    ap.add_argument("--delay", type=float, default=0.10, help="Delay after write (s) before reading for ACK")
    args = ap.parse_args()

    layers_mask = parse_layers(args.layers)

    # Build items
    if args.set == "physical":
        items = build_items_from_profile(PROFILE_PHYSICAL)
    elif args.set == "operational":
        items = build_items_from_profile(PROFILE_OPERATIONAL)
    else:  # all
        items = build_items_from_profile(PROFILE_PHYSICAL) + build_items_from_profile(PROFILE_OPERATIONAL)

    # Prepare VALSET
    valset = build_valset(items, layers_mask)

    # Pretty print summary
    print("Applying configuration:")
    for k, human_v in (PROFILE_PHYSICAL.items() if args.set == "physical"
                       else PROFILE_OPERATIONAL.items() if args.set == "operational"
                       else {**PROFILE_PHYSICAL, **PROFILE_OPERATIONAL}.items()):
        if k in KEYS:
            spec = KEYS[k]
            v_int = scale_value(k, human_v)
            print(f"  - {k:32s} -> {human_v}  (type {spec['type']}, scaled {v_int})")
    print(f"Layers mask: {args.layers} (0x{layers_mask:02X})")
    print()

    if args.dry_run:
        print("DRY-RUN: UBX-CFG-VALSET frame (hex):")
        print(valset.hex())
        return

    # Open transport
    if args.serial:
        if serial is None:
            print("ERROR: pyserial not installed. pip install pyserial", file=sys.stderr)
            sys.exit(2)
        tr = SerialTransport(args.serial, baud=args.baud)
        print(f"Opened serial: {args.serial} @ {args.baud}")
    else:
        host, port_str = args.tcp.split(':', 1)
        tr = TcpTransport(host, int(port_str))
        print(f"Opened TCP: {host}:{port_str}")

    # Send VALSET
    tr.write(valset)
    time.sleep(args.delay)
    buf = tr.read(timeout_s=0.5)
    ack = find_ack(buf)
    if ack is None:
        print("No explicit ACK received (this can be normal for VALSET).")
    else:
        ok, acls, amid = ack
        print(f"ACK received: {'ACK' if ok else 'NAK'} for class 0x{acls:02X}, id 0x{amid:02X}")

    tr.close()
    print("Done.")


if __name__ == "__main__":
    main()
