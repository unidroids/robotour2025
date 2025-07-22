#!/usr/bin/env python3
"""capture_lidar_packets.py – simple recorder for Unitree L2 UDP packets.

Usage (default 3s on standard port):
    python3 capture_lidar_packets.py

Specify custom port/duration/output:
    python3 capture_lidar_packets.py -p 6201 -d 5 -o /tmp/lidar_raw.bin

Output format per record (little‑endian):
    double   timestamp_sec   # Unix time (epoch)
    uint32   payload_len     # length of following UDP payload in bytes
    bytes    payload

The file can later be parsed with struct.unpack or converted to PCAP.
"""

import argparse
import datetime as _dt
import os
import socket
import struct
import time
from pathlib import Path

DEFAULT_PORT = 6201  # Jetson receives on this port by default
DEFAULT_DURATION = 3.0  # seconds
DEFAULT_DIR = Path("/robot/data/logs/lidar")  # agreed log root


def _default_outfile() -> Path:
    ts = _dt.datetime.now().strftime("capture_%Y%m%d_%H%M%S.bin")
    return DEFAULT_DIR / ts


def capture(port: int, duration: float, outfile: Path) -> None:
    """Listen on 0.0.0.0:<port> and write all packets for <duration> seconds."""

    # Ensure parent directory exists
    outfile.parent.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))  # listen on all interfaces
    sock.setblocking(False)

    end_time = time.time() + duration
    packets = 0

    with outfile.open("wb") as f:
        while time.time() < end_time:
            try:
                data, _addr = sock.recvfrom(65535)  # maximum UDP payload
            except BlockingIOError:
                # no data ready – yield a little to save CPU
                time.sleep(0.001)
                continue

            pkt_ts = time.time()
            f.write(struct.pack("<dI", pkt_ts, len(data)))
            f.write(data)
            packets += 1

    print(f"Captured {packets} packets → {outfile}")


def main():
    ap = argparse.ArgumentParser(description="Record Unitree L2 UDP packets to a binary log.")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_PORT, help="UDP port to listen on (default: 6201)")
    ap.add_argument("-d", "--duration", type=float, default=DEFAULT_DURATION, help="Recording length in seconds (default: 3.0)")
    ap.add_argument("-o", "--outfile", type=Path, default=_default_outfile(), help="Output file path (default: /robot/data/logs/lidar/capture_YYYYMMDD_HHMMSS.bin)")
    args = ap.parse_args()

    try:
        capture(args.port, args.duration, args.outfile)
    except KeyboardInterrupt:
        print("Interrupted by user – exiting …")


if __name__ == "__main__":
    main()
