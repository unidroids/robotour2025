"""service.py – vysoká vrstva služby DRIVE (port 9003)

Zajišťuje:
  - inicializaci UARTu a dispatcheru
  - enkodování a odesílání příkazových rámců
  - metody pro doménové příkazy (HALT/BREAK/POWER_OFF/DRIVE/PWM)
  - START/STOP služby dle specifikace (START => otevřít UART + poslat cmd=2, STOP => cmd=1 + zavřít UART)
  - get_state() pro diagnostiku (JSON‑ready dict)

Poznámka k názvosloví:
  - cmd=0   HALT (rychlé zastavení)
  - cmd=1   STOP motorů
  - cmd=2   START motorů
  - cmd=3   POWER_OFF (vypnutí napájení hoverboardu)
  - cmd=4   DRIVE (max_pwm do p1/p2, left_speed do p3, right_speed do p4)
  - cmd=5   BREAK (zastavení)
  - cmd=101 PWM (left_pwm do p1/p2, right_pwm do p3/p4)

Závislosti: serial.HoverboardSerial, dispatcher.MessageDispatcher, handlers.ack/nack
"""
from __future__ import annotations

import time
import threading
from dataclasses import dataclass, asdict
from typing import Any, Dict, Optional, Tuple

# Naše moduly
from hb_serial import HoverboardSerial, SerialConfig  # UART RX/TX + FIFO (náš serial.py)
from dispatcher import MessageDispatcher, DispatcherConfig

# Handlery (zatím jen print)
from handlers import ack as handler_ack
from handlers import nack as handler_nack

__all__ = [
    "DriveServiceConfig",
    "DriveService",
    #"STX", "MTX", "ETX",
    #"build_frame", "encode_speed", "encode_pwm",
]


# --- rámcové konstanty ---
STX = 251
MTX = 252
ETX = 253


def _assert_param(v: int) -> None:
    iv = int(v)
    if iv < 0 or iv > 250:
        raise ValueError(f"Param {iv} out of range 0..250")


def build_frame(cmd: int, p1: int, p2: int, p3: int, p4: int) -> bytes:
    """Sestaví binární rámec dle specifikace."""
    for v in (cmd, p1, p2, p3, p4):
        _assert_param(v)
    return bytes([STX, cmd, p1, p2, p3, p4, MTX, cmd, p1, p2, p3, p4, ETX])


def encode_speed(v: int) -> int:
    """Mapuje rychlost do p3/p4: v ∈ [-50, 200]  =>  p = v + 50 ∈ [0..250]."""
    v = int(v)
    p = v + 50
    _assert_param(p)
    return p


def encode_pwm(d: int) -> Tuple[int, int]:
    """Kusová mapovací funkce PWM -> (p1,p2) v rozsahu 0..250.

    d ∈ [-125..375]
      d <= 0:     p1=0,   p2=d+125
      0 < d<=250: p1=d,   p2=125
      d > 250:    p1=250, p2=d-125
    """
    d = int(d)
    if d < -125 or d > 375:
        raise ValueError(f"PWM {d} out of range [-125, 375]")
    if d <= 0:
        p1, p2 = 0, d + 125
    elif d <= 250:
        p1, p2 = d, 125
    else:
        p1, p2 = 250, d - 125
    _assert_param(p1); _assert_param(p2)
    return p1, p2


# --- konfigurace a stav ---
@dataclass(slots=True)
class DriveServiceConfig:
    serial: SerialConfig = SerialConfig()
    dispatcher: DispatcherConfig = DispatcherConfig()


class DriveService:
    """Vysoká vrstva pro ovládání hoverboardu a směrování zpráv."""

    def __init__(self, cfg: Optional[DriveServiceConfig] = None):
        self.cfg = cfg or DriveServiceConfig()
        self._ser = HoverboardSerial(self.cfg.serial)
        self._disp = MessageDispatcher(self._ser, self.cfg.dispatcher)

        # registry handlerů (prozatím základní)
        self._disp.register_handler('IAM', handler_ack.handle)
        self._disp.register_handler('INM', handler_nack.handle)
        # volitelně můžeme nastavit default handler, který jen tiskne vše ostatní
        self._disp.set_default_handler(lambda m: print(f"[MSG] {m.code} {m.values}"))

        self._lock = threading.Lock()
        self._ack_nack_event = threading.Event()
        self._running = False
        self._started_at = 0.0
        self._last_cmd_at = 0.0
        self._tx_ok = 0
        self._tx_fail = 0
        self._last_ack_nack_data: Optional[Tuple[int, int, int, int, int]] = None
        self._last_ack_nack_error: Optional[Tuple[int, int]] = None

    # --------------- lifecycle ---------------
    def start(self) -> str:
        """Spustí službu: otevře UART + dispatcher a *pošle* cmd=2 (START motorů)."""
        with self._lock:
            if self._running:
                # idempotentní start – přesto pošleme znovu START motorů
                # self._send_cmd(2, 125, 125, 125, 125)  # neutrální p1..p4 (nepovinné)
                return "RUNNING"
            self._ser.start()
            self._disp.start()
            self._running = True
            self._started_at = time.monotonic()
        # dle specifikace po startu UI/servisy poslat START motorů (cmd=2)
        self.motors_start()
        return "RUNNING"

    def stop(self) -> str:
        """Zastaví službu: pošle cmd=1 (STOP motorů) a zavře UART/dispatcher."""
        # nejprve zkus poslat STOP motorů
        try:
            self.motors_stop()
        except Exception:
            pass
        with self._lock:
            if not self._running:
                return "STOPPED"
            self._disp.stop()
            self._ser.stop()
            self._running = False
        return "STOPPED"

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    # --------------- API – jednoduché příkazy ---------------
    def ping(self) -> str:
        return "PONG DRIVE"

    def motors_start(self) -> bool:
        return self._send_cmd(2, 125, 125, 125, 125)

    def motors_stop(self) -> bool:
        return self._send_cmd(1, 125, 125, 125, 125)

    def power_off(self) -> bool:
        return self._send_cmd(3, 125, 125, 125, 125)

    def halt(self) -> bool:
        return self._send_cmd(0, 125, 125, 125, 125)

    def brake(self) -> bool:  # „BREAK“ v textu, ale pojmenováno jako „brake“ kvůli klíčovému slovu
        return self._send_cmd(5, 125, 125, 125, 125)

    def drive(self, max_pwm: int, left_speed: int, right_speed: int) -> bool:
        p1, p2 = encode_pwm(max_pwm)
        p3 = encode_speed(left_speed)
        p4 = encode_speed(right_speed)
        return self._send_cmd(4, p1, p2, p3, p4)

    def pwm(self, left_pwm: int, right_pwm: int) -> bool:
        p1, p2 = encode_pwm(left_pwm)
        p3, p4 = encode_pwm(right_pwm)
        return self._send_cmd(101, p1, p2, p3, p4)



    # --------------- low‑level TX ---------------
    def received_ack_nack(self, cmd: int, p1: int, p2: int, p3: int, p4: int, ie:int, ce:int):
        """Voláno handlery ACK/NACK při přijetí potvrzení.
        """
        # keep last ack/nack data
        with self._lock:
            self._last_ack_nack_data = (cmd, p1, p2, p3, p4)
            self._last_ack_nack_error = (ie, ce)
        # zde můžeme přidat kontrolu shody s posledním odeslaným příkazem
        self._ack_nack_event.set()
        return 

    def _send_cmd(self, cmd: int, p1: int, p2: int, p3: int, p4: int) -> bool:
        # build  frame
        frame = build_frame(cmd, p1, p2, p3, p4)
        # remember
        frame_data = (cmd, p1, p2, p3, p4)
        with self._lock:
            self._last_ack_nack_error = None
            self._last_ack_nack_data = None
            self._ack_nack_event.clear()
        # send   frame
        ok = self._ser.send_frame(frame)
        # update stats
        with self._lock:
            self._last_cmd_at = time.monotonic()
            if ok:
                self._tx_ok += 1
            else:
                self._tx_fail += 1
                raise RuntimeError("Failed to send frame")
        # wait for ack/nack
        if not self._ack_nack_event.wait(timeout=0.005):
            raise TimeoutError("No ACK/NACK received within timeout")

        # check data
        if self._last_ack_nack_error is None:
            raise RuntimeError("ACK/NACK error data missing")

        # check ACK/NACK errors
        ie, ce = self._last_ack_nack_error
        if ce != 0:
            raise RuntimeError(f"Command error (CE={ce})")
        if ie != 0:
            # error in transport - resend frame 
            with self._lock:
                self._last_ack_nack_error = None
                self._last_ack_nack_data = None
                self._ack_nack_event.clear()
            # send frame again
            ok = self._ser.send_frame(frame)
            # update stats
            with self._lock:
                self._last_cmd_at = time.monotonic()
                if ok:
                    self._tx_ok += 1
                else:
                    self._tx_fail += 1
                    raise RuntimeError("Failed to re-send frame")

            # wait for ack/nack after resend
            if not self._ack_nack_event.wait(timeout=0.005):
                raise TimeoutError("No ACK/NACK received within timeout")
            # check error data from resend
            if self._last_ack_nack_error is None:
                raise RuntimeError("ACK/NACK error data missing")

            # check ACK/NACK errors
            ie, ce = self._last_ack_nack_error
            if ce != 0:
                raise RuntimeError(f"Command error (CE={ce})")
            if ie != 0:
                raise RuntimeError(f"Transport error after resend (CE={ie})")


        # double check send data
        if self._last_ack_nack_data is None:
            raise RuntimeError("ACK/NACK data missing")
        if self._last_ack_nack_data != frame_data:
            raise RuntimeError(f"ACK/NACK data mismatch: sent {frame_data}, received {self._last_ack_nack_data}")
        return ok





    # --------------- stav/diagnostika ---------------
    def get_state(self) -> Dict[str, Any]:
        rx_bytes, tx_bytes, rx_msgs, tx_frames, rx_over, tx_over, bad, too_long, unknown = self._ser.stats()
        disp_stats = self._disp.stats()
        with self._lock:
            running = self._running
            started_at = self._started_at
            last_cmd_at = self._last_cmd_at
            tx_ok = self._tx_ok
            tx_fail = self._tx_fail
        return {
            "service": "DRIVE",
            "status": "RUNNING" if running else "STOPPED",
            "started_at_mono": started_at,
            "last_cmd_at_mono": last_cmd_at,
            "tx_ok": tx_ok,
            "tx_fail": tx_fail,
            "serial": {
                "device": self.cfg.serial.device,
                "baud": self.cfg.serial.baudrate,
                "rx_bytes": rx_bytes,
                "tx_bytes": tx_bytes,
                "rx_msgs": rx_msgs,
                "tx_frames": tx_frames,
                "rx_overflows": rx_over,
                "tx_overflows": tx_over,
                "parser": {
                    "bad_lines": bad,
                    "too_long_lines": too_long,
                    "unknown_codes": unknown,
                },
            },
            "dispatcher": {
                "processed": disp_stats.processed,
                "unhandled": disp_stats.unhandled,
                "handler_errors": disp_stats.handler_errors,
                "per_code": dict(disp_stats.per_code),
                "last_error": disp_stats.last_error,
                "started_at_mono": disp_stats.started_at,
            },
        }



# --- jednoduchý self‑test ---
if __name__ == "__main__":
    svc = DriveService()
    print("PING:", svc.ping())
    print("START:", svc.start())
    print("STATE:", svc.get_state())
    print("HALT:", svc.halt())
    print("DRIVE:", svc.drive(120, 0, 0))
    print("PWM:", svc.pwm(100, 100))
    print("STOP:", svc.stop())
    print("STATE:", svc.get_state())
