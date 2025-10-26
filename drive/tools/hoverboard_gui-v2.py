#!/usr/bin/env python3
# hoverboard_gui.py
# Desktop GUI for hoverboard communication (Windows-friendly, also works on Linux/Mac)
# - Left pane: RX plain-text lines (entries separated by CRLF)
# - Right pane: controls + log (outgoing messages and errors)
# - TX encodes fixed-length frames: STX,CMD,P1,P2,P3,P4,MTX,CMD,P1,P2,P3,P4,ETX
# - Two threads: RX reader (byte-by-byte -> CRLF lines), TX sender (from FIFO queue)
#
# Requires: pyserial
#   pip install pyserial
#
# Usage:
#   py hoverboard_gui.py
#
import threading
import queue
import time
import sys
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

try:
    import serial
except ImportError:
    serial = None

STX = 251
MTX = 252
ETX = 253

# ---------------------- Encoding helpers ----------------------
def encode_pwm(d):
    """Map desired PWM d (-125..375) to (p1,p2) params 0..250 via piecewise rule:
       d <= 0:    p1=0,   p2=d+125
       0 < d<=250:p1=d,   p2=125
       d > 250:   p1=250, p2=d-125
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

def encode_speed(v):
    """max speed: v ∈ [-50, 200]  ->  p3 = v + 50 (0..250)"""
    v = int(v)
    p3 = v + 50
    _assert_param(p3)
    return p3

def encode_omega(w):
    """max angular velocity: w ∈ [-125, 125]  ->  p4 = w + 125 (0..250)"""
    w = int(w)
    p4 = w + 125
    _assert_param(p4)
    return p4

def encode_corr(c):
    """tune correction: c ∈ [-125, 125] -> param = c + 125"""
    c = int(c)
    p = c + 125
    _assert_param(p)
    return p

def build_frame(cmd, p1, p2, p3, p4):
    #for v in (cmd, p1, p2, p3, p4):
    #    _assert_param(v)
    return bytes([STX, cmd, p1, p2, p3, p4, MTX, cmd, p1, p2, p3, p4, ETX])

def _assert_param(v):
    iv = int(v)
    if iv < 0 or iv > 250:
        raise ValueError(f"Param {iv} out of range 0..250")

# ---------------------- GUI App ----------------------
class HoverboardGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Hoverboard UART Tool")
        self.geometry("1100x700")
        self.minsize(980, 600)

        # Shared state
        self.ser = None
        self.stop_event = threading.Event()
        self.tx_queue = queue.Queue(maxsize=200)
        self.rx_thread = None
        self.tx_thread = None

        # --- Layout: two columns ---
        self.grid_columnconfigure(0, weight=1)  # left pane grows
        self.grid_columnconfigure(1, weight=1)  # right pane grows
        self.grid_rowconfigure(0, weight=1)     # main row grows

        # Left: RX
        left_frame = ttk.Frame(self)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(8,4), pady=8)
        left_frame.grid_rowconfigure(1, weight=1)
        left_frame.grid_columnconfigure(0, weight=1)

        ttk.Label(left_frame, text="RX (plain text; lines separated by CRLF)").grid(row=0, column=0, sticky="w")
        self.rx_text = ScrolledText(left_frame, wrap="none", height=10)
        self.rx_text.grid(row=1, column=0, sticky="nsew", pady=(4,8))

        # Right: Controls + Log
        right_frame = ttk.Frame(self)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=(4,8), pady=8)
        right_frame.grid_columnconfigure(0, weight=1)
        right_frame.grid_rowconfigure(4, weight=1)

        # Port controls
        port_frame = ttk.Frame(right_frame)
        port_frame.grid(row=0, column=0, sticky="ew", pady=(0,6))
        port_frame.grid_columnconfigure(3, weight=1)

        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="COM4")
        self.port_entry = ttk.Entry(port_frame, textvariable=self.port_var, width=12)
        self.port_entry.grid(row=0, column=1, padx=(6,10))

        self.start_btn = ttk.Button(port_frame, text="Start", command=self.on_start)
        self.start_btn.grid(row=0, column=2, padx=(0,6))
        self.stop_btn = ttk.Button(port_frame, text="Stop", command=self.on_stop, state="disabled")
        self.stop_btn.grid(row=0, column=3, sticky="w")

        # Command panel
        cmds = ttk.LabelFrame(right_frame, text="Commands")
        cmds.grid(row=1, column=0, sticky="ew", pady=(0,6))
        for i in range(8):
            cmds.grid_columnconfigure(i, weight=0)
        cmds.grid_columnconfigure(7, weight=1)

        # echo (cmd=0) + p1..p4
        self.echo_p = [tk.StringVar(value="0") for _ in range(4)]
        ttk.Label(cmds, text="echo (cmd=0)").grid(row=0, column=0, sticky="w")
        for i in range(4):
            ttk.Entry(cmds, textvariable=self.echo_p[i], width=5).grid(row=0, column=1+i, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_echo).grid(row=0, column=5, padx=4)

        # stop (cmd=1)
        ttk.Label(cmds, text="stop (cmd=1)").grid(row=1, column=0, sticky="w")
        ttk.Button(cmds, text="Send", command=self.send_stop).grid(row=1, column=5, padx=4)

        # start (cmd=2)
        ttk.Label(cmds, text="start (cmd=2)").grid(row=2, column=0, sticky="w")
        ttk.Button(cmds, text="Send", command=self.send_start).grid(row=2, column=5, padx=4)

        # off (cmd=3) - timeout in p1
        self.off_timeout = tk.StringVar(value="0")
        ttk.Label(cmds, text="off (cmd=3) timeout:").grid(row=3, column=0, sticky="w")
        ttk.Entry(cmds, textvariable=self.off_timeout, width=6).grid(row=3, column=1, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_off).grid(row=3, column=5, padx=4)

        # drive (cmd=4) - PWM (-125..375), MAX SPEED (-50..200), MAX OMEGA (-125..125)
        self.drive_pwm = tk.StringVar(value="0")
        self.drive_left_speed = tk.StringVar(value="0")
        self.drive_right_speed = tk.StringVar(value="0")
        ttk.Label(cmds, text="drive (cmd=4)").grid(row=4, column=0, sticky="w")
        ttk.Label(cmds, text="PWM").grid(row=4, column=1, sticky="e")
        ttk.Entry(cmds, textvariable=self.drive_pwm, width=6).grid(row=4, column=2, padx=2)
        ttk.Label(cmds, text="L Speed").grid(row=4, column=3, sticky="e")
        ttk.Entry(cmds, textvariable=self.drive_left_speed, width=6).grid(row=4, column=4, padx=2)
        ttk.Label(cmds, text="R Speed").grid(row=4, column=5, sticky="e")
        ttk.Entry(cmds, textvariable=self.drive_right_speed, width=6).grid(row=4, column=6, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_drive).grid(row=4, column=7, padx=4, sticky="w")

        # pwm (cmd=101) - left/right PWM (encoded into p1,p2 and p3,p4)
        self.pwm_left = tk.StringVar(value="0")
        self.pwm_right = tk.StringVar(value="0")
        ttk.Label(cmds, text="pwm (cmd=101)").grid(row=5, column=0, sticky="w")
        ttk.Label(cmds, text="LeftPWM").grid(row=5, column=1, sticky="e")
        ttk.Entry(cmds, textvariable=self.pwm_left, width=6).grid(row=5, column=2, padx=2)
        ttk.Label(cmds, text="RightPWM").grid(row=5, column=3, sticky="e")
        ttk.Entry(cmds, textvariable=self.pwm_right, width=6).grid(row=5, column=4, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_pwm).grid(row=5, column=7, padx=4, sticky="w")

        # tune (cmd=102) - 4x correction (-125..125)
        self.tune_vals = [tk.StringVar(value="0") for _ in range(4)]
        ttk.Label(cmds, text="tune (cmd=102) L.cw L.ccw R.cw R.ccw").grid(row=6, column=0, sticky="w")
        for i in range(4):
            ttk.Entry(cmds, textvariable=self.tune_vals[i], width=6).grid(row=6, column=1+i, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_tune).grid(row=6, column=7, padx=4, sticky="w")

        # raw - direct values
        self.raw_vals = [tk.StringVar(value="0") for _ in range(5)]  # cmd, p1..p4
        ttk.Label(cmds, text="raw (cmd p1 p2 p3 p4)").grid(row=7, column=0, sticky="w")
        for i in range(5):
            ttk.Entry(cmds, textvariable=self.raw_vals[i], width=6).grid(row=7, column=1+i, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_raw).grid(row=7, column=7, padx=4, sticky="w")

        # Log pane (right-bottom)
        ttk.Label(right_frame, text="Log (actions, errors, outgoing text before encoding)").grid(row=3, column=0, sticky="w", pady=(6,0))
        self.log_text = ScrolledText(right_frame, wrap="word", height=10)
        self.log_text.grid(row=4, column=0, sticky="nsew", pady=(4,0))

        # Footer status
        self.status_var = tk.StringVar(value="Stopped")
        status_bar = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")
        status_bar.grid(row=1, column=0, columnspan=2, sticky="ew")

        # Close handling
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Update UI loop
        self.after(500, self._tick_status)

    # ---------------------- UI logging helpers ----------------------
    def log_right(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        def _append():
            self.log_text.insert("end", f"{ts} | {msg}\n")
            self.log_text.see("end")
        self.log_text.after(0, _append)

    def log_left(self, msg):
        def _append():
            self.rx_text.insert("end", msg + "\n")
            self.rx_text.see("end")
        self.rx_text.after(0, _append)

    def set_status(self, text):
        self.status_var.set(text)

    # ---------------------- Port control ----------------------
    def on_start(self):
        if serial is None:
            self.log_right("pyserial not installed. Run: pip install pyserial")
            return
        if self.ser is not None:
            self.log_right("Already running.")
            return
        port = self.port_var.get().strip()
        if not port:
            self.log_right("Please enter serial port (e.g., COM5)")
            return
        try:
            #self.ser = serial.Serial(port, 115200, timeout=0.05)
            self.ser = serial.Serial(port, 921600, timeout=0.05)
        except Exception as e:
            self.log_right(f"[ERR] Opening port '{port}': {e}")
            self.ser = None
            return
        self.log_right(f"[OK] Opened port {port} @115200")
        self.stop_event.clear()

        # Spawn RX thread
        self.rx_thread = threading.Thread(target=self._rx_loop, name="RX", daemon=True)
        self.rx_thread.start()

        # Spawn TX thread
        self.tx_thread = threading.Thread(target=self._tx_loop, name="TX", daemon=True)
        self.tx_thread.start()

        self.start_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.set_status(f"Running on {port}")

    def on_stop(self):
        self.stop_event.set()
        time.sleep(0.1)
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.start_btn.configure(state="normal")
        self.stop_btn.configure(state="disabled")
        self.set_status("Stopped")
        self.log_right("[OK] Port closed, threads stopping...")

    def on_close(self):
        self.on_stop()
        time.sleep(0.2)
        self.destroy()

    # ---------------------- RX: read bytes, emit lines on CRLF ----------------------
    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_event.is_set():
            try:
                b = self.ser.read(1) if self.ser else b""
            except Exception as e:
                self.log_right(f"[RX ERR] {e}")
                time.sleep(0.1)
                continue
            if not b:
                continue
            ch = b[0]
            if ch == 0x0D:  # CR
                # wait for LF
                continue
            if ch == 0x0A:  # LF
                line = buf.decode("utf-8", errors="replace")
                self.log_left(line)
                buf.clear()
                continue
            buf.append(ch)
            if len(buf) > 4096:
                line = buf.decode("utf-8", errors="replace")
                self.log_left(line)
                buf.clear()

    # ---------------------- TX: take frames from queue and write ----------------------
    def _tx_loop(self):
        while not self.stop_event.is_set():
            try:
                item = self.tx_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                continue
            try:
                frame_desc, frame_bytes = item
                if self.ser is None:
                    self.log_right("[TX ERR] Port not open. Discarding frame.")
                    continue
                self.ser.write(frame_bytes)
                self.log_right(f"[TX OK] {frame_desc} | hex: {' '.join(f'{b:02X}' for b in frame_bytes)}")
            except Exception as e:
                self.log_right(f"[TX ERR] {e}")

    # ---------------------- Command handlers (enqueue) ----------------------
    def send_echo(self):
        try:
            p = [int(v.get() or "0") for v in self.echo_p]
            cmd = 0
            desc = f"echo cmd={cmd} p1={p[0]} p2={p[1]} p3={p[2]} p4={p[3]}"
            self.log_right(desc)  # before encoding
            frame = build_frame(cmd, p[0], p[1], p[2], p[3])
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[echo ERR] {e}")

    def send_stop(self):
        try:
            cmd=1; p1=p2=p3=p4=0
            desc = "stop cmd=1 p1=0 p2=0 p3=0 p4=0"
            self.log_right(desc)
            frame = build_frame(cmd,p1,p2,p3,p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[stop ERR] {e}")

    def send_start(self):
        try:
            cmd=2; p1=p2=p3=p4=0
            desc = "start cmd=2 p1=0 p2=0 p3=0 p4=0"
            self.log_right(desc)
            frame = build_frame(cmd,p1,p2,p3,p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[start ERR] {e}")

    def send_off(self):
        try:
            t = int(self.off_timeout.get() or "0")
            cmd=3; p1=t; p2=p3=p4=0
            desc = f"off cmd=3 timeout={t} -> p1={p1}, p2=0 p3=0 p4=0"
            self.log_right(desc)
            frame = build_frame(cmd,p1,p2,p3,p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[off ERR] {e}")

    def send_drive(self):
        try:
            pwm = int(self.drive_pwm.get() or "0")
            lspd = int(self.drive_left_speed.get() or "0")
            rspd = int(self.drive_right_speed.get() or "0")
            p1,p2 = encode_pwm(pwm)
            p3 = encode_speed(lspd)
            p4 = encode_speed(rspd)
            cmd=4
            desc = f"drive cmd=4 pwm={pwm} -> (p1={p1},p2={p2}), L speed={lspd} -> p3={p3}, R speed={rspd} -> p4={p4}"
            self.log_right(desc)
            frame = build_frame(cmd,p1,p2,p3,p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[drive ERR] {e}")

    def send_pwm(self):
        try:
            lp = int(self.pwm_left.get() or "0")
            rp = int(self.pwm_right.get() or "0")
            lp1,lp2 = encode_pwm(lp)
            rp1,rp2 = encode_pwm(rp)
            cmd=101
            desc = f"pwm cmd=101 left={lp} -> (p1={lp1},p2={lp2}) right={rp} -> (p3={rp1},p4={rp2})"
            self.log_right(desc)
            frame = build_frame(cmd, lp1, lp2, rp1, rp2)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[pwm ERR] {e}")

    def send_tune(self):
        try:
            vals = [int(v.get() or "0") for v in self.tune_vals]  # L.cw, L.ccw, R.cw, R.ccw
            p = [encode_corr(vals[0]), encode_corr(vals[1]), encode_corr(vals[2]), encode_corr(vals[3])]
            cmd=102
            desc = f"tune cmd=102 Lcw={vals[0]} Lccw={vals[1]} Rcw={vals[2]} Rccw={vals[3]} -> (p1..p4)={p}"
            self.log_right(desc)
            frame = build_frame(cmd, p[0], p[1], p[2], p[3])
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[tune ERR] {e}")

    def send_raw(self):
        try:
            cv = [int(v.get() or "0") for v in self.raw_vals]  # cmd, p1..p4
            cmd,p1,p2,p3,p4 = cv
            desc = f"raw cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4}"
            self.log_right(desc)
            frame = build_frame(cmd,p1,p2,p3,p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[raw ERR] {e}")

    # ---------------------- Status ticker ----------------------
    def _tick_status(self):
        qsz = self.tx_queue.qsize()
        ser_open = self.ser is not None
        self.set_status(f"{'Running' if ser_open else 'Stopped'} | TX queue: {qsz}")
        self.after(500, self._tick_status)

if __name__ == "__main__":
    app = HoverboardGUI()
    app.mainloop()
