#!/usr/bin/env python3
# pygame čtení + výpočty + ring buffer + toggling režimu SELECT
import os, time, threading, json, pygame
from collections import deque

# --- Konstants ---
POLL_PERIOD_SEC = 0.10          # 100 ms
DEADZONE_WHEELS = 5             # ±5 -> 0 (pro -100..100)
DEADZONE_SPEED  = 5
STEER_RANGE     = 20            # ±20 rozdíl mezi koly (DRIVE)
SELECT_BUTTONS  = {6, 8}        # typické indexy "select/back/share"

# --- Sdílený stav (bez tříd) ---
joystick = None
mode = "DRIVE"                  # default po startu
stop_event = threading.Event()
compute_thread_started = False

axes = {'axis_0': 0.0, 'axis_1': 0.0, 'axis_2': 0.0, 'axis_3': 0.0}
last_button = None

wheels_left = 0
wheels_right = 0
drive_left = 0
drive_right = 0

last_ts = 0.0
payload_deque = deque(maxlen=3)
latest_payload = None

# signalizace pro DataLoger
data_ready_event = threading.Event()

# zámek na sdílený stav
lock = threading.Lock()

# --- Pomocné funkce ---
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def apply_deadzone_int(v, dz):
    return 0 if -dz <= v <= dz else v

def scale_to_int(x, out_max):
    return int(round(x * out_max))  # x∈[-1,1] -> [-out_max, out_max]

# --- Inicializace pygame gamepadu ---
def init_gamepad():
    global joystick
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # headless
    pygame.init()
    pygame.joystick.init()

    start = time.time()
    while pygame.joystick.get_count() == 0 and (time.time() - start) < 3.0:
        time.sleep(0.1)
        pygame.joystick.quit(); pygame.joystick.init()

    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[GAMEPAD] Používám: {joystick.get_name()} (axes={joystick.get_numaxes()})")
    else:
        joystick = None
        print("[GAMEPAD] Upozornění: Joystick nenalezen, poběží v nulových hodnotách.")

# --- Čtení os/tlačítek ---
def read_axes_and_buttons():
    global last_button, mode
    if joystick:
        for e in pygame.event.get():  # zpracuj události
            if e.type == pygame.JOYBUTTONDOWN:
                last_button = f"BTN_{e.button}"
                print(f"[GAMEPAD] Stisk tlačítka: {last_button}")
                if e.button in SELECT_BUTTONS:
                    mode = "WHEEL" if mode == "DRIVE" else "DRIVE"
                    print(f"[GAMEPAD] Přepnuto na režim: {mode}")

        a0 = joystick.get_axis(0) if joystick.get_numaxes() > 0 else 0.0  # levá horiz
        a1 = joystick.get_axis(1) if joystick.get_numaxes() > 1 else 0.0  # levá vert
        a2 = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0.0
        a3 = joystick.get_axis(3) if joystick.get_numaxes() > 3 else 0.0  # pravá vert
    else:
        pygame.event.pump()
        a0 = a1 = a2 = a3 = 0.0

    with lock:
        axes['axis_0'] = a0
        axes['axis_1'] = a1
        axes['axis_2'] = a2
        axes['axis_3'] = a3

# --- Výpočty ---
def compute_wheels_from_axes():
    global wheels_left, wheels_right
    ly = -axes['axis_1']  # dopředu kladné
    ry = -axes['axis_3']
    l = apply_deadzone_int(scale_to_int(ly, 100), DEADZONE_WHEELS)
    r = apply_deadzone_int(scale_to_int(ry, 100), DEADZONE_WHEELS)
    wheels_left  = clamp(l, -100, 100)
    wheels_right = clamp(r, -100, 100)

def compute_drive_from_axes():
    global drive_left, drive_right
    speed = apply_deadzone_int(scale_to_int(-axes['axis_1'], 100), DEADZONE_SPEED)  # levá vert
    steer = scale_to_int(axes['axis_0'], STEER_RANGE)                               # levá horiz
    left  = clamp(speed + steer, -100, 100)
    right = clamp(speed - steer, -100, 100)
    drive_left, drive_right = left, right

def build_payload():
    return {
        "mode": mode,
        "axes": dict(axes),
        "wheels": {"left": wheels_left, "right": wheels_right},
        "drive":  {"left": drive_left,  "right": drive_right},
        "last_button": last_button,
        "ts": time.time()
    }

# --- Vlákno výpočtů (100 ms) ---
def compute_loop():
    global last_ts, latest_payload
    print("[GAMEPAD] Vlákno výpočtů START")
    try:
        while not stop_event.is_set():
            read_axes_and_buttons()
            with lock:
                if mode == "WHEEL":
                    compute_wheels_from_axes()
                else:
                    compute_drive_from_axes()
                last_ts = time.time()
                latest_payload = build_payload()
                payload_deque.append(latest_payload)
            data_ready_event.set()  # probuď DataLoger
            time.sleep(POLL_PERIOD_SEC)
    except Exception as e:
        print(f"[GAMEPAD] Chyba ve vlákně výpočtů: {e}")
    finally:
        print("[GAMEPAD] Vlákno výpočtů STOP")

def start_compute_once():
    global compute_thread_started
    if compute_thread_started:
        return False
    compute_thread_started = True
    t = threading.Thread(target=compute_loop, daemon=True)
    t.start()
    return True

# --- API pro ostatní moduly ---
def get_latest_payload():
    with lock:
        return latest_payload if latest_payload is not None else build_payload()

def stop_all():
    stop_event.set()
