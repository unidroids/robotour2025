#!/usr/bin/env python3
# pygame čtení + výpočty + ring buffer + toggling režimu SELECT
import os, time, threading, json, pygame

# --- Konstants ---
POLL_PERIOD_SEC = 0.025          # 25 ms
DEADZONE_WHEELS = 5             # ±5 -> 0 (pro -100..100)
DEADZONE_SPEED  = 5
STEER_RANGE     = 40            # ±40 rozdíl mezi koly (DRIVE)
SELECT_BUTTONS  = {10}          # typické indexy "select/back/share"

# --- Sdílený stav (bez tříd) ---
joystick = None
mode = "DRIVE"                  # default po startu
stop_event = threading.Event()
compute_thread_started = False
lock = threading.Lock()
cond = threading.Condition(lock)   # ⬅ místo Event budeme vysílat notify_all()

axes = {'axis_0': 0.0, 'axis_1': 0.0, 'axis_2': 0.0, 'axis_3': 0.0}
buttons = {'b_0': 0, 'b_1': 0, 'b_2': 0, 'b_3': 0, 'b_4': 0}
last_button = None

left_wheel = 0
right_wheel = 0

last_ts = 0.0
latest_payload = None           # vždy str 
latest_buttons = None           # vždy str 
msg_index = 0                   # čítač zpráv

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

    while pygame.joystick.get_count() == 0 and not stop_event.is_set():
        time.sleep(0.1)
        pygame.joystick.quit()
        pygame.joystick.init()

    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[GAMEPAD] Používám: {joystick.get_name()} (axes={joystick.get_numaxes()})")
        return True
    else:
        joystick = None
        print("[GAMEPAD] Upozornění: Joystick nenalezen, poběží v nulových hodnotách.")
        return False

# --- Čtení os/tlačítek ---
def read_axes_and_buttons():
    global last_button, mode, axes
    if joystick:
        for e in pygame.event.get():  # zpracuj události
            if e.type == pygame.JOYBUTTONDOWN:
                last_button = f"b_{e.button}"
                buttons[last_button] = 1
                print(f"[GAMEPAD] Stisk tlačítka: {last_button}")
                if e.button in SELECT_BUTTONS:
                    mode = "WHEEL" if mode == "DRIVE" else "DRIVE"
                    print(f"[GAMEPAD] Přepnuto na režim: {mode}")
            if e.type == pygame.JOYBUTTONUP:
                last_button = f"b_{e.button}"
                buttons[last_button] = 0

        a0 = joystick.get_axis(0) if joystick.get_numaxes() > 0 else 0.0  # levá horiz
        a1 = joystick.get_axis(1) if joystick.get_numaxes() > 1 else 0.0  # levá vert
        a2 = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0.0
        a3 = joystick.get_axis(3) if joystick.get_numaxes() > 3 else 0.0  # pravá vert
        a4 = joystick.get_axis(4) if joystick.get_numaxes() > 4 else 0.0  # left trottle
        a5 = joystick.get_axis(5) if joystick.get_numaxes() > 5 else 0.0  # right trottle
    else:
        pygame.event.pump()
        a0 = a1 = a2 = a3 = a4 = a5 = 0.0

    axes['axis_0'] = a0
    axes['axis_1'] = a1
    axes['axis_2'] = a2
    axes['axis_3'] = a3
    axes['axis_4'] = a4
    axes['axis_5'] = a5

# --- Výpočty ---
def compute_wheels_from_axes():
    global left_wheel, right_wheel
    ly = axes['axis_5'] #axis_1 # dopředu kladné 
    ry = axes['axis_4'] #axis_3
    l = apply_deadzone_int(scale_to_int(ly, 50)+50, DEADZONE_WHEELS)
    r = apply_deadzone_int(scale_to_int(ry, 50)+50, DEADZONE_WHEELS)
    left_wheel  = clamp(l, 0, 100)
    right_wheel = clamp(r, 0, 100)

def compute_drive_from_axes():
    global left_wheel, right_wheel
    speed = apply_deadzone_int(scale_to_int(axes['axis_4'], 60)+60, DEADZONE_SPEED)  # vert
    steer = scale_to_int(axes['axis_2'], STEER_RANGE)                               # horiz
    boost = 0 # scale_to_int(axes['axis_5'], 125) + 125  # vert
    left_wheel  = clamp(speed + steer + boost, -50, 375)
    right_wheel = clamp(speed - steer + boost, -50, 375)

def build_payload():
    # obj = {
    #     "idx": msg_index,
    #     "mode": mode,
    #     "axes": dict(axes),
    #     "left_wheel": left_wheel,
    #     "right_wheel": right_wheel,
    #     "last_button": last_button,
    #     "ts": time.time()
    # }
    # return json.dumps(obj, ensure_ascii=False)   # ⬅ vrací str (NE bytes)
    return f"PWM {left_wheel} {right_wheel}#I:{msg_index} M:{mode} B:{last_button} T:{time.time()} A:{dict(axes)}"

# --- Vlákno výpočtů (100 ms) ---
def compute_loop():
    global last_ts, latest_payload, msg_index, compute_thread_started, latest_buttons
    print("[GAMEPAD] Vlákno výpočtů START")
    try:
        while not stop_event.is_set():
            read_axes_and_buttons()
            if mode == "WHEEL":
                compute_wheels_from_axes()
            else:
                compute_drive_from_axes()
            last_ts = time.time()
            msg_index += 1
            payload = build_payload()
            payload_buttons = f"{buttons['b_0']} {buttons['b_1']} {buttons['b_2']} {buttons['b_3']} {buttons['b_4']}"
            with cond:
                latest_payload = payload  # str
                latest_buttons = payload_buttons
                cond.notify_all()
            #print("[GAMEPAD] Vlákno výpočtů LOOP", latest_payload)
            time.sleep(POLL_PERIOD_SEC)
    except Exception as e:
            print("[GAMEPAD] Vlákno výpočtů Error:", e)            
    finally:
        compute_thread_started = False
        print("[GAMEPAD] Vlákno výpočtů STOP")

def start_compute_once():
    global compute_thread_started, msg_index
    if compute_thread_started:
        return False
    stop_event.clear()
    msg_index = 0
    compute_thread_started = True
    threading.Thread(target=compute_loop, daemon=True).start()
    return True

def stop_all():
    global latest_payload
    stop_event.set()
    with cond:
        latest_payload = None
        cond.notify_all()
