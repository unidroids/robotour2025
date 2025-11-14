import pygame
import time
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # headless

pygame.init()
pygame.joystick.init()


while pygame.joystick.get_count() == 0:
    time.sleep(0.1)
    pygame.joystick.quit()
    pygame.joystick.init()

if pygame.joystick.get_count() > 0:
    print("count:",pygame.joystick.get_count())
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[GAMEPAD] Používám: {joystick.get_name()} (axes={joystick.get_numaxes()})")
else:
    joystick = None
    print("[GAMEPAD] Upozornění: Joystick nenalezen, poběží v nulových hodnotách.")
    exit()

js = pygame.joystick.Joystick(0)
js.init()

print("Joystick:", js.get_name())
print("Počet os:", js.get_numaxes())
print("Počet tlačítek:", js.get_numbuttons())
print("Počet hat switchů:", js.get_numhats())

print("Čekám na vstupy... (Ctrl+C pro ukončení)")

cnt=0
while True:
    pygame.event.pump()
    cnt+=1
    axes = [js.get_axis(i) for i in range(js.get_numaxes())]
    buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
    hats = [js.get_hat(i) for i in range(js.get_numhats())]
    
    print("#",cnt ,"axes:", axes, "buttons:", buttons, "hats:", hats)
    time.sleep(0.3)
