import pygame
import serial
import time
import struct

# Inicializace Pygame a joysticku
pygame.init()
pygame.joystick.init()

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


# Otevření prvního dostupného joysticku
#joystick = pygame.joystick.Joystick(0)
#joystick.init()

# Nastavení sériové linky (upravte port a rychlost dle potřeby)
#ser = serial.Serial('COM6', 921600, timeout=1)
ser = serial.Serial(
    port='/dev/howerboard',
    baudrate=115200,
    timeout=1,
    dsrdtr=False,   # nepoužívat DTR
    rtscts=False    # nepoužívat RTS/CTS
)
ser.setDTR(False)  # explicitně vypni DTR
ser.setRTS(False)  # explicitně vypni RTS
time.sleep(2)  # Počáteční čekání na stabilizaci linky

while (not pygame.joystick.get_init()):
    pass

print(pygame.joystick.get_count())

joy = pygame.joystick.Joystick(0)
while (not joy.get_init()):
    pass


print(joy.get_instance_id())
print(joy.get_guid())
print(joy.get_power_level())
print(joy.get_name())    
print(joy.get_numaxes())    

ser.write(bytes([13]))

try:
    while True:
        pygame.event.pump()  # Zpracování událostí Pygame

        # Čtení hodnot os (např. 2 analogy)
        axis_0 = joy.get_axis(0)
        axis_1 = joy.get_axis(1)
        axis_2 = joy.get_axis(2)
        axis_3 = joy.get_axis(3)
        # axis_4 = joy.get_axis(4)
        # axis_5 = joy.get_axis(5)
        
        # Vytvoření řetězce s hodnotami
        data_str = f"{axis_0:.2f},{axis_1:.2f},{axis_2:.2f},{axis_3:.2f}"

        left_motor = clamp(int(round((-axis_1*30))),-30,30)+158
        right_motor = clamp(int(round((-axis_3*30))),-30,30)+219
        # data_serial = struct.pack('BB', left_motor, right_motor)

        # Odeslání po sériové lince
        print(data_str, left_motor, right_motor)
        ser.write(bytes([left_motor]))
        ser.write(bytes([right_motor]))
        
        last_resp = ""
        response = ser.read_all().decode('utf-8')
        if (last_resp != response):
            print("\t\t\t" + response.split("\n")[2])
            #print(response)
            last_resp = response

        time.sleep(0.1)  # Krátká pauza test
        
except KeyboardInterrupt:
    pass

finally:
    # Uklid a zavření portu
    ser.write(bytes([27]))
    ser.close()
    pygame.quit()
