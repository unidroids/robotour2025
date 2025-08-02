# jetson_send.py
import socket
import cv2
from pyzbar import pyzbar

IP_LAPTOP = "192.168.55.100"
PORT = 5000

def gst_pipeline(sensor_id=0, w=1640, h=1232, fps=10):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )

#def stereo_pipeline(w=1640, h=1232, fps=10):
def stereo_pipeline(w=540, h=480, fps=10):
    return (
        "nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! videoconvert ! videoflip method=clockwise ! "
        "queue leaky=downstream max-size-buffers=1 ! "
        "compositor name=comp sink_0::xpos=0 sink_0::ypos=0 "
        f"sink_1::xpos={h} sink_1::ypos=0 ! "
        "nvvidconv ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false "
        "nvarguscamerasrc sensor-id=1 ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! videoconvert ! videoflip method=counterclockwise ! "
        "queue leaky=downstream max-size-buffers=1 ! comp.sink_1"
    ).format(w=w)

cap = cv2.VideoCapture(stereo_pipeline(), cv2.CAP_GSTREAMER)


#cap = cv2.VideoCapture(0)  # nebo pipeline pro Jetson

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_LAPTOP, PORT))
print("Connected to laptop!")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        codes = pyzbar.decode(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if len(codes) > 0:
            print(codes[0])
        # Zmenšit/komprimovat
        _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        data = buf.tobytes()
        # Pošli délku + data (4 bajty délka)
        s.sendall(len(data).to_bytes(4, 'big') + data)
        # pauza (10fps)
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("Interrupted by Ctrl+C")
finally:
    cap.release()          # přepne celou pipeline do NULL
    cv2.destroyAllWindows()
    s.close()
