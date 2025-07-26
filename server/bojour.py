# requirements: pip install zeroconf psutil
import time, socket, psutil
from zeroconf import Zeroconf, ServiceInfo

SERVICE_TYPE = "_robot-http._tcp.local."
SERVICE_NAME = "RobotServer._robot-http._tcp.local."
SERVICE_PORT = 8080

def first_valid_ip():
    """Vrátí IPv4 adresu použitelného rozhraní usb/ndis/enx."""
    for iface, addrs in psutil.net_if_addrs().items():
        if any(tag in iface for tag in ("usb", "enx", "Ethernet", "NDIS")):
            for addr in addrs:
                if addr.family == socket.AF_INET and not addr.address.startswith("169.254."):
                    return iface, addr.address
    return None, None

def advertise(ip):
    info = ServiceInfo(
        SERVICE_TYPE,
        SERVICE_NAME,
        port=SERVICE_PORT,
        addresses=[socket.inet_aton(ip)],
        properties={},              # volitelně JSON s kapacitami robota
        server="robot.local."       # hostname; Avahi i Zeroconf doplní další IP sama
    )
    zc = Zeroconf()
    zc.register_service(info)
    return zc, info

if __name__ == "__main__":
    zc = info = None
    current_ip = None
    #print(f"echo {psutil.net_if_addrs()}")
    print("Waiting for tethering IP …")
    try:
        while True:            
            iface, ip = first_valid_ip()
            #print(f"Get IP {ip} over {iface}, current IP {current_ip}")
            if ip and ip != current_ip:
                # IP nově přiděleno nebo se změnilo
                if zc:
                    print(f"Service unpublished as {info}")
                    zc.unregister_service(info)
                    zc.close()
                zc, info = advertise(ip)
                current_ip = ip
                print(f"[OK] Service published as http://{ip}:8080 over {iface}")
            time.sleep(2)          # jednoduché pollování, stačí
    except KeyboardInterrupt:
        if zc:
            zc.unregister_service(info)
            zc.close()
