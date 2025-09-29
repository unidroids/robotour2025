import socket
from data.nav_fusion_data import NavFusionData

HOST = "127.0.0.1"
PORT = 9006

def main():
    # Hlavička CSV
    print("ts_mono,hAcc,speed,gSpeed,sAcc,heading,vehHeading,motHeading,headingAcc,lastGyroZ,gyroZ,gyroZAcc,gnssFixOK,drUsed")
    with socket.create_connection((HOST, PORT)) as sock:
        f = sock.makefile("rwb", buffering=0)
        f.write(b"GET_BINARY_STREAM\n")
        f.flush()
        while True:
            data = f.read(NavFusionData.byte_size())
            if len(data) != NavFusionData.byte_size():
                print(f"Chyba: načteno {len(data)} bajtů, očekáváno {NavFusionData.byte_size()}")
                return
            
            fusion = NavFusionData.from_bytes(data)
            if fusion.gSpeed > 0.2:
                break
        cnt=0    
        while cnt<300:
            cnt+=1
            data = f.read(NavFusionData.byte_size())
            if len(data) != NavFusionData.byte_size():
                print(f"Chyba: načteno {len(data)} bajtů, očekáváno {NavFusionData.byte_size()}")
                return
            
            fusion = NavFusionData.from_bytes(data)
            print(f"{fusion.ts_mono:.3f},{fusion.hAcc:.2f},"
                f"{fusion.speed:.3f},{fusion.gSpeed:.3f},{fusion.sAcc:.3f},"
                f"{fusion.heading:.2f},{fusion.vehHeading:.2f},{fusion.motHeading:.2f},{fusion.headingAcc:.2f},"
                f"{fusion.lastGyroZ:.4f},{fusion.gyroZ:.4f},{fusion.gyroZAcc:.4f},"
                f"{fusion.gnssFixOK},{fusion.drUsed}"
                )

if __name__ == "__main__":
    main()