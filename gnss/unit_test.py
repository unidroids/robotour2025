import socket
from data.nav_fusion_data import NavFusionData

HOST = "127.0.0.1"
PORT = 9006

def main():
    with socket.create_connection((HOST, PORT)) as sock:
        f = sock.makefile("rwb", buffering=0)
        # Pošli příkaz
        f.write(b"GET_BINARY_STREAM\n")
        f.flush()
        # Načti binární data (očekáváme 51 B)
        while True:
            data = f.read(NavFusionData.byte_size())
            if len(data) != NavFusionData.byte_size():
                print(f"Chyba: načteno {len(data)} bajtů, očekáváno {NavFusionData.byte_size()}")
                return
            # Dekóduj a vypiš jako JSON
            fusion = NavFusionData.from_bytes(data)
            print(fusion.to_json()[:30])

if __name__ == "__main__":
    main()