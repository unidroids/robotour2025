
import socket
import struct
from data.nav_fusion_data import NavFusionData

class FusionClient:
    def __init__(self, host='127.0.0.1', port=9009):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(1.0)
        self.sock.sendall(b'GET_BINARY_STREAM\n')  # Request binary data stream       
    
    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                pass
            finally:
                self.sock = None

    def read_nav_fusion_data(self):
        data_len = NavFusionData._STRUCT.size
        raw = b'' 
        try:
            if not self.sock:
                self.connect()
            raw = self.sock.recv(data_len)
            if len(raw) != data_len:
                print(f"[GNSS CLIENT] Incomplete data received: expected {data_len} bytes, got {len(raw)} bytes. Meaage: {raw.hex()}")
                self._close()
                return None # incomplete data
            nav_data = NavFusionData.from_bytes(raw)
        except socket.timeout as e:
            print(f"[GNSS CLIENT] Timeout: {e}")
            self.disconnect()
            return None # socket timeout           
        except socket.error as e:
            print(f"[GNSS CLIENT] Socket error: {e}. Message: {raw.hex()}")
            self.disconnect()
            return None # socket error
        except struct.error as e:
            print(f"[GNSS CLIENT] Struct error: {e}. Message: {raw.hex()}")
            self.disconnect()
            return None # struct unpacking error
        except ValueError as e:
            print(f"[GNSS CLIENT] Error parsing NavFusionData: {e}. Message: {raw.hex()}")
            self.disconnect()
            return None # parsing error
        return nav_data

