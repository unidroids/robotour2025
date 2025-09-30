
import socket
import struct
from data.nav_fusion_data import NavFusionData

class GnssClient:
    def __init__(self, host='127.0.0.1', port=9006):
        self.host = host
        self.port = port
        self.sock = None

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(1.0)
        self.sock.sendall(b'GET_BINARY_STREAM\n')  # Request binary data stream       
    
    def _close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                pass
            finally:
                self.sock = None

    def start(self):
        pass
        #self._connect()

    def stop(self):
        self._close()

    def read_nav_fusion_data(self):
        data_len = NavFusionData._STRUCT.size
        raw = b'' 
        try:
            if not self.sock:
                self._connect()
            raw = self.sock.recv(data_len)
            if len(raw) != data_len:
                print(f"[GNSS CLIENT] Incomplete data received: expected {data_len} bytes, got {len(raw)} bytes. Meaage: {raw.hex()}")
                self._close()
                return None # incomplete data
            nav_data = NavFusionData.from_bytes(raw)
        except socket.timeout as e:
            print(f"[GNSS CLIENT] Timeout: {e}")
            self._close()
            return None # socket timeout           
        except socket.error as e:
            print(f"[GNSS CLIENT] Socket error: {e}. Message: {raw.hex()}")
            self._close()
            return None # socket error
        except struct.error as e:
            print(f"[GNSS CLIENT] Struct error: {e}. Message: {raw.hex()}")
            self._close()
            return None # struct unpacking error
        except ValueError as e:
            print(f"[GNSS CLIENT] Error parsing NavFusionData: {e}. Message: {raw.hex()}")
            self._close()
            return None # parsing error
        return nav_data

