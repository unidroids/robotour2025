# odm_handler.py
from __future__ import annotations

import socket
import traceback
from typing import Optional

__all__ = ["UniHeadinAHandler"]


class UniHeadinAHandler:
    """
    Handler příjmu UNIHEADINGA zpráv a jejich přeposílání.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9009,
        timeout: float = 2.0,
        autoconnect: bool = True,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout

        self._sock: Optional[socket.socket] = None
        self._stream_opened = False

        self._lastest: Optional[bytes] = None  

        if autoconnect:
            self._ensure_socket()

    # --- veřejné API ---

    def handle(self, message_bytes: bytes, wait: bool = False):
        """
        Zpracuje jednu syrovou zprávu.
        """
        # kontrola zprávy
        if not message_bytes.startswith(b'#UNIHEADINGA,'):
            raise Exception("Not a #UNIHEADINGA message")
        
        semi_idx = message_bytes.index(b';')
        
        header = message_bytes[13:semi_idx]
        #print(header)
        body = message_bytes[semi_idx+1:-11]
        #print(body)
        short_idx = nth_index(body,b',',-1,8)
        short = body[0:short_idx]
        #print(short)

        # 1) uložit na _lastest
        self._lastest = short

        # 2) poslat na stream
        self._send_heading(short, wait)

    def get_lastest(self) -> Optional[bytes]:
        """Vrátí naposledy přijatá ODM data (nebo None)."""
        return self._lastest

    # --- interní pomocné metody ---

    def _ensure_socket(self) -> None:
        """připojení k serveru"""
        if self._sock is not None:
            return

        try:
            # open socket with non waiting for sedning
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM )
            s.settimeout(self._timeout)
            s.connect((self._host, self._port))
            #s.settimeout(None)  # blokující režim po spojení
            s.setblocking(True)  # <<< NEBLOKUJÍCÍ režim
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # <<< nižší latence malých zpráv
            self._sock = s
            self._stream_opened = True  # bude otevřeno po handshake
            print(f"[UniHeadingAHandler] Connected to {self._host}:{self._port}")
        except Exception as e:
            # necháme zavřené; zkusíme znovu při další zprávě
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            self._stream_opened = False
            print(f"[UniHeadingA] Unable to connect to {self._host}:{self._port}: {e!r}")
            

    def _send_heading(self, message: bytes, wait:bool = False) -> None:
        """Pošle UNIHEADINGA data. Při chybě socket zavře (reconnect proběhne při další zprávě)."""
        self._ensure_socket()
        if self._sock is None or not self._stream_opened:
            return
        try:
            self._sock.sendall(b'HEADING\n' + message + b'\n')
            if wait:
                result = self._sock.recv(512)
                print(f"[UniHeadingAHandler] recieved: {result}")
        except Exception as e:
            print(f"[UniHeadigAHandler] sendall failed: {e!r}")
            self._close_socket()

    def _close_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._stream_opened = False

    def __del__(self):
        # best-effort úklid
        self._close_socket()

def nth_index(data: bytes, sub: bytes, start:int, n: int) -> int:
    pos = start
    for _ in range(n):
        pos = data.index(sub, pos + 1)
    return pos

# --- jednoduchý lokální test bez sítě ---
if __name__ == "__main__":
    h = UniHeadinAHandler()  
    msg = b'#UNIHEADINGA,92,GPS,FINE,2392,519230000,0,0,18,8;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*f25b9a39\r\n'
    h.handle(msg, True)
    last = h.get_lastest()
    print("Lastest:", last)
    if last:
        print("Serialized length:", len(last), "bytes")
    msg = b'#UNIHEADINGA,92,GPS,FINE,2392,519238000,0,0,18,8;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*e914be33\r\n'
    h.handle(msg)
    last = h.get_lastest()
    print("Lastest:", last)
    if last:
        print("Serialized length:", len(last), "bytes")
