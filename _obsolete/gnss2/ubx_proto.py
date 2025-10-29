# ubx_proto.py
import struct, threading, time, queue, logging

SYNC1, SYNC2 = 0xB5, 0x62

def ck_a_b(data: bytes):
    a=b=0
    for x in data:
        a=(a+x)&0xFF; b=(b+a)&0xFF
    return bytes([a,b])

def build_ubx(msg_class, msg_id, payload=b''):
    hdr = bytes([SYNC1,SYNC2,msg_class,msg_id]) + struct.pack('<H', len(payload))
    return hdr + payload + ck_a_b(hdr[2:]+payload)

def parse_stream(byte_iter):
    """Generator rámců UBX (class,id,payload,ts) – tolerantní ke šumu."""
    state = 0
    buf = bytearray()
    while True:
        b = next(byte_iter)
        if state==0:
            if b==SYNC1: state=1
        elif state==1:
            if b==SYNC2: state=2
            else: state=0
        elif state==2:
            buf = bytearray([b])  # class
            state=3
        elif state==3:
            buf.append(b)         # id
            state=4
        elif state==4:
            buf.extend([b,0])     # len LSB, placeholder MSB
            state=5
        elif state==5:
            buf[-1]=b             # len MSB
            paylen = buf[-2] | (buf[-1]<<8)
            payload = bytearray()
            state=6 if paylen==0 else 7
            if paylen==0:  # rovnou čekáme CK_A,CK_B
                pass
        elif state==7:
            payload.append(b)
            if len(payload) == (buf[-2] | (buf[-1]<<8)):
                state=6
        elif state==6:
            # b je CK_A, další CK_B přečteme
            ck_a = b
            ck_b = next(byte_iter)
            frame = bytes([buf[0], buf[1]]) + bytes([buf[-2], buf[-1]]) + payload
            if ck_a_b(frame) == bytes([ck_a, ck_b]):
                ts = time.time()
                yield (buf[0], buf[1], bytes(payload), ts)
            state=0

class SerialBytes:
    def __init__(self, ser):
        self.ser = ser
    def __iter__(self):
        while True:
            b = self.ser.read(1)
            if not b: 
                # non-blocking: malé spoždění, aby se neprotočila CPU
                time.sleep(0.0005)
                continue
            yield b[0]

# ---- Pomocné extrakce ----
def get_iTOW_from(payload, offset=0):
    # iTOW je U4 v ms na offsetu 0 u NAV* a mnoha ESF*, EOE
    if len(payload) >= offset+4:
        return struct.unpack_from('<I', payload, offset)[0]
    return None

def build_cfg_valset(items, layer_ram=True, layer_bbr=False, layer_flash=False):
    # UBX-CFG-VALSET (0x06,0x8A): version(U1)=0, layers(U1), reserved2(U2), [key(U4), val(variable)]
    layers = (1 if layer_ram else 0) | ((1<<1) if layer_bbr else 0) | ((1<<2) if layer_flash else 0)
    payload = struct.pack('<BBH', 0, layers, 0)
    for key, val, vtype in items:
        payload += struct.pack('<I', key)
        if vtype == 'L' or vtype == 'U1':  # bool / 1B
            payload += struct.pack('<B', int(val))
        elif vtype == 'U2':
            payload += struct.pack('<H', int(val))
        elif vtype == 'U4':
            payload += struct.pack('<I', int(val))
        else:
            raise ValueError(f'Unsupported vtype {vtype}')
    return build_ubx(0x06, 0x8A, payload)

def build_cfg_valsave(layers_ram=True, layers_bbr=False, layers_flash=False):
    # nepovinné pro test – můžeme uložit později
    sel = (1 if layers_ram else 0) | ((1<<1) if layers_bbr else 0) | ((1<<2) if layers_flash else 0)
    # UBX-CFG-VALSAVE (0x06,0x8E): version, layers, reserved2
    payload = struct.pack('<BBH', 0, sel, 0)
    return build_ubx(0x06,0x8E,payload)

def build_poll(msg_class, msg_id):
    return build_ubx(msg_class, msg_id, b'')
