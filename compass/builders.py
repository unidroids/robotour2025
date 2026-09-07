# builders.py
import struct

# ==============================================================================
# Register 0x02: Output Content Mask (RSW)
# ==============================================================================
RSW_TIME      = 0x0001  # 0x50: Time
RSW_ACC       = 0x0002  # 0x51: Acceleration
RSW_GYRO      = 0x0004  # 0x52: Angular velocity
RSW_ANGLE     = 0x0008  # 0x53: Angle
RSW_MAG       = 0x0010  # 0x54: Magnetic field
RSW_PORT      = 0x0020  # 0x55: Port status
RSW_PRESS     = 0x0040  # 0x56: Barometric altitude
RSW_GPS       = 0x0080  # 0x57: Latitude & Longitude
RSW_VELOCITY  = 0x0100  # 0x58: Ground speed
RSW_QUATER    = 0x0200  # 0x59: Quaternion
RSW_GSA       = 0x0400  # 0x5A: GPS positioning accuracy

# Kombinovaná maska pro ACC + GYRO + ANGLE + MAG (0x001E)
RSW_ACC_GYRO_ANGLE_MAG = RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG

# ==============================================================================
# Register 0x03: Output Rate (RRATE)
# ==============================================================================
RATE_0_2HZ = 0x01
RATE_0_5HZ = 0x02
RATE_1HZ   = 0x03
RATE_2HZ   = 0x04
RATE_5HZ   = 0x05
RATE_10HZ  = 0x06
RATE_20HZ  = 0x07
RATE_50HZ  = 0x08
RATE_100HZ = 0x09
RATE_200HZ = 0x0B

# ==============================================================================
# Register 0x1F: Low-Pass Filter Bandwidth (BANDWIDTH)
# ==============================================================================
BW_256HZ = 0x00
BW_188HZ = 0x01
BW_98HZ  = 0x02
BW_42HZ  = 0x03
BW_20HZ  = 0x04
BW_10HZ  = 0x05
BW_5HZ   = 0x06

# ==============================================================================
# Register 0x04: Serial Port Baud Rate (BAUD)
# ==============================================================================
BAUD_4800   = 0x01
BAUD_9600   = 0x02
BAUD_19200  = 0x03
BAUD_38400  = 0x04
BAUD_57600  = 0x05
BAUD_115200 = 0x06
BAUD_230400 = 0x07


def build_command(addr: int, data: int) -> bytes:
    """Helper to build standard write command: FF AA ADDR DATAL DATAH"""
    return bytes([0xFF, 0xAA, addr, data & 0xFF, (data >> 8) & 0xFF])

def build_unlock() -> bytes:
    """Unlock register to allow writing."""
    return build_command(0x69, 0xB588)

def build_save() -> bytes:
    """Save configuration."""
    return build_command(0x00, 0x0000)

def build_read(addr: int) -> bytes:
    """Read register."""
    return bytes([0xFF, 0xAA, 0x27, addr & 0xFF, 0x00])

def build_reboot() -> bytes:
    """Reboot (drops unsaved config, acts like cancel)."""
    return build_command(0x00, 0x00FF)

def build_factory_reset() -> bytes:
    """Factory reset."""
    return build_command(0x00, 0x0001)

def build_calibrate_compass() -> bytes:
    """Set calibration mode: Magnetic Field Calibration (Spherical Fitting)."""
    return build_command(0x01, 0x0007)

def build_calibrate_acc() -> bytes:
    """Set calibration mode: Auto add-up calibration."""
    return build_command(0x01, 0x0001)

def build_rsw(mask: int) -> bytes:
    """Set output content mask."""
    return build_command(0x02, mask)

def build_rrate(rate_code: int) -> bytes:
    """Set output rate."""
    return build_command(0x03, rate_code)

def build_baud(baud_code: int) -> bytes:
    """Set baud rate (0x02=9600, 0x06=115200 etc)."""
    return build_command(0x04, baud_code)

def build_bandwidth(bw_code: int) -> bytes:
    """Set bandwidth."""
    return build_command(0x1F, bw_code)

