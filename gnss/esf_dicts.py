# esf_dicts.py
# Číselníky pro UBX-ESF zprávy (status, meas, atd.)

# Fusion mode (UBX-ESF-STATUS)
fusion_modes = {
    0: "init",
    1: "fusion",
    2: "suspend",
    3: "disabled",
}

# Sensor type (UBX-ESF-STATUS, UBX-ESF-MEAS)
sensor_types = {
    0: "none",
    1: "gyro z",
    2: "accel y",
    3: "wheel tick L",
    4: "wheel tick R",
    5: "gyro z (alt)",
    8: "rear-left wheel",
    9: "rear-right wheel",
    10: "single tick",
    11: "speed",
    12: "gyro temp",
    13: "gyro y",
    14: "gyro x",
    16: "accel x",
    17: "accel y",
    18: "accel z",
}
