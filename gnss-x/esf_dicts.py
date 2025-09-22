# esf_dicts.py
# Číselníky pro UBX-ESF zprávy (status, meas, atd.)

fusion_modes = {
    0: "init",
    1: "fusion",
    2: "suspend",
    3: "disabled",
}

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

calib_status = {
    0: "not calibrated",
    1: "calibrating",
    2: "calibrated",
    3: "calibrated",
}

time_status = {
    0: "no data",
    1: "byte reception",
    2: "event input",
    3: "time tag provided",
}

wt_init_status = {
    0: "off",
    1: "initializing",
    2: "initialized",
}

mnt_alg_status = {
    0: "off",
    1: "initializing",
    2: "initialized",
    3: "initialized",
}

ins_init_status = {
    0: "off",
    1: "initializing",
    2: "initialized",
}

imu_init_status = {
    0: "off",
    1: "initializing",
    2: "initialized",
}