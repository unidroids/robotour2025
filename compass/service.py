# service.py
import threading
import zmq
from typing import Optional
from imu_serial import ImuSerialIO
from dispatcher import MessageDispatcher
from handlers.angle_handler import AngleHandler
from handlers.acc_handler import AccHandler
from handlers.gyro_handler import GyroHandler
from handlers.mag_handler import MagHandler
from handlers.dummy_handler import DummyHandler
import builders
import time

class CompassService:
    def __init__(self, config=None):
        from configuration import CompassConfig
        self.config = config or CompassConfig()
        self.device = self.config.device
        self.baudrate = self.config.baudrate
        
        self.running = False
        self.configuring = False
        self._initialized = False
        self._lock = threading.Lock()
        
        self.imu_serial = None
        self.dispatcher = None
        self.zmq_context = None
        self.zmq_pub = None

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if self.configuring:
                return "ERR BUSY"
            
            if not self._initialized:
                self.zmq_context = zmq.Context.instance()
                self.zmq_pub = self.zmq_context.socket(zmq.PUB)
                self.zmq_pub.bind("ipc:///tmp/robot-compass")
                
                self.imu_serial = ImuSerialIO(self.device, self.baudrate)
                self.dispatcher = MessageDispatcher(self.imu_serial)
                
                self.acc_handler = AccHandler(self.zmq_pub)
                self.gyro_handler = GyroHandler(self.zmq_pub)
                self.angle_handler = AngleHandler(self.zmq_pub)
                self.mag_handler = MagHandler(self.zmq_pub)
                self.dummy_handler = DummyHandler()
                
                self.dispatcher.register_handler(0x51, self.acc_handler)
                self.dispatcher.register_handler(0x52, self.gyro_handler)
                self.dispatcher.register_handler(0x53, self.angle_handler)
                self.dispatcher.register_handler(0x54, self.mag_handler)
                
                self._initialized = True
                
            self.dispatcher.start()
            self.imu_serial.open()
            self.running = True
            print("[SERVICE] COMPASS STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
            
            if self.dispatcher:
                self.dispatcher.stop()
            
            self.running = False
            if not self.configuring:
                if self.imu_serial:
                    self.imu_serial.close()
                    
            print("[SERVICE] COMPASS STOPPED")
            return "OK"
            
    def get_status(self):
        with self._lock:
            if self.configuring:
                return "CONFIGURING"
            if not self.running:
                return "STOPPED"
        stats = self.dispatcher.stats()
        chk_err = self.imu_serial.get_error_counters()
        return f"RUNNING handled={stats[0]} unknown={stats[1]} errors={stats[2]} chk_err={chk_err}"
        
    def setup_unit(self):
        with self._lock:
            if self.running or self.configuring:
                return "ERR BUSY"
            if not self._initialized:
                self.imu_serial = ImuSerialIO(self.device, self.baudrate)
            self.configuring = True
            self.imu_serial.open()

        print(f"[SERVICE] COMPASS setup_unit START on {self.device} at {self.baudrate} baud")
        try:
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_rsw(builders.RSW_ACC_GYRO_ANGLE_MAG)) # MAG + ANGLE + GYRO + ACC (0x001E)
            time.sleep(0.05)
            
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_rrate(builders.RATE_20HZ)) # 20Hz Output rate (0x07)
            time.sleep(0.05)
            
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_bandwidth(builders.BW_5HZ)) # 5Hz Bandwidth (0x06)
            time.sleep(0.05)
            
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_baud(builders.BAUD_230400)) # 230400bps (0x07)
            time.sleep(0.05)
            
            print("[SERVICE] COMPASS settings applied, saving and rebooting...")
            
            # Save settings
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_save())
            time.sleep(0.1)

            # Reboot
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_reboot())
            time.sleep(0.5)
            
            # Close old serial port
            self.imu_serial.close()
            
            # Update config and reopen with new baudrate
            self.baudrate = 230400
            self.config.baudrate = 230400
            self.config.save()
            
            self.imu_serial.baudrate = self.baudrate
            self.imu_serial.open()
            time.sleep(0.5)
            
            print(f"[SERVICE] COMPASS reopened at {self.baudrate} baud. Verifying...")
            
            # Helper for reading register
            def read_reg(addr: int) -> Optional[int]:
                while self.imu_serial.get_message(timeout=0.0) is not None:
                    pass
                self.imu_serial.send_command(builders.build_unlock())
                time.sleep(0.05)
                self.imu_serial.send_command(builders.build_read(addr))
                st = time.time()
                while time.time() - st < 1.0:
                    msg = self.imu_serial.get_message(timeout=0.1)
                    if msg and len(msg) >= 11 and msg[1] == 0x5F:
                        return msg[2] | (msg[3] << 8)
                return None

            rsw = read_reg(0x02)
            rrate = read_reg(0x03)
            bw = read_reg(0x1F)
            baud = read_reg(0x04)

            res_str = f"OK (Verify: RSW={hex(rsw) if rsw else None}, RATE={hex(rrate) if rrate else None}, BW={hex(bw) if bw else None}, BAUD={hex(baud) if baud else None})"
            print(f"[SERVICE] COMPASS setup_unit DONE: {res_str}")
            return res_str
            
        finally:
            with self._lock:
                self.configuring = False
                if not self.running:
                    self.imu_serial.close()

    def calibrate_compass(self):
        with self._lock:
            if self.running or self.configuring:
                return "ERR BUSY"
            if not self._initialized:
                self.imu_serial = ImuSerialIO(self.device, self.baudrate)
            self.configuring = True
            self.imu_serial.open()
            
        self.imu_serial.send_command(builders.build_unlock())
        time.sleep(0.05)
        self.imu_serial.send_command(builders.build_calibrate_compass())
        return "OK"
        
    def calibrate_acc(self):
        with self._lock:
            if self.running or self.configuring:
                return "ERR BUSY"
            if not self._initialized:
                self.imu_serial = ImuSerialIO(self.device, self.baudrate)
            self.configuring = True
            self.imu_serial.open()

        self.imu_serial.send_command(builders.build_unlock())
        time.sleep(0.05)
        self.imu_serial.send_command(builders.build_calibrate_acc())
        return "OK"
        
    def save_calibration(self):
        with self._lock:
            if not self.configuring:
                return "ERR NOT CONFIGURING"
                
        self.imu_serial.send_command(builders.build_unlock())
        time.sleep(0.05)
        self.imu_serial.send_command(builders.build_save())
        
        with self._lock:
            self.configuring = False
            if not self.running:
                self.imu_serial.close()
        return "OK"

    def cancel(self):
        with self._lock:
            if not self.configuring:
                return "ERR NOT CONFIGURING"
                
        self.imu_serial.send_command(builders.build_unlock())
        time.sleep(0.05)
        self.imu_serial.send_command(builders.build_reboot())
        
        with self._lock:
            self.configuring = False
            if not self.running:
                self.imu_serial.close()
        return "OK"

    def factory_reset(self):
        with self._lock:
            if self.running or self.configuring:
                return "ERR BUSY"
            if not self._initialized:
                self.imu_serial = ImuSerialIO(self.device, self.baudrate)
            self.configuring = True
            self.imu_serial.open()
            
        try:
            self.imu_serial.send_command(builders.build_unlock())
            time.sleep(0.05)
            self.imu_serial.send_command(builders.build_factory_reset())
            time.sleep(0.05)
        finally:
            with self._lock:
                self.configuring = False
                if not self.running:
                    self.imu_serial.close()
        return "OK"
