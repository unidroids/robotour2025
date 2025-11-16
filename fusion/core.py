from utils.sliding_angle_average import SlidingAngleAverage


class FusionCore:
    def __init__(self):
        self.ready = False


    def update_position(self, lat, lon, height, vAcc, hAcc):
        pass

    def update_global_heading(self, heading, gstddev, lenght):
        pass    

    def update_global_roll(self, roll, gstddev, lenght):
        pass        

    def update_local_heading(self, heading, omega):
        pass            

    def update_whell_speed(self, left_wheel_speed, right_wheel_speed):
        pass                

    def get(self):
        return #TODO