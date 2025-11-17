from utils.sliding_angle_average import SlidingAngleAverage


class FusionCore:
    def __init__(self):
        self.ready = False
        #TODO open data log



    def update_position(self, iTow, lat, lon, height, vAcc, hAcc):
        pass

    def update_global_heading(self, iTow, heading, gstddev, lenght):
        pass    

    def update_global_roll(self, iTow, roll, gstddev, lenght):
        pass        

    def update_local_heading(self, tmark, heading, omega):
        pass            

    def update_whell_speed(self, tmark, left_wheel_speed, right_wheel_speed):
        pass                

    def get(self):
        return #TODO NavFusoinData