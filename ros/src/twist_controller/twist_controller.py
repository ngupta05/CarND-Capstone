
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
import math

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.max_lat_accel = max_lat_accel

    def control(self, proposed_v, proposed_w, current_v, dbw_status):
        steer = self.yaw_controller.get_steering(proposed_v, proposed_w, current_v)

        throttle = 0.0
        brake = 0.0

        # Assuming 1 sec to reach the desired vel
        # accel = final_v - current_v
        TIME = 1.0
        if proposed_v - current_v > 1e-5:
            throttle = min((proposed_v - current_v) / TIME, self.max_lat_accel) / self.max_lat_accel
        elif proposed_v - current_v < -1e-5:
            brake = min((proposed_v - current_v) / TIME, self.max_lat_accel) / self.max_lat_accel

        # Return throttle, brake, steer
        return throttle, brake, -1.0 * steer
