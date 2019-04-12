from math import atan

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

        self.last_steering = 0.0


    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity, current_angular_vel):
        # If the difference between current angular velocity and target angular velocity is less than a set threshold,
        # use previous steering value
        if abs(current_angular_vel - angular_velocity) > 0.02:
            angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

            if abs(current_velocity) > 0.1:
                max_yaw_rate = abs(self.max_lat_accel / current_velocity);
                angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

            steering = self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
        else:
            steering = self.last_steering

        self.last_steering = steering
        return steering

    def reset(self):
        self.last_steering = 0.0
