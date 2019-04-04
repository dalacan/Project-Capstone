from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02 # Sample time (50 hertz)
        # ts = .033 # Sample time (30 hertz)
        self.vel_lpf = LowPassFilter(tau, ts)

        self.ang_vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, linear_vel, angular_vel, current_vel, current_ang_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # rospy.logwarn("Curr ang vel: {0}".format(current_ang_vel))

        current_vel = self.vel_lpf.filt(current_vel)
        current_ang_vel = self.ang_vel_lpf.filt(current_ang_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target vel: {0}".format(linear_vel))
        # rospy.logwarn("Current vel: {0}".format(current_vel))

        # If the difference between current angular velocity and target angular velocity is less than a set threshold, do not update steering
        if abs(current_ang_vel - angular_vel) > 0.02:
            steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        else:
            steering = None
            
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        # Throttle to steering ratio function Reduce throttle for larger steerng
        # if steering is not None:
        #     throttle = -0.0015625 * steering * steering + throttle

        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 #N*m - to hold the car in place if we are stopped at a light. Accl - 1m/s^2
        elif throttle < .1 and vel_error <0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        # rospy.logwarn("Throttle: {0}".format(throttle))
        # rospy.logwarn("Brake: {0}".format(brake))
        # rospy.logwarn("Steering: {0}".format(steering))

        # Return throttle, brake, steer
        # return 1., 0., 0.
        return throttle, brake, steering
