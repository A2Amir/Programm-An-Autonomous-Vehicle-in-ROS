import math
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, max_throttle_percent):
        
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.5
        ki = 0.0001
        kd = 0.15
        mn = decel_limit #minial throttle value
        mx = max_throttle_percent #maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.05 # 1 / (2pi*tau) = cutoff frequency
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        self.last_throttle = 0
        self.last_brake = 100

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled, max_throttle_percent):
        # return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        filt_current_vel = self.vel_lpf.filt(current_vel)
        #rospy.loginfo('angular_vel: %f', angular_vel)
        #steering = self.yaw_controller.get_steering(twist.twist.linear.x, twist.twist.angular.z, velocity.twist.linear.x)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, filt_current_vel)

        vel_error = linear_vel - filt_current_vel
        
        #rospy.loginfo('linear_vel: %f', linear_vel)
        #rospy.loginfo('filt_current_vel: %f', filt_current_vel)
        #rospy.loginfo('vel_error: %f', vel_error)
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        acceleration = self.throttle_controller.step(vel_error, sample_time)
        
        # smooth acceleration: http://ijssst.info/Vol-17/No-30/paper19.pdf
        smooth_acc = ((linear_vel * linear_vel) - (filt_current_vel * filt_current_vel)) / (2 * 30)
          
        rospy.loginfo('smooth acceleration: %f', smooth_acc)
        rospy.loginfo('linear_vel: %f', linear_vel)
        #rospy.loginfo('angular_vel: %.3f   linear_vel: %.3f   filt_current_vel: %.3f   vel_error: %.3f  acceleration: %.3f', angular_vel, linear_vel, filt_current_vel, vel_error, acceleration)
        
        if smooth_acc >= 0:
            # converting smooth_acc values to throttle acceptable values
            throttle = smooth_acc * (max_throttle_percent - (linear_vel * 0.018)) / ((11.1111 * 11.1111)/(2*30)) + (linear_vel * 0.018)
        else:
            throttle = 0
        
        if throttle > max_throttle_percent:
            throttle = max_throttle_percent
        
        #smoothing throttle acceleration and deceleration    
        if (throttle > 0.025) and (throttle - self.last_throttle) > 0.005:
            throttle = max((self.last_throttle + 0.0025), 0.005)
        if throttle > 0.025 and (throttle - self.last_throttle) < -0.05:
            throttle = self.last_throttle - 0.05
        
        self.last_throttle = throttle
        
        brake = 0.

        if linear_vel == 0. and filt_current_vel < 0.1:
            throttle = 0.
            brake = 700. # N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif throttle < 0.025 and vel_error < 0.:
            throttle = 0.
            #decel = max(vel_error, self.decel_limit)
            decel = max((smooth_acc * 5), self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m
            #smoothing brake
            if brake > 100 and (brake - self.last_brake) > 20:
                brake = max((self.last_brake + 20), 100)
        
        if brake > 20 and (brake - self.last_brake) > 20:
            brake = max((self.last_brake + 20), 20)
        
        #rospy.loginfo('brake: %f', brake)
        #rospy.loginfo('trottle: %f', throttle)
        self.last_brake = brake
        
        
        return throttle, brake, steering