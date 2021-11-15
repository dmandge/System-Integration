#code from the video tutorial in the lecture is used as a starting point 
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  #min throttle value 
        mx = 0.2 # max throttle value 
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
         
        tau = 0.5 #1/(2pi * tau) = cutoff frequency
        ts = 0.02 #sample time
        
        self.vel_lpf = LowPassFilter(tau, ts) # velocity incoming is noisy so its filters all high freuqency noise
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit 
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio 
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        self.last_time = rospy.get_time()
        self.log_time = rospy.get_time()
        
        
        
        
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel): #gets called in dbw.py
        
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        current_vel = self.vel_lpf.filt(current_vel)
        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0 
#         accel = self.vel_lpf.get()
        
#         if accel > 0.0:
#             throttle = self.throttle_controller.step(accel, self.dt)
        
        if linear_vel == 0.0 and current_vel <0.1:
            throttle = 0 
            brake = 700 #to hold a car in place if we are stopped at a traffic light 1m/s2 accel 
        elif throttle <0.1 and vel_error <0.0:
            throttle =0.0
            decel = max(vel_error, self.decel_limit)
#             brake = abs(accel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * 2
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
#             brake =  min(700, (abs(decel) * self.vehicle_mass * self.wheel_radius))

        return throttle, brake, steer
            
        

#         vel_error = target_linear_velocity - current_velocity
#         raw_accel = self.accel_controller.step(vel_error, self.dt)

#         self.lowpass_filter.filt(raw_accel)
#         accel = self.lowpass_filter.get()


#         brake = 0.0
#         throttle = 0.0
#         steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_velocity)

#         if accel > 0.0:
#             throttle = self.throttle_controller.step(accel, self.dt)

#         if target_linear_velocity == 0 and current_velocity < 0.1:
#             throttle = 0
#             brake = 400

#         elif throttle < .1 and accel < 0:
#             throttle = 0
#             accel = max(accel,self.decel_limit)
#             brake = abs(accel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * 2

#         return throttle, brake, steer
        
#         return 1., 0., 0.
