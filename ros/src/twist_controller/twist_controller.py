GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from lowpass import LowPassFilter
from yaw_controller import YawController
class Controller(object):
    def __init__(self, Car_Param):
        # TODO: Implement
        Kp=0.3
        Kd=2.0
        Ki=0.004
        self.Car_Param=Car_Param
        self.yawcontroller = YawController(wheel_base= Car_Param.wheel_base, steer_ratio = Car_Param.steer_ratio, min_speed= Car_Param.min_speed, max_lat_accel= Car_Param.max_lat_accel, max_steer_angle=Car_Param.max_steer_angle)
        tau=0.5
        ts=0.02
        self.lowpass=LowPassFilter(tau, ts)

        pass

    def reset(self):
        self.pid.reset()
    def control(self, twist_cmd, current_velocity, duration):
        # TODO: Change the arg, kwarg list to suit your needs
        #calculate steering angle
        #angular_velocity=twist_cmd.twist.angular.z
        linear_vel = abs(twist_cmd.twist.linear.x)
        angular_vel = twist_cmd.twist.angular.z
        vel_err = linear_vel - current_velocity.twist.linear.x
        steer=self.yawcontroller.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer=self.lowpass.filt(steer) #pass the value through low pass fileter to remove noise

        #caluclate brake and throttle
        #linear_velocity=abs(twist_cmd.twist.linear.x)
        error_velocity=linear_velocity-current_velocity #cross track error for PID Controller
        acc = self.pid.step(error_velocity, duration) #PID controlls to determine

        #apply throttle and brake only when neccessary
        if (error_velocity)>0.0 :
            throttle=acc
            brake=0.0
        else :
            throttle=0.0
            decel = abs(acc)
           if decel <self.Car_Param.brake_deadband:
              decel = 0.0

           brake = decel* (self. Car_Param.vehicle_mass +self.Car_Param.fuel_capacity*GAS_DENSITY)*self.Car_Param.wheel_radius

        return throttle, brake, steer
