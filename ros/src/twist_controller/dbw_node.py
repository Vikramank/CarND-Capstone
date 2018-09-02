#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller


'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.
You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.
One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.
We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.
Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.
'''
class Car_Param(object):
      def __init__(self):
          self.vehicle_mass=None
          self.fuel_capacity=None
          self.brake_deadband=None
          self.decel_limit= None
          self.accel_limit=None
          self.wheel_radius=None
          self.wheel_base=None
          self.steer_ratio=None
          self.max_lat_accel=None
          self.max_steer_angle=None
          self.min_speed= None

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        cp = Car_Param()

        cp.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        cp.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        cp.brake_deadband = rospy.get_param('~brake_deadband', .1)
        cp.decel_limit = rospy.get_param('~decel_limit', -5)
        cp.accel_limit = rospy.get_param('~accel_limit', 1.)
        cp.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        cp.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        cp.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        cp.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        cp.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        cp.min_speed = 5


        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.sampling_rate = 20.0
        # TODO: Create `TwistController` object
        #self.controller = Controller(sampling_rate = self.sampling_rate, Car_Param = cp)
        self.controller = Controller(Car_Param = cp)

        # TODO: Subscribe to all the topics you need to 
        self.dbw_status=True
        self.throttle=0.0
        self.brake=0.0
        self.steering=0.0
        self.current_velocity=None
        self.linear_veloctiy=None
        self.current_twist_cmd=None
        self.angular_velocity=None #required by yaw_controller
        self.duration = 1.0

        rospy.Subscriber('/twist_cmd',TwistStamped, self.twist_cmd_cb, queue_size=5)
        rospy.Subscriber('/current_velocity',TwistStamped, self.current_velocity_cb, queue_size=5)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_enabled_cb, queue_size=1)

        self.reset_flag =True
        
        self.previous_timestamp = rospy.get_time()

        self.loop()
    
    def dbw_enabled_cb(self,dbw_enabled):
        try:
           self.dbw_enabled=bool(dbw_enabled.data)
        except Exception:
           self.dbw_enabled = dbw_enabled

    def current_velocity_cb(self,current_velocity):
        self.current_velocity = current_velocity

    def twist_cmd_cb(self, twist_cmd):
        self.current_twist_cmd = twist_cmd


    def loop(self):
            
           rate = rospy.Rate(self.sampling_rate) # 50Hz
           while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            current_timestamp = rospy.get_time()
            #duration = current_timestamp - self.previous_timestamp
            duration = 1.0 # Greg - Not sure if should do this to keep car from slowing down to 0 before stoplight
            self.previous_timestamp = current_timestamp
            if self.dbw_status and self.current_velocity is not None and self.current_twist_cmd is not None:
               if self.reset_flag:
                  #self.controller.reset()
                  self.reset_flag =False
     
               throttle, brake, steering = self.controller.control(twist_cmd=self.current_twist_cmd, current_velocity=self.current_velocity,duration = duration)
               
            # if <dbw is enabled>:
               self.publish(throttle, brake, steering)
            else:
               self.reset_flag = True
            
            rate.sleep()
            '''
            rate = rospy.Rate(50) # 50Hz

            while not rospy.is_shutdown():
                # TODO: Get predicted throttle, brake, and steering using `twist_controller`
                # You should only publish the control commands if dbw is enabled
                # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
                #                                                     <proposed angular velocity>,
                #                                                     <current linear velocity>,
                #                                                     <dbw status>,
                #                                                     <any other argument you need>)
                # if <dbw is enabled>:
                #   self.publish(throttle, brake, steer)
                duration=1

                self.throttle,self.brake,self.steering = self.controller.control(twist_cmd=self.current_twist_cmd, current_velocity=self.current_velocity,duration = duration)

                if self.dbw_status:
                    self.publish(self.throttle,self.brake,self.steering)

                rate.sleep()
                '''

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
