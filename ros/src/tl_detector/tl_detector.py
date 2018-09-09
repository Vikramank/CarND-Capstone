#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from scipy.spatial import KDTree
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import numpy as np
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree =None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.threshold_distance=60 # threshold distance between car and traffic lights to start applying brakes if red signal is detected

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if not self.waypoints:
            self.waypoints = waypoints
        if not self.waypoints_2d:
           self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
           self.waypoint_tree=KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def distance_funct(self,x1,x2,y1,y2):
        array1=np.asarray([x1,y1])
        array2=np.asarray([x2,y2])
        dist=np.linalg.norm(array1-array2)

        return dist
    def stop_loc(self, pos_x, pos_y):
        # light state initialization
        light = TrafficLight()

        # pose position
        light.pose = PoseStamped()
        light.pose.header.stamp = rospy.Time.now()
        light.pose.header.frame_id = 'world'
        light.pose.pose.position.x = pos_x
        light.pose.pose.position.y = pos_y
        #light.pose.pose.position.z = 0.0
        return light
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        #return 0
        current_pose=np.asarray([self.pose.pose.position.x,self.pose.pose.position.y])
        _,closest_index=self.waypoint_tree.query(current_pose)
        closest_coordinate=np.asarray(self.waypoints_2d[closest_index])
        previous_coordinate=np.asarray(self.waypoints_2d[closest_index-1])
        #check whether this waypoint is ahead or behind the car
        a_vector=closest_coordinate-previous_coordinate
        b_vector=current_pose-closest_coordinate
        dot_product=np.dot(a_vector,b_vector)
        bool=dot_product>0

        if bool:
             closest_index = (closest_index + 1) % len(self.waypoints_2d)

        return closest_index

    #def get_light_state(self, light):
    def get_light_state(self):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        """    
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
        #return light.state
        #return TrafficLight.RED 

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light_present = False
        light_wp=-1
        closest_wp=-1
        state=TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        min_dist=np.finfo(np.float32).max
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_position_index = self.get_closest_waypoint(self.pose)
            #TODO find the closest visible traffic light (if one exists)
            for i,stop_position in enumerate(self.stop_line_positions):
                stop_pose=self.stop_loc(stop_position[0], stop_position[1])
                stop_pos_index=self.get_closest_waypoint(stop_pose)
                dist=self.distance_funct(self.waypoints_2d[car_position_index][0],
                                    self.waypoints_2d[stop_pos_index][0],
                                    self.waypoints_2d[car_position_index][1],
                                    self.waypoints_2d[stop_pos_index][1]
                                    )

                if (min_dist> dist):
                    min_dist=dist
                if (dist< self.threshold_distance) and (stop_pos_index>car_position_index):
                    light_present=True
                    min_dist=dist
                    closest_wp=stop_pos_index

        if light_present:
                #state = self.get_light_state(light)
                state = self.get_light_state()
                light_wp = closest_wp
                rospy.logwarn("Traffic light id: {}, and its color state: {}".format(closest_wp, state))
        else:
                state = TrafficLight.UNKNOWN
                light_wp = -1
        return light_wp, state
        #self.waypoints = None
        #return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
