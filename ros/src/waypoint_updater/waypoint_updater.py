#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import math
import numpy as np

MAX_DECEL=0.5
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb )
        rospy.Subscriber('/obstacle_waypoint',Lane,self.obstacle_cb )
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        #self.waypoints= None
        self.pose= None
        self.base_lane = None
        self.stopline_wp_idx = -1
        #self.base_waypoints=None
        self.waypoints_2d = None
        self.waypoint_tree =None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.pose=msg #current position of the car

    def waypoints_cb(self, waypoints):
            if not self.base_lane:
                  #self.waypoints=waypoints #collect the base_waypoints of the system
                  self.base_lane=waypoints
            if not self.waypoints_2d:
               self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
               self.waypoint_tree=KDTree(self.waypoints_2d)
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        print self.stopline_wp_idx

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_closest_waypoint_index(self):
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

    def publish_waypoints(self): # , closest_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_index()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        print (self.stopline_wp_idx)

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            rospy.logwarn('Decelerating the car')
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            rospy.logwarn('Decelerating the car')
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from line so front of car stops at line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1. :
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
