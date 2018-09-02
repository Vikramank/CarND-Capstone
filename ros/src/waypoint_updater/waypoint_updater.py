#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb )
        rospy.Subscriber('/obstacle_waypoint',Lane,self.obstacle_cb )

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints=None
        self.pose=None
        self.base_waypoints=None
        self.waypoints_2d=None
        self.waypoint_tree=None
        self.loop()

        #rospy.spin()
    #loop to publish waypoint data at the rate of 50 Hz. Note, Carla will accept messages published at a rate greater than 10 Hz

    def loop(self):
        rate= rospy.Rate(50)
        #run the following code till the system is live
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                closest_waypoint_idx=self.get_closest_waypoint_index() #get closest_index
                self.publish_waypoints(closest_waypoint_idx)
                rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose=msg.pose #current position of the car
        pass

    def waypoints_cb(self, waypoints):
        if not self.waypoints:
              self.waypoints=waypoints #collect the base_waypoints of the system
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        if (not self.waypoint_tree) and (not self.waypoints_2d) :
              self.waypoint_tree=KDTree(self.waypoints_2d)

        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    def publish_waypoints(self, closest_index):
        lane=Lane()
        lane.header=self.waypoints.header
        lane.waypoints=self.waypoints[closest_index:closest_index+LOOKAHEAD_WPS]
        self.final_waypoints_pub(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
