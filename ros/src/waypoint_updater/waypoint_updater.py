#!/usr/bin/env python
#code from the video tutorial in the lecture is used as a starting point 
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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5 #max decelration when approaching traffic light


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)        
        
        

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below



        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None 
        self.stopline_wp_idx = -1 
        self.waypoints_2d = None 
        self.waypoint_tree = None 
        self.decelerate_count = 0
        
        self.loop()

#         rospy.spin()

    def loop(self):
        rate = rospy.Rate(50) #gives control over the publishing frequency we can go as low as 30Hz as these msgs go to the waypoint follower which is running at 30Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                #Get closest waypoint
#                 closest_waypoint_idx = self.get_closest_waypoint_idx()
#                 self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()
            
            
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x #coordinates of the car 
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:#modulate length of the waypoint 2d 
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def publish_waypoints(self):
#         lane = Lane()
#         lane.header = self.base_waypoints.header()
#         lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
#         self.final_waypoints_pub.publish(lane)
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane(self):
        lane = Lane()
        
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints 
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_i):
        temp = []
        for i, wp in enumerate(waypoints):
            
            p = Waypoint()
            p.pose = wp.pose 
            stop_idx = max(self.stopline_wp_idx - closest_i - 2, 0) #two waypoints back from the line so that front of the car stops at the line 
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist) #math.sqrt(2*MAX_DECEL*dist) #change this relation
            if vel < 1.0:
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
            
        return temp 

    def pose_cb(self, msg):
        # TODO: Implement
        #step 1 - lessson 
        self.pose = msg #stores the cars position around 50hz 
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # step 1 (lesson)
        # take a chunk of the base waypoints and use first 200 of them in front of the car. KDT from scipy allows to lookup closest 200 waypoints in the space 
        self.base_lane = waypoints
#         self.base_waypoints = waypoints 
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints] # convert each waypoint into 2d coordinates 
            self.waypoint_tree = KDTree(self.waypoints_2d) # use the 2d coords to construct the KD tree
        

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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
