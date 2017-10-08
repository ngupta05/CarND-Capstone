#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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

LOOKAHEAD_WPS = 200 #TODO reset later to a suitable number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.max_speed_kph = rospy.get_param('/waypoint_loader/velocity', 20)
        self.waypoints = list()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.last_closest_waypoint = None 

        rospy.spin()

    def pose_cb(self, msg):
        if len(self.waypoints) == 0:
            return
        n = self.get_next_waypoint(msg.pose)
        l = self.waypoints[n:] + self.waypoints[:n]
        wps = l[:LOOKAHEAD_WPS]
        for i in range(LOOKAHEAD_WPS):
            wps[i].pose.header = msg.header
            wps[i].twist.header = msg.header
        lane = Lane()
        lane.header = msg.header
        lane.waypoints = wps
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_heading(self, p1, p2):
        return math.atan2(p2.y - p1.y, p2.x - p1.x)

    def get_closest_waypoint(self, pose):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        tl = len(self.waypoints)
        min_dist = 1e10
        closest_wp = self.last_closest_waypoint if self.last_closest_waypoint is not None else 0
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                      pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        
        for i in range(tl):
            idx = (i + closest_wp) % tl
            dist = dl(pose.position, self.waypoints[idx].pose.pose.position) 

            heading = self.get_heading(pose.position, self.waypoints[idx].pose.pose.position)
            angle = math.fabs(heading - yaw)
            if angle < math.pi/4 and dist < 1.0:
                min_dist = dist
                closest_wp = idx
                break
            elif dist < min_dist:
                min_dist = dist
                closest_wp = idx
        return closest_wp

    def get_next_waypoint(self, pose):
        cwp = self.get_closest_waypoint(pose)
        heading = self.get_heading(pose.position, self.waypoints[cwp].pose.pose.position)
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                      pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        angle = math.fabs(heading - yaw)
        if angle > math.pi/4:
            cwp = (cwp + 1) % len(self.waypoints)
        self.last_closest_waypoint = cwp
        return cwp
        
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
