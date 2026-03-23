#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

""" A ROS planning node that subscribes to a costmap
  and generates a path by using the Djikstra algorithm"""

import sys
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from myplanner import Planner

class PlannerNode:
    def __init__(self):
        
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")
        self.update_map = rospy.get_param("~update_map", False)
        self.listener = tf.TransformListener()

        self.map = None
        self.planner = None
        self.goal = None
        
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10, latch=True)
        rospy.Subscriber('costmap_2d/costmap/costmap', OccupancyGrid, self.map_callback)
        rospy.Subscriber('goal', PoseStamped, self.goal_callback) 


    def map_callback(self, map):
        self.map = map
        if self.planner is None or self.update_map == True:
            rospy.loginfo("map received!")
            self.planner = Planner(self.map)


            
    def goal_callback(self, goal):
        rospy.loginfo("Goal received! x: %.2f, y:%.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal = [goal.pose.position.x, goal.pose.position.y]

        # First, get the robot position in the map 
        base_pos = PointStamped()
        base_pos.header.frame_id = self.base_frame_id
        base_pos.header.stamp = rospy.Time()
        initial_pos = None
        try:
            initial_pos = self.listener.transformPoint(self.global_frame_id, base_pos)
        except:
            rospy.logerr("Error getting robot position in map!")
            initial_pos = None

        if initial_pos is not None:
            ix = initial_pos.point.x
            iy = initial_pos.point.y
            gx = self.goal[0]
            gy = self.goal[1] 
            rospy.loginfo("Computing path...")
            self.compute_path(ix, iy, gx, gy)



    def compute_path(self, ix, iy, gx, gy):

        # if the planner has an updated map, let's plan!
        if self.planner is not None:
            path = self.planner.plan(ix, iy, gx, gy)

            if path is not None:
                xpath = path[0]
                ypath = path[1]
            
                path_msg = Path()
                path_msg.header.stamp = rospy.Time.now()
                path_msg.header.frame_id = self.global_frame_id
                path_msg.header.seq = 0

                for i in range(len(xpath)):
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = xpath[i]
                    pose.pose.position.y = ypath[i]
                    pose.pose.position.z = 0
                    path_msg.poses.append(pose)

                rospy.loginfo("Publishing computed path!")
                self.path_publisher.publish(path_msg)
            else:
                rospy.logerr("Error computing path!")
  
        
if __name__ == '__main__':
    #try:
        # initiliaze
        rospy.init_node('path_planner', anonymous=False)

        # Tell the user we are waiting for a costmap in order to operate
        rospy.loginfo("Starting planning node. Waiting for valid map. Press CTRL+C to exit")

        planner = PlannerNode()     

        # The planning node will wait for new goals and the map at this rate
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            r.sleep()
    #except:
    #    rospy.loginfo("Planning node terminated.")