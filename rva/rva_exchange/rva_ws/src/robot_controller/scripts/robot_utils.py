#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry


class Utils():
    
    def __init__(self, tf, odom_topic):
        
        self.listener = tf
        # Subscription to the odometry topic
        rospy.Subscriber(odom_topic, Odometry, self.odomCallback)
        self.odom = Odometry()
        
     
    def odomCallback(self, msg):
        """
        Callback to receive and store the odometry msgs
        """
        self.odom = msg
        

    def getOdom(self):
        """
        return the current odometry msgs
        This method can be called from our robot controller
        to obtain the robot position and velocity
        in the odometry frame
        """
        return self.odom   

    
    
    def computeNewXYPositions(self, xi, yi, vx, vy, theta, dt):
        """
        Compute x and y positions based on velocity
        xi The current x position (m)
        yi The current y position (m)
        vx The current x velocity (m/s)
        vy The current y velocity (m/s)
        theta The current orientation (rad)
        dt The timestep to take (secs)
        Return The new x and y positions (m)
        """
        newx = xi + (vx * math.cos(theta) + vy * math.cos(math.pi/2.0 + theta)) * dt
        newy = yi + (vx * math.sin(theta) + vy * math.sin(math.pi/2.0 + theta)) * dt
        return newx, newy
    
    
    def computeNewThetaPosition(self, thetai, vth, dt):
        """
        Compute orientation based on velocity
        thetai The current orientation (rad)
        vth The current theta velocity (rad/s)
        dt The timestep to take (s)
        The new orientation (rad)
        """
        return thetai + vth * dt
  


    def computeNewVelocity(self, vg, vi, a_max, dt): 
        """
        Compute velocity based on acceleration
        vg The desired velocity, what we're accelerating up to (m/s)
        vi The current velocity (m/s)
        a_max An acceleration limit (m/s^2)
        dt The timestep to take (s)
        return The new velocity (m/s)
        """
        if ((vg - vi) >= 0):
            return min(vg, vi + a_max * dt)
        return max(vg, vi - a_max * dt)
  

    def transformPose(self, pose, to_frame):
        """
        Transform a PoseStamped to given frame
        pose poseStamped to be transformed
        to_frame desired frame
        return the transformed PoseStamped
        """
        pose.header.stamp = rospy.Time()
        pose_trans = PoseStamped()
        try:
            pose_trans = self.listener.transformPose(to_frame, pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Problem Transforming pose to frame %s", to_frame)
            
        return pose_trans
    

    def transformPoint(self, point, to_frame):
        """
        Transform a PointStamped to given frame
        point pointStamped to be transformed
        to_frame desired frame
        return the transformed PoseStamped
        """
        point.header.stamp = rospy.Time()
        point_trans = PointStamped()
        try:
            point_trans = self.listener.transformPoint(to_frame, point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Problem Transforming point to frame %s", to_frame)
            
        return point_trans