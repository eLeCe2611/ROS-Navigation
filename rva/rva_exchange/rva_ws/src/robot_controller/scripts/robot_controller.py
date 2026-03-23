#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from robot_utils import Utils
import numpy as np


class TurtlebotController():
    
    def __init__(self, rate):
        
        robot_vel_topic = rospy.get_param('~robot_vel_topic', 'speed')
        robot_scan_topic = rospy.get_param('~robot_scan_topic', 'laser') 
        
        # read the maximum linear and angular velocities
        # from the parameter server!!!!
        self.max_lin_vel = rospy.get_param('~max_lin_vel', 0.2)  # m/s
        self.max_ang_vel = rospy.get_param('~max_ang_vel', 0.4)  # rad/s

        self.r_soi = 1 # Sphere of influence radius
        
        self.rate = rate # Hz  (1/Hz = secs)
        # we store the received path here
        self.path = Path()
        self.path_received = False
        self.laser = LaserScan()
        self.laser_received = False
        self.is_collision = False
        
        # Declare the velocity command publisher
        self.cmd_vel = rospy.Publisher(robot_vel_topic, Twist, queue_size=10)
        
        self.listener = tf.TransformListener()

        self.utils = Utils(self.listener, "odom")
        
        # subscription to the scan topic [sensor_msgs/LaserScan]
        rospy.Subscriber(robot_scan_topic, LaserScan, self.laserCallback)
        
        # subscription to the path topic [nav_msgs/Path]
        rospy.Subscriber("path", Path, self.pathCallback)
     
        
    def command(self):

        # TODO: check if the final goal has been reached
        if(self.goalReached()==True):
            rospy.loginfo("GOAL REACHED! Chilling...")
            self.publish(0.0, 0.0)
            return True
                
        # Determine the local path point to be reached
        # TODO: fill the method getSubGoal
        current_goal = self.getSubGoal() 
        if not current_goal:
            self.publish(0.0, 0.0)
            return False
            
        # TODO: use current_goal 
        # Put your control law here (copy from EPD1)
        angular = 0.0
        linear = 0.0
        min_d = 1.0
        angle_range_difference = False

        # Normalize directionection towards goal
        current_goal = np.array([current_goal.pose.position.x, current_goal.pose.position.y])
        current_distance = np.linalg.norm(current_goal)
        current_goal_normalized = current_goal / current_distance

        # Calculate linear and angular velocities

        target = np.array([current_goal_normalized[0], current_goal_normalized[1]])
        angle = math.atan2(current_goal_normalized[1], current_goal_normalized[0])

        allowed_angle = 15
        allowed_angle = allowed_angle * math.pi / 180

        angular = (angle / abs(angle)) * self.max_ang_vel
        if abs(angle) <= allowed_angle * 1.25:
            angular = angular * 0.25
             
        speed_factor = 1
        if min_d < self.r_soi:
            speed_factor = (min_d / self.r_soi) * 0.5 + 0.5
        if angle_range_difference:
            speed_factor = 0.33
        if abs(angle) <= allowed_angle:
            linear = self.max_lin_vel * speed_factor
            linear = min(linear, self.max_lin_vel)

        # check the maximum speed values allowed

        linear, angular = self.checkMaximumSpeedValues(linear,angular,self.max_lin_vel,self.max_ang_vel)

        # If the computed commands does not provoke a collision,
        # send the commands to the robot
        # TODO: fill the checkCollision function
        if(self.checkCollision(linear, angular)==False):
            self.publish(linear,angular)
            return False

        # if a possible collision is detected,
        # try to find an alternative command to
        # avoid the collision
        # TODO: fill the CollisionAvoidance function
        linear, angular = self.collisionAvoidance() 

        # check the maximum speed values allowed
        linear, angular = self.checkMaximumSpeedValues(linear,angular,self.max_lin_vel,self.max_ang_vel)

        self.publish(linear,angular)
        return False
    
    def checkMaximumSpeedValues(self,linear,angular,max_lin_vel,max_ang_vel):

        if(abs(angular) > max_ang_vel):
            if(angular < 0):
                angular = -max_ang_vel
            else:
                angular = max_ang_vel
        if(linear > max_lin_vel):
            linear = max_lin_vel

        return linear,angular

    
    def goalReached(self):
        # -------------------------------------------------------------
        # TODO: use the last point of the path to check if the robot
        # has reached the final goal (the robot is in a close position).
        # return True if the FINAL goal was reached, False otherwise
        # -------------------------------------------------------------
        if len(self.path.poses) == 0:
            return False

        # Get goal position
        final_goal = self.path.poses[-1]
        final_goal_rf = self.utils.transformPose(final_goal, 'base_footprint')

        # Calculate current_distance
        final_goal = np.array([final_goal_rf.pose.position.x, final_goal_rf.pose.position.y])
        current_distance = np.linalg.norm(final_goal)

        if current_distance > 0 and current_distance < 0.15:
            return True

        return False


    def getSubGoal(self):
        # -------------------------------------------------------------
        # TODO: use self.path.poses to find the subgoal to be reach
        # You could transform the path points to the robot reference
        # to find the closest point:
        # path_pose = self.path.poses[index]
        # path_pose_rf = self.utils.transformPose(path_pose, 'base_footprint')

        # Return False in case that path has not loaded
        if len(self.path.poses) == 0:
            return False

        # Define lookahead in the path positions
        lookahead = 2

        # Calculate index of point with lowest current_distance to the robot
        current_distance = math.inf
        index = -1
        for i, point in enumerate(self.path.poses):
            # Calculate current_distance for point
            point_rf = self.utils.transformPose(point, 'base_footprint')
            point_rf = np.array([point_rf.pose.position.x, point_rf.pose.position.y])
            distance_to_goal = np.linalg.norm(point_rf)

            if distance_to_goal < current_distance:
                index = i
                current_distance = distance_to_goal

        # Get index and point according to the lowest current_distance + lookahead
        index = min(index + lookahead, len(self.path.poses) - 1)
        subgoal_pose = self.path.poses[index]
        subgoal = self.utils.transformPose(subgoal_pose, 'base_footprint')
        # -------------------------------------------------------------
        return subgoal


    def checkCollision(self, linear, angular):
        # -------------------------------------------------------------
        # TODO: use self.laser to check possible collisions
        # Optionally, you can also use the velocity commands
        # return True if possible collision, False otherwise
        
        # When laser data has not loaded, return False
        ## -------------------------------------------------------------
        if not self.laser:
            rospy.loginfo("No information laser received")
            return False
        if len(self.laser.ranges) == 0:
            rospy.loginfo("No information laser received")
            return False

        if np.min(self.laser.ranges) > self.r_soi + 0.12:
            self.is_collision = False
            return False

        if self.is_collision or np.min(self.laser.ranges) < self.r_soi:
            self.is_collision = True
            return True

        
        return False
        


    def collisionAvoidance(self):
        # -------------------------------------------------------------
        # TODO: try to find an alternative command to avoid the collision
        # Here you must try to implement one of the reactive methods
        # seen in T4: bug algorithm, potential fields, velocity obstacles,
        # Dynamic Window Approach, others...
        # Feel free to add the new variables and methods that you may need 

        index = np.argmin(self.laser.ranges)
        angle = (self.laser.angle_min + index * self.laser.angle_increment)
        min_d = min(np.min(self.laser.ranges), 1)

        # Calculate nearest obstacle
        x = self.laser.ranges[index] * math.cos(angle)
        y = self.laser.ranges[index] * math.sin(angle)

        repulsive_force = np.array([-x, -y])

        current_goal_pose = self.getSubGoal()
        goal_direction = np.array([current_goal_pose.pose.position.x, current_goal_pose.pose.position.y])
        goal_direction_normalized = (goal_direction / np.linalg.norm(goal_direction))

        goal_force = goal_direction_normalized
        c = 0
        if min_d <= self.r_soi:
            c = (self.r_soi - min_d) / min_d
        repulsive_force_normalized = repulsive_force / np.linalg.norm(repulsive_force)
        potential_field_force = (repulsive_force_normalized) * c

        goal_w = 4
        potential_field_w = 1.25
        target_force = goal_force * goal_w + potential_field_force * potential_field_w

        allowed_angle = 45
        allowed_angle = allowed_angle * math.pi / 180
        target_force_normalized = target_force / np.linalg.norm(target_force)
        angle_range_difference = abs(np.arccos(np.clip(np.dot(goal_direction_normalized, target_force_normalized), -1, 1))) > allowed_angle

        angular = 0.0
        linear = 0.0
        
        target = np.array([target_force[0], target_force[1]])
        angle = math.atan2(target_force[1], target_force[0])

        allowed_angle = 15
        allowed_angle = allowed_angle * math.pi / 180

        angular = (angle / abs(angle)) * self.max_ang_vel
        if abs(angle) <= allowed_angle * 1.25:
            angular = angular * 0.25
             
        speed_factor = 1
        if min_d < self.r_soi:
            speed_factor = (min_d / self.r_soi) * 0.5 + 0.5
        if angle_range_difference:
            speed_factor = 0.33
        if abs(angle) <= allowed_angle:
            linear = self.max_lin_vel * speed_factor
            linear = min(linear, self.max_lin_vel)

        lin_vel = linear
        ang_vel = angular
        
        # -------------------------------------------------------------
        return lin_vel, ang_vel

        
    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # Copy the forward velocity
        move_cmd.linear.x = lin_vel
        # Copy the angular velocity
        move_cmd.angular.z = ang_vel
        #rospy.loginfo("Commanding lv: %.2f, av: %.2f", lin_vel, ang_vel)
        self.cmd_vel.publish(move_cmd)


    def laserCallback(self,data):
        self.laser = data
        self.laser_received = True
        #rospy.loginfo("Laser received " + str(len(data.ranges)))
        

    def pathCallback(self,path):
        self.path = path
        self.path_received = True
        

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
 
if __name__ == '__main__':
    #try:
        # initiliaze
        rospy.init_node('TurtlebotController', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        rate = 10 # Frecuency (Hz) for commanding the robot
        robot=TurtlebotController(rate)
        # What function to call when you ctrl + c    
        rospy.on_shutdown(robot.shutdown)
        
        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(rate)
        end = False
        # as long as you haven't ctrl + c keeping doing...
        while not (rospy.is_shutdown() or end==True):
            #rospy.loginfo("Loop")
	        # publish the velocity
            robot.command()
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    #except:
    #    rospy.loginfo("TurtlebotController node terminated.")
        
