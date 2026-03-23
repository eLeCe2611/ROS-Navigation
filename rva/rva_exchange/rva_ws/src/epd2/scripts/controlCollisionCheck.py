#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan

class Turtlebot():
    
    def __init__(self):
        # Obtener parámetros desde el server
        self.max_lin_vel = rospy.get_param('~max_linear_velocity', 0.5)  # m/s
        self.max_ang_vel = rospy.get_param('~max_angular_velocity', 1.0)  # rad/s
        self.angle_threshold = rospy.get_param('~angle_threshold', 0.2)  # Rango en radianes para moverse recto
        
        rospy.loginfo("Max Linear Velocity: %.2f, Max Angular Velocity: %.2f", self.max_lin_vel, self.max_ang_vel)

        # Suscripciones y publicadores
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.listener = tf.TransformListener()

        rospy.Subscriber("scan", LaserScan, self.laserCallback)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        
        self.goal = PointStamped()
        self.goal.header.frame_id = "odom"

    def command(self):
        base_goal = PointStamped()
        self.goal.header.stamp = rospy.Time()

        try:
            base_goal = self.listener.transformPoint('base_footprint', self.goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            print()
            return

        # Obtener posición del objetivo en coordenadas del robot
        x = base_goal.point.x
        y = base_goal.point.y
        distance = math.sqrt(x**2 + y**2)
        angle_to_goal = math.atan2(y, x)

        # Comprobar si ya llegó al punto antes de asignar velocidades
        if distance < 0.1:
            rospy.loginfo("Goal reached! Stopping robot.")
            self.publish(0.0, 0.0)
            return

        # Control proporcional
        K_linear = 0.5
        K_angular = 1.0

        # Ajustar velocidad angular primero antes de avanzar demasiado rápido
        if abs(angle_to_goal) > self.angle_threshold:
            linear = min(self.max_lin_vel * 0.5, 0.1)  # Reducimos la velocidad lineal mientras giramos
            angular = max(min(K_angular * angle_to_goal, self.max_ang_vel), -self.max_ang_vel)
        else:
            linear = self.max_lin_vel
            angular = 0.0
            
        if(self.checkCollision()):
            rospy.loginfo("possible collision! Stopping!!!!")
            linear = 0.0
            angular = 0.0

        rospy.loginfo("Distance: %.2f, Angle: %.2f, Lin: %.2f, Ang: %.2f", distance, angle_to_goal, linear, angular)
        self.publish(linear, angular)


    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)
        
    def checkCollision(self):
        # Verificar si los datos del láser están disponibles
        if not hasattr(self, 'laser'):
            rospy.logwarn("Laser data not received yet.")
            return False

        # Obtener los datos del láser
        laser_data = self.laser

        # Umbral de distancia para colisión (en metros)
        collision_threshold = 0.25

        # Revisar si alguna de las distancias del láser es menor que el umbral de colisión
        for distance in laser_data.ranges:
            if distance < collision_threshold and not math.isnan(distance):
                # Si la distancia es menor que el umbral y no es NaN, hay una posible colisión
                rospy.loginfo("Collision detected! Distance: %.2f", distance)
                return True

        # Si no se detectó ninguna colisión
        return False

    def laserCallback(self, data):
        self.laser = data

    def goalCallback(self, goal):
        rospy.loginfo("Goal received! x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal.header = goal.header
        self.goal.point.x = goal.pose.position.x
        self.goal.point.y = goal.pose.position.y

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('controlCollisionCheck', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL + C")
        robot = Turtlebot()
        rospy.on_shutdown(robot.shutdown)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            robot.command()
            r.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("ControlCollisionCheck node terminated.")
