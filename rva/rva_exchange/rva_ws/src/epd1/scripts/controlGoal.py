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
        
        robot_vel_topic = rospy.get_param('~robot_vel_topic', 'speed')
        robot_scan_topic = rospy.get_param('~robot_scan_topic', 'laser') 
        
        # -------------------------------------------------------------
        # Leer las velocidades máximas desde la parameter server
        self.max_lin_vel = rospy.get_param('~max_lin_vel', 0.2)  # m/s, valor por defecto: 0.2
        self.max_ang_vel = rospy.get_param('~max_ang_vel', 0.4)  # rad/s, valor por defecto: 0.4
        rospy.loginfo("Max Linear Velocity: %.2f, Max Angular Velocity: %.2f", self.max_lin_vel, self.max_ang_vel)
        # -------------------------------------------------------------
        
        self.goal = PointStamped()
        self.goal.header.frame_id = "odom"
        
        self.cmd_vel = rospy.Publisher(robot_vel_topic, Twist, queue_size=10)
        
        self.listener = tf.TransformListener()
        
        # subscription to the scan topic [sensor_msgs/LaserScan]
        rospy.Subscriber(robot_scan_topic, LaserScan, self.laserCallback)
        
        # subscription to the goal topic [geometry_msgs/PoseStamped]
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
     
        
    def command(self):
        
        base_goal = PointStamped()
        
        # we update the goal timestamp
        self.goal.header.stamp = rospy.Time()
      
        try:
            base_goal = self.listener.transformPoint('base_footprint', self.goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return
            
        # Extraer la posición del objetivo en coordenadas del robot
        x = base_goal.point.x
        y = base_goal.point.y
        
        # Calcular la distancia al objetivo
        distance = math.sqrt(x**2 + y**2)
        
        # Calcular el ángulo al objetivo
        angle_to_goal = math.atan2(y, x)
        
        # Definir ganancias para control proporcional
        K_linear = 0.5  # Ganancia para velocidad lineal
        K_angular = 1.0  # Ganancia para velocidad angular
        
        # Si la distancia es menor a un umbral, detener el robot
        if distance < 0.1:
            linear = 0.0
            angular = 0.0
        else:
            # Aplicar control proporcional
            linear = min(K_linear * distance, self.max_lin_vel)  # Limitar a la velocidad máxima
            angular = max(min(K_angular * angle_to_goal, self.max_ang_vel), -self.max_ang_vel)  # Limitar a [-max_ang_vel, max_ang_vel]
        
        rospy.loginfo(f"Distance: {distance}, Angle: {angle_to_goal}, Lin: {linear}, Ang: {angular}")
        
        self.publish(linear, angular)


    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # Copiar la velocidad lineal
        move_cmd.linear.x = lin_vel
        # Copiar la velocidad angular
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)
        
    def laserCallback(self, data):
        self.laser = data
        
        
    def goalCallback(self, goal):
        rospy.loginfo("¡Meta recibida! x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal.header = goal.header
        self.goal.point.x = goal.pose.position.x
        self.goal.point.y = goal.pose.position.y
        
        
    def shutdown(self):
        # Detener el robot
        rospy.loginfo("Deteniendo TurtleBot")
        # Un Twist por defecto tiene linear.x de 0 y angular.z de 0, por lo que detendrá al robot
        self.cmd_vel.publish(Twist())
        # Dormir para asegurarse que el comando de detención se recibe antes de apagar el script
        rospy.sleep(1)
 
 
if __name__ == '__main__':
    try:
        # Inicializar el nodo
        rospy.init_node('controlCollisionCheck', anonymous=False)

        # Indicar al usuario cómo detener el TurtleBot
        rospy.loginfo("Para detener TurtleBot presiona CTRL + C")

        robot = Turtlebot()
        # Qué función llamar cuando presionas ctrl + c    
        rospy.on_shutdown(robot.shutdown)

        goalx = float(sys.argv[1])
        goaly = float(sys.argv[2])

        rospy.loginfo(f"Meta recibida: x={goalx}, y={goaly}")

        # TurtleBot se detendrá si no le seguimos diciendo que se mueva. ¿Cada cuánto debemos decirle que se mueva? 10 Hz
        r = rospy.Rate(10)

        # Mientras no se haya presionado ctrl + c, seguir haciendo...
        while not rospy.is_shutdown():
            # publicar la velocidad
            robot.command()
            # Esperar 0.1 segundos (10 Hz) y publicar nuevamente
            r.sleep()

    except:
        rospy.loginfo("El nodo controlCollisionCheck ha terminado.")
