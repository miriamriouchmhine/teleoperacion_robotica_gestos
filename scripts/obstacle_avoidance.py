#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('obstacle_avoidance')

        # Suscriptor para la nube de puntos de los obstáculos captados por el lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Suscriptor para comandos de control Ackermann
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publicador para comandos Ackermann modificados
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)
        # Almacenar el último mensaje Ackermann recibido
        self.last_ackermann_cmd = AckermannDrive()
        self.obstacle_detected = False

    def obstacle_callback(self, msg):
        # TODO Procesar la nube de puntos con los obstáculos teniendo en cuenta el último mensaje de movimiento recibido para evitar colisiones
        points = list(pc2.read_points(msg, skip_nans=True))
        self.obstacle_detected = self.check_for_obstacles(points)

        # Crear y enviar mensaje de ackermann
        self.send_ackermann_command()        
        return

    def check_for_obstacles(self, points):
        # Verificar si hay obstáculos en la trayectoria del robot
        for point in points:
            x, y, z = point[:3]
            if -0.5 < y < 0.5 and 0 < x < 1.0:
                rospy.loginfo(f"Obstacle detected at x: {x}, y: {y}")
                return True
        return False
    
    def send_ackermann_command(self):
        # Modifica el comando Ackermann para evitar obstáculos
        cmd = AckermannDrive()
        cmd = self.last_ackermann_cmd
        if self.obstacle_detected:
            cmd.speed = 0.0  # Detener el robot si se detecta un obstáculo
            cmd.steering_angle = 0.0
            rospy.loginfo("Obstacle detected! Stopping the robot.")
        else:
            rospy.loginfo("Path clear. Proceeding with the given command.")

        rospy.loginfo(f"Publishing command: speed={cmd.speed}, steering_angle={cmd.steering_angle}")
        self.cmd_pub.publish(cmd)

    def ackermann_callback(self, msg):
        # Almacena el último comando recibido
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self):
        # Modifica el comando Ackermann para evitar obstáculos (Se puede modificar si es necesario)
        cmd = self.last_ackermann_cmd
        cmd.drive.speed = 0.0  # Reduce la velocidad
        cmd.drive.steering_angle = 0.0 # Cambia el ángulo de dirección
        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
