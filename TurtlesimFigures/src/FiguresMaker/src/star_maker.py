#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
#Creamos la clase
class MoveTurtlesim():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("StarTurtlesim")
        #Creamos el publisher
        self.pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)        
        
        self.subPose = rospy.Subscriber("turtle1/pose",Pose,self.odom_callback)

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(100)
        self.msg = Twist()

        self.current_pose = None

        self.target_index = 0

        self.targets = [[2,2],[5,3.5],[8,2],[6.5,5],[9,7],[6,6.5],[5,9],[4,6.5],[1,7],[3.5,5],[2,2]]

        self.current_target = self.targets[self.target_index]
        
        self.state = "pointingTowardsGoal"

        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)

    def odom_callback(self, odom):
        self.current_pose = odom


    def move(self,v_lin,v_ang):

        self.msg.linear.x = v_lin
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = v_ang
        #Publicamos la velocidad
        self.pub.publish(self.msg)

    def end_callback(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        #Publicamos las velocidades
        self.pub.publish(self.msg)



if __name__ == "__main__":
    #iniciamos la clase
    mov = MoveTurtlesim()
    #mientras este corriendo el nodo movemos el carro el circulo
    Kpw = 5.0
    Kpv = 5.0
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if mov.current_pose != None:
            #print("None condition passed")
            theta_target = np.arctan2(mov.current_target[1]-mov.current_pose.y, mov.current_target[0]-mov.current_pose.x)
            e_theta = theta_target - mov.current_pose.theta
            e_pos = np.sqrt((mov.current_target[0]-mov.current_pose.x)**2 + (mov.current_target[1]-mov.current_pose.y)**2)
            
            if mov.state == "pointingTowardsGoal":
                w = Kpw*e_theta
                v = 0.0          
                if w >= 2:
                    w = 1.9
                elif w <= -2:
                    w = -1.9               

                mov.move(v,w)

            if mov.state == "travelingTowardsGoal":
                w = Kpw*e_theta
                v = Kpv*e_pos

                           
                if w >= 2:
                    w = 1.9
                elif w <= -2:
                    w = -1.9
                if v >= 2:
                    v = 1.9
                elif v <= -2:
                    v = -1.9               
                
                mov.move(v,w)
        
            if (abs(e_theta) <= 0.025) and mov.state == "pointingTowardsGoal":

                mov.state = "travelingTowardsGoal"
                    
            if (abs(e_theta) <= 0.025) and (abs(e_pos) <= 0.02 and mov.state == "travelingTowardsGoal"):
                mov.move(0.0,0.0)
                print("============ target {target_num} reached, pose based in odometry is: =============".format(target_num = mov.target_index))
                print(mov.current_pose)
                if mov.target_index == len(mov.targets)-1:
                    mov.state = "goalReached"
                else:                    
                    mov.target_index += 1
                    mov.current_target = mov.targets[mov.target_index]
                    mov.state = "pointingTowardsGoal"
                
            if mov.state == "goalReached":
                mov.move(0.0,0.0)
                mov.end_callback()
                
        mov.rate.sleep()