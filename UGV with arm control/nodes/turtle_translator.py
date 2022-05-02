#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import UInt16MultiArray
reset = False
def motor(lin, ang):
    Speed_Val = 80
    M_C = [Speed_Val, 0, 0, 0, 0]
    if lin == 2 and ang == 0:
        M_C[1] = 1
        M_C[2] = 0
        M_C[3] = 1
        M_C[4] = 0
        return M_C
    elif lin == -2 and ang == 0:
        M_C[1] = 0
        M_C[2] = 1
        M_C[3] = 0
        M_C[4] = 1
        return M_C
    elif ang == 2 and lin == 0:
        M_C[1] = 0
        M_C[2] = 1
        M_C[3] = 1
        M_C[4] = 0
        return M_C
    elif ang == -2 and lin == 0:
        M_C[1] = 1
        M_C[2] = 0
        M_C[3] = 0
        M_C[4] = 1
        return M_C
    elif lin == 0 and ang == 0:
        return M_C
        
def callback1(turtle):
    
    Motor_Data = UInt16MultiArray()
    Motor_Data.data = motor(turtle.linear.x, turtle.angular.z)
    pub.publish(Motor_Data)
    global reset
    reset = True

def callback2(pose):

    lin_vel = pose.linear_velocity
    ang_vel = pose.angular_velocity
    halt = UInt16MultiArray()
    global reset
    if reset:
        if lin_vel == 0 and ang_vel == 0 :
            halt.data = motor(lin_vel, ang_vel)
            pub.publish(halt)
            reset = False

def listen():

    sub1 = rospy.Subscriber('/turtle1/cmd_vel', Twist, callback1)
    sub2 = rospy.Subscriber('/turtle1/pose', Pose, callback2)
    rospy.spin()

if __name__ == '__main__':


    rospy.init_node('Turtle2GuiNE')

    pub = rospy.Publisher('/GuiNE_Bot/Motor', UInt16MultiArray, queue_size = 10)

    try:
        
        listen()

    except rospy.ROSInterruptException:
        pass
    
