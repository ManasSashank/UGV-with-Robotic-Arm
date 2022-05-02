#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import math

class Proportional:
    

    def __init__(self, x, y):

        

        print("Initializing Node")

        # Initializes the Node.
        

        # Initializes a publihser within the node
        self.controlPub = rospy.Publisher("/robot_base_velocity_controller/cmd_vel", Twist, queue_size=10)

        self.odomSub = rospy.Subscriber("/robot_base_velocity_controller/odom", Odometry, self.control)

        # K gain for angle
        self.ka = 0.7

        # K gain for velocity
        self.kx = 0.5

        self.goal = [x, y]

    def control(self, msg):

        error_x = msg.pose.pose.position.x - self.goal[0]
        error_y = msg.pose.pose.position.y - self.goal[1]

        quat = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        error_theta = math.atan2(self.goal[1], self.goal[0]) - yaw

        error_linear = math.sqrt(error_x**2 + error_y**2)
        msg_ = Twist()

        if(abs(error_theta) > 0.1):
            msg_.angular.z = self.ka*error_theta
            msg_.linear.x = 0

        else:
            if(abs(error_linear) > 0.2):
                msg_.linear.x = self.kx*error_linear
                msg_.angular.z = 0
            
            else:
                msg_.linear.x = 0
                msg_.angular.z = 0

        self.controlPub.publish(msg_)



if __name__ == '__main__':
    
    rospy.init_node("publisher_node")
    p = Proportional(1,1)
    rospy.spin()