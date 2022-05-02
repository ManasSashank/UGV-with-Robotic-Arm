#!/usr/bin/python3
from __future__ import print_function 
import rospy 
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal 
from std_msgs.msg import Float64 
from trajectory_msgs.msg import JointTrajectoryPoint 
 
 
def move_robot_arm(joint_values):
  
  
  arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
  arm_client.wait_for_server()
  arm_goal = FollowJointTrajectoryGoal()
  arm_goal.trajectory.joint_names = ['j0', 'j1','j2']
  point = JointTrajectoryPoint()
  point.positions = joint_values
  point.time_from_start = rospy.Duration(5)
  arm_goal.trajectory.points.append(point)
  exec_timeout = rospy.Duration(10)
  prmpt_timeout = rospy.Duration(5)
  arm_client.send_goal_and_wait(arm_goal, exec_timeout, prmpt_timeout)
 
 
if __name__ == '__main__':

  try:

    rospy.init_node('send_goal_to_arm_py')
    l=[]
    # Move the joints of the robot arm to the desired angles in radians
    print("Input in radian")
    for i in range(2):
        i=float(input())
        l.append(i)

    move_robot_arm(l)
    print("Robotic arm has successfully reached the goal!")
    l.clear()
    move_robot_arm([0,0,0]) #move back goal to initial position
    
     
  except rospy.ROSInterruptException:
    print("Program interrupted before completion.", file=sys.stderr)