#!/usr/bin/env python  
import roslib
roslib.load_manifest('rescuer_project')
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String

#==============================================================================
# 
# Controller
#     Define goal point -> Start
#     Send 2D goal to turtlebot
#     Take off drone
# Quadrotor
#     Start with tracking enabled
#     If no tag is received ask turtlebot for position
#     Follow given position until tag is detected
# Turtlebot
#     Go to defined goal
#     Listen for quadrotor request
#     If received publish 3D tracking point for quadrotor
#     When reach final point, if drone tracking tag -> command drone to land
#
#==============================================================================
    


if __name__ == '__main__':
    rospy.init_node('gotogether')

    base_goalPub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped,queue_size=1)    
    rospy.sleep(0.2)
    base_goal = geometry_msgs.msg.PoseStamped()
    base_goal.header.frame_id = "/map";
    base_goal.header.stamp = Time.now();
    base_goal.pose.position.x = 2.0
    base_goal.pose.position.y = 0.5
    base_goal.pose.position.z = 0.0
    base_goal.pose.orientation.x = 0.0
    base_goal.pose.orientation.y = 0.0
    base_goal.pose.orientation.z = 0.0
    base_goal.pose.orientation.w = 1.0
    base_goalPub.publish(base_goal)
    
    quad_takeOffPub = rospy.Publisher('/ardrone/takeoff', std_msgs.msg.Empty,queue_size=1)    
    rospy.sleep(0.2)
    quad_takeOff = std_msgs.msg.Empty()
    quad_takeOffPub.publish()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        rate.sleep()
