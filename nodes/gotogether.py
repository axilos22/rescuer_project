#!/usr/bin/env python  
import roslib
roslib.load_manifest('rescuer_project')
import rospy
import math
import tf
import geometry_msgs.msg
import std_srvs.srv
from std_msgs.msg import String, Empty
from cmvision.msg import Blobs, Blob

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

def comm_callback(data):   
    try:
        (trans,rot) = listener.lookupTransform('/quad_odom', '/track_frame', rospy.Time(0))
        (r,p,y) =  tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
        
        comm_Pub.publish("c clearCommands")
        
        rospy.sleep(0.6)
        pose=String()
        pose.data = "c goto " + "{0:.2f}".format(-trans[1])+ " {0:.2f}".format(trans[0]) + " {0:.2f}".format(trans[2]) + " {0:.2f}".format(y)
        comm_Pub.publish(pose)        
        print(pose.data)               
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Error during tf")
        

if __name__ == '__main__':
    rospy.init_node('gotogether')

#==============================================================================
#     #Define goal for mobile base
#     base_goalPub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped,queue_size=1)    
#     rospy.sleep(0.6)
#     base_goal = geometry_msgs.msg.PoseStamped()
#     base_goal.header.frame_id = "/mobile_map";
#     base_goal.header.stamp = rospy.Time.now();
#     base_goal.pose.position.x = -1.0
#     base_goal.pose.position.y = -4.0
#     base_goal.pose.position.z = 0.0
#     base_goal.pose.orientation.x = 0.0
#     base_goal.pose.orientation.y = 0.0
#     base_goal.pose.orientation.z = 0.0
#     base_goal.pose.orientation.w = 1.0
#     base_goalPub.publish(base_goal)
#==============================================================================
    
    #Subscribe from Turtlebot
    rospy.Subscriber('/turtlebot_quad', String, comm_callback)  
    
    listener = tf.TransformListener()
    
    comm_Pub = rospy.Publisher('/tum_ardrone/com', String,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
             
        rate.sleep()
