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

def mode_callback(data):
    global mode    
    mode=data.data #Assigns coupled or decoupled mode
 
def blob_detect_callback(data):
    global detected
    if "de" not in mode:
        if data.blob_count > 0:        
            if data.blobs[0].name == 'Black':
                if not detected:  
                    detected = True	  
                    #Stop controller
                    msg = String()
                    msg.data = "c stop"
                    comm_Pub.publish(msg)            
                       	     
                #print ('detected')
                vel = geometry_msgs.msg.Twist()
                if data.image_height/2 > data.blobs[0].y:
                    vel.linear.x = default_vel
                    #print('Forward')                                
                else:
                    vel.linear.x = -default_vel
                    #print('Backward')  
                if data.image_width/2 > data.blobs[0].x:
                    vel.linear.y = default_vel 
                    #print('Right')                                
                else:
                    vel.linear.y = -default_vel
                    #print('Left') 
                vel_Pub.publish(vel)
        else:
            if detected:
                detected = False
                msg = String()
                msg.data = "c start"
                comm_Pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('quad_blob_tracker')

    default_vel = 0.2

    #Change to bottom camera for the drone
    rospy.wait_for_service('/quadrotor/ardrone/togglecam')
    try:
        toggle_cam = rospy.ServiceProxy('/quadrotor/ardrone/togglecam', std_srvs.srv.Empty)
        toggle_cam()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    
    
    
    #Take off with the drone
    quad_takeOffPub = rospy.Publisher('/ardrone/takeoff', Empty,queue_size=1)    
    rospy.sleep(0.6)
    quad_takeOff = Empty()
    quad_takeOffPub.publish(quad_takeOff)
  
    #Subscribe to blob detector
    detected = False
    rospy.Subscriber('/blobs', Blobs, blob_detect_callback)
    
    
    #Define coupled (follow) or decoupled (autonomous) mode
    mode = "decoupled"
    rospy.Subscriber('/behaviour_mode', String, mode_callback)
    
    #Publish from Drone
    quad_Pub = rospy.Publisher('/ask_for_point', String,queue_size=1)    
    vel_Pub = rospy.Publisher('/quadrotor/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    comm_Pub = rospy.Publisher('/tum_ardrone/com', String,queue_size=1)

    rospy.sleep(0.5)
    auto_start = String()
    auto_start.data = "c start"
    comm_Pub.publish(auto_start)
    
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        #Check if there is a blob detected, if not ask for a point
        if not detected and "de" not in mode:
            quad_msg = String()
            quad_msg.data = "No tag detected"
            quad_Pub.publish(quad_msg)            
            
        rate.sleep()
