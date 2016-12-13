#!/usr/bin/env python  
import roslib
roslib.load_manifest('rescuer_project')
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String

def mode_callback(data):
    global mode    
    mode=data.data #Assigns coupled or decoupled mode

if __name__ == '__main__':
    rospy.init_node('followpoint_drone')

    listener = tf.TransformListener()
   
    command = rospy.Publisher('/tum_ardrone/com', String,queue_size=1)
    
    #Define coupled (follow) or decoupled (autonomous) mode
    mode = "decoupled"
    rospy.Subscriber('/behaviour_mode', String, mode_callback)

    rospy.wait_for_service('/quadrotor/ardrone/togglecam')
    rospy.sleep(0.6)
    auto_start = String()
    auto_start.data = "c start"
    command.publish(auto_start)

    #auto_start.data ="c autoInit 500 800"
    #command.publish(auto_start)   
    #auto_start.data = "c setStayWithinDist 0.2"
    #command.publish(auto_start)
    #auto_start.data = "c setReference $POSE$"
    #command.publish(auto_start)
#==============================================================================
#     
#     command.publish("c setMaxControl 0.1")
#     command.publish("c setStayTime 3")
#     command.publish("c lockScaleFP")
#==============================================================================

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        if "de" not in mode:
            try:
                (trans,rot) = listener.lookupTransform('/quad_odom', '/track_frame', rospy.Time(0))
                (r,p,y) =  tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            command.publish("c clearCommands")
            
            rospy.sleep(0.2)
            pose=String()
            #pose.data = "c goto {0} {1} {2} {3}".format(trans[0],trans[1],trans[2]-0.5,y)
    
            #with turtlebot 1m offset in x and y	
            pose.data = "c goto " + "{0:.2f}".format(-trans[1])+ " {0:.2f}".format(trans[0]) + " {0:.2f}".format(trans[2]) + " {0:.2f}".format(y)
            print(pose.data)
            command.publish(pose)

        rate.sleep()
