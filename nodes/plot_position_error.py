#!/usr/bin/env python  
import roslib
roslib.load_manifest('rescuer_project')
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String
import tum_ardrone.msg
import nav_msgs.msg

def quad_callback(data):
    global quad_received, quad_pos
    quad_pos.x = data.y
    quad_pos.y = -data.x
    quad_received = True
    
def base_callback(data):
    global base_received, base_pos
    base_pos.x = data.x
    base_pos.y = data.y
    base_received = True

if __name__ == '__main__':
    rospy.init_node('plot_position_error')

    error = rospy.Publisher('/position_error', geometry_msgs.msg.Pose2D,queue_size=1)
    rospy.Subscriber('/ardrone/predictedPose', tum_ardrone.msg.filter_state, quad_callback)    
    rospy.Subscriber('/mobile_base/abs_pos', geometry_msgs.msg.Pose2D, base_callback)    
    
    quad_received = False  
    quad_pos = geometry_msgs.msg.Pose2D()
    base_received = False
    base_pos = geometry_msgs.msg.Pose2D()
    
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        if quad_received and base_received:
            error_pos = geometry_msgs.msg.Pose2D()            
            error_pos.x = -quad_pos.x + base_pos.x + 1.0
            error_pos.y = -quad_pos.y + base_pos.y + 1.0            
            error.publish(error_pos)
            quad_received = False
            base_received = False
        rate.sleep()
