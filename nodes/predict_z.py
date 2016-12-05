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

def pose_callback(data):
    new_msg = data    
    new_msg.z = real_z
    command.publish(new_msg)
    
def nav_callback(data):
    global real_z
    real_z = data.pose.pose.position.z
    print("Real z= " + str(real_z))    

if __name__ == '__main__':
    rospy.init_node('predict_z')

    command = rospy.Publisher('/ardrone/predictedPose', tum_ardrone.msg.filter_state,queue_size=1)
    rospy.Subscriber('/tum_ardrone/predictedPose', tum_ardrone.msg.filter_state, pose_callback)
    rospy.Subscriber('/quadrotor/ground_truth/state', nav_msgs.msg.Odometry, nav_callback)

    real_z = 1.0
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        rate.sleep()
