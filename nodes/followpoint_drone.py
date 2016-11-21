#!/usr/bin/env python  
import roslib
roslib.load_manifest('rescuer_project')
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('followpoint_drone')

    br = tf.TransformBroadcaster()    

    listener = tf.TransformListener()
   
    command = rospy.Publisher('/tum_ardrone/com', String,queue_size=1)

    rospy.sleep(2.0)
    auto_start = String()
    auto_start.data = "c start"
    command.publish(auto_start)
    rospy.sleep(2.0)
    auto_start.data ="c autoInit 500 800"
    command.publish(auto_start)    
#==============================================================================
#     command.publish("c setReference $POSE$")
#     command.publish("c setMaxControl 0.1")
#     command.publish("c setStayWithinDist 0.3")
#     command.publish("c setStayTime 3")
#     command.publish("c lockScaleFP")
#==============================================================================
    rospy.sleep(1.0)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():

        br.sendTransform((-0.3, 0.0, 1.0),
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     "track_frame",
                     "base_footprint")
        rospy.sleep(0.2)
        try:
            (trans,rot) = listener.lookupTransform('/map', '/track_frame', rospy.Time(0))
            (r,p,y) =  tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        command.publish("c clearCommands")
        
        rospy.sleep(0.2)
        pose=String()
        pose.data = "c goto {0} {1} {2} {3}".format(trans[1],trans[0],trans[2],y)
        command.publish(pose)

        rate.sleep()
