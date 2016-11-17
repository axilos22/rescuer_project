#!/usr/bin/env python  
import roslib
roslib.load_manifest('rescuer_project')
import rospy
import math
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('position_rescuer')

    listener = tf.TransformListener()
   
    base_pos = rospy.Publisher('/guardian/abs_pos', geometry_msgs.msg.Pose2D,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pose = geometry_msgs.msg.Pose2D()
        pose.x = trans[0]
        pose.y = trans[0]
	pose.theta = rot[0]
        base_pos.publish(pose)

        rate.sleep()
