#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Twist

def takeoff():
	pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
	loop=0
	while not rospy.is_shutdown() and loop<5:
		pub.publish(Empty())
		rate.sleep()
		loop+=1
		rospy.loginfo("[%d] take off")

def land():
	loop=0
	pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
	while not rospy.is_shutdown() and loop<5:
		pub.publish(Empty())
		rate.sleep()
		loop+=1
		rospy.loginfo("[%d] Land")

def turnAround():
	#~ geometryMsg=[[0,0,0],[0,0,1.0]]
	t = Twist()
	t.angular.z=1.0
	loop=0
	pub = rospy.Publisher("ardrone/cmd_vel",Twist, queue_size=10 )
	while not rospy.is_shutdown() and loop<5:
		pub.publish(Empty())
		rate.sleep()
		loop+=1
		rospy.loginfo("[%d] Turn")

def stopNode():
	land()
	rospy.loginfo("Ending node")

if __name__ == '__main__':
	rospy.on_shutdown(stopNode)
	try:
		rospy.init_node('droneNode', anonymous=True)
		rate = rospy.Rate(10) # 10hz
		takeoff()
		rospy.sleep(5)
		turnAround()
		rospy.sleep(10)
		land()
	except rospy.ROSInterruptException:
		pass
