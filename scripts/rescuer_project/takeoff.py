#!/usr/bin/env python
import roslib
roslib.load_manifest('rescuer_project')
import rospy
from ardrone_autonomy.msg import Navdata
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
	t.angular.z=.4
	loop=0
	pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10 )
	while not rospy.is_shutdown() and loop<5:
		pub.publish(t)
		rate.sleep()
		loop+=1
		rospy.loginfo("[%d] Turn")

def stopNode():
	land()
	rospy.loginfo("Ending node")

def navDataUpdate(navData):
	bat = navData.batteryPercent
	print("battery:%d" % bat)
	state = navData.state
	print("state: %d" % state)
	rospy.sleep(.5)

if __name__ == '__main__':
	rospy.on_shutdown(stopNode)
	try:
		rospy.init_node('droneNode', anonymous=True)
		rate = rospy.Rate(10) # 10hz
		sub = rospy.Subscriber("ardrone/navdata",Navdata,navDataUpdate)
		takeoff()
		rospy.sleep(5)
		turnAround()
		rospy.sleep(5)
		land()
	except rospy.ROSInterruptException:
		pass
