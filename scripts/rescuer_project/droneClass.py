#!/usr/bin/env python
import roslib
roslib.load_manifest('rescuer_project')
import rospy
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

class ARDrone:
	"Common clase for AR Drone ROS Node"
	def __init__(self,name="ARDrone_1",rate=10):
		self.name=name
		self.navData=[]
		self.odom=[]
		rospy.init_node(self.name, anonymous=True)
		self.rate=rospy.Rate(rate) # 10hz
	def initNode(self):
		rospy.on_shutdown(stopNode)
		subNav = rospy.Subscriber("ardrone/navdata",Navdata,self.navDataUpdate)
		subImu = rospy.Subscriber("ardrone/odometry",Odometry,self.odometryUpdate)
	def navDataUpdate(self,navData):
		self.navData=navData
		tagCount = self.navData.tags_count
		if tagCount != 0:
			print("tagCount=%d"%tagCount)
		else:
			print("No tag")
	def odometryUpdate(self,odom):
		self.odom=odom
	def takeOff(self):
		pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
		loop=0
		while not rospy.is_shutdown() and loop<8:
			pub.publish(Empty())
			self.rate.sleep()
			loop+=1
			rospy.loginfo("[%d] take off")
	def land(self):
		loop=0
		pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
		while not rospy.is_shutdown() and loop<5:
			pub.publish(Empty())
			self.rate.sleep()
			loop+=1
			rospy.loginfo("[%d] Land")

def stopNode():
	loop=0
	pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
	while not rospy.is_shutdown() and loop<5:
		pub.publish(Empty())
		loop+=1
		rospy.loginfo("[%d] Land")
	rospy.loginfo("Ending node")

if __name__ == '__main__':
	try:
		drone= ARDrone("myDrone")
		drone.initNode()
		#~ drone.takeOff()
		#~ rospy.sleep(6)
		#~ drone.land()
		while not rospy.is_shutdown():
			drone.rate.sleep()
	except rospy.ROSInterruptException:
		pass
