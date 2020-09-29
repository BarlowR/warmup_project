import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from wallfollow.cfg import WallFollowConfig





class ObstacleAvoid:
	def __init__(self):
		rospy.init_node('wall_follow')

		rospy.Subscriber('/scan', LaserScan, self.process_scan)

		self.ESTOP = False;
		self.update_hz = rospy.Rate(100)
		self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.vel_x = 0
		self.ang_vel_y = 0
		self.twist = Twist()
		self.laserScan = [0]*360

		self.offset_index = 0
		self.weightArray = [0] * 360
		for i in range (0,90):
			self.weightArray[self.offset_index+i] = math.cos(i*3.14/180)
			self.weightArray[self.offset_index-i] = math.cos(i*3.14/180)
		#print(self.weightArray)

	def process_scan(self, msg):
		self.laserScan = msg.ranges

	def weightMap(self):
		leftWeight = 0
		rightWeight = 0
		for i in range(0,90):
			if self.laserScan[i] < 20: leftWeight += (1/(1+self.laserScan[i]))*self.weightArray[i]
			if self.laserScan[-i] < 20: rightWeight += (1/(1+self.laserScan[-i]))*self.weightArray[-i]
		return(leftWeight, rightWeight)


	def pubVel(self, input_vel, input_ang):
		if not rospy.is_shutdown():

			if not self.ESTOP:
				self.vel_x = input_vel
				self.ang_vel_y = input_ang
			else:
				self.vel_x = 0
				self.ang_vel_y = 0

				
			self.twist.linear.x = self.vel_x
			self.twist.angular.z = self.ang_vel_y
			self.twist_publisher.publish(self.twist)


if __name__ == "__main__":
	oa = ObstacleAvoid()

	while not rospy.is_shutdown():
		weights = oa.weightMap()
		turn = (weights[1]-weights[0])/10
		forwardVel = 10/(10+(weights[1]+weights[0]))
		#print(turn, forwardVel)
		oa.pubVel(forwardVel, turn)
		oa.update_hz.sleep()