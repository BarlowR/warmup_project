import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from wallfollow.cfg import WallFollowConfig





class FollowWall:
	def __init__(self):
		rospy.init_node('wall_follow')

		if rospy.has_param('~wall_distance'):
			self.wall_distance = rospy.get_param('~wall_distance')
		else:
			self.wall_distance = 1
		if rospy.has_param('~wall_distance'):
			self.k = rospy.get_param('~k')
		else:
			self.k = .5

		#srv = Server(WallFollowConfig, self.dynamicCB)

		rospy.Subscriber('/scan', LaserScan, self.process_scan)

		self.ESTOP = False;
		self.update_hz = rospy.Rate(100)
		self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.vel_x = 0
		self.ang_vel_y = 0
		self.twist = Twist()
		self.laserScan = [0]*360

		self.offset_index = 90
		self.weightArray = [0] * 360
		for i in range (0,90):
			self.weightArray[self.offset_index+i] = math.cos(i*3.14/180)
			self.weightArray[self.offset_index-i] = math.cos(i*3.14/180)
		print(self.weightArray)



	def dynamicCB(self, config, level):
		self.k = config.k
		self.wall_distance = config.wall_distance
		return config


	def process_scan(self, msg):
		self.laserScan = msg.ranges

	def computeWallAngle(self):
		centerOfInfluence = 0
		for i in range(0,90):
			if self.laserScan[self.offset_index+i] < 50:
				if self.laserScan[self.offset_index-i] < 50:
					difference = self.laserScan[self.offset_index+i] - self.laserScan[self.offset_index-i] 
				else: difference = -1
			elif self.laserScan[self.offset_index-i] < 50: difference = 1
			else: difference = 0
			#print(self.offset_index-i, self.laserScan[self.offset_index-i], difference * self.weightArray[self.offset_index+i])
			#print(self.offset_index+i, self.laserScan[self.offset_index+i])

			centerOfInfluence += difference * self.weightArray[self.offset_index+i]
		#print("     ")
		return centerOfInfluence/25

	def computeWallDistance(self):
		difference = 0
		for i in range(0,90):
			if self.laserScan[self.offset_index+i] < 50:
				if self.laserScan[self.offset_index-i] < 50:
					difference += self.weightArray[self.offset_index+i] * ( self.weightArray[self.offset_index+i] * (self.laserScan[self.offset_index-i] + self.laserScan[self.offset_index+i])/2 - self.wall_distance)
		#print(difference)			
		if difference*difference > 1600:
			if difference > 0: difference = 40
			if difference < 0: difference = -40
		return difference/15


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
	fw = FollowWall()

	while not rospy.is_shutdown():
		angle_to_turn = (fw.computeWallDistance() - fw.computeWallAngle())
		fw.pubVel(.8,angle_to_turn)
		fw.update_hz.sleep()