import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from wallfollow.cfg import WallFollowConfig





class FollowPerson:
	def __init__(self):
		rospy.init_node('wall_follow')

		if rospy.has_param('~wall_distance'):
			self.wall_distance = rospy.get_param('~wall_distance')
		else:
			self.wall_distance = .5
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

		self.offset_index = 180
		self.weightArray = [0] * 360
		for i in range (0,90):
			self.weightArray[self.offset_index+i] = math.cos(i*3.14/360)
			self.weightArray[self.offset_index-i] = math.cos(i*3.14/360)
		#print(self.weightArray)



	def dynamicCB(self, config, level):
		self.k = config.k
		self.wall_distance = config.wall_distance
		return config


	def process_scan(self, msg):
		self.laserScan = msg.ranges

	def largestObjectOfInterest(self, ranges, threshold):
		if len(ranges) <= 1: return (0, 0)
		else:
			index_begin = 0
			index_end = 1

			for i in range(0,len(ranges)-1):
				if ranges[i] != float('inf') or ranges[i+1] != float('inf'):
					if (ranges[i] - ranges[i+1])**2 > threshold**2:
						if index_begin == 0:
							index_begin = i+1
						else: 
							index_end = i
							break

			next = self.largestObjectOfInterest(ranges[(index_end+1):], threshold)
			if (next[1]-next[0])>(index_end-index_begin): return (next[0] +1+ index_end, next[1]+1+index_end)
			else: return(index_begin, index_end)



	def turnToObject(self):
		lObj = self.largestObjectOfInterest(self.laserScan, .3)
		#print(lObj)
		if (lObj[1]-lObj[0] > 1):
			center = (lObj[1]+lObj[0])/2
			#print(lObj[1]-lObj[0])
			return(-(self.offset_index - center)/45)
		return 0
		
		#print("     ")
		return centerOfInfluence

	def computeObjectForwardDistance(self):
		difference = 0
		for i in range(0,90):
			if self.laserScan[self.offset_index+i] < 50:
				if self.laserScan[self.offset_index-i] < 50:
					difference += (self.weightArray[self.offset_index+i] * (self.laserScan[self.offset_index-i] + self.laserScan[self.offset_index+i])/2 - self.wall_distance)
		
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
	fw = FollowPerson()

	while not rospy.is_shutdown():
		vel = fw.computeObjectForwardDistance()
		if vel > 0.6: vel = .6
		fw.pubVel(-vel,fw.turnToObject())
		fw.update_hz.sleep()