import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

#from wallfollow.cfg import WallFollowConfig





class DrivetoPoints:
	def __init__(self):
		rospy.init_node('wall_follow')

		rospy.Subscriber('/odom', Odometry, self.process_odom)

		self.ESTOP = False;
		self.update_hz = rospy.Rate(10)
		self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.marker_publisher = rospy.Publisher("/nav_points", Marker, queue_size=10)
		self.vel_x = 0
		self.ang_vel_y = 0
		self.twist = Twist()
		self.marker = Marker()
		self.marker.header.frame_id = "odom"
		self.marker.pose.orientation.x = 0
		self.marker.pose.orientation.y = 0
		self.marker.pose.orientation.z = 0
		self.marker.pose.orientation.w = 1
		self.marker.scale.x = .1
		self.marker.scale.y = 0.1
		self.marker.scale.z = 0.5
		self.marker.color.a = 1.0
		self.marker.color.r = 0.0
		self.marker.color.g = 1.0
		self.marker.color.b = 0.0

		self.x_odom = 0
		self.y_odom = 0
		self.angle_odom = 0

		self.num_states = 0;
		self.position_state = 0;

	def process_odom(self, msg):
		odom_pose = msg.pose.pose
		self.x_odom = odom_pose.position.x
		self.y_odom = odom_pose.position.y
		orientation_tuple = (odom_pose.orientation.x,
							odom_pose.orientation.y,
							odom_pose.orientation.z,
							odom_pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.angle_odom = angles[2]

	def driveToListPoints(self, points):
		self.num_states = len(points)
		if self.position_state >= self.num_states: 
			self.pubVel(0,0)
			self.ESTOP = 1
		else:
			#print(self.num_states)

			goal = points[self.position_state] 
			x_goal = goal[0]
			y_goal = goal[1]

			self.marker.pose.position.x = x_goal;
			self.marker.pose.position.y = y_goal;
			self.marker_publisher.publish(self.marker)

			xdis = x_goal - self.x_odom
			ydis = y_goal - self.y_odom

			for_vel = math.sqrt(xdis*xdis+ ydis*ydis)/2
			if for_vel > 1: for_vel = 1

			angle_to_point = math.atan2(ydis,xdis)
			
			ang_vel = self.angle_odom - angle_to_point/1

			if ang_vel > 1: ang_vel = 1
			elif ang_vel < -1: ang_vel = -1

			for_vel -= ang_vel*ang_vel


			self.pubVel(for_vel, -ang_vel)


			if ((x_goal-self.x_odom)**2 < .2 and (y_goal-self.y_odom)**2 < .1):
				self.position_state += 1


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
	neato = DrivetoPoints()

	while not rospy.is_shutdown():
		neato.driveToListPoints([(1,1), (2, 1), (0,0), (2, 5), (4, -3)])
		neato.update_hz.sleep()