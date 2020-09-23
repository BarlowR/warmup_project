#!/usr/bin/env python3

from visualization_msgs.msg import Marker
import rospy


class SphereMarker:
	def __init__(self):
		rospy.init_node('test_vis')
		self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
		self.marker = Marker();
		
		
	

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.marker.header.frame_id = "bodom";
			self.marker.header.stamp = rospy.Time.now();
			self.marker.ns = "my_namespace";
			self.marker.id = 0;
			self.marker.type = Marker.SPHERE;
			self.marker.action = Marker.ADD;
			self.marker.pose.position.x = 1;
			self.marker.pose.position.y = 2;
			self.marker.pose.position.z = 0;
			self.marker.pose.orientation.x = 0.0;
			self.marker.pose.orientation.y = 0.0;
			self.marker.pose.orientation.z = 0.0;
			self.marker.pose.orientation.w = 1.0;
			self.marker.scale.x = 0.1;
			self.marker.scale.y = 0.1;
			self.marker.scale.z = 0.1;
			self.marker.color.a = 1.0;
			self.marker.color.r = 1.0;
			self.marker.color.g = 1.0;
			self.marker.color.b = 0.0;
			self.vis_pub.publish( self.marker );
			r.sleep()

if __name__ == "__main__":
	node = SphereMarker()
	node.run()
