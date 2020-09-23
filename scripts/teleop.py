import rospy
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios
from inputs import get_gamepad
import time
import threading


class GetKeyboard:
	def __init__(self):
		self.settings = termios.tcgetattr(sys.stdin)
		self.key = None

	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		self.key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return self.key


	def run(self):
		while self.key != '\x03':
			self.key = self.getKey()
			print(self.key)

class GetGamepad:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.rx = 0
		self.ry = 0
		self.but_a = 0
		self.but_b = 0
		self.but_x = 0
		self.but_y = 0

		

	def getGPEvent(self):
		self.events = get_gamepad()
		return self.events


	def update(self):
		events = self.getGPEvent()
		
		for event in events:
			if event.ev_type == "Absolute":
				if event.code == "ABS_X" : self.x = event.state
				elif event.code == "ABS_Y" : self.y = event.state
				elif event.code == "ABS_RX" : self.rx = event.state
				elif event.code == "ABS_RY" : self.ry = event.state

			elif event.ev_type == "Key":
				if event.code == "BTN_SOUTH" : self.but_a = event.state
				elif event.code == "BTN_EAST" : self.but_b = event.state
				elif event.code == "BTN_NORTH" : self.but_y = event.state
				elif event.code == "BTN_WEST" : self.but_x = event.state
	def runLoop(self):
		while 1:
			self.update()
			time.sleep(.001)



class TeleOp:
	def __init__(self):
		rospy.init_node("tele_op")

		self.ESTOP = False;


		self.update_hz = rospy.Rate(10)

		self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

		self.vel_x = 0
		self.ang_vel_y = 0

		self.twist = Twist()


	def update(self, input_x, input_y):
		if not rospy.is_shutdown():

			if not self.ESTOP:
				self.vel_x = input_x
				self.ang_vel_y = input_y
			else:
				self.vel_x = 0
				self.ang_vel_y = 0

				self.twist.linear.x = self.vel_x
				self.twist.angular.z = self.ang_vel_y

			self.twist_publisher.publish(self.twist)







if __name__ == "__main__":
	gamepad = GetGamepad()
	neato = TeleOp()
	
	"""gamepadThread = threading.Thread(target = gamepad.runLoop)
	gamepadThread.start()
"""
	
	while 1:
		neato.update(1,0)
		neato.update_hz.sleep()
	