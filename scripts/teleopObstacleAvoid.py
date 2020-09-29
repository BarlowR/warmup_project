import rospy
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios
import time
import threading

import obstacleAvoidance
import teleop

try:
	from inputs import get_gamepad
except ImportError:
	print("please install inputs (pip install inputs)")









if __name__ == "__main__":
	gamepad = teleop.GetGamepad()
	oa = obstacleAvoidance.ObstacleAvoid()

	
	gamepadThread = threading.Thread(target = gamepad.runLoop)
	gamepadThread.start()

	
	while not rospy.is_shutdown():

		if gamepad.but_a:
			weights = oa.weightMap()
			turn = (weights[1]-weights[0])/10
			forwardVel = 10/(10+(weights[1]+weights[0]))
			oa.pubVel(forwardVel, turn)
			oa.update_hz.sleep()
		else:
			joystick = gamepad.gamepadUtility()
			oa.pubVel(-joystick[1],-joystick[0])
			oa.update_hz.sleep()
	