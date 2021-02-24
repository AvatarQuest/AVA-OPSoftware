#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64
import time

#Called when a new message arrives with joystick data
def callback_pantilt_joystick(msg):
	global pan_angle
	global tilt_angle
	global pan_delta
	global tilt_delta

	x = int(msg.x* -100.0)
	if(abs(x)> 5):
		pan_delta = x/15
	else:
		pan_delta =0

	y = int(msg.y* -100.0)
	if(abs(y)> 5):
		tilt_delta = y/15
	else:
		tilt_delta =0


#called when the pantilt home button is pressed or released
def callback_pantilt_home(msg):
	if(msg.data == True):
		global pan_angle
		global tilt_angle

		pan_angle = 96
		tilt_angle  = 48


#called when the pantilt dock button is pressed or released
def callback_pantilt_dock(msg):
	if(msg.data == True):
		global pan_angle
		global tilt_angle

		pan_angle = 180
		tilt_angle  = 157



#global variables
pan_angle = 90			#initialize with the same value used to init the pan servo in servo_settings.py
tilt_angle = 90			#initialize with the same value used to init the tilt servo in servo_settings.py


pan_delta = 0.0
tilt_delta = 0.0


#ROS setup stuff
#Initialize this ROS node
rospy.init_node('pantilt')

#setup the subscribers and their callbacks
rospy.Subscriber("pantilt_joystick", Vector3, callback_pantilt_joystick)
rospy.Subscriber("pantilt_home", Bool, callback_pantilt_home)
rospy.Subscriber("pantilt_dock", Bool, callback_pantilt_dock)

#setup the publishers
publisher_servo_angle_0 = rospy.Publisher('servo_angle_0', Float64)
publisher_servo_angle_1 = rospy.Publisher('servo_angle_1', Float64)


#the loop will run 20 times/sec
rate = rospy.Rate(20)
while not rospy.is_shutdown():

	pan_angle +=  pan_delta
	tilt_angle += tilt_delta

	#constrain
	pan_angle  = int(max(0, min(pan_angle,180)))		#keep the pan_angle between 0 and 180. Change to match your servo settings
	tilt_angle = int(max(0,min(tilt_angle,180)))		#keep the tilt_angle between 0 and 180. Change to match your servo settings


	#send the pan and tilt angles to the servo_controller
	publisher_servo_angle_1.publish(pan_angle)
	publisher_servo_angle_0.publish(tilt_angle)

	rate.sleep()
