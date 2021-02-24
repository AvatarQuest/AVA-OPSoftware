#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64

def callback_change_state(msg):
    global drive_state, drive_state_publisher
    if msg.data:
        drive_state = not drive_state
        print(drive_state)
        drive_state_publisher.publish(drive_state)

def callback_arm_estop(msg):
    global drive_state, estop_publisher
    if not drive_state:
        estop_publisher.publish(msg.data)

def callback_arm_x_joystick(msg):
    global drive_state, arm_joystick_x_publisher
    if not drive_state:
        arm_joystick_x_publisher.publish(Vector3(msg.x, msg.y, msg.z))

def callback_arm_y_joystick(msg):
    global drive_state, arm_joystick_uy_publisher
    if not drive_state:
        arm_joystick_y_publisher.publish(Vector3(msg.x, msg.y, msg.z))

def callback_arm_torque(msg):
    global drive_state, torque_publisher
    if not drive_state:
        torque_publisher.publish(msg.data)

def callback_arm_reboot(msg):
    global drive_state, reboot_publisher
    if not drive_state:
        reboot_publisher.publish(msg.data)

def callback_drive_joystick(msg):
    global drive_state, drive_joystick_publisher
    if drive_state:
        drive_joystick_publisher.publish(Vector3(msg.x, msg.y, msg.z))

def callback_arm_claw_open(msg):
    global drive_state, claw_open_publisher
    if drive_state:
        claw_open_publisher.publish(msg.data)

def callback_arm_claw_close(msg):
    global drive_state, claw_close_publisher
    if drive_state:
        claw_close_publisher.publish(msg.data)

rospy.init_node("controller_state_node")

drive_state = True
rospy.Subscriber("controller_state", Bool, callback_change_state)
rospy.Subscriber("controller_drive_joystick", Vector3, callback_drive_joystick)
rospy.Subscriber("controller_arm_estop", Bool, callback_arm_estop)
rospy.Subscriber("controller_arm_x_joystick", Vector3, callback_arm_x_joystick)
rospy.Subscriber("controller_arm_y_joystick", Vector3, callback_arm_y_joystick)
rospy.Subscriber("controller_arm_torque", Bool, callback_arm_torque)
rospy.Subscriber("controller_arm_reboot", Bool, callback_arm_reboot)
rospy.Subscriber("controller_arm_claw_open", Bool, callback_arm_claw_open)
rospy.Subscriber("controller_arm_claw_close", Bool, callback_arm_claw_close)

drive_state_publisher = rospy.Publisher("drive_state", Bool, queue_size=1)
drive_joystick_publisher = rospy.Publisher("drive_joystick", Vector3, queue_size=5)
estop_publisher = rospy.Publisher("dynamixel_arm_estop", Bool, queue_size=5)
arm_joystick_x_publisher = rospy.Publisher("dynamixel_arm_x_joystick", Vector3, queue_size=3)
arm_joystick_y_publisher = rospy.Publisher("dynamixel_arm_y_joystick", Vector3, queue_size=3)
torque_publisher = rospy.Publisher("dynamixel_arm_torque", Bool, queue_size=5)
reboot_publisher = rospy.Publisher("dynamixel_arm_reboot", Bool, queue_size=5)
claw_open_publisher = rospy.Publisher("dynamixel_arm_claw_open", Bool, queue_size=1)
claw_close_publisher = rospy.Publisher("dynamixel_arm_claw_close", Bool, queue_size=1)

drive_state_publisher.publish(drive_state)
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    rate.sleep()