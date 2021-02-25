#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64
import time
from Arm import Arm, ArmPositionController
from constants import *

def callback_arm_x_joystick(msg):
    global delta_x

    delta_x = msg.y * 0.5 if abs(msg.y) > 0.01 else 0

def callback_arm_y_joystick(msg):
    global delta_y, delta_rotation
    delta_y = msg.y * 0.5 if abs(msg.y) > 0.1 else 0
    delta_rotation = msg.x * -2 if abs(msg.x) > 0.1 else 0

def callback_arm_reboot(msg):
    global reboot
    if msg.data:
        reboot = True

def callback_arm_torque(msg):
    global torque
    if msg.data:
        torque = not torque
        arm.set_torque(ids, torque)

def callback_arm_reset(msg):
    global controller, position
    if msg.data:
        controller.move(0, 0)
        position = (0, 0)

def callback_arm_estop(msg):
    rospy.signal_shutdown("Estop was pressed")

def callback_arm_claw_open(msg):
    global arm_close, arm_open
    if msg.data:
        arm_close = False 
        arm_open = True 
    else:
        arm_open = False

def callback_arm_claw_close(msg):
    global arm_close, arm_open
    if msg.data:
        arm_close = True
        arm_open = False
    else:
        arm_close = False

arm = Arm(ids, offsets)
controller = ArmPositionController(arm, rotation_motor)
reboot = False
torque = True
delta_x = 0
delta_y = 0
delta_rotation = 0
rotation = 180
position = (0, 0)
arm_open = False
arm_close = False


rospy.init_node('dynamixel_arm')

rospy.Subscriber("dynamixel_arm_estop", Bool, callback_arm_estop)
rospy.Subscriber("dynamixel_arm_x_joystick", Vector3, callback_arm_x_joystick)
rospy.Subscriber("dynamixel_arm_y_joystick", Vector3, callback_arm_y_joystick)
rospy.Subscriber("dynamixel_arm_torque", Bool, callback_arm_torque)
rospy.Subscriber("dynamixel_arm_reboot", Bool, callback_arm_reboot)
rospy.Subscriber("dynamixel_arm_reset", Bool, callback_arm_reset)
rospy.Subscriber("dynamixel_arm_claw_open", Bool, callback_arm_claw_open)
rospy.Subscriber("dynamixel_arm_claw_close", Bool, callback_arm_claw_close)

#setup the publishers
publisher_arm_x = rospy.Publisher('arm_out_x', Float64, queue_size=3)
publisher_arm_y = rospy.Publisher('arm_out_y', Float64, queue_size=3)
publisher_arm_reboot = rospy.Publisher('arm_out_reboot', Bool, queue_size=1)
publisher_arm_torque = rospy.Publisher('arm_out_torque', Bool, queue_size=1)
publisher_arm_rotation = rospy.Publisher('arm_out_rotation', Float64, queue_size=3)

rate = rospy.Rate(20)
arm.set_torque(ids, True)
arm.set_angle(12, 190)
controller.rotate(180)
controller.move(position[0], position[1], delay=1)
try:
    while not rospy.is_shutdown():
        publisher_arm_reboot.publish(reboot)
        publisher_arm_torque.publish(torque)
        
        if reboot:
            controller.start_reboot_sequence()
            reboot = False
            continue

        if abs(delta_x) > 0.1 or abs(delta_y) > 0.1:
            x, y = position
            position = (x + delta_x, y + delta_y)
            controller.move(position[0], position[1])
            publisher_arm_x.publish(position[0])
            publisher_arm_y.publish(position[1])
        
        if abs(delta_rotation) > 0:
            rotation += delta_rotation 
            controller.rotate(rotation)
            publisher_arm_rotation.publish(rotation)
        
        if arm_close:
            arm.move(15, 1, 70)
        elif arm_open:
            arm.move(15, 1, -70)
        
        rate.sleep()
except rospy.ROSInterruptException:
    pass