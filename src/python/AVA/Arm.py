"""
Arm classes for controlling the dynamixel arm
"""

from __future__ import division
from dynamixel_helper import DxlHelper
from time import sleep
from constants import *
from numpy import deg2rad, cos, sin, sqrt, arctan2, rad2deg
import math

class Arm:
    """
    This class creates an instance of a complete daisy chained dynamixel arm from one USB port.

    This class gives access to easily manipulate each motor of the dynamixel arm easily and can be used in the ArmPositionController class to control the arm using inverse kinematics.

    :param int[] ids: An array of 5 dynamixel ids that corrospond the the ids of the 5 dynamixel arm motors
    :param offsets: A dictionary of <id: offset> where offset in the offset in degrees of the motor id
    :type offsets: dict<int: float>

    """
    def __init__(self, ids, offsets):
        self.helper = DxlHelper("preset.json", verbosity="detailed")
        self.ids = ids
        self.motors = {}
        self.motor_offsets = offsets
        for id in self.ids:
            self.motors.__setitem__(id, self.helper.get_motor(id))

    def set_torque(self, motors, value):
        """
        This method sets the torque of one or more motors

        :param int[] motors: The array of motor ids to change the torque of 
        :param bool value: The value of the torque to be changed to
        """
        for motor in motors:
        	self.motors[motor].set_torque(value)

    def theta_to_pos(self, theta): 
        """
        This method converts from theta in degrees to a dynamixel position

        :param theta: The value in degrees to be converted
        :type theta: float or int

        :return: The theta value converted to dynamixel poistion
        :rtype: int or float
        """
        # print(int((theta/360)*4096))
        return int((theta/360)*4096)

    def set_angle(self, id, theta):
        """
        This method sets a dynamixel motor to the angle specified

        :param int id: The id of the dynamixel motor 
        :param theta: The angle to set the motor to
        :type theta: float or int
        """
        position = self.theta_to_pos(theta + self.motor_offsets[id])
        self.motors[id].set_goal_position(position)

    def set_angles(self, angles, delay=0):
        """
        This method sets the angles of mulitple dynamixel motors

        :param angles: A dictionary in the format <id: angle>
        :type angles: dict<int: int>

        """
        for id, theta in angles.items():
            position = self.theta_to_pos(theta + self.motor_offsets[id])
            self.motors[id].set_goal_position(position)
            sleep(delay)
    
    def read_write_position(self, motor,js_value):
        """
        This method reads the position of a motor and increments the position based on the js_value

        :param int motor: The id of the motor
        :param float js_value: The value to increment the motor position by
        """
        dxl_unit,res= self.motors[motor].get_present_position()
        if (dxl_unit+js_value) >= 4096:
	        return
        self.motors[motor].set_goal_position(int(dxl_unit+js_value)%4096)

    def move(self, motor, axis, scaler):
        """
        This method moves a motor based on a joystick value

        :param int motor: The id of the motor
        :param float axis: The value of an axis of a joystick
        :param float scalar: The value to multiply the axis value by (speed)
        """
        value = axis*scaler
        self.read_write_position(motor, value)

    def reboot(self, motor):
        """
        This method reboots a dynamixel motor

        :param int motor: The id of the motor
        """
        self.motors[motor].reboot()
        sleep(1)

    def reboot_all(self):
        """
        This method reboots all the motors in the arm
        """
        for id, motor in self.motors.items():
            motor.reboot()

        sleep(3)

class ArmPositionController:
    """
    This class controls the position (x, y) of the Arm using inverse kinematics

    Using the Arm class, this class does inverse kinematics to control the position of the Arm and holds other methods to control the position of the Arm

    :param Arm arm: The arm class to control
    :param int rotation_motor: The id of the motor that rotates the arm 
    """
    def __init__(self, arm, rotation_motor):
        self.arm = arm
        self.rotation_motor = rotation_motor

    def ik(self, py, px):
        """
        This method does inverse kinematics to find the angles of motors of move the arm to a (x, y) position. Returns NaN if the position is invalid

        :param float px: The desired x position
        :param float py: The desired y position
        """
        # Desired Position of End effector
        # px = -14
        # py = 3
        # x and y are flipped


        phi = 270
        phi = deg2rad(phi)

        # Equations for Inverse kinematics
        wx = px - a3*cos(phi)
        wy = py - a3*sin(phi)

        delta = wx**2 + wy**2
        c2 = ( delta -a1**2 -a2**2)/(2*a1*a2)
        s2 = sqrt(1-c2**2)  # elbow down
        theta_2 = arctan2(s2, c2)

        s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
        c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
        theta_1 = arctan2(s1,c1)
        theta_3 = phi-theta_1-theta_2

        # print('theta_1: ', rad2deg(theta_1)+180-14.25)
        # print('theta_2: ', rad2deg(theta_2)+90+14.25)
        # print('theta_3: ', rad2deg(theta_3))

        return (rad2deg(theta_1)+180, rad2deg(theta_2)+90, rad2deg(theta_3))

    def move(self, x, y, delay=0):
        """
        This method calculates the angles and moves the arm to an (x, y) position

        :param float x: The x desired position
        :param float y: The y desired position
        :param float delay: Delay in seconds between the individual movements of the arm. Defualt is 0
        """
        angles = self.ik(x, y)
        for angle in angles:
            if math.isnan(angle):
                print("INVALID POSITION")
                return
                
        thetas = {12: angles[0], 13: angles[1], 14: angles[2]}
        self.arm.set_angles(thetas, delay)

    def rotate(self, angle):
        """
        This method rotates the arm based on the given rotation motor

        :param float angle: The desired angle
        """
        self.arm.set_angle(self.rotation_motor, angle)

    def start_reboot_sequence(self):
        """
        This method starts moves the arm to (0, 0) with a delay of 1 second in between each arm movement
        """
        self.arm.reboot_all()
        self.arm.set_torque(ids, True)
        self.move(0, 0, delay=1)


if __name__ == "__main__":
    arm = Arm(ids, offsets)
    controller = ArmPositionController(arm, rotation_motor)
    print(controller.ik(0, 0))
    # controller.move(10, 10)
    # controller.start_reboot_sequence()
    # arm.set_angle(11, 180)
    # arm.reboot(15)
    
# sleep(1)
# controller.move(5, 0)
# sleep(1)
# controller.move(5, 5)
# sleep(1)
# controller.move(10, 10)
# # arm.set_torque(ids, True)
# arm.set_angles({12: 180, 13: 180, 14: 180})
