from __future__ import division
from dynamixel_helper import DxlHelper
from time import sleep
from constants import *
from numpy import deg2rad, cos, sin, sqrt, arctan2, rad2deg
import math

class Arm:
    def __init__(self, ids, offsets):
        self.helper = DxlHelper("preset.json", verbosity="detailed")
        self.ids = ids
        self.motors = {}
        self.motor_offsets = offsets
        for id in self.ids:
            self.motors.__setitem__(id, self.helper.get_motor(id))

    def set_torque(self, motors, value):
        for motor in motors:
        	self.motors[motor].set_torque(value)

    def theta_to_pos(self, theta): 
        # print(int((theta/360)*4096))
        return int((theta/360)*4096)

    def set_angle(self, id, theta):
        position = self.theta_to_pos(theta + self.motor_offsets[id])
        self.motors[id].set_goal_position(position)

    def set_angles(self, angles, delay=0):
        for id, theta in angles.items():
            position = self.theta_to_pos(theta + self.motor_offsets[id])
            self.motors[id].set_goal_position(position)
            sleep(delay)
    
    def read_write_position(self, motor,js_value):
        dxl_unit,res= self.motors[motor].get_present_position()
        if (dxl_unit+js_value) >= 4096:
	        return
        self.motors[motor].set_goal_position(int(dxl_unit+js_value)%4096)

    def move(self, motor, axis, scaler):
        value = axis*scaler
        self.read_write_position(motor, value)

    def reboot(self, motor):
        self.motors[motor].reboot()
        sleep(1)

    def reboot_all(self):
        for id, motor in self.motors.items():
            motor.reboot()

        sleep(3)

class ArmPositionController:
    def __init__(self, arm, rotation_motor):
        self.arm = arm
        self.rotation_motor = rotation_motor

    def ik(self, py, px):
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
        angles = self.ik(x, y)
        for angle in angles:
            if math.isnan(angle):
                print("INVALID POSITION")
                return
                
        thetas = {12: angles[0], 13: angles[1], 14: angles[2]}
        self.arm.set_angles(thetas, delay)

    def rotate(self, angle):
        self.arm.set_angle(self.rotation_motor, angle)

    def start_reboot_sequence(self):
        self.arm.reboot_all()
        self.arm.set_torque(ids, True)
        self.move(0, 0, delay=1)

if __name__ == "__main__":
    arm = Arm(ids, offsets)
    controller = ArmPositionController(arm, rotation_motor)
    # controller.move(10, 10)
    controller.start_reboot_sequence()
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
