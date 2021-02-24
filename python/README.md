# DynamixelArm
Code for manipulating a a 3DOF arm made of 6 XM430-W350 daisy chained dynamixels in 2d space using inverse kinematics

also didnt do the math which is the hard part, just implemented it in code

Dependencies: DynamixelSDk, dynamixel_helper, pygame(for now), numpy 

in the dynamixel_helper in the dxl_motor.py file, delete line 159 "raise RuntimeError" (this line prevents daisy chaining motors on one serial port) and replace the tables folder with the tables folder in this repo

in the dxl_motor.py file add the function in reboot.py as a function in the motor class 
