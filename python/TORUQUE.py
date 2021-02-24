from dynamixel_helper import DxlHelper
import time

helper = DxlHelper("preset.json", verbosity="detailed")

motor_id = 14
motor = helper.get_motor(motor_id)
motor.set_torque(True)
# motor.set_goal_position(2650)
# motor.set_goal_position(2650)
# motor.set_goal_position(2650)
motor.set_torque(False)
# for i in range(100):
#     print(motor.get_present_position())
#     time.sleep(1)
