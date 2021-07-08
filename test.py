from mirobot import Mirobot
import model
import math
import time
import numpy as np

command = "disconnect"
if command == "connect":

    arm_model = model.Motivation()
    arm_model.robot_initial()
    arm_model.run_robot_angle()
    time.sleep(5)
    print(f"状态查询: {arm_model.get_status()}")
    print(arm_model.get_position())

leg_model = model.Motivation()
leg_model.set_angle([0, 0, 20, 0, 0, 0])
# leg_model.set_position(211, 20, 30, 0, 0, 0)
leg_model.get_jacobi()
print(leg_model.get_jacobi())

