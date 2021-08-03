from mirobot import Mirobot
import model
import math
import time
import numpy as np
import cv2

command = "disconnect"
if command == "connect":

    arm_model = model.Motivation()
    arm_model.robot_initial()
    arm_model.run_robot()
    time.sleep(5)
    print(f"状态查询: {arm_model.get_status()}")
    print(arm_model.get_position())

leg_model = model.Motivation()
leg_model.set_angle([0, 0, 0, 0, 0, 40])
# leg_model.set_position(211, 20, 30, 0, 0, 0)
position = leg_model.get_position()
T = leg_model.get_matrix()
print(T)
print(position)
leg_model.set_position(position)
T_ = leg_model.get_matrix()
print(T_)
print(leg_model.get_angle())
