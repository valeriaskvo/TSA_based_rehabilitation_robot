import sys
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

from rj_stand import Stand_RJ
import numpy as np
from time import sleep, perf_counter

stand_rj = Stand_RJ()
stand_rj.dont_collect_data()
stand_rj.get_state()

zero_x = stand_rj.state["x"]
zero_q = stand_rj.state["q"]

t_turn = perf_counter()
turns = 0

calibration_speed = 25

moving_threshold = 3
time_threshold = 1

accurate_threshold = np.deg2rad(0.01)
gains = {"kp": 5,
         "kd": 2}

try:
    while turns < 3:
        speed = (-1)**turns * calibration_speed
        stand_rj.motor_set_speed(speed)
        if zero_x > stand_rj.state["x"]:
            zero_x = stand_rj.state["x"]
            zero_q = stand_rj.state["q"]
        
        t = perf_counter()
        if np.abs(zero_x - stand_rj.state["x"]) > moving_threshold and (t - t_turn) > time_threshold:
            turns +=1
            t_turn = t
            print('Turn', turns)

    print("\nNew zero position", zero_x,"[mm]")
    print("Zero motor angle is", zero_q, "[deg]")

except KeyboardInterrupt:
    stand_rj.motor_pause()
finally:
    stand_rj.stand_disable(zero_q = zero_q)

    print("Callibration is done!")
    print("Reboot stand physically\n")   