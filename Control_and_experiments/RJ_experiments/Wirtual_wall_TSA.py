from cmath import pi
import sys

from pyrsistent import v
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

from rj_stand import Stand_RJ
import numpy as np
from time import perf_counter

def jacobian_TSA(state, calib_data):
    if state["q"] < pi:
        q = pi
    else:
        q = state["q"]

    x = state["x"]
    J = q*calib_data["r"]**2/(calib_data["R"]*(calib_data["L"]-x))
    return J

def rj_position_control(phi_des, dphi_des, Kp, state, calib_data):
    J = jacobian_TSA(state, calib_data)
    dq = (dphi_des + Kp * (phi_des - state["phi"])) / J
    return dq

def virtual_wall(phi_des, impidance, state, calib_data):
    tau = state["F_hand"] * calib_data ["l"]
    phi = phi_des - tau/impidance["k"]

    return phi



path = "RJ_experiments/Results/"
filename = "wirtual_wall_k_2000_1"

stand_rj = Stand_RJ()
stand_rj.force_sensor_initialization()
stand_rj.dont_collect_data()

calib_data = stand_rj.calib_data

phi_des = np.deg2rad(80)

impidance = {"m": 1,
             "b": 2,
             "k": 2000}

Kp = 15
tn = 15

desired_data = {"t":[], "phi":[]}

try:
    while stand_rj.state["phi"] < phi_des:
        dq = rj_position_control(phi_des, 0, Kp, stand_rj.state, calib_data)
        stand_rj.motor_set_speed(dq)
    stand_rj.motor.pause()

    print("\nStand at the desired position!\n")
    input("Press any key to continue...")
    stand_rj.collect_data()
    t0 = perf_counter()
    t = 0
    while t < tn:
        phi_wall = virtual_wall(phi_des, impidance, stand_rj.state, calib_data)
        print("t:",t, "[sec], phi:", np.rad2deg(phi_wall),"[deg]")
        dq = rj_position_control(phi_wall, 0, Kp, stand_rj.state, calib_data)
        stand_rj.motor_set_speed(dq)
        t = perf_counter() - t0

        desired_data["t"].append(stand_rj.state["t"])
        desired_data["phi"].append(np.rad2deg(phi_wall))
           
        
except KeyboardInterrupt:
    stand_rj.motor_pause()

except Exception as e:
    print(e)

finally:
    stand_rj.stand_disable(filename=path+filename, desired=desired_data)