from cmath import pi
import sys
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
    pass

def virtual_wall(phi_des, K, K_force, state, calib_data):
    tau_des = K * (phi_des - state["phi"])
    if state["F_hand"] < 2:
        tau_hand = 0
    else:
        tau_hand = state["F_hand"] * calib_data["l"]
    J = jacobian_TSA(state, calib_data)
    tau_motor = K_force*(tau_des - tau_hand) / J
    return tau_motor



path = "RJ_experiments/Results/"
filename = "wirtual_wall_1"

stand_rj = Stand_RJ()
stand_rj.force_sensor_initialization()
stand_rj.dont_collect_data()

calib_data = stand_rj.calib_data

phi_des = np.deg2rad(90)
K = 5
K_force = 10
tn = 10

nominal_velocity = 50

desired_data = {"t":[], "tau":[], "phi":[]}

try:
    while stand_rj.state["phi"] < phi_des:
        stand_rj.motor_set_speed(nominal_velocity)

    print("\nWirtual wall is running!!!!\n Save yourself!\n")

    stand_rj.collect_data()

    t0 = perf_counter()
    t = 0
    while t < tn:
        tau_motor = virtual_wall(phi_des, K, K_force, stand_rj.state, calib_data)

        desired_data["t"].append(stand_rj.state["t"])
        desired_data["tau"].append(tau_motor)
        desired_data["phi"].append(np.rad2deg(phi_des))

        I = tau_motor/calib_data["motor_K"]
        stand_rj.motor_set_current(I)

        t = perf_counter()-t0
            
        
except KeyboardInterrupt:
    stand_rj.motor_pause()

except Exception as e:
    print(e)

finally:
    stand_rj.stand_disable(filename=path+filename, desired=desired_data)