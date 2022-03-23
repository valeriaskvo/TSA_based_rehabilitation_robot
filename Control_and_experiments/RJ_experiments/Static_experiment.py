import sys
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

from rj_stand import Stand_RJ
import numpy as np

path = "RJ_experiments/Results/"
# filename = "static_experiment_v_const_1"
filename = "test"

stand_rj = Stand_RJ()
stand_rj.get_state()

nominal_velocity = 10
max_phi = np.deg2rad(150)

t = 0
try:
    while stand_rj.state["phi"] < max_phi:
        stand_rj.motor_set_speed(nominal_velocity)
        if stand_rj.state["t"]-t > 0.5:
            print(stand_rj.state["F_hand"])
            t = stand_rj.state["t"]
        
except KeyboardInterrupt:
    stand_rj.motor_pause()

except Exception as e:
    print(e)

finally:
    stand_rj.stand_disable(filename=path+filename, desired={"phi": np.rad2deg(max_phi)})