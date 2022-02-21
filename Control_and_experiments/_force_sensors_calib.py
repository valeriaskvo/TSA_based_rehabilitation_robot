import pickle
from sensors import SensorRJ
from can import CAN_Bus
from time import sleep, perf_counter
import numpy as np

def save_obj(obj, name ):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

# calib_data =   {"force_A":0,
#                 "force_B":0,
#                 "motor_K":0}

# save_obj(calib_data,"calib_data")

def calibration_force(m, tf, sensor, sensor_data, weight):
    msg = input("Put "+str(m)+" kg weight")
    g = 9.8
    t0 = perf_counter()
    t = 0
    while t<tf:
        force_a, force_b = sensors.read_force()
        if sensor=="A":
            sensor_data.append(force_a)
        else:
            sensor_data.append(force_b)
        weight.append(m*g)
        t = perf_counter()-t0
    return sensor_data, weight


calib_data = load_obj("calib_data")
bus = CAN_Bus(interface = 'can0')
sensors = SensorRJ(bus)
sensor = "B"


sensor_data = []
weight = []
tf = 5
ms = [0.5, 1, 1.25, 0.5+1.25, 2.25]

for m in ms:
    sensor_data, weight = calibration_force(m, tf, sensor, sensor_data, weight)

weight = np.array(weight)
sensor_data = np.array(sensor_data)

k = np.mean(weight/sensor_data)
if sensor=="A":
    calib_data["force_A"] = k
else:
    calib_data["force_B"] = k

print(calib_data)

save_obj(calib_data,"calib_data")