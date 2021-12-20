import sys
sys.path.append('../Control_and_experiments')

from can import CAN_Bus
from motors.gyems import GyemsDRC
from sensors import SensorRJ
import numpy as np
from time import perf_counter, sleep
import pandas as pd
import pickle


def reset_data():
    global START_TIME
    START_TIME = perf_counter()

    stand_data = {"Time [sec]": [],
                  "Motor angle [rad]": [],
                  "Motor speed [rad/sec]": [],
                  "Motor current [units]": [],
                  "Motor torque [Nm]": [],
                  "Joint angle [rad]": [],
                  "Force sensor TSA [units]": [],
                  "Force sensor TSA [N]": [],
                  "Desired motor angle [rad]": [],
                  "Desired motor speed [rad/sec]": [],
                  "Desired motor current [units]": [],
                  "Desired motor torque [Nm]": []}
    return stand_data

def save_data(name, data):
    data_to_save = pd.DataFrame.from_dict(data)
    data_to_save.to_csv(name)
    return

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

def run_stand(stand_param):
    bus = CAN_Bus(interface=stand_param['interface'])

    print('CAN BUS connected successfully')

    motor = GyemsDRC(can_bus=bus, device_id=stand_param['id_motor'])
    motor.set_radians()
    motor.current_limit = stand_param['current_limit']
    motor.enable()
    print('Motor is enable')

    sensors = SensorRJ(bus)
    print('Sensors is enable')

    stand_data = reset_data()

    global calib_data
    calib_data = load_obj("/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data")

    return motor, sensors, stand_data


def get_state(stand_data, motor, sensors, q_des=0, dq_des=0, I_des=0):
    t = perf_counter() - START_TIME
    q, dq, I = motor.state['angle'], motor.state['speed'], motor.state['current']
    _, rotation_angle = sensors.read_encoders()
    force_a, _ = sensors.read_force()

    stand_data["Time [sec]"].append(t)
    stand_data["Motor angle [rad]"].append(q)
    stand_data["Motor speed [rad/sec]"].append(dq)
    stand_data["Motor current [units]"].append(I)
    stand_data["Motor torque [Nm]"].append(I*calib_data["motor_K"])

    stand_data["Joint angle [rad]"].append(rotation_angle)
    stand_data["Force sensor TSA [units]"].append(force_a)
    stand_data["Force sensor TSA [N]"].append(force_a*calib_data["force_A"])

    stand_data["Desired motor angle [rad]"].append(q_des)
    stand_data["Desired motor speed [rad/sec]"].append(dq_des)
    stand_data["Desired motor current [units]"].append(I_des)
    stand_data["Desired motor torque [Nm]"].append(I_des*calib_data["motor_K"])

    return q, dq, t, stand_data


def velocity_control(stand_data, motor, sensors, q_des, dq_des=0, gains={"kp": 1.5, "kd": 0.8}, v_max=400, threshold=np.pi/360):
    q, dq, t, stand_data = get_state(stand_data, motor, sensors)

    while np.abs(q-q_des) > threshold:
        e, de = q_des - q, dq_des - dq

        v = gains["kp"]*e + gains["kd"]*de
        if np.abs(v) > v_max:
            v = np.sign(v)*v_max

        motor.set_speed(v)
        q, dq, t, stand_data = get_state(
            stand_data, motor, sensors, q_des=q_des, dq_des=v)

    motor.pause()
    return q, dq, t, stand_data


stand_param = {'interface': 'can0', 'id_motor': 0x141, 'current_limit': 400}
motor, sensors, stand_data = run_stand(stand_param)
_, _, t, stand_data = get_state(stand_data, motor, sensors)



tf = 5
I_des = 200

const_speed = 30
F_constr = 0.06
n_avg = 50

experiment_name = "Current_"+str(I_des)+"_weight_0"

try:
    while t < tf:
        motor.set_current(I_des)
        _, _, t, stand_data = get_state(stand_data, motor, sensors, I_des=I_des)

except KeyboardInterrupt:
    motor.pause()
    print("Exit...")
finally:
    save_data("/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/Exoskeleton_experiments/experiment_results" +
              experiment_name+".csv", stand_data)

    q, dq, t, stand_data = velocity_control(stand_data, motor, sensors, 0)
    print("Actuator at the initial position")
    motor.disable()
