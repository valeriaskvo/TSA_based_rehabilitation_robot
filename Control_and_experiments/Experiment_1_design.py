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
                  "Linear encoder [mm]":[],
                  "Joint angle [rad]": [],
                  "Force sensor TSA [units]": [],
                  "Force sensor TSA [N]": [],
                  "Force sensor handle [units]": [],
                  "Force sensor handle [N]": [],
                  "Desired motor angle [rad]": [],
                  "Desired motor speed [rad/sec]": []
                  }
    return stand_data

def save_data(name, data):
    data_to_save = pd.DataFrame.from_dict(data)
    data_to_save.to_csv(name)
    return

def save_obj(obj, name ):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

def run_stand(stand_param):
    bus = CAN_Bus(interface = stand_param['interface'])

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
    print('Calibration data is enable')

    return motor, sensors, stand_data

def get_state(stand_data, motor, sensors, q_des = 0, dq_des = 0):
    t = perf_counter() - START_TIME
    q, dq, I = motor.state['angle'], motor.state['speed'], motor.state['current']
    linear_displacement, rotation_angle = sensors.read_encoders()
    force_a, force_b = sensors.read_force()

    stand_data["Time [sec]"].append(t)
    stand_data["Motor angle [rad]"].append(q)
    stand_data["Motor speed [rad/sec]"].append(dq)
    stand_data["Motor current [units]"].append(I)
    stand_data["Motor torque [Nm]"].append(I*calib_data["motor_K"])

    stand_data["Linear encoder [mm]"].append(linear_displacement)
    stand_data["Joint angle [rad]"].append(rotation_angle)

    stand_data["Force sensor TSA [units]"].append(force_a)
    stand_data["Force sensor handle [units]"].append(force_b)
    stand_data["Force sensor TSA [N]"].append(force_a*calib_data["force_A"])
    stand_data["Force sensor handle [N]"].append(force_b*calib_data["force_B"])

    stand_data["Desired motor angle [rad]"].append(q_des)
    stand_data["Desired motor speed [rad/sec]"].append(dq_des)

    return q, dq, t, stand_data

def velocity_control(stand_data, motor, sensors, q_des, dq_des = 0, gains = {"kp": 1.5, "kd": 0.8}, v_max = 400, threshold = np.pi/360):
    q, dq, t, stand_data = get_state(stand_data, motor, sensors)

    while np.abs(q-q_des)>threshold:
        e, de = q_des - q, dq_des - dq
        
        v = gains["kp"]*e + gains["kd"]*de

        motor.set_speed(v)
        q, dq, t, stand_data = get_state(stand_data, motor, sensors, q_des=q_des, dq_des = v)
    
    motor.pause()
    return q, dq, t, stand_data

stand_param = {'interface':'can0', 'id_motor':0x141, 'current_limit':400}
motor, sensors, stand_data = run_stand(stand_param)
q, dq, t, stand_data = get_state(stand_data, motor, sensors)

A = 55 *2*np.pi
w = 0.025 *2*np.pi
tf = 15

experiment_name = "Static_experiment_upd"

try:
    while t<tf:
        v = A * np.cos(w*t) * w
        x = A *np.sin(w*t)
        motor.set_speed(v)
        q, dq, t, stand_data = get_state(stand_data, motor, sensors, q_des=x, dq_des = v)

except KeyboardInterrupt:
    motor.pause()
    print("Exit...")
finally:
    save_data("experiment_results/Experiment_1/"+experiment_name+".csv", stand_data)

    q, dq, t, stand_data = velocity_control(stand_data, motor, sensors, 0)
    print("Actuator at the initial position")

    motor.disable()
