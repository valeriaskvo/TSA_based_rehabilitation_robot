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
                  "Linear encoder [mm]": [],
                  "Joint angle [rad]": [],
                  "Force sensor TSA [units]": [],
                  "Force sensor TSA [N]": [],
                  "Force sensor handle [units]": [],
                  "Force sensor handle [N]": []
                  }
    return stand_data

def save_data(name, data):
    data_to_save = pd.DataFrame.from_dict(data)
    data_to_save.to_csv(name)
    return

def save_obj(obj, name):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

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
    calib_data = load_obj(
        "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data")
    print('Calibration data is enable')

    return motor, sensors, stand_data

def get_state(stand_data, motor, sensors):
    t = perf_counter() - START_TIME
    q, dq, I = motor.state['angle'], motor.state['speed'], motor.state['current']
    x, phi = sensors.read_encoders()
    force_a, force_b = sensors.read_force()

    stand_state = {}
    stand_state["t"] = t
    stand_state["q"] = q
    stand_state["dq"] = dq
    stand_state["x"] = x
    stand_state["phi"] = phi
    stand_state["I"] = I
    stand_state["tau"] = I*calib_data["motor_K"]
    stand_state["F_tsa"] = force_a*calib_data["force_A"]
    stand_state["F_hand"] = force_b*calib_data["force_B"]

    stand_data["Time [sec]"].append(t)
    stand_data["Motor angle [rad]"].append(q)
    stand_data["Motor speed [rad/sec]"].append(dq)
    stand_data["Motor current [units]"].append(I)
    stand_data["Motor torque [Nm]"].append(stand_state["tau"])

    stand_data["Linear encoder [mm]"].append(x)
    stand_data["Joint angle [rad]"].append(phi)

    stand_data["Force sensor TSA [units]"].append(force_a)
    stand_data["Force sensor handle [units]"].append(force_b)
    stand_data["Force sensor TSA [N]"].append(stand_state["F_tsa"])
    stand_data["Force sensor handle [N]"].append(stand_state["F_hand"])

    return stand_state, stand_data


# TODO: Rewrite velocity control for this stand
def velocity_control(stand_data,
                     motor,
                     sensors,
                     q_des,
                     dq_des=0,
                     gains={"kp": 1.5, "kd": 0.8},
                     v_max=400, threshold=np.pi/180):

    stand_state, stand_data = get_state(stand_data, motor, sensors)

    while np.abs(stand_state["q"]-q_des) > threshold:
        e, de = q_des - stand_state["q"], dq_des - stand_state["dq"]

        v = gains["kp"]*e + gains["kd"]*de

        if np.abs(v) > v_max:
            v = np.sign(v)*v_max

        motor.set_speed(v)

        stand_state, stand_data = get_state(stand_data, motor, sensors)

    motor.pause()
    return stand_state, stand_data


stand_param = {'interface': 'can0', 'id_motor': 0x141, 'current_limit': 400}
motor, sensors, stand_data = run_stand(stand_param)
stand_state, stand_data = get_state(stand_data, motor, sensors)

nominal_velocity = 10
max_phi = np.deg2rad(90)

experiment_name = "Static_experiment_upd"

try:
    while stand_state["phi"] < max_phi:
        motor.set_speed(nominal_velocity)
        stand_state, stand_data = get_state(stand_data, motor, sensors)

except KeyboardInterrupt:
    motor.pause()
    print("Exit...")
finally:
    save_data("experiment_results/Experiment_1/" +
              experiment_name+".csv", stand_data)

    stand_state, stand_data = velocity_control(stand_data, motor, sensors, 0)
    print("Actuator at the initial position")
    print(stand_state)

    motor.disable()
