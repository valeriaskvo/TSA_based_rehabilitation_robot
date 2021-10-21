from can import CAN_Bus
from motors.gyems import GyemsDRC
from sensors import SensorRJ
import numpy as np
from time import perf_counter, sleep
import pandas as pd


def reset_data():
    global START_TIME
    START_TIME = perf_counter()

    stand_data = {"Time [sec]": [],
                  "Motor angle [rad]": [],
                  "Motor speed [rad/sec]": [],
                  "Motor current [units]": [],
                  "Linear encoder [mm]": [],
                  "Joint angle [rad]": [],
                  "Force sensor on motor [units]": [],
                  "Force sensor on handle [units]": [],
                  "Desired motor angle [rad]": [],
                  "Desired motor speed [rad/sec]": [],
                  "Desired motor current[units]": [],
                  "Desired w [Hz]": []}
    return stand_data


def save_data(name, data):
    data_to_save = pd.DataFrame.from_dict(data)
    data_to_save.to_csv(name)
    return


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

    return motor, sensors, stand_data


def get_state(stand_data, motor, sensors, q_des=0, dq_des=0, I_des=0, w_des=False, w0=None, wf=None, tf=None, get_I_F=False):
    t = perf_counter() - START_TIME
    q, dq, I = motor.state['angle'], motor.state['speed'], motor.state['current']
    linear_displacement, rotation_angle = sensors.read_encoders()
    force_a, force_b = sensors.read_force()

    stand_data["Time [sec]"].append(t)
    stand_data["Motor angle [rad]"].append(q)
    stand_data["Motor speed [rad/sec]"].append(dq)
    stand_data["Motor current [units]"].append(I)
    stand_data["Linear encoder [mm]"].append(linear_displacement)
    stand_data["Joint angle [rad]"].append(rotation_angle)
    stand_data["Force sensor on motor [units]"].append(force_a)
    stand_data["Force sensor on handle [units]"].append(force_b)

    stand_data["Desired motor angle [rad]"].append(q_des)
    stand_data["Desired motor speed [rad/sec]"].append(dq_des)
    stand_data["Desired motor current[units]"].append(I_des)
    
    if w_des:
        w = w_desired(t, w0, wf, tf)
    else:
        w = 0

    stand_data["Desired w [Hz]"].append(w)

    if get_I_F:
        return q, dq, t, stand_data, I, force_b
    else:
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


def w_desired(t, w0, wf, tf):
    k = (wf - w0) / tf
    w = k*t + w0
    return w


def chirp(t, A, w0, wf, tf, x0):
    k = (wf - w0) / tf
    w = 0.5 * k*t + w0
    return A * np.sin((0.5 * k * t**2 + w0) * 2*np.pi) + x0


stand_param = {'interface': 'can0', 'id_motor': 0x141, 'current_limit': 400}
motor, sensors, stand_data = run_stand(stand_param)
_, _, t, stand_data, I, F = get_state(stand_data, motor, sensors, get_I_F=True)


parameters = "_angle_5_exp_3"
experiment_name = "Chirp" + parameters
wall_detection_name = "Wall_detection" + parameters

A = 100
w0 = 0
wf = 10
tf = 120

const_speed = 30
F_constr = 0.06
n_avg = 50

try:
    while F < F_constr:
        motor.set_speed(const_speed)
        _, _, t, stand_data, I, F = get_state(
            stand_data, motor, sensors, get_I_F=True)

    # n = len(stand_data["Motor current [units]"])-n_avg
    # I0 = sum(stand_data["Motor current [units]"][n:])/n_avg + A/2
    I0 = 90
    print("I0: = ", I0)

    t0 = t
    while t - t0 < 1:
        motor.set_current(I0)
        _, _, t, stand_data, I, F = get_state(
            stand_data, motor, sensors, get_I_F=True)

    # save_data("experiment_results/Experiment_2/" +
    #           wall_detection_name+"_A_"+str(A)+".csv", stand_data)

    stand_data = reset_data()
    _, _, t, stand_data, I, F = get_state(
        stand_data, motor, sensors, I_des=I0, w_des=True, w0=w0, wf=wf, tf=tf, get_I_F=True)
    while t < tf:
        I_des = chirp(t, A, w0, wf, tf, I0)
        motor.set_current(I_des)
        _, _, t, stand_data, I, F = get_state(
            stand_data, motor, sensors, I_des=I_des, w_des=True, w0=w0, wf=wf, tf=tf, get_I_F=True)

except KeyboardInterrupt:
    motor.pause()
    print("Exit...")
finally:
    save_data("experiment_results/Experiment_2/" +
              experiment_name+"_A_"+str(A)+".csv", stand_data)

    q, dq, t, stand_data = velocity_control(stand_data, motor, sensors, 0)
    print("Actuator at the initial position")
    motor.disable()
