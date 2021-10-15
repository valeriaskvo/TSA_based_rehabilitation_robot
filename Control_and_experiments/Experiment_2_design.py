from can import CAN_Bus
from motors.gyems import GyemsDRC
from sensors import SensorRJ
import numpy as np
from time import perf_counter, sleep
import pandas as pd

def reset_data():
    stand_data =    {"Time [sec]":[],  
                    "Motor angle [rad]":[],
                    "Motor speed [rad/sec]": [],
                    "Motor current [units]": [],
                    "Linear encoder [mm]":[],
                    "Joint angle [rad]":[],
                    "Force sensor on motor [units]": [],
                    "Force sensor on handle [units]": [],
                    "Desired motor angle [rad]": [],
                    "Desired motor speed [rad/sec]": [],
                    "Desired motor current[units]":[]}
    return stand_data

def save_data(name, data):
    data_to_save = pd.DataFrame.from_dict(data)
    data_to_save.to_csv(name)
    return

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
    
    global START_TIME
    START_TIME = perf_counter()

    return motor, sensors, stand_data

def get_state(stand_data, motor, sensors, t0 = 0, q_des = 0, dq_des = 0, I_des = 0, get_I_F = False):
    t = perf_counter() - t0 - START_TIME
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

    if get_I_F:
        return q, dq, t, stand_data, I, force_b
    else:
        return q, dq, t, stand_data

def velocity_control(stand_data, motor, sensors, q_des, dq_des = 0, gains = {"kp": 1.5, "kd": 0.8}, v_max = 400, threshold = np.pi/360):
    q, dq, t, stand_data = get_state(stand_data, motor, sensors)

    while np.abs(q-q_des)>threshold:
        e, de = q_des - q, dq_des - dq
        
        v = gains["kp"]*e + gains["kd"]*de
        if np.abs(v)>v_max:
            v = np.sign(v)*v_max

        motor.set_speed(v)
        q, dq, t, stand_data = get_state(stand_data, motor, sensors, q_des=q_des, dq_des = v)
    
    motor.pause()
    return q, dq, t, stand_data

def chirp_x(t, A,  dw, w, x0 = 0):
    return A * np.sin(dw * t**2/2 + w * t) + x0

def chirp_dx(t, A,  dw, w):
    return A * np.cos(dw * t**2/2 + w * t) * (dw * t + w)

stand_param = {'interface':'can0', 'id_motor':0x141, 'current_limit':400}
motor, sensors, stand_data = run_stand(stand_param)
_, _, t, stand_data, I, F = get_state(stand_data, motor, sensors, get_I_F=True)

A = 80
dw = 0.03 *2*np.pi
w = 0.2 *2*np.pi
tf = 20

experiment_name = "Chirp"

const_speed = 30
F_constr = 0.06
n_avg = 50

try:
    while F<F_constr:
        motor.set_speed(const_speed)
        _, _, t, stand_data, I, F = get_state(stand_data, motor, sensors, get_I_F=True)

    n = len(stand_data["Motor current [units]"])-n_avg
    I0 = sum(stand_data["Motor current [units]"][n:])/n_avg + A/2

    t0 = t
    while t- t0 < 1:
        motor.set_current(I0)
        _, _, t, stand_data, I, F = get_state(stand_data, motor, sensors, get_I_F=True)

    save_data("experiment_results/Experiment_2/Wall_detection"+"_I0_"+str(int(I0))+"_A_"+str(A)+".csv", stand_data)
    
    t0 = t
    stand_data = reset_data()
    _, _, t, stand_data, I, F = get_state(stand_data, motor, sensors, t0=t0, I_des=I0, get_I_F=True)
    while t<tf:
        I_des = chirp_x(t, A,  dw, w, x0 = I0)
        motor.set_current(I_des)
        _, _, t, stand_data, I, F = get_state(stand_data, motor, sensors, t0=t0, I_des=I_des, get_I_F=True)


except KeyboardInterrupt:
    motor.pause()
    print("Exit...")
finally:
    save_data("experiment_results/Experiment_2/"+experiment_name+"_I0_"+str(int(I0))+"_A_"+str(A)+".csv", stand_data)

    q, dq, t, stand_data = velocity_control(stand_data, motor, sensors, 0)
    print("Actuator at the initial position")
    motor.disable()
