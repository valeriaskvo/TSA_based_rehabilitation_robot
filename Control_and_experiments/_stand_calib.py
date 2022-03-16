import pickle

from numpy import disp
from can import CAN_Bus
from time import sleep, perf_counter
import numpy as np
from sensors import SensorRJ
from motors.gyems import GyemsDRC

def save_obj(obj, name ):
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

    global calib_data
    calib_data = load_obj("/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data")
    print('Calibration data is enable')

    return motor, sensors

stand_param = {'interface': 'can0', 'id_motor': 0x141, 'current_limit': 400}
motor, sensors = run_stand(stand_param)

calibration_speed = 70

moving_threshold = 3
time_threshold = 1

accurate_threshold = np.deg2rad(0.01)
gains = {"kp": 5,
         "kd": 2}

turns = 0

zero_disp, _ = sensors.read_encoders()
zero_q = motor.state['angle']
t_turn = perf_counter()

try:
    while turns < 3:
        speed = (-1)**turns * calibration_speed
        motor.set_speed(speed)
        
        cur_disp, _ = sensors.read_encoders()

        if zero_disp > cur_disp:
            zero_disp = cur_disp
            zero_q = motor.state['angle']

        t = perf_counter()
        if np.abs(zero_disp - cur_disp) > moving_threshold and (t - t_turn) > time_threshold:
            turns +=1
            t_turn = t
            print('Turn', turns)

    q, dq = motor.state['angle'], motor.state['speed']
    print("New zero position", zero_disp,"mm")
    print("Zero motor angle is", np.rad2deg(zero_q), "deg")

    while np.abs(q - zero_q) > accurate_threshold:
        q, dq = motor.state['angle'], motor.state['speed']
        e, de = zero_q - q, - dq

        v = gains["kp"]*e + gains["kd"]*de
        motor.set_speed(v)

    print("Final position error is", np.rad2deg(q - zero_q), "deg\n")

    print("Callibration is done!")
    print("Reboot stand physically \n")      

except KeyboardInterrupt:
    motor.pause()
    print("Exit...")
finally:
    motor.disable()