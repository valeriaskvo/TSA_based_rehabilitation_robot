import sys
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

from can import CAN_Bus
from motors.gyems import GyemsDRC
from sensors import SensorRJ
import numpy as np
from time import perf_counter
import pandas as pd
import pickle

import matplotlib.pyplot as plt
plt.style.use('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/rj_stand/paper_plots_style.mplstyle')

def load_calib(name = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data"):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

def save_calib(obj, name = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data"):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

class Stand_RJ:
    def __init__(self, stand_param = {'interface': 'can0', 'id_motor': 0x141,'current_limit': 400}):
                                
        self.stand_param = stand_param
        self.calib_data_path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data"
        self.calib_data = load_calib(self.calib_data_path)
        
        self.bus = CAN_Bus(interface=self.stand_param['interface'])
        print('CAN BUS connected successfully')

        self.motor = GyemsDRC(can_bus=self.bus, device_id=self.stand_param['id_motor'])
        self.motor.set_radians()
        self.motor.current_limit = self.stand_param['current_limit']
        self.motor.enable()
        print('Motor is enable')

        self.sensors = SensorRJ(self.bus)
        print('Sensors is enable')

        self.from_state_to_data = {"t": "Time [sec]",
                                   "q": "Motor angle [rad]",
                                   "dq": "Motor speed [rad/sec]",
                                   "x": "Linear encoder [mm]",
                                   "phi": "Joint angle [rad]",
                                   "I": "Motor current [units]",
                                   "tau": "Motor torque [Nm]",
                                   "F_tsa": "Force sensor TSA [N]",
                                   "F_hand": "Force sensor handle [N]"}
        
        self.state_headers = list(self.from_state_to_data.keys())
        self.state = dict((name, 0) for name in self.state_headers)
        self.state ['phi_prev'] = 0
        self.state ['dphi'] = 0

        self.data_headers =list(self.from_state_to_data.values())
        self.data = dict((name, []) for name in self.data_headers)
        
        self.collect_data_flag = True
                
        self.save_path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/"
        self.START_TIME = perf_counter()

        self.filter_force_sensor_flag = False
        self.filtering_N = 100

        self.filtering_sample = {"F_tsa": [],
                                 "F_hand": [],
                                 "tau": []}
        return

    def __del__(self):
        self.motor.disable()
        print('Motor is down')
        return

    def reset_data(self):
        self.data = dict((name, []) for name in self.data_headers)
        self.START_TIME = perf_counter()
        return

    def collect_data(self, with_reset = True):
        self.collect_data_flag = True
        if with_reset:
            self.reset_data()
        return
    
    def dont_collect_data(self, with_reset = True):
        self.collect_data_flag = False
        if with_reset:
            self.reset_data()
        return

    def save_data(self, data = {}, filename = "last_log"):
        if not data:
            data = self.data        
        data_to_save = pd.DataFrame.from_dict(data)
        data_to_save.to_csv(self.save_path+filename+".csv")
        return

    def get_state(self):
        t = perf_counter() - self.START_TIME
        q, dq, I = self.motor.state['angle'], self.motor.state['speed'], self.motor.state['current']
        x, phi = self.sensors.read_encoders()
        force_a, force_b = self.sensors.read_force()
        dt = t - self.state["t"]
        self.state["phi_prev"] = self.state["phi"]

        self.state["t"] = t
        self.state["q"] = q
        self.state["dq"] = dq
        self.state["x"] = x 
        self.state["phi"] = x / self.calib_data["R"]
        self.state["dphi"] = (self.state["phi"] - self.state["phi_prev"]) / dt
        self.state["I"] = I

        forces = {"F_tsa": force_a*self.calib_data["force_A"] - self.calib_data["bias_a"],
                  "F_hand": force_b*self.calib_data["force_B"] - self.calib_data["bias_b"],
                  "tau": I*self.calib_data["motor_K"]}

        if self.filter_force_sensor_flag:
            for key in forces.keys():
                self.filtering_sample[key].pop(0)
                self.filtering_sample[key].append(forces[key])
                self.state[key] = np.mean(self.filtering_sample[key])
        else:
            for key in forces.keys():
                self.state[key] = forces[key]

        if self.collect_data_flag:
            for state_idx in self.state_headers:
                data_idx = self.from_state_to_data[state_idx]                
                self.data[data_idx].append(self.state[state_idx])

        return


    def force_sensor_initialization(self):
        self.dont_collect_data()
        N = 0
        while N < self.filtering_N:
            self.get_state()
            for key in self.filtering_sample.keys():
                self.filtering_sample[key].append(self.state[key])
            N += 1
        
        self.filter_force_sensor_flag = True
        self.collect_data()
        return
            
    def motor_set_current(self, I):
        self.motor.set_current(I)
        self.get_state()
        return        

    def motor_set_speed(self, v):
        self.motor.set_speed(v)
        self.get_state()
        return

    def motor_position_control(self, q_des, dq_des = 0, gains = {"kp": 5}, v_max=500, threshold = np.pi/180):
        self.get_state()

        while np.abs(self.state["q"]-q_des)>threshold:
            e = q_des - self.state["q"]
            v = gains["kp"]*e

            if np.abs(v) > v_max:
                v = np.sign(v)*v_max

            self.motor_set_speed(v)
        
        self.motor.pause()
        print("\nMotor at the desired position", np.rad2deg(q_des), "[deg], with position error", np.rad2deg(self.state["q"]-q_des), "[deg]\n")
        return

    def motor_pause(self):
        self.motor.pause()
        print("Exit...")
        return

    def show_state(self, data, desired = {}):
        t = np.asarray(data[self.from_state_to_data["t"]])
        q = np.asarray(data[self.from_state_to_data["q"]])
        dq = np.asarray(data[self.from_state_to_data["dq"]])
        phi = np.asarray(data[self.from_state_to_data["phi"]])
        tau = np.asarray(data[self.from_state_to_data["tau"]])
        F_tsa = np.asarray(data[self.from_state_to_data["F_tsa"]])
        F_hand = np.asarray(data[self.from_state_to_data["F_hand"]])

        f, axs = plt.subplots(3, 2, sharex=True, figsize=(10,6))

        axs[0,0].plot(t, q / (2*np.pi), color = 'red')
        axs[0,0].set(ylabel='Motor angle [rev]')
        
        axs[1,0].plot(t, dq, color = 'red')
        axs[1,0].set(ylabel='Motor speed [rad/sec]')

        axs[2,0].plot(t, np.rad2deg(phi), color = 'red')
        axs[2,0].set(ylabel='Rotation joint angle [deg]', xlabel = self.from_state_to_data["t"])

        axs[0,1].plot(t, tau, color = 'red')
        axs[0,1].set(ylabel=self.from_state_to_data["tau"])
        
        axs[1,1].plot(t, F_tsa, color = 'red')
        axs[1,1].set(ylabel=self.from_state_to_data["F_tsa"])

        axs[2,1].plot(t, F_hand, color = 'red')
        axs[2,1].set(ylabel=self.from_state_to_data["F_hand"], xlabel = self.from_state_to_data["t"])
        
        axes_plot = {"q": axs[0,0],
                     "dq": axs[1,0],
                     "phi": axs[2,0],
                     "tau": axs[0,1],
                     "F_tsa": axs[1,1],
                     "F_hand": axs[2,1]}
        
        for key in desired:
            if key in axes_plot:
                if isinstance(desired[key], (int, float)):
                    des_y = np.zeros(t.shape) + desired[key]
                else:
                    t = desired["t"]
                    des_y = desired[key]

                axes_plot[key].plot(t, des_y, color = 'black', linestyle = '--')

        plt.show()
        return

    def stand_disable(self, filename = "", zero_q = 0, show_state = True, desired = {}):       
        if self.collect_data_flag:
            if filename != "":
                self.save_data(filename=filename)
            else:
                self.save_data()

        data_to_show = {}
        if show_state and self.collect_data_flag:
            data_to_show = self.data

        self.dont_collect_data()
        self.motor_position_control(q_des=zero_q)

        if show_state and data_to_show:
            self.show_state(data_to_show, desired=desired)
        return





