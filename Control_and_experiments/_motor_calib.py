import pickle
from can import CAN_Bus
from time import sleep, perf_counter
import numpy as np
from motors.gyems import GyemsDRC
from numpy import pi

def save_obj(obj, name ):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

def get_state(motor):
    return motor.state["angle"], motor.state["speed"], motor.state["current"]

def velocity_control(I_data, q_data, m_data, motor, q_des, m_i, dq_des=0, tf = 5, gains={"kp": 1.5, "kd": 0.8}, v_max=400, threshold=np.pi/360):
    q, dq, I = get_state(motor)
    de_i = 100
    e_i = 0

    while np.abs(q-q_des) > threshold and de_i > threshold:
        e, de = q_des - q, dq_des - dq
        de_i = np.abs(e_i - e)
        e_i = e

        v = gains["kp"]*e + gains["kd"]*de
        if np.abs(v) > v_max:
            v = np.sign(v)*v_max

        motor.set_speed(v)
        q, dq, I = get_state(motor)
        

    print("Motor at the desired position")
    sleep(3)
    t0 = perf_counter()
    t = 0
    while t<tf:
        e, de = q_des - q, dq_des - dq

        v = gains["kp"]*e + gains["kd"]*de
        if np.abs(v) > v_max:
            v = np.sign(v)*v_max

        motor.set_speed(v)
        q, dq, I = get_state(motor)
        t = perf_counter()-t0
        I_data.append(I)
        q_data.append(q)
        m_data.append(m_i)

    motor.set_current(0)

    return I_data, q_data, m_data

def calculate_K(I_data, q_data, m_data, l):
    g = 9.8
    I = np.array(I_data)
    q = np.array(q_data)
    m = np.array(m_data)
    k = m*g*l*np.sin(q)/I
    return np.mean(k)

motor_param = {'interface':'can0', 'id':0x141, 'current_limit':500}
bus = CAN_Bus(interface = motor_param['interface'])

calib_data = load_obj("calib_data")
print(calib_data)

calib_data["motor_K"] = calib_data["motor_K_34"]
print(calib_data)
save_obj(calib_data,"calib_data")

# motor = GyemsDRC(can_bus=bus, device_id=motor_param['id'])
# motor.set_radians()
# motor.current_limit = motor_param['current_limit']
# motor.enable()

# tf = 10

# t0 = perf_counter()
# t = 0
# q_des = np.pi/2

# # m = [(16 + 24 + 102 + 99*2)*10**(-3), (16 + 24 + 102 + 99)*10**(-3), (16 + 24 + 102)*10**(-3)]
# # l = 100 *10**(-3)

# # m = [120 * 10**(-3), 2*120 * 10**(-3), 3*120 * 10**(-3)]
# m = [ 57* 10**(-3), 2*57 * 10**(-3), 137* 10**(-3), (2*57 +137) * 10**(-3)]
# l = [100 *10**(-3), 150 *10**(-3)]

# calib_data["motor_K_49"] = calib_data["motor_K"]

# I_data, q_data, m_data = [], [], []
# try:
#     for j in range(len(l)):
#         print("Put the weights on L=", l[j]*10**3, "mm")
#         for i in range(len(m)):
#             _ = input("Put weight "+str(i+1)+" on pendulum")
#             I_data, q_data, m_data = velocity_control(I_data, q_data, m_data, motor, q_des, m[i])
#             k = calculate_K(I_data, q_data, m_data, l[j])
#             print("K = ", k)
#             calib_data["motor_K_34"] = k
#             save_obj(calib_data,"calib_data")

# except KeyboardInterrupt:
#     print("Motor stop!")
# finally:
#     # velocity_control(I_data, q_data, motor, 0)
#     motor.set_current(0)
#     motor.disable()
        
#     save_obj(calib_data,"calib_data")

