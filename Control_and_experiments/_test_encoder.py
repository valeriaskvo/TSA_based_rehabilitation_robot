from can import CAN_Bus
from time import sleep, perf_counter
from struct import unpack
from sensors import SensorRJ
import pickle


def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

# bus = CAN_Bus(interface = 'can0')


# protocol = {'encoders': b'\x01', 
#             'reset': b'\x02',
#             'config': b'\x03',
#             'torque': b'\x04', 
#             }

# message = protocol['encoders'] + 7*b'\x00'
# device_id = 0x1 ### ИД девайса проверила, не работает если ставишь ид моторов, все норм
#                 ### Соосность датчиков нужно фиксить

# # # TODO: Проверить датчики силы, можно ли подключить два и тд
# # # TODO: Переделать либу, с корректировкой по датчикам

# # try:
# #     while 1:
# #         bus.send_bytes(device_id, message)
# #         can_id, can_dlc, can_data = bus.recive_frame()
# #         sensor_a = unpack('h',can_data[:2])
# #         sensor_b = unpack('h',can_data[2:4])


# #         print(sensor_a[0], sensor_b[0])
# #         sleep(0.01)
# # except KeyboardInterrupt:
# #     print("Exit...")

# message = protocol['torque'] + 7*b'\x00'
# try:
#     while 1:
#         bus.send_bytes(device_id, message)
#         can_id, can_dlc, can_data = bus.recive_frame()
#         sensor_a = unpack('f',can_data[:4])
#         sensor_b = unpack('f',can_data[4:])        


#         print(sensor_a[0],sensor_b[0])
#         sleep(0.05)
# except KeyboardInterrupt:
#     print("Exit...")

calib_data = load_obj("/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/calib_data")

bus = CAN_Bus(interface = 'can0')

# sens_prop = {   "encoder_a": "lin",
#                 "encoder_b": "rot"}

calibration_data = {"encoder_a": 1,
                    "encoder_b": 1,
                    "force_a": 1,
                    "force_b": 1}
sensors = SensorRJ(bus)

t = perf_counter()

dt = 0
n = 1000


try:
    while True:
        linear_displacement, rotation_angle = sensors.read_encoders()
        force_a, force_b = sensors.read_force()
        ti = perf_counter()
        dt = dt + ti - t
        t = ti
        # msg = "Zero B: {:-f}, Force B: {:-f}".format(force_b, force_b*calib_data["force_B"]/9.8)
        msg = "Force A: {:f}; Force B: {:-f}; Linear encoder: {:-f}; Angular encoder: {:-f}".format(force_a*calib_data["force_A"], force_b*calib_data["force_B"], linear_displacement, rotation_angle*180/3.1415926)
        
        # msg = "Linear encoder: {:-f}; Angular encoder: {:-f}".format(linear_displacement, rotation_angle*180/3.1415926)
        print(msg)

        # print(rotation_angle*180/3.1415926)

        sleep(0.1)
except KeyboardInterrupt:
    print("Exit...")

# T = dt/n
# w = 1/T

# print(T, "sec")
# print(w,"Hz")
