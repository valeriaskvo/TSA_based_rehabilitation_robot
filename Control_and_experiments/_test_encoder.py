from can import CAN_Bus
from time import sleep, perf_counter
from struct import unpack
from sensors import SensorRJ

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
        # print("A:", force_a, "B:", force_b)
        print(linear_displacement, rotation_angle*180/3.14)

        sleep(0.05)
except KeyboardInterrupt:
    print("Exit...")

# T = dt/n
# w = 1/T

# print(T, "sec")
# print(w,"Hz")
