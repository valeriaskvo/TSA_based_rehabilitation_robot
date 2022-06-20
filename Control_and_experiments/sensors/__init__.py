import numpy as np
from can import CAN_Bus
from math import pi
from struct import unpack

def rot_lin_scale(sensor):
    if sensor == "rot":
        return 2*pi /(2048 * 4)
    else:
        return 1 / 360 * 25.4 / 4

class SensorRJ:
    def __init__(self, can_bus = None,  sens_prop = {"encoder_a": "lin",
                                                     "encoder_b": "rot"},
                                        calibration_data = {"encoder_a": None,
                                                            "encoder_b": None,
                                                            "force_a": 1,
                                                            "force_b": 1}):
        if not can_bus:
            print('Provide can_bus as argument')
            self.__del__()

        self.transmiter = can_bus.send_bytes
        self.reciver = can_bus.recive_frame
    
        self.protocol = {'encoders': b'\x01', 
            'reset': b'\x02',
            'config': b'\x03',
            'force': b'\x04', 
            }
        
        self.device_id = 0x1

        self.state = {  "encoder_a": 0.,
                        "encoder_b":0.,
                        "force_a": 0.,
                        "force_b":0.}

        self.scale = calibration_data
        if not self.scale["encoder_a"]:
            self.scale["encoder_a"] = rot_lin_scale(sens_prop["encoder_a"])


        if not self.scale["encoder_b"]:
            self.scale["encoder_b"] = rot_lin_scale(sens_prop["encoder_b"])




    def read_encoders(self):
        message = self.protocol['encoders'] + 7*b'\x00'
        self.transmiter(self.device_id, message)
        _, _, can_data = self.reciver()
        sensor_a = unpack('h',can_data[:2])
        sensor_b = unpack('h',can_data[2:4])

        self.state["encoder_a"] = sensor_a[0] * self.scale["encoder_a"]
        self.state["encoder_b"] = sensor_b[0] * self.scale["encoder_b"]
        return self.state["encoder_a"], self.state["encoder_b"]

    def read_force(self):
        message = self.protocol['force'] + 7*b'\x00'
        self.transmiter(self.device_id, message)
        _, _, can_data = self.reciver()
        sensor_a = unpack('f',can_data[:4])
        sensor_b = unpack('f',can_data[4:])

        self.state["force_a"] = sensor_a[0] * self.scale["force_a"]
        self.state["force_b"] = sensor_b[0] * self.scale["force_b"]
        return self.state["force_a"], self.state["force_b"]  
        




    


    