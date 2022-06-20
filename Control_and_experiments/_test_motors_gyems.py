from can import CAN_Bus
from motors.gyems import GyemsDRC
from numpy import pi
import numpy as np

from time import perf_counter, sleep

def go_to_position(motor,position,current,threshold = pi/30):
    angle_0 = motor.state['angle']
    t_0 = perf_counter()
    d_angle = 0
    count = 0
    while np.abs(motor.state['angle']-position) > threshold:
        motor.set_current(current)
        t = perf_counter()
        d_angle += (motor.state['angle']-angle_0)/(t-t_0)
        count += 1
        angle_0 = motor.state['angle']
        t_0 = t

    motor.pause()
    return d_angle/count


motor_param = {'interface':'can0', 'id':0x141, 'current_limit':1000}
bus = CAN_Bus(interface = motor_param['interface'])

# sleep(3)
print('CAN BUS connected successfully')

motor = GyemsDRC(can_bus=bus, device_id=motor_param['id'])
motor.set_radians()
motor.current_limit = motor_param['current_limit']
motor.enable()
print('Motor is enable')

goal_current = 100
goal_angle = 50*2*pi
time_to_sleep = 5

try:
    speed = go_to_position(motor,goal_angle,goal_current)

    print("Speed =",speed/(2*pi))

    sleep(time_to_sleep)

except KeyboardInterrupt:
    motor.set_current(0)
    print('Motors stopped by keyboard')
finally:
    go_to_position(motor,0,-goal_current)
    motor.disable()
    print('Motor is disable')
