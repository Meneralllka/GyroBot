import odrive
from odrive.enums import *
import time
import fibre

dev0 = odrive.find_any()
print(str(dev0.vbus_voltage))
dev0.clear_errors()
print(dev0.axis1.motor.error)
#dev0 = fibre.libfibre.find_devices(vid = 0x1209, pid = 0x0d32)
#dev0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
#dev0.axis1.motor.config.startup_motor_calibration = False
#dev0.axis1.motor.config.startup_sensorless_ramp = False

dev0.axis1.motor.config.current_lim = 0
dev0.axis1.motor.config.pre_calibrated = False

if dev0.axis1.current_state != AXIS_STATE_IDLE:
    print("lol")
    dev0.axis1.requested_state = AXIS_STATE_IDLE


dev0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
time.sleep(5)#this
print('I am here2!')
dev0.clear_errors()
print(str(odrive.utils.dump_errors(dev0)))
print(dev0.axis1.motor.error)

dev0.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL #this
#dev0.axis0.config.startup_sensorless_control = True #this
print('I am here3!')
print(str(odrive.utils.dump_errors(dev0))) #this
print('I am here4!')
dev0.axis0.controller.input_vel = 20
print(str(odrive.utils.dump_errors(dev0)))
time.sleep(5)

