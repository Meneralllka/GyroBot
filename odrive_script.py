import odrive
from odrive.enums import *
import time

odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))

print("Erasing pre-exsisting configuration...")
try:
    odrv0.erase_configuration()
except Exception:
    pass

odrv0 = odrive.find_any()

odrv0.config.brake_resistance = 2.0  #(based on the resistor used for the Odrive) #this
odrv0.config.dc_max_positive_current = 20  #(whatever is the max current that the power supply or the battery can give) #this
odrv0.config.dc_max_negative_current = -1  #(whatever is the max current that the power supply or the battery can absorb) #this
odrv0.config.max_regen_current = 0  #(usually 0) #this
odrv0.config.dc_bus_undervoltage_trip_level = 8.0 #this
odrv0.config.dc_bus_overvoltage_trip_level = 25.5  #(the max current that the Odrive can handle) #this
odrv0.save_configuration()
print('Config 1 pass!')
odrv0.axis0.motor.config.motor_type = odrive.utils.MotorType.HIGH_CURRENT #this
odrv0.axis0.controller.config.pos_gain = 20 #this
odrv0.axis0.controller.config.vel_gain = 0.02 #this
odrv0.axis0.controller.config.vel_integrator_gain = 0.01 #this
odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.VELOCITY_CONTROL #this
odrv0.axis0.controller.config.vel_limit = 110 #this
odrv0.axis0.motor.config.current_lim = 60 #(depends on motor 80 for gt5323 80) #this
odrv0.axis0.motor.config.pole_pairs = 7 #this
odrv0.axis0.motor.config.direction = 1 #this
odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 325)  #(7 is the pole pairs of the motor and the 230 is kv) #this
print('I am here!')
odrv0.axis0.requested_state = odrive.utils.AxisState.MOTOR_CALIBRATION
time.sleep(5)#this
print('I am here2!')
print(str(odrive.utils.dump_errors(odrv0)))

odrv0.axis0.requested_state = odrive.utils.AxisState.SENSORLESS_CONTROL #this

odrv0.axis0.config.startup_sensorless_control = True #this
print('I am here3!')
print(str(odrive.utils.dump_errors(odrv0))) #this
print('I am here4!')
#odrv0.axis0.controller.input_vel = 50
print(str(odrive.utils.dump_errors(odrv0)))
time.sleep(5)


