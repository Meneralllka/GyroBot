import pygame
from RPLCD.i2c import CharLCD
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time
import math
import odrive
from odrive.enums import *
from datetime import datetime  # Added for timestamp

LCD_ENABLE = 0
if LCD_ENABLE:
    lcd = CharLCD('PCF8574', 0x27)
    lcd.write_string("Loading...")
    time.sleep(3)
    lcd.clear()
# Dynamixel settings
MY_DXL = 'P_SERIES'  # Dynamixel model type
ADDR_TORQUE_ENABLE = 512
ADDR_GOAL_POSITION = 564
ADDR_PRESENT_POSITION = 580
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
DXL_MINIMUM_POSITION_VALUE = -303454
DXL_MAXIMUM_POSITION_VALUE = 303454
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'
DXL_ID = [0, 1]  # Dynamixel IDs
TORQUE_ENABLE = 1  # Enable torque
TORQUE_DISABLE = 0  # Disable torque

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

t1 = 1.0
t2 = 3.0
w1 = 2 * math.pi / t1
w11 = math.pi
w2 = 2 * math.pi / t2
amp = 0.5
odrive_init = False
logging_active = False  # Track logging state
log_file = None  # File handle for logging


def degrees_to_position(deg):
    return int((deg / 360.0) * 303454)


def initialize_odrive():
    global odrive_init
    odrive_init = True
    odrv0 = odrive.find_any()
    print(str(odrv0.vbus_voltage))

    print("Erasing pre-exsisting configuration...")
    try:
        odrv0.erase_configuration()
    except Exception:
        pass

    odrv0 = odrive.find_any()

    odrv0.config.brake_resistance = 2.0
    odrv0.config.dc_max_positive_current = 20
    odrv0.config.dc_max_negative_current = -1
    odrv0.config.max_regen_current = 0
    odrv0.config.dc_bus_undervoltage_trip_level = 8.0
    odrv0.config.dc_bus_overvoltage_trip_level = 25.5
    odrv0.save_configuration()
    print('Config 1 pass!')
    odrv0.axis0.motor.config.motor_type = odrive.utils.MotorType.HIGH_CURRENT
    odrv0.axis0.controller.config.pos_gain = 20
    odrv0.axis0.controller.config.vel_gain = 0.02
    odrv0.axis0.controller.config.vel_integrator_gain = 0.01
    odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.VELOCITY_CONTROL
    odrv0.axis0.controller.config.vel_limit = 110
    odrv0.axis0.motor.config.current_lim = 60
    odrv0.axis0.motor.config.pole_pairs = 7
    odrv0.axis0.motor.config.direction = 1
    odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 325)
    print('I am here!')
    odrv0.axis0.requested_state = odrive.utils.AxisState.MOTOR_CALIBRATION
    time.sleep(5)
    print('I am here2!')
    print(str(odrive.utils.dump_errors(odrv0)))

    odrv0.axis0.requested_state = odrive.utils.AxisState.SENSORLESS_CONTROL
    odrv0.axis0.config.startup_sensorless_control = True
    print('I am here3!')
    print(str(odrive.utils.dump_errors(odrv0)))
    print('I am here4!')
    return odrv0


def initialize_dynamixels():
    """Initialize the Dynamixel motors."""
    if not portHandler.openPort():
        print("Failed to open the port")
        quit()
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate")
        quit()
    for dxl_id in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to enable torque for motor {dxl_id}")
        elif dxl_error != 0:
            print(f"Error enabling torque for motor {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Dynamixel motor {dxl_id} is connected and torque enabled")


def move_motors(axis0_value, axis1_value):
    """Convert axis values to Dynamixel positions and move motors."""
    positions = [axis0_value, axis1_value]
    for i, value in enumerate(positions):
        position = int(value)
        param_goal_position = [
            DXL_LOBYTE(DXL_LOWORD(position)),
            DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)),
            DXL_HIBYTE(DXL_HIWORD(position))
        ]
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
        if not dxl_addparam_result:
            print(f"Failed to add parameter for motor {DXL_ID[i]}")
            return False
    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to send goal positions: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False

    return True


def move_motor_pattern(dxl_id, position):
    # Convert signed int to unsigned 32-bit
    if position < 0:
        position = (1 << 32) + position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Communication error for motor {dxl_id}")
    elif dxl_error != 0:
        print(f"Motor {dxl_id} error: {packetHandler.getRxPacketError(dxl_error)}")


def read_motor_positions():
    """Read the present positions of the Dynamixel motors."""
    present_positions = []
    for dxl_id in DXL_ID:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id,
                                                                                       ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to read position for motor {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
            present_positions.append(0)
        elif dxl_error != 0:
            print(f"Error reading position for motor {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
            present_positions.append(0)
        else:
            # Convert to signed int if necessary
            if dxl_present_position > (1 << 31):
                dxl_present_position -= (1 << 32)
            present_positions.append(dxl_present_position)
    return present_positions


# Initialize pygame for joystick input
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

initialize_dynamixels()

POS_90 = degrees_to_position(180)
NEG_90 = degrees_to_position(-180)
input_vel_prev = None

try:
    while True:
        pygame.event.pump()
        hat = joystick.get_hat(0)
        axis_0 = joystick.get_axis(1)  # Axis 0
        axis_1 = joystick.get_axis(3)  # Axis 1
        axis_5 = joystick.get_axis(5)  # Axis 5 (trigger to enable motion)
        axis_6 = joystick.get_axis(4)
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        # Convert axis values from [-1,1] to [-40,40]
        axis_0_value = int(axis_0 * 90)
        axis_1_value = int(axis_1 * 90)
        try:
            if odrv0.axis0.controller.input_vel != input_vel_prev and LCD_ENABLE:
                lcd.clear()
                lcd.write_string('Gyro Vel: ' + str(odrv0.axis0.controller.input_vel))
                input_vel_prev = odrv0.axis0.controller.input_vel
        except Exception:
            pass
        # Handle button 10 (start logging)
        if buttons[10] == 1 and not logging_active:
            logging_active = True
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = open(f"joystick_log_{timestamp}.txt", "w")
            log_file.write("Time,Goal_Position_0,Goal_Position_1,Present_Position_0,Present_Position_1\n")
            print(f"Started logging to joystick_log_{timestamp}.txt")

        # Handle button 11 (stop logging)
        if buttons[11] == 1 and logging_active:
            logging_active = False
            if log_file:
                log_file.close()
                log_file = None
                print("Stopped logging and saved file")

        if buttons[12] == 1 and odrive_init == False:
            odrv0 = initialize_odrive()
            odrv0.axis0.controller.input_vel = 0
            print(str(odrive.utils.dump_errors(odrv0)))
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN and (
                    event.button in [7] and event.button not in [6]) and odrive_init == True:
                if odrv0.axis0.controller.input_vel <= 70:
                    odrv0.axis0.controller.input_vel += 10
                    odrive.utils.dump_errors(odrv0)
                    print(odrv0.axis0.controller.input_vel)
            elif event.type == pygame.JOYBUTTONDOWN and (
                    event.button not in [7] and event.button in [6]) and odrive_init == True:
                if odrv0.axis0.controller.input_vel > 0:
                    odrv0.axis0.controller.input_vel -= 10
                    odrive.utils.dump_errors(odrv0)
                    print(odrv0.axis0.controller.input_vel)

        if axis_5 > 0.8:
            if hat[0] == 0 and hat[1] == 0:  # Move motors only if axis 5 is pressed
                goal_pos0 = int((axis_0_value / 180) * DXL_MAXIMUM_POSITION_VALUE)
                goal_pos1 = int((axis_1_value / 180) * DXL_MAXIMUM_POSITION_VALUE)
                move_motors(goal_pos0, goal_pos1)
            if hat[0] == -1 and hat[1] == 0:
                time_start = time.time()
                while (time.time() - time_start < (t1 + t2) / 2):
                    time_now = time.time() - time_start
                    if time_now < t1 / 2:
                        pos = -amp * DXL_MAXIMUM_POSITION_VALUE * math.cos(w1 * time_now)
                    elif time_now > t1 / 2:
                        pos = amp * DXL_MAXIMUM_POSITION_VALUE * math.cos(w2 * (time_now - t1 / 2))
                    else:
                        pos = 0
                    move_motors(pos, 0)
                    # Log when hat button is pressed
                    if logging_active and log_file:
                        current_time = time.time()
                        present_positions = read_motor_positions()
                        log_file.write(f"{current_time},{pos},0,{present_positions[0]},{present_positions[1]}\n")
            elif hat[0] == 1 and hat[1] == 0:
                time_start = time.time()
                while (time.time() - time_start < t1):
                    time_now = time.time() % 10000
                    pos = amp * DXL_MAXIMUM_POSITION_VALUE * math.cos(w11 * time_now)
                    pos1 = -amp * 0.111 * DXL_MAXIMUM_POSITION_VALUE * math.cos(w11 * time_now)
                    move_motors(pos, pos1)
                    # Log when hat button is pressed
                    if logging_active and log_file:
                        current_time = time.time()
                        present_positions = read_motor_positions()
                        log_file.write(f"{current_time},{pos},{pos1},{present_positions[0]},{present_positions[1]}\n")
            elif hat[0] == 0 and hat[1] == -1:
                time_start = time.time()
                while (time.time() - time_start < (t1 + t2) / 2):
                    time_now = time.time() - time_start
                    if time_now < t1 / 2:
                        pos = 0.667 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w1 * time_now) + 0.5)
                        pos2 = 0.5 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w1 * time_now) + 1)
                    elif time_now > t1 / 2:
                        pos = -0.667 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w2 * (time_now - t1 / 2)) - 0.5)
                        pos2 = -0.5 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w2 * (time_now - t1 / 2)) - 1)
                    else:
                        pos = 0
                    move_motors(pos, -pos)
                    # Log when hat button is pressed
                    if logging_active and log_file:
                        current_time = time.time()
                        present_positions = read_motor_positions()
                        log_file.write(f"{current_time},{pos},{-pos},{present_positions[0]},{present_positions[1]}\n")
            elif hat[0] == 0 and hat[1] == 1:
                time_start = time.time()
                while (time.time() - time_start < (t1 + t2) / 2):
                    time_now = time.time() - time_start
                    if time_now < t1 / 2:
                        pos = 0.667 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w1 * time_now) + 0.5)
                        pos2 = 0.5 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w1 * time_now) + 1)
                    elif time_now > t1 / 2:
                        pos = -0.667 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w2 * (time_now - t1 / 2)) - 0.5)
                        pos2 = -0.5 * amp * DXL_MAXIMUM_POSITION_VALUE * (math.cos(w2 * (time_now - t1 / 2)) - 1)
                    else:
                        pos = 0
                    move_motors(pos, pos)
                    # Log when hat button is pressed
                    if logging_active and log_file:
                        current_time = time.time()
                        present_positions = read_motor_positions()
                        log_file.write(f"{current_time},{pos},{pos},{present_positions[0]},{present_positions[1]}\n")

        if buttons[4] == 1:
            for i in range(5):
                print(f"Cycle {i + 1}")
                move_motor_pattern(0, POS_90)
                time.sleep(0.8)

                move_motor_pattern(1, POS_90)
                time.sleep(0.8)

                move_motor_pattern(0, NEG_90)
                time.sleep(0.8)

                move_motor_pattern(1, NEG_90)
                time.sleep(0.8)

except KeyboardInterrupt:
    print("Exiting...")
    for dxl_id in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    if logging_active and log_file:
        log_file.close()  # Ensure file is closed on exit
    pygame.quit()