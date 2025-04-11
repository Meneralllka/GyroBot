import pygame
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time
import math
import odrive
from odrive.enums import *

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

# Goal positions for ±90°
def degrees_to_position(deg):
    return int((deg / 360.0) * 303454)

POS_90 = degrees_to_position(180)
NEG_90 = degrees_to_position(-180)

def move_motor(dxl_id, position):
    # Convert signed int to unsigned 32-bit
    if position < 0:
        position = (1 << 32) + position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Communication error for motor {dxl_id}")
    elif dxl_error != 0:
        print(f"Motor {dxl_id} error: {packetHandler.getRxPacketError(dxl_error)}")

time.sleep(10)

initialize_dynamixels()

for i in range(5):
    print(f"Cycle {i+1}")
    move_motor(0, POS_90)
    time.sleep(0.5)

    move_motor(1, POS_90)
    time.sleep(0.5)

    move_motor(0, NEG_90)
    time.sleep(0.5)

    move_motor(1, NEG_90)
    time.sleep(0.5)

# Disable torque before exit
for dxl_id in DXL_ID:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
print("Done.")
