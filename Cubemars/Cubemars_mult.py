import can
import math
import time
from dataclasses import dataclass

# Constants
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -30.0
V_MAX = 30.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0
NUM_MOTORS = 2

@dataclass
class Motor:
    can_id: int
    p_in: float = 0.0
    v_in: float = 0.0
    kp_in: float = 30.0
    kd_in: float = 1.0
    t_in: float = 0.0
    p_out: float = 0.0
    v_out: float = 0.0
    t_out: float = 0.0
    s: int = 1
    prev_time: float = 0.0

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Converts a float to an unsigned int, given range and number of bits."""
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return int((x - offset) * 4095.0 / span)
    elif bits == 16:
        return int((x - offset) * 65535.0 / span)
    return 0

def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """Converts unsigned int to float, given range and number of bits."""
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return (float(x_int) * span / 4095.0) + offset
    elif bits == 16:
        return (float(x_int) * span / 65535.0) + offset
    return 0.0

def pack_cmd(bus: can.Bus, motor: Motor) -> None:
    """Packs and sends a CAN command packet for the motor."""
    p_des = max(min(motor.p_in, P_MAX), P_MIN)
    v_des = max(min(motor.v_in, V_MAX), V_MIN)
    kp = max(min(motor.kp_in, KP_MAX), KP_MIN)
    kd = max(min(motor.kd_in, KD_MAX), KD_MIN)
    t_ff = max(min(motor.t_in, T_MAX), T_MIN)

    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    data = [
        p_int >> 8,
        p_int & 0xFF,
        v_int >> 4,
        ((v_int & 0xF) << 4) | (kp_int >> 8),
        kp_int & 0xFF,
        kd_int >> 4,
        ((kd_int & 0xF) << 4) | (t_int >> 8),
        t_int & 0xFF
    ]

    msg = can.Message(arbitration_id=motor.can_id, data=data, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.001)

def unpack_reply(motor: Motor, msg: can.Message) -> None:
    """Unpacks a CAN reply packet and updates motor output values."""
    buf = msg.data
    p_int = (buf[1] << 8) | buf[2]
    v_int = (buf[3] << 4) | (buf[4] >> 4)
    i_int = ((buf[4] & 0xF) << 8) | buf[5]

    motor.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
    motor.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
    motor.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12)

def enter_motor_mode(bus: can.Bus, motor: Motor) -> None:
    """Sends command to enter motor mode."""
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    msg = can.Message(arbitration_id=motor.can_id, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"Motor {motor.can_id} ENTER")

def exit_motor_mode(bus: can.Bus, motor: Motor) -> None:
    """Sends command to exit motor mode."""
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    msg = can.Message(arbitration_id=motor.can_id, data=data, is_extended_id=False)
    bus.send(msg)

def zero(bus: can.Bus, motor: Motor) -> None:
    """Sends command to zero the motor."""
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
    msg = can.Message(arbitration_id=motor.can_id, data=data, is_extended_id=False)
    bus.send(msg)

def main():
    # Initialize CAN bus
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
    except Exception as e:
        print(f"Failed to initialize CAN bus: {e}")
        return

    # Initialize motors
    motors = [Motor(0x00), Motor(0x01)]

    # Open CSV file for logging
    with open('motor_data.csv', 'w') as f:
        f.write('Timestamp,Motor0_p_in,Motor0_p_out,Motor1_p_in,Motor1_p_out\n')
    
    # Enter motor mode for all motors
    for motor in motors:
        enter_motor_mode(bus, motor)

    start_time = time.time()
    
    try:
        while True:
            current_time = time.time() - start_time
            
            # Update motor commands
            for i, motor in enumerate(motors):
                p_step = 6.28 * math.sin(3.14 * 0.5 * (time.time() - start_time))
                motor.p_in = max(min(p_step / (i + 1), P_MAX), P_MIN)
                #if i == 0:
                pack_cmd(bus, motor)
            
            # Receive CAN messages
                msg = bus.recv(timeout=0.0001)
                if msg:
                    for motor in motors:
                        if msg.arbitration_id == motor.can_id:
                            unpack_reply(motor, msg)
                            print(f'Motor: {motor.can_id}, Position: {motor.p_out}, Ref: {motor.p_in}')
                            break
            
            # Log data to CSV
            with open('motor_data.csv', 'a') as f:
                f.write(f'{current_time},{motors[0].p_in},{motors[0].p_out},{motors[1].p_in},{motors[1].p_out}\n')
            
            time.sleep(0.0001)

    except KeyboardInterrupt:
        print("Shutting down...")
        for motor in motors:
            exit_motor_mode(bus, motor)
        bus.shutdown()

if __name__ == "__main__":
    main()
