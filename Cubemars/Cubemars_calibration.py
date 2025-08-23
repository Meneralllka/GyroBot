import can
import time
import RPi.GPIO as GPIO
from motor_init import Motor, pack_cmd, unpack_reply, enter_motor_mode, zero, exit_motor_mode

def calibrate_motors() -> tuple[can.Bus, list[Motor]]:
    """
    Initializes and calibrates motors with limit switch pins.
    Returns the CAN bus and list of calibrated motors.
    """
    # Initialize CAN bus
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
    except Exception as e:
        raise Exception(f"Failed to initialize CAN bus: {e}")

    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    
    # Initialize motors with their respective limit switch pins
    motors = [Motor(can_id=0x00, limit_switch_pin=12), Motor(can_id=0x01, limit_switch_pin=16)]
    
    # Setup GPIO pins for both motors
    for motor in motors:
        GPIO.setup(motor.limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Enter motor mode for all motors
    for motor in motors:
        enter_motor_mode(bus, motor)
        time.sleep(0.1)  # Small delay to ensure motor mode is set

    print("Starting calibration. Motors will move until limit switches are pressed.")
    print_position = False
    last_print_time = time.time()

    while any(not motor.limit_switch_pressed for motor in motors):
        for motor in motors:
            if not motor.limit_switch_pressed:
                # Check limit switch state
                if GPIO.input(motor.limit_switch_pin) == GPIO.LOW:
                    print(f"Motor {motor.can_id}: Limit switch pressed")
                    motor.limit_switch_pressed = True
                    if motor.motor_moving:
                        # Stop the motor
                        motor.v_in = 0.0
                        motor.kp_in = 0.0
                        motor.kd_in = 0.0
                        pack_cmd(bus, motor)
                        motor.motor_moving = False
                        print(f"Motor {motor.can_id} stopped due to limit switch")

                    # Zero the motor position
                    zero(bus, motor)
                    time.sleep(0.1)
                    #motor.p_in = 0.0
                    #motor.p_out = 0.0
                else:
                    # Move motor if limit switch not pressed
                    if not motor.motor_moving:
                        print(f"Starting motor {motor.can_id} movement...")
                        motor.v_in = 0.3  # Very slow speed
                        motor.kp_in = 0.0
                        motor.kd_in = 5.0
                        motor.motor_moving = True

            # Update motor commands
            if motor.motor_moving:
                pack_cmd(bus, motor)

        # Receive CAN messages
            msg = bus.recv(timeout=0.001)
            if msg:
                for motor in motors:
                    if msg.arbitration_id == motor.can_id:
                        try:
                            unpack_reply(motor, msg)
                        except IndexError:
                            print(f"Motor {motor.can_id}: unpack error, message is not full")
                        break

        # Print motor positions if enabled
        if print_position and time.time() - last_print_time >= 0.1:  # Print every 0.1s
            for motor in motors:
                print(f"Motor {motor.can_id} position: {motor.p_out:.3f}")
            last_print_time = time.time()

        time.sleep(0.001)  # Small delay to prevent CPU overload

    print("Calibration complete for all motors.")
    return bus, motors
