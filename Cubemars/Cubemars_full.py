import can
import time
import RPi.GPIO as GPIO
import sys
import math
import pygame
import select
import odrive
from odrive.enums import *
from Cubemars_calibration import calibrate_motors
from trajectory import generate_quintic_trajectory
from motor_init import Motor, pack_cmd, unpack_reply, exit_motor_mode, zero

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def start():
    try:
        #pygame.init()
        #joystick = pygame.joystick.Joystick(0)
        #joystick.init()
        # Calibrate motors first
        bus, motors = calibrate_motors()
        print("Calibration complete. Starting trajectory for both motors.")

        # Define trajectory parameters for both motors
        trajectories = []
        end_pos = -3.14 * 0.75  # Desired end position for both motors
        duration = 1.5          # Trajectory duration in seconds
        msg = bus.recv(timeout=0.001)
        # Generate trajectories for each motor
        for motor in motors:
            start_pos = 0#motor.p_out  # Current position after calibration
            print(motor.can_id, motor.p_in, motor.p_out)
            coeffs = generate_quintic_trajectory(start_pos, end_pos, duration)
            print(motor.can_id, coeffs)
        t = time.time()
        print("2 sec rest")
        while time.time() - t < 0.25:
            for motor in motors:
                motor.v_in = 0
                motor.kp_in = 0.0
                motor.kd_in = 2.0
                pack_cmd(bus, motor)
                msg = bus.recv(timeout=0.001)
                if msg:
                #print(msg)
                    for motor in motors:
                        if msg.arbitration_id == motor.can_id:
                            try:
                                unpack_reply(motor, msg)
                                #if motor.can_id == 1:
                                print(f"Motor {motor.can_id} position: {motor.p_out:.3f}")
                            except IndexError:
                                print(f"Motor {motor.can_id}: unpack error")
                            break
        #time.sleep(2)
        print("now we move")
        # Control loop to follow trajectories for both motors
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                t = time.time() - start_time
                for i, motor in enumerate(motors):
                    #print(coeffs)
                    motor.p_in = coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5 # Set desired position
                    #motor.v_in = vel_func(t)  # Set desired velocity
                    motor.kp_in = 30.0        # Position gain
                    motor.kd_in = 2.0         # Damping gain
                    pack_cmd(bus, motor)      # Send command to motor
                    #time.sleep(0.01)
                # Receive and process feedback for both motors
                    msg = bus.recv(timeout=0.0001)
                    if msg:
                        for motor in motors:
                            if msg.arbitration_id == motor.can_id:
                                try:
                                    unpack_reply(motor, msg)
                                    print(f"Motor {motor.can_id} position: {motor.p_out:.3f}, Target position: {motor.p_in:.3f}")
                                except IndexError:
                                    print(f"Motor {motor.can_id}: unpack error")
                                break
                #print(t, motors[0].p_in, motors[0].p_out, motors[1].p_out)
                #time.sleep(0.01)  # Control loop frequency

        except KeyboardInterrupt:
            print("Trajectory interrupted.")

        finally:
            # Stop both motors
            zero(bus, motors[0])
            time.sleep(0.001)
            zero(bus, motors[1])
            time.sleep(0.001)
            for motor in motors:
                motor.p_in = 0 #-3.14*0.75
                motor.v_in = 0.0
                motor.kp_in = 30.0
                motor.kd_in = 2.0
                pack_cmd(bus, motor)
                #exit_motor_mode(bus, motor)
                time.sleep(0.001)
                msg = bus.recv(timeout=0.0001)
                if msg:
                    #print(msg)
                    for motor in motors:
                        if msg.arbitration_id == motor.can_id:
                            try:
                                unpack_reply(motor, msg)
                                print(f"Motor {motor.can_id} position: {motor.p_out:.3f}")
                            except IndexError:
                                print(f"Motor {motor.can_id}: unpack error")
                            break
            #bus.shutdown()
            #GPIO.cleanup()
            return [bus, motors]
            

    except Exception as e:
        print(f"Calibration or trajectory failed: {e}")
        GPIO.cleanup()

def initialize_odrive():
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
    odrv0.config.max_regen_current = 0.5
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


def main():
    m1 = -14
    c1 = math.pi/2
    d1 = -math.pi/2
    t1 = 0.2
    t2 = -t1 + 2*(d1-c1)/m1
    a1 = 0.5 * m1/(t1 - t2)
    move_en = False
    move_mode = 0
    print(m1, c1, d1, t1, t2, a1)
    odrv0 = initialize_odrive()
    time.sleep(10)
    bus, motors = start()
    try:
        while True:
            pygame.event.pump()
            hat = joystick.get_hat(0)
            axis_0 = joystick.get_axis(1)  # Axis 0
            axis_1 = joystick.get_axis(3)  # Axis 1
            axis_2 = joystick.get_axis(4)  
            axis_3 = joystick.get_axis(5)  # Axis 5 (trigger to enable motion)
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            # Convert axis values from [-1,1] to [-40,40]
            axis_0_value = max(min(axis_0 * 3.14 * 0.5, 3.14*0.5), -3.14*0.5)
            axis_1_value = max(min(axis_1 * 3.14 * 0.5, 3.14*0.5), -3.14*0.5) 
            msg = bus.recv(timeout=0.0001)
            #print(axis_0_value, axis_1_value)
            vals = [axis_0_value, axis_1_value]
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN and (event.button in [7] and event.button not in [6]):
                    if odrv0.axis0.controller.input_vel <= 70:
                        odrv0.axis0.controller.input_vel += 10 
                        odrive.utils.dump_errors(odrv0)
                        print(odrv0.axis0.controller.input_vel)
                elif event.type == pygame.JOYBUTTONDOWN and (event.button not in [7] and event.button in [6]):
                    if odrv0.axis0.controller.input_vel > 0:
                        odrv0.axis0.controller.input_vel -= 10 
                        odrive.utils.dump_errors(odrv0)
                        print(odrv0.axis0.controller.input_vel)
        
            for i, motor in enumerate(motors):
                    #print(coeffs)
                    if axis_3 > 0.8 or move_en:
                        if hat[0] == -1 and hat[1] == 0 and not move_en:
                            time_start = time.time()
                            move_en = True
                            move_mode = 1
                            coeffs = generate_quintic_trajectory(d1, c1, 2 - t2)
                        if hat[0] == 1 and hat[1] == 0 and not move_en:
                            time_start = time.time()
                            move_en = True
                            move_mode = 2
                            coeffs = generate_quintic_trajectory(d1, c1, 2 - t2)
                        if hat[0] == 0 and hat[1] == -1 and not move_en:
                            time_start = time.time()
                            move_en = True
                            move_mode = 3
                            coeffs = generate_quintic_trajectory(d1, c1, 2 - t2)
                        if hat[0] == 0 and hat[1] == 1 and not move_en:
                            time_start = time.time()
                            move_en = True
                            move_mode = 4
                            coeffs = generate_quintic_trajectory(d1, c1, 2 - t2)
                        if not move_en:
                            motor.p_in = vals[i] # Set desired position
                            #motor.v_in = vel_func(t)  # Set desired velocity
                            motor.kp_in = 30.0        # Position gain
                            motor.kd_in = 2.0         # Damping gain
                            pack_cmd(bus, motor)      # Send command to motor
                        elif time.time() - time_start < 3 and move_en:
                            time_now = time.time() - time_start
                            if time_now <= t1:
                                pos = m1*time_now + c1
                            elif t1 < time_now <= t2:
                                pos = a1*(time_now - t2)**2 + d1
                            elif time_now > t2:
                                t = time_now - t2
                                if coeffs is not None:
                                    pos = coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5
                                    print(pos)
                            if move_mode == 1:
                                if i == 0:
                                    motor.p_in = max(min(pos, 3.14*0.5), -3.14*0.5) # Set desired position
                                else:
                                    motor.p_in = 0
                                #motor.v_in = vel_func(t)  # Set desired velocity
                                motor.kp_in = 30.0        # Position gain
                                motor.kd_in = 2.0         # Damping gain
                                pack_cmd(bus, motor)      # Send command to motor
                            elif move_mode == 2:
                                if i == 1:
                                    motor.p_in = max(min(pos, 3.14*0.5), -3.14*0.5) # Set desired position
                                else:
                                    motor.p_in = 0
                                #motor.v_in = vel_func(t)  # Set desired velocity
                                motor.kp_in = 30.0        # Position gain
                                motor.kd_in = 2.0         # Damping gain
                                pack_cmd(bus, motor)      # Send command to motor
                            elif move_mode == 3:
                                if i == 0:
                                    motor.p_in = max(min(pos, 3.14*0.5), -3.14*0.5) # Set desired position
                                else:
                                    motor.p_in = math.pi/2
                                #motor.v_in = vel_func(t)  # Set desired velocity
                                motor.kp_in = 30.0        # Position gain
                                motor.kd_in = 2.0         # Damping gain
                                pack_cmd(bus, motor)      # Send command to motor
                            elif move_mode == 4:
                                if i == 1:
                                    motor.p_in = max(min(pos, 3.14*0.5), -3.14*0.5) # Set desired position
                                else:
                                    motor.p_in = math.pi/2
                                #motor.v_in = vel_func(t)  # Set desired velocity
                                motor.kp_in = 30.0        # Position gain
                                motor.kd_in = 2.0         # Damping gain
                                pack_cmd(bus, motor)      # Send command to motor
                        elif time.time() - time_start > 3:
                            move_en = False
                    #time.sleep(0.01)
                # Receive and process feedback for both motors
                    msg = bus.recv(timeout=0.0001)
                    if msg:
                        for motor in motors:
                            if msg.arbitration_id == motor.can_id:
                                try:
                                    unpack_reply(motor, msg)
                                    print(f"Motor {motor.can_id} position: {motor.p_out:.3f}, Target position: {motor.p_in:.3f}")
                                except IndexError:
                                    print(f"Motor {motor.can_id}: unpack error")
                                break
            #time.sleep(0.001)
    except Exception as e:
        print(f"Calibration or trajectory failed: {e}")
        bus.shutdown()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
