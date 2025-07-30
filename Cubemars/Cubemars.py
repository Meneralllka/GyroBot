import can
import time
import RPi.GPIO as GPIO
import sys
import math
import select
from Cubemars_calibration import calibrate_motors
from trajectory import generate_quintic_trajectory
from motor_init import Motor, pack_cmd, unpack_reply, exit_motor_mode, zero

def main():
    try:
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
        while time.time() - t < 1:
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
                    motor.kd_in = 5.0         # Damping gain
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
            t_s = time.time()
            
            while time.time()-t_s <= 10.0:
                for motor in motors:
                    motor.p_in = 0 #.5*math.pi*math.sin(2*math.pi*1*(time.time()-t_s)) #-3.14*0.75
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
            
            t_s = time.time()
            while time.time()-t_s <= 600.0:
                for motor in motors:
                    motor.p_in = 0.5*math.pi*math.sin(2*math.pi*2*(time.time()-t_s)) #-3.14*0.75
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
                                    print(f"Time: {time.time()-t_s}, Motor {motor.can_id} position: {motor.p_out:.3f}")
                                except IndexError:
                                    print(f"Motor {motor.can_id}: unpack error")
                                break
            while 1:
                for motor in motors:
                    motor.p_in = 0 #.5*math.pi*math.sin(2*math.pi*1*(time.time()-t_s)) #-3.14*0.75
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
            bus.shutdown()
            GPIO.cleanup()
            

    except Exception as e:
        print(f"Calibration or trajectory failed: {e}")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
