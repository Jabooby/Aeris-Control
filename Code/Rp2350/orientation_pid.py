import A4988_with_PIO_with_class as A4988
import UART_JSON_Handler as UART_JSON
import BMI160_Handler as BMI160
import AS5600_Handler as AS5600
import time
import math as m
import uasyncio as asyncio


# Create IMU Sensor object + Calibration i2c_0 and i2c_1 comme from BMI160
# i2c_0 will need to be mapped to the correct x,y,z axes
imu_0 = IMUSensor(i2c_0)    # on mast
imu_1 = IMUSensor(i2c_1)    # on boat
imu_0.auto_calibrate()
imu_1.auto_calibrate()
time_manager = TimeManager()

HEIGHT_OF_MAST = 0.3
X_DIST_MOTOR_TO_MAST = 0.3
old_rope_magnitude = 0.3 #  just as a starting position
motor_ratio = 21
radius_output_gear = 0.05
steps_per_rev = 200

Kp = 1
Ki = 1
Kd = 1
last_error = 0
error = 0

def pid_controller(target_direction, real_direction, time_delta):
    error = target - current

    integral += error*time_delta
    derivative = (error - last_error)/time_delta

    output = (Kp*error) + (Ki * integral) + (Kd * derivative)

    last_error = error

    return output


async def automatic_orientation(target_direction, time_delta):

    boat_yaw = imu_1.yaw
    mast_roll = 2*m.pi*imu_0.roll/360
    magnitude_rope_to_mast = m.sqrt(X_DIST_MOTOR_TO_MAST^2 + (HEIGHT_OF_MAST*m.sin(mast_roll))^2 + (HEIGHT_OF_MAST*m.cos(mast_roll))^2)

    if (time_delta > 0.1): #10Hz timer on PID

        new_mast_angular_speed_roll = pid_controller(target_direction, boat_yaw, time_delta)

        if (new_mast_angular_speed_roll > MAX_MAST_ANGULAR_SPEED):
            new_mast_angular_speed_roll = MAX_MAST_ANGULAR_SPEED
        elif (new_mast_angular_speed_roll < MIN_MAST_ANGULAR_SPEED):
            new_mast_angular_speed_roll = MIN_MAST_ANGULAR_SPEED

        if (new_mast_angular_speed_roll > 0 & mast_roll > 45):
            new_mast_angular_speed_roll = 0
        elif (new_mast_angular_speed_roll < 0 & mast_roll < -45):
            new_mast_angular_speed_roll = 0

        # Mettre le calcul qui lie la vitesse des moteur a la vitesse du roll, 
        # pour trouver le deplacement a faire en step selon v_moteur/delta_t = N_step

        rope_speed = new_mast_angular_speed_roll - new_mast_angular_speed_roll
        motor_anglular_velocity = rope_speed/(motor_ratio*radius_output_gear)  
        old_rope_magnitude = magnitude_rope_to_mast
        motor_speed_steps = motor_angular_velocity * steps_per_rev / (2 * math.pi)  # steps/s
        motor.set_speed(motor_speed_steps)
        motor.go_to_position(motor_speed_steps*time_delta)

        # The effect on the z_motor to turn based on the lateral position of the mast



target_direction = 20 #degrees (should come from an input in the json)
uart = UART_JSON.UARTJsonHandler()
motor = A4988.StepperControl(sm_id=0, stpPin=14, dirPin=15)


start_time = time.ticks_ms()
while True:
    current_time = time.ticks_ms()
    
    if time.ticks_diff(current_time, start_time) >= 100: 
        time_delta = time.ticks_diff(current_time, start_time) / 1000.0  # Convert ms to seconds
        start_time = current_time  
        automatic_orientation(target_direction, time_delta)
        print(f"{time_delta:.3f} seconds")

        



# Reset the motor
motor.reset_motor()


