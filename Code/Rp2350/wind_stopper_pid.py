import A4988_with_PIO_with_class as A4988
import UART_JSON_Handler as UART_JSON
import BMI160_Handler as BMI160
import AS5600_Handler as AS5600
import time
import math as m
import uasyncio as asyncio



###//////// WE NEED TO ADD WIND DIRECTION FOR THE TARGET ANGLE, AND MAKE SURE WE HAVE A CALIBRATED 0 FOR THE MAST POSITION///////###




# Create IMU Sensor object + Calibration i2c_0 and i2c_1 comme from BMI160
# i2c_0 will need to be mapped to the correct x,y,z axes
imu_0 = BMI160.IMUSensor(BMI160.i2c_0) # on mast
imu_0.auto_calibrate()

MAX_MAST_ANGULAR_SPEED = 0.2
MIN_MAST_ANGULAR_SPEED = 0
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
    error = target_direction - real_direction

    integral += error*time_delta
    derivative = (error - last_error)/time_delta

    output = (Kp*error) + (Ki * integral) + (Kd * derivative)

    last_error = error

    return output


async def automatic_wind_sail_lineup(target_direction, time_delta):

    boat_yaw = imu_0.yaw

    if (time_delta > 0.1): #10Hz timer on PID

        new_z_motor_angular_velocity = pid_controller(target_direction, boat_yaw, time_delta) # rad/second I guess

        if (new_mast_angular_speed_roll > MAX_MAST_ANGULAR_SPEED):
            new_mast_angular_speed_roll = MAX_MAST_ANGULAR_SPEED
        elif (new_mast_angular_speed_roll < MIN_MAST_ANGULAR_SPEED):
            new_mast_angular_speed_roll = MIN_MAST_ANGULAR_SPEED

        # Mettre le calcul qui lie la vitesse des moteur a la vitesse du roll, 
        # pour trouver le deplacement a faire en step selon v_moteur/delta_t = N_step

        motor_speed_steps = new_z_motor_angular_velocity * steps_per_rev / (2 * m.pi)  # steps/s
        motor.set_speed(motor_speed_steps)
        motor.go_to_position(motor_speed_steps*time_delta)

        # The effect on the z_motor to turn based on the lateral position of the mast



target_direction = 20 #degrees, should come from wind indication
uart = UART_JSON.UARTJsonHandler()
motor = A4988.StepperControl(sm_id=0, stpPin=14, dirPin=15)


start_time = time.ticks_ms()
while True:
    current_time = time.ticks_ms()
    
    if time.ticks_diff(current_time, start_time) >= 100: 
        time_delta = time.ticks_diff(current_time, start_time) / 1000.0  # Convert ms to seconds
        start_time = current_time  
        automatic_wind_sail_lineup(target_direction, time_delta)
        print(f"{time_delta:.3f} seconds")

        



# Reset the motor
motor.reset_motor()


