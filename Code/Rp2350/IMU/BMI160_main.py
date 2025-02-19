import time
from machine import Pin, I2C
from micropython_bmi160 import bmi160
import math as m

i2c = I2C(0, sda=Pin(4), scl=Pin(5))  # Correct I2C pins for RP2040
bmi = bmi160.BMI160(i2c)
PI = 3.14
WEIGHT_OF_SENSORS = 0.98

roll, pitch, yaw = 0,0,0
gyrox, gyroy, gyroz = 0,0,0
gy_roll, gy_pitch, gy_yaw = 0,0,0
acc_roll, acc_pitch = 0,0
prev_time = time.ticks_ms()
time_delta = 0
sum_time_delta = 0.0
# Setup time



def rad_to_deg(val_rad):
    return val_rad * 180 / PI

def get_gyro_angle(omega_roll, omega_pitch, omega_yaw, time_delta):
    gyro_roll = omega_roll*time_delta
    gyro_pitch = omega_pitch*time_delta
    gyro_yaw = omega_yaw*time_delta
    return gyro_roll, gyro_pitch, gyro_yaw

def get_acc_angle(accx, accy, accz):
    acceleration_roll = m.atan2(accy, accz)
    acceleration_pitch = m.atan2(-accx, m.sqrt(accy**2 + accz**2))
    return rad_to_deg(acceleration_roll), rad_to_deg(acceleration_pitch)

def angle_filter(acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw):
    # weight of sensors can help use choose which angle value coming 
    # from which sensor should get more priority : between (0,1)
    # We can add a yaw filter by checking the gyro and the yaw motor encoder in the future
    f_roll = WEIGHT_OF_SENSORS*gy_roll + (1-WEIGHT_OF_SENSORS)*acc_roll
    f_pitch = WEIGHT_OF_SENSORS*gy_pitch + (1-WEIGHT_OF_SENSORS)*acc_pitch
    f_yaw = gy_yaw
    return f_roll, f_pitch, f_yaw

while True:
    current_time = time.ticks_ms()
    time_delta = (time.ticks_diff(current_time, prev_time)) / 1000.0  # Convert ms to seconds
    #sum_time_delta = sum_time_delta + time_delta
    prev_time = current_time

    accx, accy, accz = bmi.acceleration # accelerometer in m/s2
    acc_roll, acc_pitch = get_acc_angle(accx, accy, accz) # accelerometer angles in °

    gyrox, gyroy, gyroz = bmi.gyro # gyroscope angular velocity in °/s
    gy_roll, gy_pitch, gy_yaw  = get_gyro_angle(gyrox, gyroy, gyroz, time_delta) # gyroscope angles in °
    f_roll, f_pitch, f_yaw = angle_filter(acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw) # filtered angled weighted with the accelerometer and gyroscope
    
    # Integration overtime
    roll += f_roll
    pitch += f_pitch
    yaw += f_yaw

    #print(f"x:{gyrox:.2f}°/s, y:{gyroy:.2f}°/s, z{gyroz:.2f}°/s, x:{accx:.2f}m/s2, y:{accy:.2f}m/s2, z{accz:.2f}m/s2")
    
    if (sum_time_delta >= 1.0):
        print(f"x:{acc_roll:.2f}°")
        sum_time_delta = 0.0