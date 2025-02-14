# SPDX-FileCopyrightText: Copyright (c) 2023 Jose D. Montoya
#
# SPDX-License-Identifier: MIT

import time
from machine import Pin, I2C
from micropython_bmi160 import bmi160
import math as m

i2c = I2C(1, sda=Pin(2), scl=Pin(3))  # Correct I2C pins for RP2040
bmi = bmi160.BMI160(i2c)
PI = 3.14
WEIGHT_OF_SENSORS = 0.98

roll, pitch, yaw = 0,0,0
gy_roll, gy_pitch, gy_yaw = 0,0,0
acc_roll, acc_pitch = 0,0
time_delta = 0

# Setup time
prev_time = time.ticks_ms()


def deg_to_rad(val_deg):
    return val_deg * PI / 180

def get_gyro_angle(omega_roll, omega_pitch, omega_yaw, time_delta):
    roll = roll + deg_to_rad(omega_roll)*time_delta
    pitch = pitch + deg_to_rad(omega_pitch)*time_delta
    yaw = yaw + deg_to_rad(omega_yaw)*time_delta
    return roll, pitch, yaw

def get_acc_angle(accx, accy, accz):
    roll = m.atan2(accy, accz)
    pitch = m.atan2(-accx, m.sqrt(accy**2 + accx**2))
    return roll, pitch

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
    prev_time = current_time  

    accx, accy, accz = bmi.acceleration
    acc_roll, acc_pitch = get_acc_angle(accx, accy, accz)

    gyrox, gyroy, gyroz = bmi.gyro
    gy_roll, gy_pitch, gy_yaw  = get_gyro_angle(gyrox, gyroy, gyroz, time_delta)
    roll, pitch, yaw = angle_filter(acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw)


    
    #print(f"x:{gyrox:.2f}°/s, y:{gyroy:.2f}°/s, z{gyroz:.2f}°/s")
    #print(f"x:{accx:.2f}m/s2, y:{accy:.2f}m/s2, z{accz:.2f}m/s2")
    





    