import time
from machine import Pin, I2C
from micropython_bmi160 import bmi160
import math as m

# BMI I2C setup
i2c_0 = I2C(0, sda=Pin(4), scl=Pin(5))  
#i2c_1 = I2C(1, sda=Pin(6), scl=Pin(7))  

class TimeManager:
    """ """
    def __init__(self)
        self.prev_time = time.ticks_ms()
        self.time_delta = 0.0
        self.current_time = 0.0

    def update()
        self.current_time = time.ticks_ms()
        self.time_delta = (time.ticks_diff(self.current_time, self.prev_time)) / 1000.0  # Convert ms to seconds
        self.prev_time = self.current_time

    def get_time_delta(self)
        return self.time_delta

class IMUSensor:
    """ """
    def __init__(self, i2c, weight_of_sensors=0.98):
        self.bmi = bmi160.BMI160(i2c)
        self.weight_of_sensors = weight_of_sensors
        self.PI = 3.14159265359
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accx = 0.0
        self.accy = 0.0
        self.accz = 0.0
        self.speedx = 0.0
        self.speedy = 0.0
        self.speedz = 0.0
        self.distancex = 0.0
        self.distancey = 0.0
        self.distancez = 0.0
    
    def rad_to_deg(self, val_rad):
        return val_rad * (180 / self.PI)

    def get_acc_angle(self):
        self.accx, self.accy, self.accz = self.bmi.acceleration
        roll = math.atan2(self.accy, self.accz)
        pitch = math.atan2(-self.accx, math.sqrt(self.accy**2 + self.accz**2))
        return self.rad_to_deg(roll), self.rad_to_deg(pitch)

    def get_gyro_angle(self, time_delta):
        gyrox, gyroy, gyroz = self.bmi.gyro
        return gyrox * time_delta, gyroy * time_delta, gyroz * time_delta
    
    def get_speed(self, time_delta):
        return self.speedx, self.speedy, self.speedz

    def get_distance(self, time_delta):
        return self.distancex, self.distancey, self.distancez

    def angle_filter(self, acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw):
        f_roll = self.weight_of_sensors * gy_roll + (1 - self.weight_of_sensors) * acc_roll
        f_pitch = self.weight_of_sensors * gy_pitch + (1 - self.weight_of_sensors) * acc_pitch
        f_yaw = gy_yaw
        return f_roll, f_pitch, f_yaw

    def update(self, time_delta):
        acc_roll, acc_pitch = self.get_acc_angle()
        gy_roll, gy_pitch, gy_yaw = self.get_gyro_angle(time_delta)
        f_roll, f_pitch, f_yaw = self.angle_filter(acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw)

        # Update angles
        self.roll += f_roll
        self.pitch += f_pitch
        self.yaw += f_yaw

        # Update values by integration
        self.speedx += self.accx*time_delta
        self.speedy += self.accy*time_delta
        self.speedz += self.accz*time_delta
        self.distancex += self.speedx*time_delta
        self.distancey += self.speedy*time_delta
        self.distancez += self.speedz*time_delta

# Create IMU Sensor object
imu_0 = IMUSensor(i2c_0)
time_manager = TimeManager()

# Read and update sensor data in a loop
while True:
    time_manager.update()

    imu.update(time_manager.get_time_delta())
    roll, pitch, yaw = imu.get_angles()
    print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
    
    sum_time_delta += time_manager.get_time_delta()
    if (sum_time_delta >= 1.0):
        print(f"x:{acc_roll:.2f}°")
        sum_time_delta = 0.0

