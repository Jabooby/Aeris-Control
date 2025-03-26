from machine import I2C, Pin
from AS5600_Handler import AS5600
import time

# Initialize I2C
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100_000)  # Adjust pins as needed

# Create an instance of the AS5600 class
sensor = AS5600(i2c)

pi = 3.14159

old_time = time.time_ns()
old_angle = (sensor.get_angle()*2*pi)/360
curr_time = time.time_ns()
curr_angle = (sensor.get_angle()*2*pi)/360

while True :
    curr_angle = (sensor.get_angle()*2*pi)/360 #rad
    curr_time = time.time_ns()
    if old_angle>pi and curr_angle<pi :
        curr_angle += 360
    speed = 0.047*((curr_angle-old_angle)/((curr_time-old_time)/10000000000))
    print(speed)
    old_time = time.time_ns()
    old_angle = (sensor.get_angle()*2*pi)/360
    
    time.sleep(0.2)



# Read and print the angle
if sensor.is_ready():
    angle = sensor.get_angle()
    print(f"Angle: {angle:.2f}°")

    magnitude = sensor.get_magnitude()
    print(f"Magnitude: {magnitude}")
