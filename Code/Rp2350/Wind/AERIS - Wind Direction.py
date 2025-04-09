from machine import I2C, Pin
from AS5600_Handler import AS5600
import time
import math

class WindVane :
    def __init__(self) :
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
        self.sensor = AS5600(i2c)
        self.zero_angle = self.sensor.get_angle()
        
    def get_wind_direction(self) :
        return ((self.zero_angle + self.sensor.get_angle()) % 360)
        #In Degrees : 0 to 360
    
    def set_0(self) :
        self.zero_angle = self.sensor.get_angle()
        
windvane = WindVane()

while True :
    print(windvane.get_wind_direction())
    time.sleep(0.1)
        
        
        
        
        
    