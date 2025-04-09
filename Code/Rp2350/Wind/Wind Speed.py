from machine import I2C, Pin
from AS5600_Handler import AS5600
import time
import math

class Anemometer:
    def __init__(self):
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
        self.sensor = AS5600(i2c)
        self.sample_size = 10
        self.position_readings = [0] * self.sample_size  # Degrees
        self.time_readings = [0] * self.sample_size  # Nanoseconds
        self.wind_speed_readings = [0] * (self.sample_size - 1)  # 9 speeds from 10 samples
        self.rotation_radius = 0.047

    def get_wind_speed(self):
        # Collect 10 samples
        for i in range(self.sample_size):
            self.position_readings[i] = self.sensor.get_angle()  # In degrees
            self.time_readings[i] = time.time_ns()  # In nanoseconds
            if i < self.sample_size - 1:  # Sleep except after the last sample
                time.sleep(0.01)

        # Calculate wind speeds between consecutive samples
        for i in range(1, self.sample_size):
            # Angle difference with wraparound handling
            angle_diff = self.position_readings[i] - self.position_readings[i - 1]
            if angle_diff > 180:  # Crossing from 360° to 0°
                angle_diff -= 360
            elif angle_diff < -180:  # Crossing from 0° to 360°
                angle_diff += 360
            
            # Convert to radians
            angle_diff_rad = angle_diff * 2 * math.pi / 360
            
            # Time difference in seconds
            time_diff = (self.time_readings[i] - self.time_readings[i - 1]) / 1_000_000_000
            
            # Angular speed (rad/s) and wind speed (m/s)
            angular_speed = angle_diff_rad / time_diff
            self.wind_speed_readings[i - 1] = self.rotation_radius * angular_speed

        # Average wind speed
        wind_speed = sum(self.wind_speed_readings) / (self.sample_size - 1)
        
        # Debugging output
        #print("Position readings:", self.position_readings)
        #print("Time readings:", self.time_readings)
        #print("Wind speed readings:", self.wind_speed_readings)
        print("Average wind speed:", wind_speed)
        
        return wind_speed

# Test the anemometer
anemometer = Anemometer()

while True :
    anemometer.get_wind_speed()  # Correct instance method call
    time.sleep(0.2)