from machine import I2C, Pin
from AS5600_Handler import AS5600
import time
import math


class Anemometer :
    def __init__(self):
        # Initialize I2C
        i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100_000)  # Adjust pins as needed
            
        # Create an instance of the AS5600 class
        self.sensor = AS5600(i2c)

        self.N = 5  # Number of samples in the FIR filter
        self.angle_buffer = [0] * self.N  # Circular buffer for past speed values
        self.buffer_index = 0  # Track where to insert new values

        self.weights = [0.54 - 0.46 * math.cos((2 * math.pi * i) / (self.N - 1)) for i in range(self.N)]
        self.weights_sum = sum(self.weights)  # Normalize to keep amplitude consistent
    
    def get_wind_speed(self):

        old_time = time.time_ns()
        old_angle = (self.sensor.get_angle()*2*math.pi)/360

        while True :
            raw_angle = (self.sensor.get_angle()*2*math.pi)/360 #rad
            curr_time = time.time_ns()
            
            if old_angle>math.pi and raw_angle<math.pi :
                raw_angle += 2*math.pi
                
            # Store the new speed in the circular buffer
            self.angle_buffer[self.buffer_index] = raw_angle
            self.buffer_index = (self.buffer_index + 1) % self.N  # Move to the next index (circular buffer)
            filtered_angle = sum(self.angle_buffer[i] * self.weights[i] for i in range(self.N)) / self.weights_sum # Apply the weighted FIR filter
                
            speed = 0.047*((filtered_angle-old_angle)/((curr_time-old_time)/10000000000))
            
            print(speed)
            old_time = curr_time
            old_angle = filtered_angle
            
            time.sleep(0.1)
            
class WindVane:
     def __init__(self):
        # Initialize I2C
        i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100_000)  # Adjust pins as needed
            
        # Create an instance of the AS5600 class
        self.sensor = AS5600(i2c)
        
        self.N = 5  # Number of samples in the FIR filter
        self.angle_buffer = [0] * self.N  # Circular buffer for past speed values
        self.buffer_index = 0  # Track where to insert new values

        self.weights = [0.54 - 0.46 * math.cos((2 * math.pi * i) / (self.N - 1)) for i in range(self.N)]
        self.weights_sum = sum(self.weights)  # Normalize to keep amplitude consistent
    
    def get_wind_direction(self):
        raw_angle = self.sensor.get_angle()
        # Store the new speed in the circular buffer
        self.angle_buffer[self.buffer_index] = raw_angle
        self.buffer_index = (self.buffer_index + 1) % self.N  # Move to the next index (circular buffer)
        filtered_angle = sum(self.angle_buffer[i] * self.weights[i] for i in range(self.N)) / self.weights_sum
        
        return filtered_angle


anemometer = Anemometer()
windvane = WindVane()

anemometer.get_wind_speed()
