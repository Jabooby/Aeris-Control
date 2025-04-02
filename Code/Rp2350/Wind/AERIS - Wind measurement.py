from machine import I2C, Pin
from AS5600_Handler_Modified import AS5600
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
        print("Starting get_wind_speed")
        old_time = time.time_ns()
        print("Got old_time:", old_time)
        sensor_angle = self.sensor.get_angle()
        print("Sensor angle:", sensor_angle)
        old_angle = (sensor_angle * 2 * math.pi) / 360
        print("Calculated old_angle:", old_angle)

        while True:
            raw_angle = (self.sensor.get_angle() * 2 * math.pi) / 360  # rad
            print("Raw angle:", raw_angle)
            curr_time = time.time_ns()
            print("Current time:", curr_time)
            
            if old_angle > math.pi and raw_angle < math.pi:
                raw_angle += 2 * math.pi
                print("Adjusted raw_angle:", raw_angle)
                    
            # Store the new speed in the circular buffer
            self.angle_buffer[self.buffer_index] = raw_angle
            self.buffer_index = (self.buffer_index + 1) % self.N
            print("Buffer updated:", self.angle_buffer)
            
            # Calculate sum explicitly
            temp_sum = 0
            for i in range(self.N):
                temp_sum += self.angle_buffer[i] * self.weights[i]
                print(f"Step {i}: {self.angle_buffer[i]} * {self.weights[i]} = {self.angle_buffer[i] * self.weights[i]}")
            filtered_angle = temp_sum / self.weights_sum
            print("Filtered angle:", filtered_angle)
            
            time_diff = (curr_time - old_time) / 10000000000
            angle_diff = filtered_angle - old_angle
            speed = 0.047 * (angle_diff / time_diff)
            print(f"Speed calculation: 0.047 * ({angle_diff} / {time_diff}) = {speed}")
            
            print("Speed:", speed)
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
        # Store the new angle in the circular buffer
        self.angle_buffer[self.buffer_index] = raw_angle
        self.buffer_index = (self.buffer_index + 1) % self.N  # Move to the next index (circular buffer)
        
        # Calculate sum using explicit loop
        temp_sum = 0
        for i in range(self.N):
            temp_sum += self.angle_buffer[i] * self.weights[i]
        filtered_angle = temp_sum / self.weights_sum
        
        return filtered_angle


anemometer = Anemometer()
windvane = WindVane()

anemometer.get_wind_speed()
