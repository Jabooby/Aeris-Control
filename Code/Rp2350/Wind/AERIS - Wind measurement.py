from machine import I2C, Pin
from AS5600_Handler import AS5600
import time
import math

class Anemometer:
    def __init__(self):
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
        self.sensor = AS5600(i2c)
        
        self.N = 5
        self.angle_buffer = [0] * self.N
        self.buffer_index = 0
        self.weights = [0.54 - 0.46 * math.cos((2 * math.pi * i) / (self.N - 1)) for i in range(self.N)]
        self.weights_sum = sum(self.weights)
        self.old_filtered_angle = 0  # Initialize for first call
        
    def get_wind_speed(self):
        print("Starting get_wind_speed")
        
        samples = []
        times = []
        start_time = time.time_ns()
        
        raw_angle = (self.sensor.get_angle() * 2 * math.pi) / 360
        cumulative_angle = raw_angle
        samples.append(cumulative_angle)
        times.append(start_time)
        
        for _ in range(9):
            time.sleep(0.01)
            curr_time = time.time_ns()
            raw_angle = (self.sensor.get_angle() * 2 * math.pi) / 360
            angle_diff_raw = raw_angle - samples[-1]
            if angle_diff_raw > math.pi:
                angle_diff_raw -= 2 * math.pi
            elif angle_diff_raw < -math.pi:
                angle_diff_raw += 2 * math.pi
            cumulative_angle += angle_diff_raw
            samples.append(cumulative_angle)
            times.append(curr_time)
        
        print("----------------------------------------------------")
        
        total_time = (times[-1] - times[0]) / 1_000_000_000
        total_angle_diff = samples[-1] - samples[0]
        raw_speed = 0.047 * (total_angle_diff / total_time)
        print(f"Raw speed calculation: 0.047 * ({total_angle_diff} / {total_time}) = {raw_speed}")
        
        # Filter the final cumulative angle
        self.angle_buffer[self.buffer_index] = samples[-1]
        self.buffer_index = (self.buffer_index + 1) % self.N
        temp_sum = sum(self.angle_buffer[i] * self.weights[i] for i in range(self.N))
        filtered_angle = temp_sum / self.weights_sum
        print("Filtered angle:", filtered_angle)
        
        # Calculate filtered speed using previous filtered angle
        filtered_speed = 0.047 * ((filtered_angle - self.old_filtered_angle) / total_time)
        print(f"Filtered speed calculation: 0.047 * ({filtered_angle - self.old_filtered_angle} / {total_time}) = {filtered_speed}")
        print("Filtered speed:", filtered_speed)
        
        # Update old_filtered_angle for next call
        self.old_filtered_angle = filtered_angle
        
        return filtered_speed

# Test it
anemometer = Anemometer()
while True:
    speed = anemometer.get_wind_speed()
    time.sleep(0.1)  # Pause between calls
