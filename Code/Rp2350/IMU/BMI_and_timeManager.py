import time
import json
from machine import Pin, I2C
from micropython_bmi160 import bmi160
import math as m

# BMI I2C setup
i2c_0 = I2C(0, sda=Pin(4), scl=Pin(5))  

class TimeManager:
    """Handles time updates and delta calculations."""
    
    def __init__(self):
        self.prev_time = time.ticks_ms()
        self.time_delta = 0.0
        self.current_time = 0.0

    def update(self):
        self.current_time = time.ticks_ms()
        self.time_delta = time.ticks_diff(self.current_time, self.prev_time) / 1000.0  # Convert ms to seconds
        self.prev_time = self.current_time

    def get_time_delta(self):
        return self.time_delta

class IMUSensor:
    """Handles IMU sensor readings and filtering."""
    
    def __init__(self, i2c, weight_of_sensors=0.98):
        self.bmi = bmi160.BMI160(i2c_0)
        
        self.bmi.acceleration_range = bmi160.ACCEL_RANGE_2G # the mapping of the values going from  + 2Gs to - 2Gs in 2^16 bits
        self.bmi.acceleration_output_data_rate = bmi160.BANDWIDTH_50 # rate a which data is sent 50Hz usually 100Hz
        self.bmi.gyro_range = bmi160.GYRO_RANGE_500
        self.bmi.gyro_output_data_rate = bmi160.BANDWIDTH_50
        
        self.weight_of_sensors = weight_of_sensors
        self.PI = 3.14159265359
        self.gravity  = 9.81
        self.epsilon = 1e-6
        self.gyrox_sum, self.gyroy_sum, self.gyroz_sum = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.accx, self.accy, self.accz = 0.0, 0.0, 0.0
        self.speedx, self.speedy, self.speedz = 0.0, 0.0, 0.0
        self.distancex, self.distancey, self.distancez = 0.0, 0.0, 0.0
        self.setup_accx_offset, self.setup_accy_offset, self.setup_accz_offset = 0.0, 0.0, 0.0
        self.temp_accx, self.temp_accy, self.temp_accz = 0.0, 0.0, 0.0
        
    def auto_calibrate(self):
        """Perform auto-calibration to remove noise or error."""  
        num_samples = 300
        ax_offset = 0
        ay_offset = 0
        az_offset = 0
     
        for _ in range(num_samples):
            ax_raw, ay_raw, az_raw = self.bmi.acceleration
            ax_offset += ax_raw
            ay_offset += ay_raw
            az_offset += az_raw
            time.sleep(0.01)  # Small delay between readings
     
        # Calculate average offsets
        self.setup_accx_offset //= num_samples
        self.setup_accy_offset //= num_samples
        self.setup_accz_offset //= num_samples
     
        # Assuming the sensor is stable, Z-axis should measure 1g (gravity)
        self.setup_accz_offset -= self.gravity
   
        return
    
    def rad_to_deg(self, val_rad):
        return val_rad * (180 / self.PI)

    def set_get_acceleration(self):
        acc_raw_x, acc_raw_y, acc_raw_z = self.bmi.acceleration 
        self.accx = acc_raw_x - self.setup_accx_offset - self.gravity
        self.accy = acc_raw_y - self.setup_accy_offset
        self.accz = acc_raw_z - self.setup_accz_offset - self.gravity
        ancient_accx, ancient_accy, ancient_accz = self.accx, self.accy, self.accz
        
        # If the before and after acceleration delta is too small we set it to 0, we only set temp to ancient_acc because we need to compare with true acceleration
        if (abs(self.temp_accx - self.accx) < 0.2):
            self.accx = 0.0
            
        if (abs(self.temp_accy - self.accy) < 0.2):
            self.accy = 0.0
            
        if (abs(self.temp_accz - self.accz) < 0.2):
            self.accz = 0.0
            
        self.temp_accx, self.temp_accy, self.temp_accz = ancient_accx, ancient_accy, ancient_accz
        
        return self.accx, self.accy, self.accz
        
    def get_acc_angle(self):
        if abs(self.accz) < self.epsilon:  # Prevent division by zero
            accz = self.epsilon
        if m.sqrt(self.accy**2 + self.accz**2) < self.epsilon:
            return 0, 0
        
        roll = m.atan2(self.accy, self.accz)
        pitch = m.atan2(-self.accx, m.sqrt(self.accy**2 + self.accz**2))
        return self.rad_to_deg(roll), self.rad_to_deg(pitch)

    def get_gyro_angle(self, time_delta):
        gyrox, gyroy, gyroz = self.bmi.gyro
        self.gyrox_sum += gyrox * time_delta
        self.gyroy_sum += gyroy * time_delta
        self.gyroz_sum += gyroz * time_delta
        return self.gyrox_sum, self.gyroy_sum, self.gyroz_sum 

    def get_speed(self):
        return self.speedx, self.speedy, self.speedz

    def get_distance(self):
        return self.distancex, self.distancey, self.distancez

    def angle_filter(self, acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw):
        f_roll = self.weight_of_sensors * gy_roll + (1 - self.weight_of_sensors) * acc_roll
        f_pitch = self.weight_of_sensors * gy_pitch + (1 - self.weight_of_sensors) * acc_pitch
        f_yaw = gy_yaw
        return f_roll, f_pitch, f_yaw

    def update(self, time_delta):
        accx, accy, accz = self.set_get_acceleration()
        acc_roll, acc_pitch = self.get_acc_angle()
        gy_roll, gy_pitch, gy_yaw = self.get_gyro_angle(time_delta)
        # Update angles
        self.roll, self.pitch, self.yaw = self.angle_filter(acc_roll, acc_pitch, gy_roll, gy_pitch, gy_yaw)
        print(f"accx: {accx:.2f}, accy: {accy:.2f}, accz: {accz:.2f}")
        # Reset velocity periodically (e.g., every 10 seconds)
        if time.ticks_ms() % 10000 < 100:  # Every 10 seconds
            self.speedx = self.speedy = self.speedz = 0.0
        
        # Update values by integration
        self.speedx += accx * time_delta
        self.speedy += accy * time_delta
        self.speedz += accz * time_delta
        self.distancex += self.speedx * time_delta + 0.5 * self.accx * time_delta**2
        self.distancey += self.speedy * time_delta + 0.5 * self.accy * time_delta**2
        self.distancez += self.speedz * time_delta + 0.5 * self.accz * time_delta**2
        #print(f"accx: {self.distancex:.2f}, accy: {self.distancey:.2f}, accz: {self.distancez:.2f}")


def update_json_file(roll, pitch, yaw, filename="imu_data.json"):
    data = {
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw
    }


    with open(filename, "w") as f:
        json.dump(data, f)
        
# Create IMU Sensor object + Calibration
imu_0 = IMUSensor(i2c_0)
imu_0.auto_calibrate()
time_manager = TimeManager()

# Initialize time tracking variable
sum_time_delta = 0.0

while True:
    time_manager.update()
    imu_0.update(time_manager.get_time_delta())
    accx, accy, accz = imu_0.accx, imu_0.accy, imu_0.accz
    roll, pitch, yaw = imu_0.roll, imu_0.pitch, imu_0.yaw
    update_json_file(roll, pitch, yaw)
    time.sleep(0.5)
    
  



