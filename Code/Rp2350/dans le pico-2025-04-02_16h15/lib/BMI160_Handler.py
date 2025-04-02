from machine import Pin, I2C
import time

# I2C Configuration for RP2040
I2C_SDA_PIN = 0  # Adjust as per your board
I2C_SCL_PIN = 1  # Adjust as per your board
BMI160_I2C_ADDR_0 = 0x68
BMI160_I2C_ADDR_1 = 0x69

# BMI160 Register addresses
BMI160_CHIP_ID_REG = 0x00
BMI160_CHIP_ID = 0xD1
BMI160_COMMAND_REG = 0x7E
BMI160_ACC_CONF = 0x40
BMI160_ACC_RANGE = 0x41
BMI160_GYRO_CONF = 0x42
BMI160_GYRO_RANGE = 0x43
BMI160_ACC_X_LSB = 0x12

class BMI160:
    
    def write_register(self, reg, data):
        self.i2c.writeto_mem(self.address, reg, bytes([data]))

    def read_register(self, reg, length=1):
        return int.from_bytes(self.i2c.readfrom_mem(self.address, reg, length), 'little')

    def initialize_sensor(self):
        # Reset the sensor
        self.write_register(BMI160_COMMAND_REG, 0xB6)
        time.sleep(0.1)  # Wait for reset
        
        # Set accelerometer configuration
        self.write_register(BMI160_ACC_CONF, 0x28)  # Normal mode, 100Hz
        self.write_register(BMI160_ACC_RANGE, 0x03) # Range +/- 2G
        
        # Set gyroscope configuration
        self.write_register(BMI160_GYRO_CONF, 0x28) # Normal mode, 100Hz
        self.write_register(BMI160_GYRO_RANGE, 0x00) # Range 2000 dps

    def read_accel_gyro(self):
        # Read accelerometer and gyroscope values
        data = self.i2c.readfrom_mem(self.address, BMI160_ACC_X_LSB, 12) #Read 12 bytes SEQUENTIALLY
        
        # Convert the raw 16 bit data to integers
        accel_x = int.from_bytes(data[0:2], 'little', signed=True)
        accel_y = int.from_bytes(data[2:4], 'little', signed=True)
        accel_z = int.from_bytes(data[4:6], 'little', signed=True)
        gyro_x = int.from_bytes(data[6:8], 'little', signed=True)
        gyro_y = int.from_bytes(data[8:10], 'little', signed=True)
        gyro_z = int.from_bytes(data[10:12], 'little', signed=True)
        
        # Scale factors for conversion (adjust based on settings)
        accel_scale = 2 / 32768  # Assuming +/- 2G range
        gyro_scale = 2000 / 32768  # Assuming 2000 dps range
        
        #Convert into G value
        accel_x *= accel_scale
        accel_y *= accel_scale
        accel_z *= accel_scale
        #Convert into dps value
        gyro_x *= gyro_scale
        gyro_y *= gyro_scale
        gyro_z *= gyro_scale

        return {
            "accel": (accel_x, accel_y, accel_z),
            "gyro": (gyro_x, gyro_y, gyro_z)
        }
    
    def __init__(self, i2c, g=2, gyro_dps= 2000, address=BMI160_I2C_ADDR_0):
        self.i2c = i2c
        self.address = address

        # Check if the device is connected
        chip_id = self.read_register(BMI160_CHIP_ID_REG)
        if chip_id != BMI160_CHIP_ID:
            raise Exception("BMI160 not found. Check wiring.")
        
        print("BMI160 found!")

        self.initialize_sensor()

# Example usage:
if __name__ == "__main__":
    # Setup I2C
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))

    try:
        bmi160 = BMI160(i2c)
        while True:
            sensor_data = bmi160.read_accel_gyro()
            print("Accel (g):", sensor_data["accel"])
            print("Gyro (dps):", sensor_data["gyro"])
            time.sleep(1)
    except Exception as e:
        print("Error:", e)
