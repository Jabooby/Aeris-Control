from machine import Pin, I2C
import time

# I2C Configuration for RP2040
I2C_SDA_PIN = 4  # Adjust as per your board
I2C_SCL_PIN = 5  # Adjust as per your board
BMI160_I2C_ADDR_0 = 0x69
BMI160_I2C_ADDR_1 = 0x68

# BMI160 Register addresses
BMI160_CHIP_ID_REG = 0x00
BMI160_CHIP_ID = 0xD1
BMI160_COMMAND_REG = 0x7E
BMI160_ACC_CONF = 0x40
BMI160_ACC_RANGE = 0x41
BMI160_GYRO_CONF = 0x42
BMI160_GYRO_RANGE = 0x43
BMI160_ACC_X_LSB = 0x12
BMI160_FIFO_CONFIG_1 = 0x48
BMI160_INT_EN_1 = 0x51
BMI160_FOC_CONF = 0x69
BMI160_SELF_TEST = 0x6D

class BMI160:
    
    def __init__(self, i2c, address=0x69):
        self.i2c = i2c
        self.address = address
        # Check if the device is connected
        chip_id = self.read_register(BMI160_CHIP_ID_REG)
        if chip_id != BMI160_CHIP_ID:
            raise Exception("BMI160 not found. Check wiring or address.")
        print("BMI160 found!")
        self.initialize_sensor()

    def write_register(self, reg, data):
        """Write data to a register."""
        self.i2c.writeto_mem(self.address, reg, bytes([data]))

    def read_register(self, reg, length=1):
        """Read data from a register."""
        data = self.i2c.readfrom_mem(self.address, reg, length)
        return int.from_bytes(data, 'big')

    def initialize_sensor(self):
        """Initialize the BMI160 sensor."""
        # Perform a soft reset on the BMI160
        self.write_register(BMI160_COMMAND_REG, 0xB6)
        time.sleep(0.01)  # Wait for reset
        
        # Set accelerometer and gyroscope to Normal mode
        self.write_register(BMI160_COMMAND_REG, 0x11)  # Accelerometer
        time.sleep(0.01)
        self.write_register(BMI160_COMMAND_REG, 0x15)  # Gyroscope
        time.sleep(0.01)Raw Gyro X: 62831, Y: 4135, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (-165.1001, 252.3804, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 22588, Y: 4136, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (1378.662, 252.4414, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 47882, Y: 4136, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (-1077.515, 252.4414, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 7640, Y: 4137, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (466.3086, 252.5024, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 32933, Y: 4137, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (-1989.929, 252.5024, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 58227, Y: 4137, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (-446.106, 252.5024, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 17984, Y: 4138, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (1097.656, 252.5635, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 43278, Y: 4138, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (-1358.521, 252.5635, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 3036, Y: 4139, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (185.3027, 252.6245, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 28329, Y: 4139, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (1729.065, 252.6245, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 53623, Y: 4139, Z: 0
Accel (g): (0.0, 0.0, 0.0)
Gyro (dps): (-727.1118, 252.6245, 0.0)
Raw Accel X: 0, Y: 0, Z: 0
Raw Gyro X: 13381, Y: 4140, Z: 0
Accel (g): (0.0, 0.0, 0.0)

        # Configure accelerometer: low power mode, 100Hz bandwidth
        self.write_register(BMI160_ACC_CONF, 0x14)  # Accel bandwidth 100Hz
        self.write_register(BMI160_ACC_RANGE, 0x03)  # ±2g range
        
        # Configure gyroscope: 2000dps range, 100Hz bandwidth
        self.write_register(BMI160_GYRO_CONF, 0x14)  # Gyro bandwidth 100Hz
        self.write_register(BMI160_GYRO_RANGE, 0x00)  # 2000dps range
        
        # Configure FIFO
        self.write_register(BMI160_FIFO_CONFIG_1, 0xC0)  # Enable FIFO
        
        time.sleep(0.25)  # Allow some time for sensor to stabilize
        
        self.write_register(BMI160_FOC_CONF, 0x00)  # Disable FOC (no factory offset correction)
        self.write_register(BMI160_COMMAND_REG, 0x03)  # Start normal mode
        time.sleep(0.25)  # Wait for reset

    def read_accel_gyro(self):
        """Read accelerometer and gyroscope data."""
        # Use a bytearray buffer to hold the read data
        buffer = bytearray(12)  # 6 bytes for accel (X, Y, Z) and 6 bytes for gyro (X, Y, Z)
        
        # Read the accelerometer and gyroscope data in one go
        self.i2c.readfrom_mem_into(self.address, BMI160_ACC_X_LSB, buffer)
        
        # Extract accelerometer data (2 bytes for each axis)
        accel_x = (buffer[1] << 8) | buffer[0]
        accel_y = (buffer[3] << 8) | buffer[2]
        accel_z = (buffer[5] << 8) | buffer[4]
        
        # Extract gyroscope data (2 bytes for each axis)
        gyro_x = (buffer[7] << 8) | buffer[6]
        gyro_y = (buffer[9] << 8) | buffer[8]
        gyro_z = (buffer[11] << 8) | buffer[10]
        
        print(f"Raw Accel X: {accel_x}, Y: {accel_y}, Z: {accel_z}")
        print(f"Raw Gyro X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}")
        # Handle signed values for accelerometer and gyroscope (16-bit signed integers)
        if accel_x > 32767:
            accel_x -= 65536
        if accel_y > 32767:
            accel_y -= 65536
        if accel_z > 32767:
            accel_z -= 65536
        if gyro_x > 32767:
            gyro_x -= 65536
        if gyro_y > 32767:
            gyro_y -= 65536
        if gyro_z > 32767:
            gyro_z -= 65536
        
        # Scale factors
        accel_scale = 2 / 32768  # ±2g
        gyro_scale = 2000 / 32768  # ±2000dps

        # Convert to G and dps values
        accel_x *= accel_scale
        accel_y *= accel_scale
        accel_z *= accel_scale
        gyro_x *= gyro_scale
        gyro_y *= gyro_scale
        gyro_z *= gyro_scale

        return {
            "accel": (accel_x, accel_y, accel_z),
            "gyro": (gyro_x, gyro_y, gyro_z)
        }

# Example usage:
if __name__ == "__main__":
    # Setup I2C
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))

    try:
        bmi160 = BMI160(i2c)
        print("Sensor Initialized")
        while True:
            sensor_data = bmi160.read_accel_gyro()
            print("Accel (g):", sensor_data["accel"])
            print("Gyro (dps):", sensor_data["gyro"])
            time.sleep(1)
    except Exception as e:
        print("Error:", e)
