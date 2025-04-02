from machine import I2C, Pin
import uasyncio as asyncio

# Register addresses as global variables
AS5600_REG_STATUS = 0x00
AS5600_REG_ANGLE_HIGH = 0x0C
AS5600_REG_ANGLE_LOW = 0x0D
AS5600_REG_MAGNITUDE_HIGH = 0x1A
AS5600_REG_MAGNITUDE_LOW = 0x1B
AS5600_ADDR_0 = 0x36
AS5600_ADDR_1 = 0x37

class AS5600:
    def __init__(self, i2c: I2C, address=0x36):
        """
        Initialize the AS5600 sensor over I2C.
        :param i2c: I2C instance (machine.I2C)
        :param address: I2C address of the AS5600 (default: 0x36)
        """
        self.i2c = i2c
        self.address = address
        
    def _read_register(self, reg: int, length: int = 1):
        """
        Helper function to read data from a register
        :param reg: Register address to read from
        :param length: Number of bytes to read
        :return: byte data read from the register
        """
        return self.i2c.readfrom_mem(self.address, reg, length)
    
    def _write_register(self, reg: int, data: bytes):
        """
        Helper function to write data to a register
        :param reg: Register address to write to
        :param data: Data to write (bytes)
        """
        self.i2c.writeto_mem(self.address, reg, data)
    
    def get_angle(self):
        """
        Get the angle from the AS5600 sensor (in degrees).
        :return: angle in degrees (float)
        """
        # Read high and low byte of angle
        angle_high = self._read_register(AS5600_REG_ANGLE_HIGH)
        angle_low = self._read_register(AS5600_REG_ANGLE_LOW)

        # Convert bytes to an integer value
        angle = (angle_high[0] << 8) | angle_low[0]
        
        # Convert angle to degrees (assuming 12-bit resolution)
        angle_deg = (angle / 4096) * 360
        return angle_deg

    def get_magnitude(self):
        """
        Get the magnetic field magnitude.
        :return: magnitude (int)
        """
        # Read high and low byte of magnitude
        magnitude_high = self._read_register(AS5600_REG_MAGNITUDE_HIGH)
        magnitude_low = self._read_register(AS5600_REG_MAGNITUDE_LOW)
        
        # Combine the two bytes into a single value
        magnitude = (magnitude_high[0] << 8) | magnitude_low[0]
        return magnitude

    def is_ready(self):
        """
        Check if the sensor is ready for a new reading (STATUS register check).
        :return: True if ready, False otherwise
        """
        status = self._read_register(AS5600_REG_STATUS)
        # Check the 'ready' bit in the STATUS register
        return (status[0] & 0x01) == 0x01


# Example usage:
if __name__ == "__main__":        # Setup I2C
    async def main():
        # Initialize I2C
        i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100_000)  # Adjust pin numbers as needed

        # Initialize AS5600 sensor
        sensor = AS5600(i2c)

        # Main loop
        while True:
            if sensor.is_ready():
                angle = sensor.get_angle()
                print(f"Angle: {angle:.2f}°")
                
                magnitude = sensor.get_magnitude()
                print(f"Magnitude: {magnitude}")
            
            await asyncio.sleep(0.1)

    # Run the asyncio event loop
    asyncio.run(main())
